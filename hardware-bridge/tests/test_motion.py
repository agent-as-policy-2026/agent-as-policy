import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pytest

from agp_yam_bridge.config import load_config
from agp_yam_bridge.kinematics import (
    I2rtKinematicsBackend,
    TrajectoryValidationError,
)
from agp_yam_bridge.motion import MotionController, MotionFault, MotionState
from agp_yam_bridge.preflight import DEFAULT_CONFIG


class _TrackingActuator:
    def __init__(self, joints: np.ndarray) -> None:
        self.joints = joints.astype(np.float64, copy=True)
        self.commands: list[np.ndarray] = []
        self.idle_count = 0
        self._sequence = 0
        self._lock = threading.Lock()

    def read_motion_state(self) -> MotionState:
        with self._lock:
            self._sequence += 1
            return MotionState(
                position=self.joints.copy(),
                velocity=np.zeros(7),
                effort=np.zeros(7),
                monotonic_ns=__import__("time").monotonic_ns(),
                sequence=self._sequence,
                motor_errors=(),
            )

    def command_joint_positions(self, target: np.ndarray) -> None:
        with self._lock:
            self.joints = target.astype(np.float64, copy=True)
            self.commands.append(self.joints.copy())

    def enter_safe_idle(self) -> None:
        self.idle_count += 1


class _SettlingActuator(_TrackingActuator):
    """Apply a target only after the planned trajectory has already ended."""

    def command_joint_positions(self, target: np.ndarray) -> None:
        with self._lock:
            command = target.astype(np.float64, copy=True)
            self.commands.append(command)
            if len(self.commands) >= 3:
                self.joints = command.copy()


class _DriftingSecondReadActuator(_TrackingActuator):
    """Move slightly between Cartesian planning and trajectory execution feedback."""

    def __init__(self, joints: np.ndarray) -> None:
        super().__init__(joints)
        self.read_count = 0
        self.precommand_read_count = 0
        self.commanded = False

    def read_motion_state(self) -> MotionState:
        self.read_count += 1
        if not self.commanded:
            self.precommand_read_count += 1
        if self.read_count == 2:
            self.joints[5] += 2e-6
        return super().read_motion_state()

    def command_joint_positions(self, target: np.ndarray) -> None:
        self.commanded = True
        super().command_joint_positions(target)


class _KnownSettleLagActuator(_TrackingActuator):
    """Track every command with a fixed error inside the established 0.03 rad band."""

    def command_joint_positions(self, target: np.ndarray) -> None:
        with self._lock:
            command = target.astype(np.float64, copy=True)
            self.commands.append(command)
            self.joints = command.copy()
            self.joints[5] -= 0.02


class _BoundaryDriftActuator(_TrackingActuator):
    """Enter the tolerance band for one frame, then drift back outside it."""

    def __init__(self, joints: np.ndarray) -> None:
        super().__init__(joints)
        self.command_count = 0
        self.reads_since_command = 0
        self.last_command = joints.astype(np.float64, copy=True)

    def read_motion_state(self) -> MotionState:
        with self._lock:
            if self.command_count == 2 and self.reads_since_command >= 1:
                self.joints = self.last_command.copy()
                self.joints[5] -= 0.011
            self.reads_since_command += 1
        return super().read_motion_state()

    def command_joint_positions(self, target: np.ndarray) -> None:
        with self._lock:
            self.command_count += 1
            self.reads_since_command = 0
            self.last_command = target.astype(np.float64, copy=True)
            self.commands.append(self.last_command.copy())
            self.joints = self.last_command.copy()
            self.joints[5] -= {1: 0.011, 2: 0.009}.get(self.command_count, 0.005)


class _UnresponsiveActuator(_TrackingActuator):
    """Record commands without moving, modelling a Cartesian axis that is stuck."""

    def command_joint_positions(self, target: np.ndarray) -> None:
        with self._lock:
            self.commands.append(target.astype(np.float64, copy=True))


def test_joint_trajectory_uses_feedback_and_records_replayable_events(tmp_path) -> None:
    """Skipping feedback or action logging would make a reported completion untrustworthy."""
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _TrackingActuator(start)
    log_path = tmp_path / "actions.jsonl"
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=log_path,
    )
    target = start[:6].copy()
    target[0] += 0.01
    try:
        result = controller.execute(
            request_id=17,
            action={
                "kind": "joint_trajectory",
                "waypoints": np.vstack((start[:6], target)).astype(np.float32),
                "waypoint_times_s": np.array([0.0, 0.1], dtype=np.float32),
                "timeout_s": 0.5,
                "position_tolerance_rad": 0.002,
            },
        )
        health = controller.health()
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["request_id"] == 17
    np.testing.assert_allclose(result["final_joint_pos_0"][:6], target, atol=0.002)
    assert health == {
        "state": "ok",
        "source_connected": True,
        "motion_enabled": True,
        "safety_state": "holding",
        "active_action_request_id": 17,
        "detail": "",
    }
    assert len(actuator.commands) >= 2
    records = [json.loads(line) for line in log_path.read_text().splitlines()]
    assert records[0]["event"] == "start"
    assert any(record["event"] == "feedback" for record in records)
    assert records[-1]["event"] == "result"
    assert records[-1]["status"] == "completed"


def test_joint_trajectory_holds_final_target_until_feedback_converges(tmp_path) -> None:
    """Actuator latency must use the timeout budget instead of causing an early failure."""
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _SettlingActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "settling.jsonl",
    )
    target = start[:6].copy()
    target[0] += 0.001
    try:
        result = controller.execute(
            request_id=171,
            action={
                "kind": "joint_trajectory",
                "waypoints": np.vstack((start[:6], target)).astype(np.float32),
                "waypoint_times_s": np.array([0.0, 0.02], dtype=np.float32),
                "timeout_s": 0.5,
                "position_tolerance_rad": 0.0001,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    np.testing.assert_allclose(result["final_joint_pos_0"][:6], target, atol=0.0001)
    assert len(actuator.commands) >= 3


def test_joint_trajectory_compensates_bounded_steady_joint_lag(tmp_path) -> None:
    """Repeatable PD steady-state error must not make a reachable target time out."""
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _KnownSettleLagActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "joint-settle.jsonl",
        heartbeat_timeout_s=2.0,
    )
    target = start[:6].copy()
    target[5] += 0.01
    try:
        result = controller.execute(
            request_id=172,
            action={
                "kind": "joint_trajectory",
                "waypoints": np.vstack((start[:6], target)).astype(np.float32),
                "waypoint_times_s": np.array([0.0, 0.1], dtype=np.float32),
                "timeout_s": 1.5,
                "position_tolerance_rad": 0.01,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    np.testing.assert_allclose(result["final_joint_pos_0"][:6], target, atol=0.01)
    assert max(command[5] for command in actuator.commands) > target[5]


def test_joint_settle_backtracks_an_unsafe_maximum_correction(
    tmp_path, monkeypatch
) -> None:
    """A smaller safe correction must be tried before rejecting a reachable target."""
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _KnownSettleLagActuator(start)
    backend = I2rtKinematicsBackend.from_config(config)
    target = start[:6].copy()
    target[5] += 0.01
    maximum_safe_bias = 0.012
    original_validate = backend.validate_trajectory

    def reject_only_large_settle_bias(waypoints, times, *, initial_joints):
        original_validate(waypoints, times, initial_joints=initial_joints)
        final = np.asarray(waypoints, dtype=np.float64)[-1]
        if final[5] - target[5] > maximum_safe_bias:
            raise TrajectoryValidationError(
                "WORKSPACE",
                "synthetic workspace boundary rejects the maximum settle bias",
            )

    monkeypatch.setattr(backend, "validate_trajectory", reject_only_large_settle_bias)
    controller = MotionController(
        actuator,
        backend,
        config,
        action_log_path=tmp_path / "joint-settle-backtrack.jsonl",
        heartbeat_timeout_s=2.0,
    )
    try:
        result = controller.execute(
            request_id=174,
            action={
                "kind": "joint_trajectory",
                "waypoints": np.vstack((start[:6], target)).astype(np.float32),
                "waypoint_times_s": np.array([0.0, 0.1], dtype=np.float32),
                "timeout_s": 1.5,
                "position_tolerance_rad": 0.009,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert max(command[5] - target[5] for command in actuator.commands) <= (
        maximum_safe_bias + 1e-9
    )


def test_joint_settle_timeout_completes_at_last_safe_boundary_command(tmp_path, monkeypatch) -> None:
    """A residual position-tolerance miss must not fail an otherwise safe move."""
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _KnownSettleLagActuator(start)
    backend = I2rtKinematicsBackend.from_config(config)
    target = start[:6].copy()
    target[5] += 0.01
    maximum_safe_bias = 0.012
    original_validate = backend.validate_trajectory

    def reject_past_boundary(waypoints, times, *, initial_joints):
        original_validate(waypoints, times, initial_joints=initial_joints)
        final = np.asarray(waypoints, dtype=np.float64)[-1]
        if final[5] - target[5] > maximum_safe_bias:
            raise TrajectoryValidationError(
                "WORKSPACE",
                "synthetic workspace boundary rejects every positive next step",
            )

    monkeypatch.setattr(backend, "validate_trajectory", reject_past_boundary)
    controller = MotionController(
        actuator,
        backend,
        config,
        action_log_path=tmp_path / "joint-settle-hold-boundary.jsonl",
        heartbeat_timeout_s=2.0,
    )
    try:
        result = controller.execute(
            request_id=175,
            action={
                "kind": "joint_trajectory",
                "waypoints": np.vstack((start[:6], target)).astype(np.float32),
                "waypoint_times_s": np.array([0.0, 0.1], dtype=np.float32),
                "timeout_s": 1.5,
                "position_tolerance_rad": 0.005,
            },
        )
        health = controller.health()
        idle_before_close = actuator.idle_count
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert np.max(np.abs(result["final_joint_pos_0"][:6] - target)) > 0.005
    assert health["safety_state"] == "holding"
    assert idle_before_close == 0
    assert max(command[5] - target[5] for command in actuator.commands) <= (
        maximum_safe_bias + 1e-9
    )


def test_joint_trajectory_requires_stable_in_tolerance_feedback(tmp_path) -> None:
    """One boundary frame must not report completion before the joint drifts out again."""
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _BoundaryDriftActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "joint-boundary-drift.jsonl",
        heartbeat_timeout_s=2.0,
    )
    target = start[:6].copy()
    try:
        result = controller.execute(
            request_id=173,
            action={
                "kind": "joint_trajectory",
                "waypoints": np.vstack((target, target)).astype(np.float32),
                "waypoint_times_s": np.array([0.0, 0.02], dtype=np.float32),
                "timeout_s": 1.0,
                "position_tolerance_rad": 0.01,
            },
        )
        stable = actuator.read_motion_state().position[:6]
    finally:
        controller.close()

    assert result["status"] == "completed"
    np.testing.assert_allclose(stable, target, atol=0.01)
    assert len(actuator.commands) >= 3


def test_command_heartbeat_timeout_cancels_motion_and_enters_safe_idle(tmp_path) -> None:
    """Continuing the last target after the client heartbeat dies is the P4 watchdog hazard."""
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _TrackingActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "watchdog.jsonl",
        heartbeat_timeout_s=0.05,
    )
    target = start[:6].copy()
    target[0] += 0.05
    try:
        result = controller.execute(
            request_id=18,
            action={
                "kind": "joint_trajectory",
                "waypoints": np.vstack((start[:6], target)).astype(np.float32),
                "waypoint_times_s": np.array([0.0, 0.5], dtype=np.float32),
                "timeout_s": 1.0,
                "position_tolerance_rad": 0.002,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "cancelled"
    assert result["detail"] == "command heartbeat timed out"
    assert actuator.idle_count >= 1
    assert actuator.commands[-1][0] < target[0]


def test_stale_feedback_fails_closed_before_first_motor_command(tmp_path) -> None:
    """A live request must not make an old motor frame safe to act on."""

    class StaleActuator(_TrackingActuator):
        def read_motion_state(self) -> MotionState:
            state = super().read_motion_state()
            return MotionState(
                position=state.position,
                velocity=state.velocity,
                effort=state.effort,
                monotonic_ns=state.monotonic_ns - 1_000_000_000,
                sequence=state.sequence,
                motor_errors=(),
            )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = StaleActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "stale.jsonl",
        feedback_stale_after_s=0.05,
    )
    try:
        with pytest.raises(MotionFault) as exc_info:
            controller.execute(
                request_id=19,
                action={
                    "kind": "joint_trajectory",
                    "waypoints": np.vstack((start[:6], start[:6])).astype(np.float32),
                    "waypoint_times_s": np.array([0.0, 0.1], dtype=np.float32),
                    "timeout_s": 0.5,
                    "position_tolerance_rad": 0.002,
                },
            )
    finally:
        controller.close()

    assert exc_info.value.code == "STALE_FEEDBACK"
    assert actuator.commands == []
    assert actuator.idle_count >= 1


def test_explicit_cancel_interrupts_matching_trajectory_and_idles(tmp_path) -> None:
    """Cancellation that only changes metadata would leave the arm moving."""
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _TrackingActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "cancel.jsonl",
        heartbeat_timeout_s=1.0,
    )
    target = start[:6].copy()
    target[0] += 0.05
    action = {
        "kind": "joint_trajectory",
        "waypoints": np.vstack((start[:6], target)).astype(np.float32),
        "waypoint_times_s": np.array([0.0, 0.5], dtype=np.float32),
        "timeout_s": 1.0,
        "position_tolerance_rad": 0.002,
    }
    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(controller.execute, request_id=20, action=action)
            deadline = time.monotonic() + 0.3
            while not actuator.commands and time.monotonic() < deadline:
                time.sleep(0.005)
            cancel_result = controller.cancel(20)
            result = future.result(timeout=0.5)
    finally:
        controller.close()

    assert cancel_result["status"] == "cancelled"
    assert result["status"] == "cancelled"
    assert result["detail"] == "cancelled by request"
    assert actuator.idle_count >= 1
    assert actuator.commands[-1][0] < target[0]


def test_sustained_tracking_error_faults_and_idles(tmp_path) -> None:
    """A responsive CAN loop that never follows its target must not report success."""

    class StuckActuator(_TrackingActuator):
        def command_joint_positions(self, target: np.ndarray) -> None:
            with self._lock:
                self.commands.append(target.astype(np.float64, copy=True))

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = StuckActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "tracking.jsonl",
        heartbeat_timeout_s=1.0,
        max_tracking_error_rad=0.01,
        tracking_error_timeout_s=0.04,
    )
    target = start[:6].copy()
    target[0] += 0.05
    try:
        with pytest.raises(MotionFault) as exc_info:
            controller.execute(
                request_id=21,
                action={
                    "kind": "joint_trajectory",
                    "waypoints": np.vstack((start[:6], target)).astype(np.float32),
                    "waypoint_times_s": np.array([0.0, 0.5], dtype=np.float32),
                    "timeout_s": 1.0,
                    "position_tolerance_rad": 0.002,
                },
            )
    finally:
        controller.close()

    assert exc_info.value.code == "TRACKING_ERROR"
    assert actuator.idle_count >= 1
    assert len(actuator.commands) < 26


def test_disabled_tracking_error_gate_records_lag_without_faulting(tmp_path) -> None:
    """Disabling the tracking gate must not hide the measured tracking error."""
    config = load_config(DEFAULT_CONFIG)
    config = config.model_copy(
        update={
            "safety": config.safety.model_copy(
                update={"max_tracking_error_deg": None}
            )
        }
    )
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _UnresponsiveActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "tracking-disabled.jsonl",
        heartbeat_timeout_s=1.0,
    )
    target = start[:6].copy()
    target[0] += 0.05
    try:
        result = controller.execute(
            request_id=22,
            action={
                "kind": "joint_trajectory",
                "waypoints": np.vstack((start[:6], target)).astype(np.float32),
                "waypoint_times_s": np.array([0.0, 0.2], dtype=np.float32),
                "timeout_s": 0.3,
                "position_tolerance_rad": 0.002,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["max_tracking_error_rad"] >= 0.049
    assert "joint settle tolerance not reached" in result["detail"]


def test_gripper_command_is_rate_limited_and_keeps_arm_feedback_target(tmp_path) -> None:
    """Sending a gripper-only vector or jumping its fraction would disturb the arm contract."""
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _TrackingActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper.jsonl",
        heartbeat_timeout_s=1.0,
    )
    try:
        result = controller.execute(
            request_id=22,
            action={
                "kind": "gripper",
                "open_fraction": 0.52,
                "stop_on_contact": False,
                "timeout_s": 0.5,
                "position_tolerance_fraction": 0.005,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["kind"] == "gripper"
    np.testing.assert_allclose(actuator.commands[-1][:6], start[:6])
    assert actuator.commands[-1][6] == pytest.approx(0.52)
    per_step = np.abs(np.diff([start[6], *[command[6] for command in actuator.commands]]))
    assert np.max(per_step) <= 0.25 / 50.0 + 1e-12


def test_gripper_overcurrent_faults_before_finishing_travel(tmp_path) -> None:
    """Ignoring excessive gripper effort would turn a jam into a successful close."""

    class OvercurrentActuator(_TrackingActuator):
        def read_motion_state(self) -> MotionState:
            state = super().read_motion_state()
            effort = state.effort.copy()
            if self.commands:
                effort[6] = 1.0
            return MotionState(
                position=state.position,
                velocity=state.velocity,
                effort=effort,
                monotonic_ns=state.monotonic_ns,
                sequence=state.sequence,
                motor_errors=(),
            )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = OvercurrentActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-overcurrent.jsonl",
        heartbeat_timeout_s=1.0,
        gripper_max_effort_nm=0.5,
    )
    try:
        with pytest.raises(MotionFault) as exc_info:
            controller.execute(
                request_id=23,
                action={
                    "kind": "gripper",
                    "open_fraction": 0.55,
                    "stop_on_contact": False,
                    "timeout_s": 1.0,
                    "position_tolerance_fraction": 0.005,
                },
            )
    finally:
        controller.close()

    assert exc_info.value.code == "GRIPPER_OVERCURRENT"
    assert actuator.idle_count >= 1
    assert actuator.commands[-1][6] < 0.55


def test_gripper_stall_is_detected_before_action_timeout(tmp_path) -> None:
    """Waiting for the hard timeout would keep pushing a mechanically stuck gripper."""

    class StuckGripperActuator(_TrackingActuator):
        def command_joint_positions(self, target: np.ndarray) -> None:
            with self._lock:
                self.commands.append(target.astype(np.float64, copy=True))

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = StuckGripperActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-stall.jsonl",
        heartbeat_timeout_s=1.0,
        gripper_max_effort_nm=10.0,
        gripper_stall_velocity_fraction_s=0.01,
        gripper_stall_error_fraction=0.02,
        gripper_stall_timeout_s=0.05,
    )
    try:
        with pytest.raises(MotionFault) as exc_info:
            controller.execute(
                request_id=24,
                action={
                    "kind": "gripper",
                    "open_fraction": 0.6,
                    "stop_on_contact": False,
                    "timeout_s": 1.0,
                    "position_tolerance_fraction": 0.005,
                },
            )
    finally:
        controller.close()

    assert exc_info.value.code == "GRIPPER_STALL"
    assert len(actuator.commands) < 20
    assert actuator.idle_count >= 1


def test_gripper_contact_is_a_completed_grasp_below_the_effort_limit(tmp_path) -> None:
    """A bounded object contact must hold the grasp instead of being classified as a jam."""

    class ContactActuator(_TrackingActuator):
        def command_joint_positions(self, target: np.ndarray) -> None:
            with self._lock:
                self.commands.append(target.astype(np.float64, copy=True))

        def read_motion_state(self) -> MotionState:
            state = super().read_motion_state()
            effort = state.effort.copy()
            if self.commands:
                effort[6] = 0.3
            return MotionState(
                position=state.position,
                velocity=state.velocity,
                effort=effort,
                monotonic_ns=state.monotonic_ns,
                sequence=state.sequence,
                motor_errors=(),
            )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = ContactActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-contact.jsonl",
        heartbeat_timeout_s=1.0,
        gripper_max_effort_nm=0.9,
        gripper_stall_timeout_s=0.05,
    )
    try:
        result = controller.execute(
            request_id=25,
            action={
                "kind": "gripper",
                "open_fraction": 0.0,
                "stop_on_contact": True,
                "timeout_s": 3.0,
                "position_tolerance_fraction": 0.005,
            },
        )
        health = controller.health()
        idle_count_before_close = actuator.idle_count
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["detail"] == "gripper contact"
    assert result["final_joint_pos_0"][6] == pytest.approx(0.5)
    assert health["safety_state"] == "holding"
    assert idle_count_before_close == 0


def test_single_gripper_effort_spike_is_not_object_contact(tmp_path) -> None:
    """One residual-effort frame at close startup must not report a successful grasp."""

    class TransientEffortActuator(_TrackingActuator):
        def read_motion_state(self) -> MotionState:
            state = super().read_motion_state()
            effort = state.effort.copy()
            velocity = state.velocity.copy()
            if len(self.commands) == 1:
                effort[6] = 0.21
            if self.commands and state.position[6] > 0.0:
                velocity[6] = -0.25
            return MotionState(
                position=state.position,
                velocity=velocity,
                effort=effort,
                monotonic_ns=state.monotonic_ns,
                sequence=state.sequence,
                motor_errors=(),
            )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.1)
    actuator = TransientEffortActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-transient-effort.jsonl",
        heartbeat_timeout_s=1.0,
        gripper_stall_timeout_s=0.05,
    )
    try:
        result = controller.execute(
            request_id=251,
            action={
                "kind": "gripper",
                "open_fraction": 0.0,
                "stop_on_contact": True,
                "timeout_s": 1.0,
                "position_tolerance_fraction": 0.005,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["detail"] == ""
    assert result["final_joint_pos_0"][6] == pytest.approx(0.0)


def test_moving_gripper_effort_is_not_object_contact(tmp_path) -> None:
    """Effort while the fingers are still moving must not be classified as contact."""

    class MovingEffortActuator(_TrackingActuator):
        def read_motion_state(self) -> MotionState:
            state = super().read_motion_state()
            effort = state.effort.copy()
            velocity = state.velocity.copy()
            if self.commands and state.position[6] > 0.0:
                effort[6] = 0.3
                velocity[6] = -0.25
            return MotionState(
                position=state.position,
                velocity=velocity,
                effort=effort,
                monotonic_ns=state.monotonic_ns,
                sequence=state.sequence,
                motor_errors=(),
            )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.1)
    actuator = MovingEffortActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-moving-effort.jsonl",
        heartbeat_timeout_s=1.0,
        gripper_stall_timeout_s=0.05,
    )
    try:
        result = controller.execute(
            request_id=252,
            action={
                "kind": "gripper",
                "open_fraction": 0.0,
                "stop_on_contact": True,
                "timeout_s": 1.0,
                "position_tolerance_fraction": 0.005,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["detail"] == ""
    assert result["final_joint_pos_0"][6] == pytest.approx(0.0)


def test_gripper_contact_stops_at_contact_effort_before_overcurrent(tmp_path) -> None:
    """Intentional contact must stop squeezing before its effort can reach the fault gate."""

    class RisingContactActuator(_TrackingActuator):
        def command_joint_positions(self, target: np.ndarray) -> None:
            with self._lock:
                self.commands.append(target.astype(np.float64, copy=True))

        def read_motion_state(self) -> MotionState:
            state = super().read_motion_state()
            effort = state.effort.copy()
            if self.commands:
                still_closing = (
                    len(self.commands) >= 2
                    and self.commands[-1][6] < self.commands[-2][6] - 1e-12
                )
                effort[6] = 1.0 if still_closing else 0.3
            return MotionState(
                position=state.position,
                velocity=state.velocity,
                effort=effort,
                monotonic_ns=state.monotonic_ns,
                sequence=state.sequence,
                motor_errors=(),
            )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = RisingContactActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-contact-before-overcurrent.jsonl",
        heartbeat_timeout_s=1.0,
        gripper_max_effort_nm=0.9,
        gripper_contact_min_effort_nm=0.2,
        gripper_stall_timeout_s=0.5,
    )
    try:
        result = controller.execute(
            request_id=26,
            action={
                "kind": "gripper",
                "open_fraction": 0.0,
                "stop_on_contact": True,
                "timeout_s": 3.0,
                "position_tolerance_fraction": 0.005,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["detail"] == "gripper contact"
    assert min(command[6] for command in actuator.commands) >= 0.495
    assert actuator.commands[-1][6] == pytest.approx(
        min(command[6] for command in actuator.commands)
    )
    assert actuator.commands[-1][6] <= result["final_joint_pos_0"][6]


def test_gripper_contact_pauses_before_slow_compliance_reaches_overcurrent(
    tmp_path,
) -> None:
    """Real contact can keep creeping faster than the stall gate while effort rises."""

    class SlowlyCompressingContactActuator(_TrackingActuator):
        def __init__(self, joints: np.ndarray) -> None:
            super().__init__(joints)
            self.velocity = np.zeros(7)
            self.effort = np.zeros(7)

        def command_joint_positions(self, target: np.ndarray) -> None:
            with self._lock:
                command = target.astype(np.float64, copy=True)
                repeated_command = bool(
                    self.commands
                    and np.isclose(command[6], self.commands[-1][6], atol=1e-12)
                )
                self.commands.append(command)
                self.joints[:6] = command[:6]
                closing = command[6] < self.joints[6] - 1e-12
                if closing and not repeated_command:
                    self.joints[6] = max(0.65, self.joints[6] - 0.001)
                    self.velocity[6] = -0.05
                    compression = self.joints[6] - command[6]
                    self.effort[6] = min(1.0, 0.15 + 80.0 * compression)
                else:
                    self.velocity[6] = 0.0
                    self.effort[6] = 0.3 if self.joints[6] > 0.05 else 0.0

        def read_motion_state(self) -> MotionState:
            with self._lock:
                self._sequence += 1
                return MotionState(
                    position=self.joints.copy(),
                    velocity=self.velocity.copy(),
                    effort=self.effort.copy(),
                    monotonic_ns=time.monotonic_ns(),
                    sequence=self._sequence,
                    motor_errors=(),
                )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.75)
    actuator = SlowlyCompressingContactActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-slow-compliance.jsonl",
        heartbeat_timeout_s=1.0,
        gripper_max_effort_nm=0.9,
        gripper_contact_min_effort_nm=0.2,
        gripper_stall_timeout_s=0.05,
    )
    try:
        result = controller.execute(
            request_id=261,
            action={
                "kind": "gripper",
                "open_fraction": 0.0,
                "stop_on_contact": True,
                "timeout_s": 1.0,
                "position_tolerance_fraction": 0.02,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["detail"] == "gripper contact"
    assert result["final_joint_pos_0"][6] > 0.65
    assert min(command[6] for command in actuator.commands) > 0.65


def test_gripper_contact_tolerates_one_effort_sample_below_acquisition_threshold(
    tmp_path,
) -> None:
    """Quantized effort jitter must not resume squeezing an already confirmed contact."""

    class ContactWithEffortDipActuator(_TrackingActuator):
        def __init__(self, joints: np.ndarray) -> None:
            super().__init__(joints)
            self.contact_command: float | None = None
            self.hold_reads = 0
            self.velocity = np.zeros(7)
            self.effort = np.zeros(7)

        def command_joint_positions(self, target: np.ndarray) -> None:
            with self._lock:
                command = target.astype(np.float64, copy=True)
                repeated_command = bool(
                    self.commands
                    and np.isclose(command[6], self.commands[-1][6], atol=1e-12)
                )
                self.commands.append(command)
                self.joints[:6] = command[:6]
                if self.contact_command is None and command[6] <= 0.70:
                    self.contact_command = float(command[6])
                    self.joints[6] = command[6]
                    self.velocity[6] = 0.0
                    self.effort[6] = 0.3
                elif repeated_command and self.contact_command is not None:
                    self.hold_reads += 1
                    self.velocity[6] = 0.0
                    self.effort[6] = 0.18 if self.hold_reads == 3 else 0.3
                elif self.contact_command is not None:
                    self.velocity[6] = 0.0
                    self.effort[6] = 1.0
                else:
                    self.joints[6] = command[6]
                    self.velocity[6] = -0.25

        def read_motion_state(self) -> MotionState:
            with self._lock:
                self._sequence += 1
                return MotionState(
                    position=self.joints.copy(),
                    velocity=self.velocity.copy(),
                    effort=self.effort.copy(),
                    monotonic_ns=time.monotonic_ns(),
                    sequence=self._sequence,
                    motor_errors=(),
                )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.75)
    actuator = ContactWithEffortDipActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-contact-effort-dip.jsonl",
        heartbeat_timeout_s=1.0,
        gripper_max_effort_nm=0.9,
        gripper_contact_min_effort_nm=0.2,
        gripper_stall_timeout_s=0.1,
    )
    try:
        result = controller.execute(
            request_id=262,
            action={
                "kind": "gripper",
                "open_fraction": 0.0,
                "stop_on_contact": True,
                "timeout_s": 1.0,
                "position_tolerance_fraction": 0.02,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["detail"] == "gripper contact"
    assert actuator.contact_command is not None
    assert min(command[6] for command in actuator.commands) == pytest.approx(
        actuator.contact_command
    )


def test_gripper_contact_confirmation_finishes_before_hold_effort_reaches_limit(
    tmp_path,
) -> None:
    """A soft grasp must not inherit the longer mechanical-stall fault timer."""

    class RisingHeldContactActuator(_TrackingActuator):
        def __init__(self, joints: np.ndarray) -> None:
            super().__init__(joints)
            self.contact_command: float | None = None
            self.hold_commands = 0
            self.velocity = np.zeros(7)
            self.effort = np.zeros(7)

        def command_joint_positions(self, target: np.ndarray) -> None:
            with self._lock:
                command = target.astype(np.float64, copy=True)
                repeated_command = bool(
                    self.commands
                    and np.isclose(command[6], self.commands[-1][6], atol=1e-12)
                )
                self.commands.append(command)
                self.joints[:6] = command[:6]
                if self.contact_command is None and command[6] <= 0.70:
                    self.contact_command = float(command[6])
                    self.joints[6] = command[6]
                    self.velocity[6] = 0.0
                    self.effort[6] = 0.3
                elif repeated_command and self.contact_command is not None:
                    self.hold_commands += 1
                    self.velocity[6] = 0.0
                    self.effort[6] = min(1.0, 0.3 + 0.08 * self.hold_commands)
                elif self.contact_command is not None:
                    self.velocity[6] = 0.0
                    self.effort[6] = 1.0
                else:
                    self.joints[6] = command[6]
                    self.velocity[6] = -0.25

        def read_motion_state(self) -> MotionState:
            with self._lock:
                self._sequence += 1
                return MotionState(
                    position=self.joints.copy(),
                    velocity=self.velocity.copy(),
                    effort=self.effort.copy(),
                    monotonic_ns=time.monotonic_ns(),
                    sequence=self._sequence,
                    motor_errors=(),
                )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.75)
    actuator = RisingHeldContactActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-contact-confirmation.jsonl",
        heartbeat_timeout_s=1.0,
        gripper_max_effort_nm=0.9,
        gripper_contact_min_effort_nm=0.2,
    )
    try:
        result = controller.execute(
            request_id=263,
            action={
                "kind": "gripper",
                "open_fraction": 0.0,
                "stop_on_contact": True,
                "timeout_s": 1.0,
                "position_tolerance_fraction": 0.02,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["detail"] == "gripper contact"
    assert 4 <= actuator.hold_commands <= 6
    assert actuator.effort[6] < 0.9


def test_gripper_contact_accepts_slow_compliance_above_mechanical_stall_velocity(
    tmp_path,
) -> None:
    """A soft object may keep compressing slowly after sustained contact begins."""

    class SlowContactActuator(_TrackingActuator):
        def __init__(self, joints: np.ndarray) -> None:
            super().__init__(joints)
            self.contact_command: float | None = None
            self.velocity = np.zeros(7)
            self.effort = np.zeros(7)

        def command_joint_positions(self, target: np.ndarray) -> None:
            with self._lock:
                command = target.astype(np.float64, copy=True)
                repeated_command = bool(
                    self.commands
                    and np.isclose(command[6], self.commands[-1][6], atol=1e-12)
                )
                self.commands.append(command)
                self.joints[:6] = command[:6]
                if self.contact_command is None and command[6] <= 0.70:
                    self.contact_command = float(command[6])
                    self.joints[6] = command[6] + 0.01
                    self.velocity[6] = -0.05
                    self.effort[6] = 0.3
                elif repeated_command and self.contact_command is not None:
                    self.joints[6] = max(
                        self.contact_command,
                        self.joints[6] - 0.001,
                    )
                    self.velocity[6] = -0.05
                    self.effort[6] = 0.4
                elif self.contact_command is not None:
                    self.velocity[6] = 0.0
                    self.effort[6] = 1.0
                else:
                    self.joints[6] = command[6]
                    self.velocity[6] = -0.25

        def read_motion_state(self) -> MotionState:
            with self._lock:
                self._sequence += 1
                return MotionState(
                    position=self.joints.copy(),
                    velocity=self.velocity.copy(),
                    effort=self.effort.copy(),
                    monotonic_ns=time.monotonic_ns(),
                    sequence=self._sequence,
                    motor_errors=(),
                )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.75)
    actuator = SlowContactActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-slow-contact-velocity.jsonl",
        heartbeat_timeout_s=1.0,
        gripper_max_effort_nm=0.9,
        gripper_contact_min_effort_nm=0.2,
    )
    try:
        result = controller.execute(
            request_id=264,
            action={
                "kind": "gripper",
                "open_fraction": 0.0,
                "stop_on_contact": True,
                "timeout_s": 1.0,
                "position_tolerance_fraction": 0.02,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["detail"] == "gripper contact"
    assert actuator.contact_command is not None
    assert min(command[6] for command in actuator.commands) == pytest.approx(
        actuator.contact_command
    )


def test_gripper_contact_rearms_after_a_rejected_effort_transient(tmp_path) -> None:
    """A one-frame effort spike must not mask the later sustained object contact."""

    class TransientThenContactActuator(_TrackingActuator):
        def __init__(self, joints: np.ndarray) -> None:
            super().__init__(joints)
            self.velocity = np.zeros(7)
            self.effort = np.zeros(7)
            self.transient_command: float | None = None

        def command_joint_positions(self, target: np.ndarray) -> None:
            with self._lock:
                command = target.astype(np.float64, copy=True)
                repeated_command = bool(
                    self.commands
                    and np.isclose(command[6], self.commands[-1][6], atol=1e-12)
                )
                self.commands.append(command)
                self.joints[:6] = command[:6]
                if self.transient_command is None and command[6] <= 0.70:
                    self.transient_command = float(command[6])
                    self.joints[6] = command[6]
                    self.velocity[6] = -0.05
                    self.effort[6] = 0.3
                elif (
                    repeated_command
                    and self.transient_command is not None
                    and np.isclose(command[6], self.transient_command, atol=1e-12)
                ):
                    self.velocity[6] = 0.0
                    self.effort[6] = 0.1
                elif (
                    self.transient_command is not None
                    and command[6] < self.transient_command - 1e-12
                ):
                    self.joints[6] = self.transient_command
                    self.velocity[6] = 0.0
                    self.effort[6] = 0.3
                else:
                    self.joints[6] = command[6]
                    self.velocity[6] = -0.05
                    self.effort[6] = 0.1

        def read_motion_state(self) -> MotionState:
            with self._lock:
                self._sequence += 1
                return MotionState(
                    position=self.joints.copy(),
                    velocity=self.velocity.copy(),
                    effort=self.effort.copy(),
                    monotonic_ns=time.monotonic_ns(),
                    sequence=self._sequence,
                    motor_errors=(),
                )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.75)
    actuator = TransientThenContactActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-rearm-after-transient.jsonl",
        heartbeat_timeout_s=1.0,
        gripper_max_effort_nm=0.9,
        gripper_contact_min_effort_nm=0.2,
        gripper_stall_timeout_s=0.05,
    )
    try:
        result = controller.execute(
            request_id=262,
            action={
                "kind": "gripper",
                "open_fraction": 0.0,
                "stop_on_contact": True,
                "timeout_s": 1.0,
                "position_tolerance_fraction": 0.02,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["detail"] == "gripper contact"
    assert actuator.transient_command is not None
    assert result["final_joint_pos_0"][6] == pytest.approx(
        actuator.transient_command
    )


def test_repeated_gripper_effort_spikes_do_not_consume_action_timeout(
    tmp_path,
) -> None:
    """Command-edge spikes must not halve the effective gripper close rate."""

    class CommandEdgeSpikeActuator(_TrackingActuator):
        def __init__(self, joints: np.ndarray) -> None:
            super().__init__(joints)
            self.velocity = np.zeros(7)
            self.effort = np.zeros(7)

        def command_joint_positions(self, target: np.ndarray) -> None:
            with self._lock:
                command = target.astype(np.float64, copy=True)
                repeated_command = bool(
                    self.commands
                    and np.isclose(command[6], self.commands[-1][6], atol=1e-12)
                )
                previous = self.joints[6]
                self.commands.append(command)
                self.joints = command.copy()
                self.velocity[6] = (
                    0.0 if repeated_command else (command[6] - previous) / 0.02
                )
                self.effort[6] = 0.1 if repeated_command else 0.3

        def read_motion_state(self) -> MotionState:
            with self._lock:
                self._sequence += 1
                return MotionState(
                    position=self.joints.copy(),
                    velocity=self.velocity.copy(),
                    effort=self.effort.copy(),
                    monotonic_ns=time.monotonic_ns(),
                    sequence=self._sequence,
                    motor_errors=(),
                )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.2)
    actuator = CommandEdgeSpikeActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-command-edge-spikes.jsonl",
        heartbeat_timeout_s=2.0,
        gripper_max_effort_nm=0.9,
        gripper_contact_min_effort_nm=0.2,
        gripper_stall_timeout_s=0.05,
    )
    try:
        result = controller.execute(
            request_id=263,
            action={
                "kind": "gripper",
                "open_fraction": 0.0,
                "stop_on_contact": True,
                "timeout_s": 1.2,
                "position_tolerance_fraction": 0.02,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["detail"] == ""
    assert result["final_joint_pos_0"][6] == pytest.approx(0.0)


def test_gripper_close_ignores_residual_effort_from_opening_direction(tmp_path) -> None:
    """A negative opening transient must not be mistaken for positive close contact."""

    class OpeningResidualActuator(_TrackingActuator):
        def read_motion_state(self) -> MotionState:
            state = super().read_motion_state()
            effort = state.effort.copy()
            velocity = state.velocity.copy()
            if self.commands:
                effort[6] = -0.3
                velocity[6] = -0.25
            return MotionState(
                position=state.position,
                velocity=velocity,
                effort=effort,
                monotonic_ns=state.monotonic_ns,
                sequence=state.sequence,
                motor_errors=(),
            )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = OpeningResidualActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-opening-residual.jsonl",
        heartbeat_timeout_s=5.0,
    )
    try:
        result = controller.execute(
            request_id=27,
            action={
                "kind": "gripper",
                "open_fraction": 0.0,
                "stop_on_contact": True,
                "timeout_s": 3.0,
                "position_tolerance_fraction": 0.02,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["detail"] == ""
    assert result["final_joint_pos_0"][6] == pytest.approx(0.0)


def test_gripper_empty_endpoint_friction_is_not_object_contact(tmp_path) -> None:
    """Positive friction inside the accepted empty band must remain an empty close."""

    class EndpointFrictionActuator(_TrackingActuator):
        def read_motion_state(self) -> MotionState:
            state = super().read_motion_state()
            effort = state.effort.copy()
            velocity = state.velocity.copy()
            if self.commands:
                effort[6] = 0.3 if state.position[6] <= 0.05 else 0.1
                velocity[6] = -0.25
            return MotionState(
                position=state.position,
                velocity=velocity,
                effort=effort,
                monotonic_ns=state.monotonic_ns,
                sequence=state.sequence,
                motor_errors=(),
            )

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = EndpointFrictionActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "gripper-empty-endpoint-friction.jsonl",
        heartbeat_timeout_s=5.0,
    )
    try:
        result = controller.execute(
            request_id=28,
            action={
                "kind": "gripper",
                "open_fraction": 0.0,
                "stop_on_contact": True,
                "timeout_s": 3.0,
                "position_tolerance_fraction": 0.02,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["detail"] == ""
    assert result["final_joint_pos_0"][6] == pytest.approx(0.0)


def test_absolute_joint_target_is_reshaped_into_a_bounded_smooth_trajectory(tmp_path) -> None:
    """Forwarding one absolute target directly would bypass step, velocity, and acceleration limits."""
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _TrackingActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "absolute.jsonl",
        heartbeat_timeout_s=2.0,
    )
    target = start[:6].copy()
    target[0] += 0.02
    try:
        result = controller.execute(
            request_id=25,
            action={
                "kind": "absolute_joints",
                "joint_target": target.astype(np.float32),
                "timeout_s": 2.0,
                "position_tolerance_rad": 0.002,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["kind"] == "absolute_joints"
    np.testing.assert_allclose(actuator.commands[-1][:6], target, atol=0.002)
    arm_commands = np.vstack((start[:6], *[command[:6] for command in actuator.commands]))
    max_step = np.radians(10.0) / 50.0
    assert np.max(np.abs(np.diff(arm_commands, axis=0))) <= max_step + 1e-9


def test_absolute_joint_target_uses_one_measured_start_for_planning_and_execution(
    tmp_path,
) -> None:
    """Planning and execution must not disagree because they sampled two idle states.

    Since 2026-09-03 a second, fresh read happens after planning (it proves the feedback
    stream is alive without re-validating the planning snapshot's age), but the trajectory
    still starts from the first read; a sub-tolerance drift between the two is accepted.
    """
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _DriftingSecondReadActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "absolute-feedback.jsonl",
        heartbeat_timeout_s=2.0,
    )
    target = start[:6].copy()
    target[0] += 0.02
    try:
        result = controller.execute(
            request_id=251,
            action={
                "kind": "absolute_joints",
                "joint_target": target.astype(np.float32),
                "timeout_s": 2.0,
                "position_tolerance_rad": 0.002,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert actuator.precommand_read_count == 2
    # the trajectory was planned from, and starts at, the FIRST read (before the drift)
    np.testing.assert_allclose(actuator.commands[0][5], start[5], atol=1e-9)


def test_hard_timeout_faults_and_idles_instead_of_leaking_active_hold(tmp_path) -> None:
    """Raising a bare timeout without changing hardware mode would leave the last PD target active."""
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _TrackingActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "timeout.jsonl",
        heartbeat_timeout_s=1.0,
    )
    target = start[:6].copy()
    target[0] += 0.05
    try:
        with pytest.raises(MotionFault) as exc_info:
            controller.execute(
                request_id=26,
                action={
                    "kind": "joint_trajectory",
                    "waypoints": np.vstack((start[:6], target)).astype(np.float32),
                    "waypoint_times_s": np.array([0.0, 0.5], dtype=np.float32),
                    "timeout_s": 0.05,
                    "position_tolerance_rad": 0.002,
                },
            )
        idle_before_close = actuator.idle_count
    finally:
        controller.close()

    assert exc_info.value.code == "ACTION_TIMEOUT"
    assert idle_before_close >= 1


def test_cartesian_pose_uses_grasp_site_ik_and_bounded_trajectory(tmp_path) -> None:
    """Dispatching Cartesian input without world/grasp-site IK would revive the invalid link_6 path."""
    config = load_config(DEFAULT_CONFIG)
    kinematics = I2rtKinematicsBackend.from_config(config)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _TrackingActuator(start)
    controller = MotionController(
        actuator,
        kinematics,
        config,
        action_log_path=tmp_path / "cartesian.jsonl",
        heartbeat_timeout_s=2.0,
    )
    target_pose = kinematics.forward(start[:6])
    target_pose[0] += 0.002
    try:
        result = controller.execute(
            request_id=27,
            action={
                "kind": "cartesian_pose",
                "frame": "world",
                "tcp": "grasp_site",
                "pose": target_pose.astype(np.float32),
                "timeout_s": 2.0,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    assert result["kind"] == "cartesian_pose"
    achieved = kinematics.forward(result["final_joint_pos_0"][:6])
    np.testing.assert_allclose(achieved[:3], target_pose[:3], atol=2e-4)


def test_cartesian_pose_does_not_resolve_the_zero_time_start_pose(
    tmp_path, monkeypatch
) -> None:
    """Numerical IK at the measured start pose must not create motion at time zero."""
    config = load_config(DEFAULT_CONFIG)
    kinematics = I2rtKinematicsBackend.from_config(config)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _TrackingActuator(start)
    controller = MotionController(
        actuator,
        kinematics,
        config,
        action_log_path=tmp_path / "cartesian-start.jsonl",
        heartbeat_timeout_s=2.0,
    )
    target_pose = kinematics.forward(start[:6])
    target_pose[0] += 0.002
    original_solve_ik = kinematics.solve_ik
    calls = 0

    def solve_ik_with_numerical_start_offset(pose, *, seed_joints):
        nonlocal calls
        solution = original_solve_ik(pose, seed_joints=seed_joints)
        calls += 1
        if calls == 1 and solution is not None:
            solution = solution.copy()
            solution[0] += 2e-6
        return solution

    monkeypatch.setattr(kinematics, "solve_ik", solve_ik_with_numerical_start_offset)
    try:
        result = controller.execute(
            request_id=271,
            action={
                "kind": "cartesian_pose",
                "frame": "world",
                "tcp": "grasp_site",
                "pose": target_pose.astype(np.float32),
                "timeout_s": 2.0,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"


def test_cartesian_pose_uses_one_measured_start_for_planning_and_execution(tmp_path) -> None:
    """Planning and execution must not disagree because they sampled two idle states."""
    config = load_config(DEFAULT_CONFIG)
    kinematics = I2rtKinematicsBackend.from_config(config)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _DriftingSecondReadActuator(start)
    controller = MotionController(
        actuator,
        kinematics,
        config,
        action_log_path=tmp_path / "cartesian-feedback.jsonl",
        heartbeat_timeout_s=2.0,
    )
    target_pose = kinematics.forward(start[:6])
    target_pose[0] += 0.002
    try:
        result = controller.execute(
            request_id=272,
            action={
                "kind": "cartesian_pose",
                "frame": "world",
                "tcp": "grasp_site",
                "pose": target_pose.astype(np.float32),
                "timeout_s": 2.0,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    # 2026-09-03: one read for planning, one fresh read after planning (liveness only);
    # the trajectory still starts from the first read.
    assert actuator.precommand_read_count == 2


def test_cartesian_pose_compensates_bounded_steady_joint_lag(tmp_path) -> None:
    """A bounded outer settle loop must remove repeatable PD steady-state error."""
    config = load_config(DEFAULT_CONFIG)
    kinematics = I2rtKinematicsBackend.from_config(config)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _KnownSettleLagActuator(start)
    controller = MotionController(
        actuator,
        kinematics,
        config,
        action_log_path=tmp_path / "cartesian-settle.jsonl",
        heartbeat_timeout_s=2.0,
    )
    target_pose = kinematics.forward(start[:6])
    target_pose[0] += 0.002
    try:
        result = controller.execute(
            request_id=273,
            action={
                "kind": "cartesian_pose",
                "frame": "world",
                "tcp": "grasp_site",
                "pose": target_pose.astype(np.float32),
                "timeout_s": 1.5,
            },
        )
    finally:
        controller.close()

    assert result["status"] == "completed"
    achieved = kinematics.forward(result["final_joint_pos_0"][:6])
    np.testing.assert_allclose(achieved[:3], target_pose[:3], atol=5e-4)
    assert max(abs(command[5] - start[5]) for command in actuator.commands) < 0.05


def test_cartesian_pose_does_not_complete_when_feedback_is_unresponsive(tmp_path) -> None:
    """Outer-loop commands must not turn unchanged Cartesian feedback into success."""
    config = load_config(DEFAULT_CONFIG)
    kinematics = I2rtKinematicsBackend.from_config(config)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _UnresponsiveActuator(start)
    controller = MotionController(
        actuator,
        kinematics,
        config,
        action_log_path=tmp_path / "cartesian-unresponsive.jsonl",
        heartbeat_timeout_s=2.0,
    )
    target_pose = kinematics.forward(start[:6])
    target_pose[0] += 0.002
    try:
        with pytest.raises(MotionFault) as exc_info:
            controller.execute(
                request_id=274,
                action={
                    "kind": "cartesian_pose",
                    "frame": "world",
                    "tcp": "grasp_site",
                    "pose": target_pose.astype(np.float32),
                    "timeout_s": 1.5,
                },
            )
    finally:
        controller.close()

    assert exc_info.value.code == "ACTION_TIMEOUT"
    assert "Cartesian pose did not converge" in str(exc_info.value)


def test_holding_watchdog_detects_control_thread_failure(tmp_path) -> None:
    """Heartbeat traffic alone must not mask a dead i2rt control producer after completion."""

    class FailingActuator(_TrackingActuator):
        fail_reads = False

        def read_motion_state(self) -> MotionState:
            if self.fail_reads:
                raise RuntimeError("control thread is not running")
            return super().read_motion_state()

    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = FailingActuator(start)
    controller = MotionController(
        actuator,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "holding-health.jsonl",
        heartbeat_timeout_s=1.0,
    )
    try:
        controller.execute(
            request_id=28,
            action={
                "kind": "joint_trajectory",
                "waypoints": np.vstack((start[:6], start[:6])).astype(np.float32),
                "waypoint_times_s": np.array([0.0, 0.02], dtype=np.float32),
                "timeout_s": 0.5,
                "position_tolerance_rad": 0.002,
            },
        )
        actuator.fail_reads = True
        deadline = time.monotonic() + 0.3
        while controller.health()["safety_state"] != "fault" and time.monotonic() < deadline:
            time.sleep(0.005)
        health = controller.health()
    finally:
        controller.close()

    assert health["state"] == "error"
    assert health["safety_state"] == "fault"
    assert "CONTROL_SOURCE" in health["detail"]
    assert actuator.idle_count >= 1
