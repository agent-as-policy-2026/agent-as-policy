"""Buffered joint programs (2026-09-14): port of the original throwing runtime's test_program_controller.py.

Real kinematics, fake motors, the left-arm throwing config. Start posture = the config's home
[0, 0, 0, 88, 90, 90] deg, so J4 test sweeps go NEGATIVE (the model's J4 upper limit is 90 deg).
"""

import json
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from agp_yam_bridge.config import load_config
from agp_yam_bridge.kinematics import I2rtKinematicsBackend
from agp_yam_bridge.motion import MotionState
from agp_yam_bridge.program_controller import ProgramController

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
THROW = CONFIG_DIR / "left_arm_throw.yaml"
PLAIN = CONFIG_DIR / "left_arm.yaml"


class Actuator:
    def __init__(self, position):
        self.position = np.asarray(position, dtype=float)
        self.commands = []
        self.idles = 0
        self.sequence = 0

    def read_motion_state(self):
        self.sequence += 1
        return MotionState(
            self.position.copy(),
            np.zeros(7),
            np.zeros(7),
            time.monotonic_ns(),
            self.sequence,
            (),
        )

    def command_joint_positions(self, target):
        self.commands.append(target.copy())
        self.position = target.copy()

    def enter_safe_idle(self):
        self.idles += 1


def build(tmp_path, config_path=THROW):
    config = load_config(config_path)
    motor = Actuator(np.append(np.radians(config.acceptance.home_joints_deg), 0.4))
    controller = ProgramController(
        motor,
        I2rtKinematicsBackend.from_config(config),
        config,
        log_dir=tmp_path,
        action_log_path=tmp_path / "actions.jsonl",
        heartbeat_timeout_s=5.0,
    )
    return controller, motor


def action():
    return {
        "kind": "joint_program",
        "program_id": "abc123",
        "program": {
            "times_s": [0, 0.1, 0.2, 0.3, 0.4],
            "joint_deltas_rad": [
                [0, 0, 0, -x, 0, 0] for x in [0, 0.0005, 0.0015, 0.0025, 0.003]
            ],
            "gripper_events": [{"time_s": 0.2, "fraction": 1.0}],
        },
    }


def test_throw_config_loads_with_program_limits():
    limits = load_config(THROW).acceptance.speed_limits
    assert limits.programs_enabled
    assert (limits.program_j4_velocity_deg_s, limits.program_j4_acceleration_deg_s2) == (180, 360)
    assert (limits.joint_velocity_deg_s, limits.joint_acceleration_deg_s2) == (20, 40)
    assert not load_config(PLAIN).acceptance.speed_limits.programs_enabled


def test_real_controller_produces_joint_and_gripper_trace(tmp_path):
    controller, motor = build(tmp_path)
    try:
        result = controller.execute(request_id=42, action=action())
        report = controller.report("abc123")
        assert result["kind"] == "joint_program"
        assert result["status"] == "completed"
        assert json.loads(result["detail"]) == {"program_id": "abc123", "measured_status": "completed"}
        assert report["status"] == "completed"
        opens = [q for q in motor.commands if q[6] == 1.0]
        assert opens and opens[0][3] < motor.commands[0][3]
        assert json.loads((tmp_path / "program_abc123.json").read_text())["trace"]
        assert controller.health()["safety_state"] == "holding"
        events = [json.loads(l)["event"] for l in (tmp_path / "actions.jsonl").read_text().splitlines()]
        assert events[0] == "start" and events[-1] == "result"
    finally:
        controller.close()


def test_kinematic_rejection_has_report_and_no_motor_command(tmp_path):
    controller, motor = build(tmp_path)
    bad = action()
    bad["program"]["joint_deltas_rad"][-1][3] = -100.0
    try:
        result = controller.execute(request_id=42, action=bad)
        assert json.loads(result["detail"])["measured_status"] == "rejected"
        assert not motor.commands
        assert controller.report("abc123")["status"] == "rejected"
        assert controller.health()["state"] == "ok"
    finally:
        controller.close()


def test_replaying_a_program_id_is_refused(tmp_path):
    controller, motor = build(tmp_path)
    try:
        controller.execute(request_id=42, action=action())
        sent = len(motor.commands)
        with pytest.raises(ValueError, match="already executed"):
            controller.execute(request_id=43, action=action())
        assert len(motor.commands) == sent
    finally:
        controller.close()


def test_unknown_report_is_a_structured_fault(tmp_path):
    controller, _ = build(tmp_path)
    try:
        with pytest.raises(Exception) as info:
            controller.report("never_ran")
        assert info.value.code == "PROGRAM_NOT_FOUND"
    finally:
        controller.close()


@pytest.mark.parametrize("kind", ["absolute_joints", "joint_program"])
def test_completed_lease_release_allows_next_session_motion(tmp_path, kind):
    controller, motor = build(tmp_path)
    ordinary = {
        "kind": "absolute_joints",
        "joint_target": motor.position[:6].copy(),
        "position_tolerance_rad": 0.03,
        "timeout_s": 1.0,
    }
    try:
        first = action() if kind == "joint_program" else ordinary
        assert controller.execute(request_id=42, action=first)["status"] == "completed"
        assert controller.health()["safety_state"] == "holding"
        sent = len(motor.commands)
        assert controller.cancel(42)["status"] == "cancelled"
        assert len(motor.commands) == sent and motor.idles == 1
        assert controller.health()["state"] == "ok"
        assert controller.health()["safety_state"] == "idle"
        assert controller.health()["active_action_request_id"] is None
        with pytest.raises(Exception, match="does not own"):
            controller.heartbeat(42)
        assert controller.execute(request_id=43, action=ordinary)["status"] == "completed"
        assert controller.health()["safety_state"] == "holding"
    finally:
        controller.close()


def test_cancellation_does_not_allow_queued_execution_to_revive_worker(tmp_path):
    controller, motor = build(tmp_path)
    entered = threading.Event()
    release = threading.Event()
    refresh = motor.read_motion_state

    def block():
        if motor.commands:
            entered.set()
            release.wait(2)
        return refresh()

    motor.read_motion_state = block
    errors = []

    def execute():
        try:
            controller.execute(request_id=42, action=action())
        except Exception as exc:
            errors.append(exc)

    worker = threading.Thread(target=execute)
    worker.start()
    try:
        assert entered.wait(2)
        canceller = threading.Thread(target=lambda: controller.cancel(42))
        canceller.start()
        sent = len(motor.commands)
        with pytest.raises(Exception, match="active"):
            controller.execute(request_id=43, action={**action(), "program_id": "other"})
        release.set()
        worker.join(2)
        canceller.join(2)
        assert not worker.is_alive() and len(motor.commands) == sent
        assert errors
        assert controller.health()["safety_state"] == "fault"
    finally:
        release.set()
        worker.join(2)
        controller.close()


def test_cancelled_owner_cannot_reapply_buffered_command_via_feedback_refresh(tmp_path):
    controller, motor = build(tmp_path)
    try:
        controller._begin(42)
        controller.cancel(42)
        before = motor.sequence
        with pytest.raises(Exception, match="cancel"):
            controller._program_feedback()
        assert motor.sequence == before
    finally:
        controller.close()


def test_cancellation_at_terminal_boundary_cannot_restore_holding(tmp_path, monkeypatch):
    import agp_yam_bridge.program_controller as module

    controller, motor = build(tmp_path)
    run = module.run_program

    def cancel_on_return(*args, **kwargs):
        result = run(*args, **kwargs)
        controller.cancel(42)
        return result

    monkeypatch.setattr(module, "run_program", cancel_on_return)
    try:
        with pytest.raises(Exception, match="cancel"):
            controller.execute(request_id=42, action=action())
        assert controller.health()["safety_state"] == "fault"
    finally:
        controller.close()


def test_watchdog_expiry_during_preparation_never_revives_motion(tmp_path, monkeypatch):
    controller, motor = build(tmp_path)
    controller._heartbeat_timeout_s = 0.5
    prepare = controller.prepare

    def slow(*args):
        time.sleep(0.6)
        return prepare(*args)

    monkeypatch.setattr(controller, "prepare", slow)
    try:
        with pytest.raises(Exception, match="heartbeat"):
            controller.execute(request_id=42, action=action())
        assert not motor.commands and motor.idles
        assert controller.report("abc123")["status"] == "fault"
    finally:
        controller.close()


def test_initial_feedback_motor_error_is_fault_not_planning_rejection(tmp_path):
    controller, motor = build(tmp_path)

    def fail():
        raise RuntimeError("CAN motor exchange failed")

    motor.read_motion_state = fail
    try:
        with pytest.raises(Exception):
            controller.execute(request_id=42, action=action())
        assert controller.report("abc123")["status"] == "fault"
        assert controller.health()["safety_state"] == "fault" and motor.idles
    finally:
        controller.close()


def test_nonfinite_authored_program_still_returns_recoverable_report(tmp_path):
    controller, motor = build(tmp_path)
    bad = action()
    bad["program"]["times_s"][-1] = float("nan")
    try:
        controller.execute(request_id=42, action=bad)
        report = controller.report("abc123")
        assert report["status"] == "rejected"
        assert report["invalid_program_repr"]
        assert not motor.commands
        assert controller.health()["state"] == "ok"
    finally:
        controller.close()


def test_j4_program_can_accelerate_to_180_degrees_per_second(tmp_path):
    controller, motor = build(tmp_path)
    try:
        # 360 deg/s² for .5 s, cruise .2 s, then brake for .5 s: 126 degrees (downwards from home).
        t = np.arange(61) * 0.02
        distance = np.where(
            t <= .5, 180 * t**2,
            np.where(t <= .7, 45 + 180 * (t - .5), 126 - 180 * (1.2 - t)**2),
        )
        delta = np.zeros((len(t), 6))
        delta[:, 3] = -np.radians(distance)
        preview = controller.preview({"times_s": t.tolist(), "joint_deltas_rad": delta.tolist()})
        assert preview["valid"]
        assert preview["peak_joint_velocity_rad_s"][3] == pytest.approx(np.pi)
        assert preview["peak_joint_acceleration_rad_s2"][3] == pytest.approx(2 * np.pi)
        assert len(preview["tcp_poses_wxyz"]) == len(preview["commands"])
        # Ordinary trajectories must still reject the same fast J4 movement.
        with pytest.raises(ValueError, match="JOINT_VELOCITY"):
            controller._kinematics.validate_trajectory(
                np.asarray(preview["commands"])[:, :6], preview["times_s"],
                initial_joints=motor.position[:6],
            )
        assert not motor.commands
    finally:
        controller.close()


def test_plain_left_config_rejects_throwing_speed_before_dispatch(tmp_path):
    controller, motor = build(tmp_path, PLAIN)
    try:
        p = {"times_s": [0, 1], "joint_deltas_rad": [[0] * 6, [0, 0, 0, -0.4, 0, 0]]}
        controller.execute(request_id=42, action={
            "kind": "joint_program", "program_id": "plain_fast", "program": p,
        })
        report = controller.report("plain_fast")
        assert report["status"] == "rejected"
        assert report["error_code"] == "JOINT_VELOCITY"
        assert not motor.commands
    finally:
        controller.close()


@pytest.mark.parametrize("joint", [0, 1, 2, 4, 5])
@pytest.mark.parametrize("positions,code", [
    ([0, -.008, -.016], "JOINT_VELOCITY"),  # .4 rad/s > 20 deg/s
    ([0, 0, -.001], "JOINT_ACCELERATION"),  # 2.5 rad/s² > 40 deg/s²
])
def test_other_program_joints_keep_low_rate_limits(tmp_path, joint, positions, code):
    controller, motor = build(tmp_path)
    try:
        delta = np.zeros((3, 6))
        # J2/J3 rest at their lower limit (0 rad) at home: move them upwards, the others down
        delta[:, joint] = -np.asarray(positions) if joint in (1, 2) else positions
        with pytest.raises(ValueError, match=code):
            controller.preview({"times_s": [0, .02, .04], "joint_deltas_rad": delta.tolist()})
        assert not motor.commands
    finally:
        controller.close()


@pytest.mark.parametrize("positions,code", [
    ([0, -.065, -.13], "JOINT_VELOCITY"),  # 3.25 rad/s > 180 deg/s
    ([0, 0, -.003], "JOINT_ACCELERATION"),  # 7.5 rad/s² > 360 deg/s²
    ([0, -.004, -.008], "JOINT_ACCELERATION"),  # abrupt start/stop at .2 rad/s
    ([0, -.001, -.003, -.006], "JOINT_ACCELERATION"),  # braking only: 7.5 rad/s²
])
def test_j4_program_rejects_excess_rates_before_dispatch(tmp_path, positions, code):
    controller, motor = build(tmp_path)
    try:
        delta = np.zeros((len(positions), 6))
        delta[:, 3] = positions
        controller.execute(request_id=42, action={
            "kind": "joint_program", "program_id": "too_fast",
            "program": {
                "times_s": (np.arange(len(positions)) * .02).tolist(),
                "joint_deltas_rad": delta.tolist(),
            },
        })
        report = controller.report("too_fast")
        assert report["status"] == "rejected"
        assert report["error_code"] == code
        assert not motor.commands
    finally:
        controller.close()


def test_ordinary_motor_failure_after_dispatch_latches_fault(tmp_path):
    controller, motor = build(tmp_path)
    read = motor.read_motion_state

    def failed_after_dispatch():
        if motor.commands:
            raise RuntimeError("CAN feedback failed during motion")
        return read()

    motor.read_motion_state = failed_after_dispatch
    try:
        with pytest.raises(Exception, match="CAN feedback failed"):
            controller.execute(
                request_id=7,
                action={
                    "kind": "absolute_joints",
                    "joint_target": motor.position[:6].copy(),
                    "position_tolerance_rad": 0.03,
                    "timeout_s": 1.0,
                },
            )
        assert controller.health()["safety_state"] == "fault"
        assert motor.idles > 0
    finally:
        controller.close()


def test_ordinary_move_after_latched_fault_still_clears_it(tmp_path):
    """Our semantics (unlike the original throwing runtime's): a new action clears a latched fault, which is what
    agp/tools/clear_bridge_fault.sh relies on after an interrupted session."""
    controller, motor = build(tmp_path)
    try:
        with controller._state_lock:
            controller._fault_detail = "command heartbeat timed out"
        assert controller.health()["safety_state"] == "fault"
        ordinary = {
            "kind": "absolute_joints",
            "joint_target": motor.position[:6].copy(),
            "position_tolerance_rad": 0.03,
            "timeout_s": 1.0,
        }
        assert controller.execute(request_id=9, action=ordinary)["status"] == "completed"
        assert controller.health()["state"] == "ok"
    finally:
        controller.close()
