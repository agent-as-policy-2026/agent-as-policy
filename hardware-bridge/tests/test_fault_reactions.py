"""Fault reactions that must not drop a held object (2026-09-03).

1. Planning slower than ``feedback_stale_after_s`` is not STALE_FEEDBACK: the old code
   re-validated the pre-planning snapshot and so measured its own planning time.
2. Arm drift between planning and execution is still refused, without idling.
3. A Cartesian settle miss keeps the last command active as a lease-monitored hold and
   reports ``SETTLE_MISS`` when ``safety.cartesian_settle_miss == "hold"`` (the legacy
   ``"idle"`` reaction is unchanged).
4. A fault raised before the action commands anything (planning phase) leaves the
   previous hold untouched: no gravity-comp idle, lease unchanged.
"""
import time

import numpy as np
import pytest
from test_motion import DEFAULT_CONFIG, _TrackingActuator, _UnresponsiveActuator

from agp_yam_bridge.config import load_config
from agp_yam_bridge.kinematics import I2rtKinematicsBackend
from agp_yam_bridge.motion import MotionController, MotionFault


def _config(settle_miss: str = "idle"):
    config = load_config(DEFAULT_CONFIG)
    safety = config.safety.model_copy(update={"cartesian_settle_miss": settle_miss})
    return config.model_copy(update={"safety": safety})


def _start(config) -> np.ndarray:
    return np.append(np.radians(config.acceptance.home_joints_deg), 0.5)


def _cartesian_action(kinematics, start, dx_m: float, timeout_s: float) -> dict:
    pose = kinematics.forward(start[:6])
    pose[0] += dx_m
    return {
        "kind": "cartesian_pose",
        "frame": "world",
        "tcp": "grasp_site",
        "pose": pose.astype(np.float32),
        "timeout_s": timeout_s,
    }


class _SlowIk:
    """Kinematics wrapper whose per-step IK sleeps, so planning exceeds the stale window."""

    def __init__(self, inner, sleep_s: float) -> None:
        self._inner = inner
        self._sleep_s = sleep_s
        self.calls = 0

    def solve_ik(self, *args, **kwargs):
        self.calls += 1
        time.sleep(self._sleep_s)
        return self._inner.solve_ik(*args, **kwargs)

    def __getattr__(self, item):
        return getattr(self._inner, item)


def test_slow_planning_is_not_reported_as_stale_feedback(tmp_path) -> None:
    config = _config()
    start = _start(config)
    actuator = _TrackingActuator(start)
    slow = _SlowIk(I2rtKinematicsBackend.from_config(config), sleep_s=0.02)
    controller = MotionController(
        actuator, slow, config,
        action_log_path=tmp_path / "slow.jsonl",
        heartbeat_timeout_s=2.0,
        feedback_stale_after_s=0.05,
    )
    try:
        result = controller.execute(
            request_id=51,
            action=_cartesian_action(slow, start, dx_m=0.002, timeout_s=3.0),
        )
        idle_before_close = actuator.idle_count
    finally:
        controller.close()

    assert slow.calls * 0.02 > 0.05, "planning must have outlasted the stale window"
    assert result["status"] == "completed"
    assert actuator.commands, "the trajectory must have been executed"
    assert idle_before_close == 0


def test_arm_drift_during_planning_is_refused_without_idling(tmp_path) -> None:
    class DriftingActuator(_TrackingActuator):
        def __init__(self, joints):
            super().__init__(joints)
            self.reads = 0

        def read_motion_state(self):
            self.reads += 1
            if self.reads == 2:                       # between planning and execution
                self.joints[0] += 0.05                # > joint_settle_tolerance_rad
            return super().read_motion_state()

    config = _config()
    start = _start(config)
    actuator = DriftingActuator(start)
    kinematics = I2rtKinematicsBackend.from_config(config)
    controller = MotionController(
        actuator, kinematics, config,
        action_log_path=tmp_path / "drift.jsonl", heartbeat_timeout_s=2.0,
    )
    try:
        with pytest.raises(MotionFault) as exc_info:
            controller.execute(
                request_id=52,
                action=_cartesian_action(kinematics, start, dx_m=0.002, timeout_s=3.0),
            )
        idle_before_close = actuator.idle_count
        health = controller.health()
    finally:
        controller.close()

    assert exc_info.value.code == "INVALID_FEEDBACK"
    assert actuator.commands == []
    assert idle_before_close == 0
    assert health["safety_state"] == "idle" and health["detail"] == ""


@pytest.mark.parametrize("settle_miss", ["hold", "idle"])
def test_cartesian_settle_miss_reaction(tmp_path, settle_miss) -> None:
    config = _config(settle_miss)
    start = _start(config)
    actuator = _UnresponsiveActuator(start)          # never moves: guaranteed settle miss
    kinematics = I2rtKinematicsBackend.from_config(config)
    controller = MotionController(
        actuator, kinematics, config,
        action_log_path=tmp_path / f"miss-{settle_miss}.jsonl", heartbeat_timeout_s=2.0,
    )
    try:
        action = _cartesian_action(kinematics, start, dx_m=0.002, timeout_s=1.0)
        if settle_miss == "hold":
            result = controller.execute(request_id=53, action=action)
            idle_before_close = actuator.idle_count
            health = controller.health()
            controller.heartbeat(53)                  # the hold is still leased to this action
        else:
            with pytest.raises(MotionFault) as exc_info:
                controller.execute(request_id=53, action=action)
            idle_before_close = actuator.idle_count
            health = controller.health()
    finally:
        controller.close()

    if settle_miss == "hold":
        assert result["status"] == "completed"
        assert result["detail"].startswith("SETTLE_MISS: Cartesian pose did not converge")
        assert idle_before_close == 0
        assert health["safety_state"] == "holding"
        assert health["active_action_request_id"] == 53
        # the last command is still the final trajectory target (plus bounded settle bias)
        assert actuator.commands
    else:
        assert exc_info.value.code == "ACTION_TIMEOUT"
        assert idle_before_close >= 1
        assert health["safety_state"] == "fault"


def test_planning_phase_fault_keeps_previous_hold(tmp_path) -> None:
    config = _config()
    start = _start(config)
    actuator = _TrackingActuator(start)
    kinematics = I2rtKinematicsBackend.from_config(config)
    controller = MotionController(
        actuator, kinematics, config,
        action_log_path=tmp_path / "planning-fault.jsonl", heartbeat_timeout_s=2.0,
    )
    try:
        held = controller.execute(
            request_id=54,
            action={
                "kind": "joint_trajectory",
                "waypoints": np.vstack((start[:6], start[:6])).astype(np.float32),
                "waypoint_times_s": np.array([0.0, 0.02], dtype=np.float32),
                "timeout_s": 0.5,
                "position_tolerance_rad": 0.002,
            },
        )
        assert held["status"] == "completed"
        commands_after_hold = len(actuator.commands)
        # 5 cm at 0.03 m/s cannot fit a 0.01 s timeout: refused while planning, before
        # any command is issued.
        with pytest.raises(MotionFault) as exc_info:
            controller.execute(
                request_id=55,
                action=_cartesian_action(kinematics, start, dx_m=0.05, timeout_s=0.01),
            )
        idle_before_close = actuator.idle_count
        health = controller.health()
        controller.heartbeat(54)                      # previous lease still valid
    finally:
        controller.close()

    assert exc_info.value.code == "ACTION_TIMEOUT"
    assert len(actuator.commands) == commands_after_hold
    assert idle_before_close == 0
    assert health["safety_state"] == "holding"
    assert health["active_action_request_id"] == 54
    assert health["detail"] == ""
