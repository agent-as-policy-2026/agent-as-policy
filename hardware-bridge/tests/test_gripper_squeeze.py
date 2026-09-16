"""Squeeze past a confirmed contact by the configured fraction (2026-09-03)."""
import numpy as np
from test_motion import DEFAULT_CONFIG, MotionState, _TrackingActuator
from agp_yam_bridge.config import load_config
from agp_yam_bridge.kinematics import I2rtKinematicsBackend
from agp_yam_bridge.motion import MotionController


class _ContactActuator(_TrackingActuator):
    def command_joint_positions(self, target: np.ndarray) -> None:
        with self._lock:
            self.commands.append(target.astype(np.float64, copy=True))

    def read_motion_state(self) -> MotionState:
        state = super().read_motion_state()
        effort = state.effort.copy()
        if self.commands:
            effort[6] = 0.3
        return MotionState(position=state.position, velocity=state.velocity, effort=effort,
                           monotonic_ns=state.monotonic_ns, sequence=state.sequence, motor_errors=())


def _run(tmp_path, squeeze):
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    actuator = _ContactActuator(start)
    controller = MotionController(actuator, I2rtKinematicsBackend.from_config(config), config,
                                  action_log_path=tmp_path / f"sq{squeeze}.jsonl", heartbeat_timeout_s=1.0,
                                  gripper_max_effort_nm=0.9, gripper_stall_timeout_s=0.05,
                                  gripper_contact_squeeze_fraction=squeeze)
    try:
        result = controller.execute(request_id=1, action={"kind": "gripper", "open_fraction": 0.0, "stop_on_contact": True,
                                                          "timeout_s": 3.0, "position_tolerance_fraction": 0.005})
    finally:
        controller.close()
    return result, actuator


def test_squeeze_closes_further_than_contact(tmp_path) -> None:
    r0, a0 = _run(tmp_path, 0.0)
    r1, a1 = _run(tmp_path, 0.04)
    assert r0["status"] == r1["status"] == "completed" and r0["detail"] == r1["detail"] == "gripper contact"
    hold0 = float(a0.commands[-1][6]); hold1 = float(a1.commands[-1][6])
    assert abs((hold0 - hold1) - 0.04) < 0.002, (hold0, hold1)     # squeezed exactly 0.04 past the contact command
    assert hold1 >= 0.05                                          # never below the min-open floor


def test_squeeze_still_respects_overcurrent(tmp_path) -> None:
    class Overcurrent(_ContactActuator):
        def read_motion_state(self):
            s = super().read_motion_state(); e = s.effort.copy()
            e[6] = 0.3 if len(self.commands) < 8 else 1.5      # spike once the squeeze starts
            return MotionState(position=s.position, velocity=s.velocity, effort=e, monotonic_ns=s.monotonic_ns, sequence=s.sequence, motor_errors=())
    config = load_config(DEFAULT_CONFIG)
    start = np.append(np.radians(config.acceptance.home_joints_deg), 0.5)
    controller = MotionController(Overcurrent(start), I2rtKinematicsBackend.from_config(config), config,
                                  action_log_path=tmp_path / "oc.jsonl", heartbeat_timeout_s=1.0,
                                  gripper_max_effort_nm=0.9, gripper_stall_timeout_s=0.05, gripper_contact_squeeze_fraction=0.04)
    import pytest
    from agp_yam_bridge.motion import MotionFault
    try:
        with pytest.raises(MotionFault) as exc:
            controller.execute(request_id=2, action={"kind": "gripper", "open_fraction": 0.0, "stop_on_contact": True,
                                                     "timeout_s": 3.0, "position_tolerance_fraction": 0.005})
        assert "during squeeze" in str(exc.value) or getattr(exc.value, "code", "") == "GRIPPER_OVERCURRENT"
    finally:
        controller.close()


def test_arm_motion_keeps_the_squeezed_gripper_command(tmp_path) -> None:
    """After a contact grasp with squeeze, a joint move must not re-command the measured
    (wider) gripper position — that would release the grip force during transport."""
    r, actuator = _run(tmp_path, 0.04)
    assert r["status"] == "completed"
    squeezed = float(actuator.commands[-1][6])
    config = load_config(DEFAULT_CONFIG)
    controller = MotionController(actuator, I2rtKinematicsBackend.from_config(config), config,
                                  action_log_path=tmp_path / "hold.jsonl", heartbeat_timeout_s=30.0,
                                  gripper_max_effort_nm=0.9, gripper_stall_timeout_s=0.05, gripper_contact_squeeze_fraction=0.04)
    controller._gripper_hold_target = squeezed                    # state carried from the grasp above
    try:
        joints = np.radians(config.acceptance.home_joints_deg).astype(float); joints[0] += 0.05
        res = controller.execute(request_id=3, action={"kind": "absolute_joints", "joint_target": joints.tolist(),
                                                       "timeout_s": 10.0, "position_tolerance_rad": 0.05})
    finally:
        controller.close()
    assert res["status"] == "completed"
    assert abs(float(actuator.commands[-1][6]) - squeezed) < 1e-6, (actuator.commands[-1][6], squeezed)
