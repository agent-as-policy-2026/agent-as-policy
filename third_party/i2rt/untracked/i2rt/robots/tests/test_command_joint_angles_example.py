"""Tests for the direct joint-angle command example."""

import importlib.util
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import ArmType, GripperType

SCRIPT_PATH = Path(__file__).parents[3] / "examples" / "command_joint_angles" / "command_joint_angles.py"


class RecordingRobot:
    """Small in-memory Robot boundary double that records position commands."""

    def __init__(self, current: np.ndarray | None = None) -> None:
        self.current = np.zeros(7) if current is None else current.copy()
        self.commands: list[np.ndarray] = []
        self.closed = False

    def get_joint_pos(self) -> np.ndarray:
        return self.current.copy()

    def get_robot_info(self) -> dict[str, np.ndarray]:
        return {"joint_limits": np.array([[-np.pi, np.pi]] * 6)}

    def command_joint_pos(self, command: np.ndarray) -> None:
        self.current = command.copy()
        self.commands.append(command.copy())

    def close(self) -> None:
        self.closed = True


def _load_example() -> ModuleType:
    spec = importlib.util.spec_from_file_location("command_joint_angles", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_move_to_joint_angles_reaches_target_and_preserves_gripper() -> None:
    """Dropping the current gripper value from the full command must fail this test."""
    example = _load_example()
    robot = get_yam_robot(arm_type=ArmType.YAM, gripper_type=GripperType.LINEAR_4310, sim=True)
    try:
        initial = robot.get_joint_pos().copy()
        initial[6] = 0.42
        robot.command_joint_pos(initial)

        example.move_to_joint_angles(
            robot,
            target_joints_deg=(0.0, 2.0, 2.0, 0.0, 1.0, 0.0),
            duration_s=0.1,
            rate_hz=100.0,
        )

        final = robot.get_joint_pos()
        np.testing.assert_allclose(final[:6], np.deg2rad([0.0, 2.0, 2.0, 0.0, 1.0, 0.0]))
        assert final[6] == pytest.approx(0.42)
    finally:
        robot.close()


def test_move_to_joint_angles_rejects_target_outside_joint_limits() -> None:
    """Removing target limit validation must fail this test."""
    example = _load_example()
    robot = get_yam_robot(arm_type=ArmType.YAM, gripper_type=GripperType.LINEAR_4310, sim=True)
    try:
        with pytest.raises(ValueError, match="outside joint limits"):
            example.move_to_joint_angles(
                robot,
                target_joints_deg=(999.0, 20.0, 30.0, 0.0, 10.0, 0.0),
                duration_s=0.1,
                rate_hz=100.0,
            )
    finally:
        robot.close()


def test_cli_accepts_joint_angles_and_runtime_parameters() -> None:
    """Breaking the documented CLI parameter interface must fail this test."""
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--sim",
            "--target-joints-deg",
            "0",
            "2",
            "2",
            "0",
            "1",
            "0",
            "--duration-s",
            "0.1",
            "--rate-hz",
            "100",
            "--skip-confirmation",
            "--no-hold",
        ],
        capture_output=True,
        check=False,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "Commanded target joint angles (deg): [0.0, 2.0, 2.0, 0.0, 1.0, 0.0]" in result.stdout


def test_move_to_joint_angles_sends_multiple_bounded_steps() -> None:
    """Collapsing interpolation into one target jump must fail this test."""
    example = _load_example()
    robot = RecordingRobot()

    example.move_to_joint_angles(
        robot,
        target_joints_deg=(2.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        duration_s=0.1,
        rate_hz=20.0,
    )

    assert len(robot.commands) == 2
    np.testing.assert_allclose(np.rad2deg(robot.commands[0][0]), 1.0)
    np.testing.assert_allclose(np.rad2deg(robot.commands[1][0]), 2.0)


def test_move_to_joint_angles_rejects_unsafe_timing() -> None:
    """Allowing low rates or excessive joint velocity must fail this test."""
    example = _load_example()

    with pytest.raises(ValueError, match="at least 20"):
        example.move_to_joint_angles(
            RecordingRobot(),
            target_joints_deg=(2.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            duration_s=1.0,
            rate_hz=1.0,
        )

    with pytest.raises(ValueError, match="maximum joint velocity"):
        example.move_to_joint_angles(
            RecordingRobot(),
            target_joints_deg=(30.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            duration_s=0.1,
            rate_hz=1000.0,
        )


def test_move_to_joint_angles_rejects_nonfinite_feedback() -> None:
    """Sending commands derived from NaN hardware feedback must fail this test."""
    example = _load_example()
    current = np.zeros(7)
    current[2] = np.nan

    with pytest.raises(ValueError, match="current joint state"):
        example.move_to_joint_angles(
            RecordingRobot(current),
            target_joints_deg=(2.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            duration_s=0.1,
            rate_hz=20.0,
        )


def test_cli_rejects_no_arm_before_joint_validation() -> None:
    """Letting a gripper-only configuration reach robot construction must fail this test."""
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--sim",
            "--arm",
            "no_arm",
            "--target-joints-deg",
            "0",
            "2",
            "2",
            "0",
            "1",
            "0",
            "--skip-confirmation",
            "--no-hold",
        ],
        capture_output=True,
        check=False,
        text=True,
    )

    assert result.returncode != 0
    assert "does not support no_arm" in result.stderr


def test_main_confirms_before_connecting_to_hardware(monkeypatch: pytest.MonkeyPatch) -> None:
    """Moving hardware construction ahead of confirmation must fail this test."""
    example = _load_example()
    events: list[str] = []

    def confirm(_: str) -> str:
        events.append("confirmation")
        return ""

    def make_robot(**_: object) -> RecordingRobot:
        events.append("hardware construction")
        return RecordingRobot()

    monkeypatch.setattr("builtins.input", confirm)
    monkeypatch.setattr(example, "get_yam_robot", make_robot)

    example.main(
        target_joints_deg=(2.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        duration_s=0.1,
        rate_hz=20.0,
        hold=False,
    )

    assert events[:2] == ["confirmation", "hardware construction"]


def test_main_requests_position_hold_during_hardware_initialization(monkeypatch: pytest.MonkeyPatch) -> None:
    """Falling back to gravity-compensation startup in this position command must fail this test."""
    example = _load_example()
    constructor_kwargs: dict[str, object] = {}

    def make_robot(**kwargs: object) -> RecordingRobot:
        constructor_kwargs.update(kwargs)
        return RecordingRobot()

    monkeypatch.setattr(example, "get_yam_robot", make_robot)

    example.main(
        target_joints_deg=(2.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        duration_s=0.1,
        rate_hz=20.0,
        skip_confirmation=True,
        hold=False,
    )

    assert constructor_kwargs["zero_gravity_mode"] is False


def test_ctrl_c_returns_to_right_safe_home_before_close(monkeypatch: pytest.MonkeyPatch) -> None:
    """Removing Ctrl+C safe-home movement must fail this test."""
    example = _load_example()
    initial = np.zeros(7)
    initial[6] = 0.42
    robot = RecordingRobot(initial)
    interrupted = False

    def interrupt_during_target(_: float) -> None:
        nonlocal interrupted
        if len(robot.commands) == 1 and not interrupted:
            interrupted = True
            raise KeyboardInterrupt

    monkeypatch.setattr(example, "get_yam_robot", lambda **_: robot)
    monkeypatch.setattr(example.time, "sleep", interrupt_during_target)

    example.main(
        target_joints_deg=(2.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        duration_s=0.1,
        rate_hz=20.0,
        skip_confirmation=True,
    )

    np.testing.assert_allclose(robot.current[:6], np.deg2rad([0.0, 0.0, 0.0, 90.0, 90.0, 90.0]))
    assert robot.current[6] == pytest.approx(0.42)
    assert robot.closed


def test_invalid_safe_home_is_rejected_before_target_motion(monkeypatch: pytest.MonkeyPatch) -> None:
    """Deferring safe-home limit validation until Ctrl+C must fail this test."""
    example = _load_example()
    robot = RecordingRobot()
    monkeypatch.setattr(example, "get_yam_robot", lambda **_: robot)

    with pytest.raises(ValueError, match="outside joint limits"):
        example.main(
            target_joints_deg=(2.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            duration_s=0.1,
            rate_hz=20.0,
            skip_confirmation=True,
            hold=False,
            safe_home_joints_deg=(999.0, 0.0, 0.0, 90.0, 90.0, 90.0),
        )

    assert robot.commands == []
    assert robot.closed


def test_safe_home_failure_still_closes_robot(monkeypatch: pytest.MonkeyPatch) -> None:
    """Losing finally-close when safe-home movement fails must fail this test."""
    example = _load_example()

    class FailingReturnRobot(RecordingRobot):
        def command_joint_pos(self, command: np.ndarray) -> None:
            if len(self.commands) >= 2:
                raise RuntimeError("safe-home command failed")
            super().command_joint_pos(command)

    robot = FailingReturnRobot()
    interrupted = False

    def interrupt_after_target(_: float) -> None:
        nonlocal interrupted
        if len(robot.commands) == 2 and not interrupted:
            interrupted = True
            raise KeyboardInterrupt

    monkeypatch.setattr(example, "get_yam_robot", lambda **_: robot)
    monkeypatch.setattr(example.time, "sleep", interrupt_after_target)

    with pytest.raises(RuntimeError, match="safe-home command failed"):
        example.main(
            target_joints_deg=(2.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            duration_s=0.1,
            rate_hz=20.0,
            skip_confirmation=True,
        )

    assert robot.closed


def test_safe_home_uses_one_feedback_snapshot_for_duration_and_interpolation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Reading a different start state for interpolation must fail this test."""
    example = _load_example()

    class SequentialFeedbackRobot(RecordingRobot):
        def __init__(self) -> None:
            super().__init__()
            first = np.zeros(7)
            second = np.zeros(7)
            second[3] = np.deg2rad(-90.0)
            self.feedback = [first, second]

        def get_joint_pos(self) -> np.ndarray:
            if self.feedback:
                return self.feedback.pop(0)
            return super().get_joint_pos()

    robot = SequentialFeedbackRobot()
    monkeypatch.setattr(example.time, "sleep", lambda _: None)

    example.return_to_safe_home(
        robot,
        safe_home_joints_deg=(0.0, 0.0, 0.0, 90.0, 90.0, 90.0),
        minimum_duration_s=3.0,
        rate_hz=20.0,
    )

    assert len(robot.feedback) == 1
    np.testing.assert_allclose(robot.current[:6], np.deg2rad([0.0, 0.0, 0.0, 90.0, 90.0, 90.0]))
