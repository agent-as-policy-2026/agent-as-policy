"""Command a YAM arm to joint angles supplied on the command line.

Examples:
    python examples/command_joint_angles/command_joint_angles.py \
        --channel can_follower_r \
        --target-joints-deg 0 20 30 0 10 0

    python examples/command_joint_angles/command_joint_angles.py \
        --sim --skip-confirmation --no-hold \
        --target-joints-deg 0 20 30 0 10 0
"""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import tyro

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.robot import Robot
from i2rt.robots.utils import ArmType, GripperType

ARM_JOINT_COUNT = 6
MIN_RATE_HZ = 20.0
MAX_JOINT_SPEED_RAD_S = np.deg2rad(30.0)
JointAngles = tuple[float, float, float, float, float, float]
RIGHT_SAFE_HOME_JOINTS_DEG: JointAngles = (0.0, 0.0, 0.0, 90.0, 90.0, 90.0)


def validate_motion_parameters(
    target_joints_deg: JointAngles,
    duration_s: float,
    rate_hz: float,
) -> np.ndarray:
    """Validate target and timing values without opening a robot connection."""
    target_arm_deg = np.asarray(target_joints_deg, dtype=np.float64)
    if target_arm_deg.shape != (ARM_JOINT_COUNT,) or not np.all(np.isfinite(target_arm_deg)):
        raise ValueError("target_joints_deg must contain six finite angles")
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("duration_s must be a finite value greater than zero")
    if not math.isfinite(rate_hz) or rate_hz < MIN_RATE_HZ:
        raise ValueError(f"rate_hz must be a finite value of at least {MIN_RATE_HZ:g}")
    return target_arm_deg


def validate_target_within_joint_limits(robot: Robot, target_joints_deg: JointAngles) -> None:
    """Reject a six-joint target outside the connected robot's arm limits."""
    target_arm_rad = np.deg2rad(np.asarray(target_joints_deg, dtype=np.float64))
    joint_limits = np.asarray(robot.get_robot_info().get("joint_limits"), dtype=np.float64)
    if joint_limits.shape != (ARM_JOINT_COUNT, 2):
        raise RuntimeError("robot did not provide six arm joint limits")

    outside_limits = (target_arm_rad < joint_limits[:, 0]) | (target_arm_rad > joint_limits[:, 1])
    if np.any(outside_limits):
        joints = ", ".join(f"j{index + 1}" for index in np.flatnonzero(outside_limits))
        raise ValueError(f"target joint angles outside joint limits: {joints}")


def move_to_joint_angles(
    robot: Robot,
    target_joints_deg: JointAngles,
    duration_s: float = 3.0,
    rate_hz: float = 50.0,
    start_joint_pos: np.ndarray | None = None,
) -> np.ndarray:
    """Move the arm smoothly to a six-joint target while preserving the gripper."""
    target_arm_deg = validate_motion_parameters(target_joints_deg, duration_s, rate_hz)

    if start_joint_pos is None:
        start_joint_pos = robot.get_joint_pos()
    current = np.asarray(start_joint_pos, dtype=np.float64).copy()
    if current.size < ARM_JOINT_COUNT:
        raise ValueError(f"robot has {current.size} joints; expected at least {ARM_JOINT_COUNT}")
    if current.ndim != 1 or not np.all(np.isfinite(current)):
        raise ValueError("current joint state must be a finite one-dimensional array")

    target = current.copy()
    target[:ARM_JOINT_COUNT] = np.deg2rad(target_arm_deg)

    validate_target_within_joint_limits(robot, target_joints_deg)

    max_joint_delta = float(np.max(np.abs(target[:ARM_JOINT_COUNT] - current[:ARM_JOINT_COUNT])))
    if max_joint_delta / duration_s > MAX_JOINT_SPEED_RAD_S:
        raise ValueError(
            "motion exceeds the maximum joint velocity of 30 deg/s; increase duration_s or choose a closer target"
        )

    steps = max(2, math.ceil(duration_s * rate_hz))
    for step in range(1, steps + 1):
        alpha = step / steps
        command = (1.0 - alpha) * current + alpha * target
        robot.command_joint_pos(command)
        time.sleep(duration_s / steps)

    return target


def return_to_safe_home(
    robot: Robot,
    safe_home_joints_deg: JointAngles,
    minimum_duration_s: float,
    rate_hz: float,
) -> np.ndarray:
    """Return to safe home, extending the duration as needed to respect the speed limit."""
    safe_home_deg = validate_motion_parameters(safe_home_joints_deg, minimum_duration_s, rate_hz)
    current = np.asarray(robot.get_joint_pos(), dtype=np.float64)
    if current.ndim != 1 or current.size < ARM_JOINT_COUNT or not np.all(np.isfinite(current)):
        raise ValueError("current joint state must contain at least six finite values")

    max_delta_rad = float(np.max(np.abs(np.deg2rad(safe_home_deg) - current[:ARM_JOINT_COUNT])))
    speed_limited_duration_s = max_delta_rad / MAX_JOINT_SPEED_RAD_S
    duration_s = max(minimum_duration_s, speed_limited_duration_s + 1e-9)
    return move_to_joint_angles(
        robot,
        safe_home_joints_deg,
        duration_s=duration_s,
        rate_hz=rate_hz,
        start_joint_pos=current,
    )


def main(
    target_joints_deg: JointAngles,
    channel: str = "can0",
    arm: str = "yam",
    gripper: str = "linear_4310",
    duration_s: float = 3.0,
    rate_hz: float = 50.0,
    sim: bool = False,
    skip_confirmation: bool = False,
    hold: bool = True,
    safe_home_joints_deg: JointAngles = RIGHT_SAFE_HOME_JOINTS_DEG,
    return_duration_s: float = 3.0,
) -> None:
    """Command a YAM arm to explicit joint angles.

    Args:
        target_joints_deg: Target [j1, j2, j3, j4, j5, j6] in degrees.
        channel: CAN interface name. Ignored in simulation.
        arm: Arm variant.
        gripper: Gripper variant.
        duration_s: Linear interpolation duration in seconds.
        rate_hz: Command rate during interpolation.
        sim: Control a simulated robot instead of real hardware.
        skip_confirmation: Start motion without waiting for Enter.
        hold: Keep the process running and hold the final position.
        safe_home_joints_deg: Ctrl+C return pose [j1, ..., j6] in degrees.
        return_duration_s: Minimum duration for the Ctrl+C return motion.
    """
    validate_motion_parameters(target_joints_deg, duration_s, rate_hz)
    validate_motion_parameters(safe_home_joints_deg, return_duration_s, rate_hz)
    arm_type = ArmType.from_string_name(arm)
    if arm_type == ArmType.NO_ARM:
        raise ValueError("command_joint_angles does not support no_arm")
    gripper_type = GripperType.from_string_name(gripper)

    robot: Robot | None = None
    try:
        print(f"Target joint angles (deg): {list(target_joints_deg)}")
        if not skip_confirmation:
            input(
                "Confirm the arm path and gripper are safe for hardware initialization, "
                "then press Enter to connect (Ctrl+C to cancel)..."
            )

        robot = get_yam_robot(
            channel=channel,
            arm_type=arm_type,
            gripper_type=gripper_type,
            zero_gravity_mode=False,
            sim=sim,
        )
        current_arm_deg = np.rad2deg(robot.get_joint_pos()[:ARM_JOINT_COUNT])
        print(f"Current joint angles (deg): {np.round(current_arm_deg, 1).tolist()}")
        validate_target_within_joint_limits(robot, safe_home_joints_deg)

        target = move_to_joint_angles(
            robot,
            target_joints_deg=target_joints_deg,
            duration_s=duration_s,
            rate_hz=rate_hz,
        )
        reached_deg = np.round(np.rad2deg(target[:ARM_JOINT_COUNT]), 6).tolist()
        print(f"Commanded target joint angles (deg): {reached_deg}")

        if hold:
            print("Holding final position. Press Ctrl+C to exit.")
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        if robot is None:
            print("Canceled before the robot connection was ready.")
        else:
            print("Ctrl+C received. Returning to the safe-home pose before shutdown...")
            return_to_safe_home(
                robot,
                safe_home_joints_deg=safe_home_joints_deg,
                minimum_duration_s=return_duration_s,
                rate_hz=rate_hz,
            )
            print("Safe-home target commanded. Closing the robot connection.")
    finally:
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    tyro.cli(main)
