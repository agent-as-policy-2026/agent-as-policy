"""Offline YAM kinematics backed by i2rt's current combined MJCF model.

This module never constructs a hardware ``Robot`` and never opens CAN.  It
delegates FK/IK to :class:`i2rt.robots.kinematics.Kinematics`, the same API
used by i2rt's MuJoCo and Viser control interfaces, and adds the frame and
trajectory checks required before a future motion RPC may dispatch anything.
"""

from __future__ import annotations

import logging
import math
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import mink
import mujoco
import numpy as np
from i2rt.robots.kinematics import Kinematics
from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml

if TYPE_CHECKING:
    from numpy.typing import ArrayLike

    from agp_yam_bridge.config import BridgeConfig

logger = logging.getLogger(__name__)

_ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
_FINGER_JOINT_NAMES = ("joint7", "joint8")
_DEFAULT_MAX_JOINT_JUMP_RAD = 0.5


#: Widening applied when validating MEASURED joints (FK/observation, IK
#: seeds, trajectory initial state) - never command waypoints/IK solutions.
_MEASUREMENT_BUFFER_RAD = 0.1


class KinematicsValidationError(ValueError):
    """A pose, transform, joint vector, or configured model is invalid."""


class TrajectoryValidationError(KinematicsValidationError):
    """A trajectory failed one fail-closed safety check."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code


@dataclass(frozen=True)
class FrameChain:
    """Explicit ``world -> base -> gripper -> grasp_site`` transforms."""

    world_from_base: np.ndarray
    base_from_gripper: np.ndarray
    gripper_from_grasp_site: np.ndarray
    world_from_grasp_site: np.ndarray


def _validate_transform(transform: ArrayLike, *, name: str) -> np.ndarray:
    matrix = np.asarray(transform, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise KinematicsValidationError(f"{name} must have shape (4, 4)")
    if not np.isfinite(matrix).all():
        raise KinematicsValidationError(f"{name} contains non-finite values")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-10):
        raise KinematicsValidationError(f"{name} must be a homogeneous transform")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8) or not np.isclose(
        np.linalg.det(rotation), 1.0, atol=1e-8
    ):
        raise KinematicsValidationError(f"{name} rotation must be orthonormal")
    return matrix.copy()


def pose_wxyz_to_matrix(pose: ArrayLike) -> np.ndarray:
    """Convert ``[x, y, z, qw, qx, qy, qz]`` to a homogeneous transform."""
    values = np.asarray(pose, dtype=np.float64)
    if values.shape != (7,):
        raise KinematicsValidationError("pose must have shape (7,) as xyz+wxyz")
    if not np.isfinite(values).all():
        raise KinematicsValidationError("pose contains non-finite values")
    quaternion = values[3:]
    norm = float(np.linalg.norm(quaternion))
    if not np.isclose(norm, 1.0, atol=1e-6):
        raise KinematicsValidationError(f"wxyz quaternion must be unit length, got {norm}")
    rotation_flat = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rotation_flat, quaternion / norm)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation_flat.reshape(3, 3)
    transform[:3, 3] = values[:3]
    return transform


def matrix_to_pose_wxyz(transform: ArrayLike) -> np.ndarray:
    """Convert a homogeneous transform to ``[x, y, z, qw, qx, qy, qz]``."""
    matrix = _validate_transform(transform, name="pose transform")
    quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion, matrix[:3, :3].reshape(-1))
    if quaternion[0] < 0.0:
        quaternion *= -1.0
    return np.concatenate((matrix[:3, 3], quaternion))


def _transform_from_mujoco_frame(position: np.ndarray, rotation_flat: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation_flat).reshape(3, 3)
    transform[:3, 3] = position
    return transform


class I2rtKinematicsBackend:
    """Single-arm YAM/linear-4310 kinematics and offline trajectory gate.

    Robot identity is explicit through i2rt's ``ArmType`` and ``GripperType``;
    no behavior is selected by guessing from a six-joint vector length.
    """

    def __init__(
        self,
        *,
        arm_type: ArmType,
        gripper_type: GripperType,
        target_frame: str,
        world_from_base: ArrayLike,
        workspace_lower_base_m: ArrayLike,
        workspace_upper_base_m: ArrayLike,
        max_joint_velocity_rad_s: float,
        max_joint_acceleration_rad_s2: float,
        control_frequency_hz: float,
        max_joint_jump_rad: float = _DEFAULT_MAX_JOINT_JUMP_RAD,
    ) -> None:
        if arm_type is not ArmType.YAM or gripper_type is not GripperType.LINEAR_4310:
            raise KinematicsValidationError(
                "P2 only accepts explicit ArmType.YAM + GripperType.LINEAR_4310"
            )
        if target_frame != "grasp_site":
            raise KinematicsValidationError("P2 target frame must be the real grasp_site")

        self.arm_type = arm_type
        self.gripper_type = gripper_type
        self.target_frame = target_frame
        self._world_from_base = _validate_transform(world_from_base, name="world_from_base")
        self._base_from_world = np.linalg.inv(self._world_from_base)
        self._workspace_lower = np.asarray(workspace_lower_base_m, dtype=np.float64)
        self._workspace_upper = np.asarray(workspace_upper_base_m, dtype=np.float64)
        if self._workspace_lower.shape != (3,) or self._workspace_upper.shape != (3,):
            raise KinematicsValidationError("workspace bounds must each have shape (3,)")
        if not np.all(self._workspace_lower < self._workspace_upper):
            raise KinematicsValidationError("workspace lower bounds must be below upper bounds")
        self._max_joint_velocity = float(max_joint_velocity_rad_s)
        self._max_joint_acceleration = float(max_joint_acceleration_rad_s2)
        self._control_frequency_hz = float(control_frequency_hz)
        self._max_joint_jump = float(max_joint_jump_rad)
        if (
            min(
                self._max_joint_velocity,
                self._max_joint_acceleration,
                self._control_frequency_hz,
                self._max_joint_jump,
            )
            <= 0.0
        ):
            raise KinematicsValidationError("trajectory limits must be positive")

        xml_path = combine_arm_and_gripper_xml(arm_type, gripper_type)
        try:
            self._model = mujoco.MjModel.from_xml_path(xml_path)
            self._kinematics = Kinematics(xml_path, target_frame)
        finally:
            Path(xml_path).unlink(missing_ok=True)
        self._data = mujoco.MjData(self._model)
        self._lock = threading.Lock()

        self._arm_qpos_addresses = self._require_joint_addresses(_ARM_JOINT_NAMES)
        self._finger_qpos_addresses = self._require_joint_addresses(_FINGER_JOINT_NAMES)
        site_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SITE, self.target_frame)
        if site_id < 0:
            raise KinematicsValidationError(
                f"i2rt combined model has no site {self.target_frame!r}"
            )
        self._joint_lower = np.array(
            [self._model.joint(name).range[0] for name in _ARM_JOINT_NAMES],
            dtype=np.float64,
        )
        self._joint_upper = np.array(
            [self._model.joint(name).range[1] for name in _ARM_JOINT_NAMES],
            dtype=np.float64,
        )

    @classmethod
    def from_config(
        cls,
        config: BridgeConfig,
        *,
        world_from_base: ArrayLike | None = None,
    ) -> I2rtKinematicsBackend:
        """Build from the same strict hardware/acceptance config as preflight."""
        arm_type = ArmType.from_string_name(config.hardware.arm)
        gripper_type = GripperType.from_string_name(config.hardware.gripper)
        workspace = config.acceptance.workspace
        if workspace.frame != "yam_base":
            raise KinematicsValidationError(
                f"P2 workspace frame must be 'yam_base', got {workspace.frame!r}"
            )
        limits = config.acceptance.speed_limits
        return cls(
            arm_type=arm_type,
            gripper_type=gripper_type,
            target_frame=workspace.tcp,
            world_from_base=np.eye(4) if world_from_base is None else world_from_base,
            workspace_lower_base_m=(workspace.x_m[0], workspace.y_m[0], workspace.z_m[0]),
            workspace_upper_base_m=(workspace.x_m[1], workspace.y_m[1], workspace.z_m[1]),
            max_joint_velocity_rad_s=np.radians(limits.joint_velocity_deg_s),
            max_joint_acceleration_rad_s2=np.radians(limits.joint_acceleration_deg_s2),
            control_frequency_hz=limits.control_frequency_hz,
        )

    def _require_joint_addresses(self, names: tuple[str, ...]) -> np.ndarray:
        addresses = []
        for name in names:
            joint_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise KinematicsValidationError(
                    f"i2rt combined model is missing required joint {name!r}"
                )
            addresses.append(int(self._model.jnt_qposadr[joint_id]))
        return np.asarray(addresses, dtype=np.int64)

    @property
    def world_from_base(self) -> np.ndarray:
        return self._world_from_base.copy()

    @property
    def joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self._joint_lower.copy(), self._joint_upper.copy()

    def _validate_joints(
        self,
        joints: ArrayLike,
        *,
        name: str = "joints",
        buffer_rad: float = 0.0,
        lower_override: ArrayLike | None = None,
        upper_override: ArrayLike | None = None,
    ) -> np.ndarray:
        """Validate six arm joints against the model range.

        ``buffer_rad`` widens the accepted range for MEASURED joints only
        (user-approved 2026-09-01): an arm resting on a mechanical hard stop
        plus the per-channel software encoder-zero corrections (i2rt
        yam_v1.yml motor_offsets_deg_by_channel) can legitimately REPORT
        joints slightly beyond the model range (seen: folded left joint3 =
        -0.37 deg vs model lower 0, which rejected every observation).
        _MEASUREMENT_BUFFER_RAD matches i2rt's runtime limit-check
        buffer_rad. COMMAND waypoints and IK solutions keep the strict
        default of 0.0.
        """
        values = np.asarray(joints, dtype=np.float64)
        if values.shape != (6,):
            raise KinematicsValidationError(f"{name} must have shape (6,)")
        if not np.isfinite(values).all():
            raise KinematicsValidationError(f"{name} contains non-finite values")
        lower = (self._joint_lower if lower_override is None
                 else np.asarray(lower_override, dtype=np.float64)) - float(buffer_rad)
        upper = (self._joint_upper if upper_override is None
                 else np.asarray(upper_override, dtype=np.float64)) + float(buffer_rad)
        violations = np.flatnonzero(
            (values < lower - 1e-10) | (values > upper + 1e-10)
        )
        if violations.size:
            index = int(violations[0])
            raise KinematicsValidationError(
                f"joint{index + 1}={values[index]:.6f} is outside "
                f"[{lower[index]:.6f}, {upper[index]:.6f}]"
            )
        return values.copy()

    def _model_q(self, joints: ArrayLike) -> np.ndarray:
        # Buffered: fed by measured joints (fk/camera compose, IK seeds).
        # Command waypoints are strictly validated in validate_trajectory
        # BEFORE any FK of them happens.
        arm = self._validate_joints(joints, buffer_rad=_MEASUREMENT_BUFFER_RAD)
        q = self._model.qpos0.copy()
        q[self._arm_qpos_addresses] = arm
        for address, name in zip(self._finger_qpos_addresses, _FINGER_JOINT_NAMES, strict=True):
            lo, hi = self._model.joint(name).range
            q[address] = (lo + hi) / 2.0
        return q

    def forward_matrix(self, joints: ArrayLike) -> np.ndarray:
        """Return ``T_world_grasp_site`` using i2rt's production FK API."""
        q = self._model_q(joints)
        with self._lock:
            base_from_site = self._kinematics.fk(q, self.target_frame).copy()
        return self._world_from_base @ base_from_site

    def forward(self, joints: ArrayLike) -> np.ndarray:
        """Return ``[x, y, z, qw, qx, qy, qz]`` in the world frame."""
        return matrix_to_pose_wxyz(self.forward_matrix(joints))

    def frame_chain(self, joints: ArrayLike) -> FrameChain:
        """Expose each link in ``world -> base -> gripper -> grasp_site``."""
        q = self._model_q(joints)
        with self._lock:
            self._data.qpos[:] = q
            mujoco.mj_forward(self._model, self._data)
            gripper = self._data.body("gripper")
            site = self._data.site(self.target_frame)
            base_from_gripper = _transform_from_mujoco_frame(
                gripper.xpos.copy(), gripper.xmat.copy()
            )
            base_from_site = _transform_from_mujoco_frame(site.xpos.copy(), site.xmat.copy())
        gripper_from_site = np.linalg.inv(base_from_gripper) @ base_from_site
        world_from_site = self.forward_matrix(joints)
        return FrameChain(
            world_from_base=self._world_from_base.copy(),
            base_from_gripper=base_from_gripper,
            gripper_from_grasp_site=gripper_from_site,
            world_from_grasp_site=world_from_site,
        )

    @staticmethod
    def _pose_errors(actual: np.ndarray, target: np.ndarray) -> tuple[float, float]:
        position_error = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
        relative_rotation = actual[:3, :3].T @ target[:3, :3]
        cosine = np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0)
        orientation_error = float(np.arccos(cosine))
        return position_error, orientation_error

    def solve_ik(
        self,
        target_world_pose: ArrayLike,
        *,
        seed_joints: ArrayLike,
        position_tolerance_m: float = 1e-4,
        orientation_tolerance_rad: float = 1e-4,
    ) -> np.ndarray | None:
        """Solve for ``grasp_site`` with i2rt, returning joint1..joint6."""
        target_world = pose_wxyz_to_matrix(target_world_pose)
        target_base = self._base_from_world @ target_world
        seed = self._validate_joints(
            seed_joints, name="seed_joints", buffer_rad=_MEASUREMENT_BUFFER_RAD
        )
        full_seed = self._model_q(seed)
        try:
            with self._lock:
                success, solution = self._kinematics.ik(
                    target_base,
                    self.target_frame,
                    init_q=full_seed,
                    pos_threshold=position_tolerance_m,
                    ori_threshold=orientation_tolerance_rad,
                )
                solution = np.asarray(solution, dtype=np.float64).copy()
        except Exception:
            logger.debug("i2rt IK raised for target", exc_info=True)
            return None
        if not success:
            return None
        arm_solution = solution[self._arm_qpos_addresses]
        try:
            arm_solution = self._validate_joints(arm_solution, name="IK solution")
        except KinematicsValidationError:
            logger.warning("i2rt IK returned an out-of-limit solution", exc_info=True)
            return None
        actual = self.forward_matrix(arm_solution)
        position_error, orientation_error = self._pose_errors(actual, target_world)
        if (
            position_error > position_tolerance_m * 1.05
            or orientation_error > orientation_tolerance_rad * 1.05
        ):
            logger.warning(
                "i2rt IK post-check failed: position_error=%.6g orientation_error=%.6g",
                position_error,
                orientation_error,
            )
            return None
        return arm_solution

    def solve_position_ik(
        self,
        target_world_position: ArrayLike,
        *,
        seed_joints: ArrayLike,
        position_tolerance_m: float = 1e-4,
    ) -> np.ndarray | None:
        """Solve for ``grasp_site`` position while leaving orientation unconstrained."""
        if isinstance(position_tolerance_m, bool):
            raise KinematicsValidationError("position_tolerance_m must be finite and positive")
        try:
            tolerance = float(position_tolerance_m)
        except (TypeError, ValueError) as exc:
            raise KinematicsValidationError(
                "position_tolerance_m must be finite and positive"
            ) from exc
        if not np.isfinite(tolerance) or tolerance <= 0.0:
            raise KinematicsValidationError("position_tolerance_m must be finite and positive")
        target_world = np.asarray(target_world_position, dtype=np.float64)
        if target_world.shape != (3,):
            raise KinematicsValidationError("target_world_position must have shape (3,)")
        if not np.isfinite(target_world).all():
            raise KinematicsValidationError("target_world_position contains non-finite values")
        target_base = (self._base_from_world @ np.append(target_world, 1.0))[:3]
        seed = self._validate_joints(
            seed_joints, name="seed_joints", buffer_rad=_MEASUREMENT_BUFFER_RAD
        )
        full_seed = self._model_q(seed)
        try:
            with self._lock:
                configuration = mink.Configuration(self._model)
                configuration.update(full_seed)
                task = mink.FrameTask(
                    frame_name=self.target_frame,
                    frame_type="site",
                    position_cost=1.0,
                    orientation_cost=0.0,
                    lm_damping=1.0,
                )
                target = np.eye(4, dtype=np.float64)
                target[:3, 3] = target_base
                task.set_target(mink.SE3.from_matrix(target))
                success = False
                for _ in range(200):
                    velocity = mink.solve_ik(
                        configuration,
                        [task],
                        0.01,
                        "quadprog",
                        damping=1e-4,
                        limits=None,
                    )
                    configuration.integrate_inplace(velocity, 0.01)
                    if np.linalg.norm(task.compute_error(configuration)[:3]) <= tolerance:
                        success = True
                        break
                solution = configuration.q.copy()
        except Exception:
            logger.debug("position-only IK raised for target", exc_info=True)
            return None
        if not success:
            return None
        arm_solution = solution[self._arm_qpos_addresses]
        try:
            arm_solution = self._validate_joints(arm_solution, name="position IK solution")
        except KinematicsValidationError:
            logger.warning("position-only IK returned an out-of-limit solution", exc_info=True)
            return None
        position_error = float(
            np.linalg.norm(self.forward_matrix(arm_solution)[:3, 3] - target_world)
        )
        if position_error > tolerance * 1.05:
            logger.warning(
                "position-only IK post-check failed: position_error=%.6g",
                position_error,
            )
            return None
        return arm_solution

    def validate_trajectory(
        self,
        joint_waypoints: ArrayLike,
        time_from_start_s: ArrayLike,
        *,
        initial_joints: ArrayLike | None = None,
        max_joint_velocity_rad_s: ArrayLike | None = None,
        max_joint_acceleration_rad_s2: ArrayLike | None = None,
    ) -> np.ndarray:
        """Fail closed on limits, timing, jumps, velocity, acceleration, and workspace.

        The rate limits default to the configured ordinary caps for all six joints; a caller
        may pass six per-joint values instead (2026-09-14: buffered joint programs raise J4).
        """
        waypoints = np.asarray(joint_waypoints, dtype=np.float64)
        times = np.asarray(time_from_start_s, dtype=np.float64)
        if waypoints.ndim != 2 or waypoints.shape[1:] != (6,) or len(waypoints) < 2:
            raise TrajectoryValidationError(
                "INVALID_SHAPE", "joint waypoints must have shape (N, 6) with N >= 2"
            )
        velocity_limits = (
            np.full(6, self._max_joint_velocity)
            if max_joint_velocity_rad_s is None
            else np.asarray(max_joint_velocity_rad_s, dtype=np.float64)
        )
        acceleration_limits = (
            np.full(6, self._max_joint_acceleration)
            if max_joint_acceleration_rad_s2 is None
            else np.asarray(max_joint_acceleration_rad_s2, dtype=np.float64)
        )
        for limits in (velocity_limits, acceleration_limits):
            if limits.shape != (6,) or not np.isfinite(limits).all() or np.any(limits <= 0):
                raise TrajectoryValidationError(
                    "INVALID_LIMITS", "joint rate limits must be six finite positive values"
                )
        if times.shape != (len(waypoints),):
            raise TrajectoryValidationError(
                "INVALID_TIMING", "time_from_start_s must have one value per waypoint"
            )
        if not np.isfinite(waypoints).all() or not np.isfinite(times).all():
            raise TrajectoryValidationError("NONFINITE", "trajectory contains non-finite values")
        original_waypoints = waypoints.copy()
        try:
            initial = (
                None
                if initial_joints is None
                else self._validate_joints(
                    initial_joints,
                    name="initial_joints",
                    buffer_rad=_MEASUREMENT_BUFFER_RAD,
                )
            )
            # NEVER-DEEPEN corridor (user-approved 2026-09-01): the measured
            # start can legally rest slightly past a model limit (hard stop +
            # encoder-zero offsets), and the min-jerk trajectory begins AT
            # that pose. Waypoints may pass through the corridor between the
            # start and the model range but never go further out than the
            # start; with no initial state the model range applies strictly.
            if initial is not None:
                _eps = 1e-6
                way_lower = np.minimum(self._joint_lower, initial - _eps)
                way_upper = np.maximum(self._joint_upper, initial + _eps)
            else:
                way_lower = way_upper = None
            for index, waypoint in enumerate(waypoints):
                self._validate_joints(
                    waypoint,
                    name=f"waypoint[{index}]",
                    lower_override=way_lower,
                    upper_override=way_upper,
                )
        except KinematicsValidationError as exc:
            raise TrajectoryValidationError("JOINT_LIMIT", str(exc)) from exc

        if times[0] < 0.0:
            raise TrajectoryValidationError(
                "INVALID_TIMING", "first waypoint time must be non-negative"
            )
        if initial is not None:
            initial_jump = float(np.max(np.abs(waypoints[0] - initial)))
            if initial_jump > self._max_joint_jump + 1e-12:
                raise TrajectoryValidationError(
                    "JOINT_JUMP",
                    f"initial-to-first jump {initial_jump:.6f} rad exceeds "
                    f"{self._max_joint_jump:.6f} rad",
                )
            if np.isclose(times[0], 0.0, atol=1e-12):
                if initial_jump > 1e-6:
                    raise TrajectoryValidationError(
                        "INVALID_TIMING",
                        "a nonzero initial-to-first move requires positive time",
                    )
            else:
                waypoints = np.vstack((initial, waypoints))
                times = np.concatenate(([0.0], times))

        dt = np.diff(times)
        if np.any(dt <= 0.0):
            raise TrajectoryValidationError(
                "INVALID_TIMING", "waypoint times must be strictly increasing"
            )
        delta = np.diff(waypoints, axis=0)
        largest_jump = float(np.max(np.abs(delta)))
        if largest_jump > self._max_joint_jump + 1e-12:
            raise TrajectoryValidationError(
                "JOINT_JUMP",
                f"largest waypoint jump {largest_jump:.6f} rad exceeds "
                f"{self._max_joint_jump:.6f} rad",
            )
        velocity = delta / dt[:, None]
        peak_velocity = np.max(np.abs(velocity), axis=0)
        exceeded = np.flatnonzero(peak_velocity > velocity_limits + 1e-9)
        if len(exceeded):
            joint = int(exceeded[0])
            raise TrajectoryValidationError(
                "JOINT_VELOCITY",
                f"J{joint + 1} velocity {peak_velocity[joint]:.6f} rad/s exceeds "
                f"{velocity_limits[joint]:.6f} rad/s",
            )
        if len(waypoints) >= 3:
            acceleration = 2.0 * np.diff(velocity, axis=0) / (dt[:-1] + dt[1:])[:, None]
            peak_acceleration = np.max(np.abs(acceleration), axis=0)
            exceeded = np.flatnonzero(peak_acceleration > acceleration_limits + 1e-9)
            if len(exceeded):
                joint = int(exceeded[0])
                raise TrajectoryValidationError(
                    "JOINT_ACCELERATION",
                    f"J{joint + 1} acceleration {peak_acceleration[joint]:.6f} rad/s^2 exceeds "
                    f"{acceleration_limits[joint]:.6f} rad/s^2",
                )

        def check_workspace(joints: np.ndarray, *, label: str) -> None:
            base_from_site = self._base_from_world @ self.forward_matrix(joints)
            position = base_from_site[:3, 3]
            if np.any(position < self._workspace_lower) or np.any(position > self._workspace_upper):
                raise TrajectoryValidationError(
                    "WORKSPACE",
                    f"{label} grasp_site {position.tolist()} is outside "
                    f"{self._workspace_lower.tolist()}..{self._workspace_upper.tolist()} in yam_base",
                )

        for index, waypoint in enumerate(waypoints):
            check_workspace(waypoint, label=f"waypoint[{index}]")
        # The future dispatcher is configured to linearly interpolate joints
        # at this fixed acceptance rate. Check every interior control tick as
        # well as the listed endpoints: Cartesian workspace boxes are not
        # convex under joint-space interpolation.
        for segment, (start, end, duration) in enumerate(
            zip(waypoints[:-1], waypoints[1:], dt, strict=True)
        ):
            interior_ticks = math.ceil(duration * self._control_frequency_hz) - 1
            for tick in range(1, interior_ticks + 1):
                elapsed = min(tick / self._control_frequency_hz, duration)
                fraction = elapsed / duration
                interpolated = start + fraction * (end - start)
                check_workspace(
                    interpolated,
                    label=f"segment[{segment}] at {elapsed:.6f}s",
                )
        return original_waypoints

    def solve_ik_trajectory(
        self,
        target_world_poses: ArrayLike,
        time_from_start_s: ArrayLike,
        *,
        seed_joints: ArrayLike,
    ) -> np.ndarray | None:
        """Solve adjacent poses with the previous result, then run the full safety gate."""
        poses = np.asarray(target_world_poses, dtype=np.float64)
        if poses.ndim != 2 or poses.shape[1:] != (7,) or len(poses) < 2:
            raise KinematicsValidationError("target poses must have shape (N, 7) with N >= 2")
        previous = self._validate_joints(seed_joints, name="seed_joints")
        waypoints = []
        for pose in poses:
            solved = self.solve_ik(pose, seed_joints=previous)
            if solved is None:
                return None
            waypoints.append(solved)
            previous = solved
        return self.validate_trajectory(
            np.asarray(waypoints),
            time_from_start_s,
            initial_joints=seed_joints,
        )


__all__ = [
    "FrameChain",
    "I2rtKinematicsBackend",
    "KinematicsValidationError",
    "TrajectoryValidationError",
    "matrix_to_pose_wxyz",
    "pose_wxyz_to_matrix",
]
