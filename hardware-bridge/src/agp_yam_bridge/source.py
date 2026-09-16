"""Observation sources owned by the read-only YAM bridge."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any, Protocol

import numpy as np

from agp_yam_bridge.camera import (
    CameraFrameError,
    OpenCvFixedRgbCamera,
    RealSenseRgbdCamera,
    RgbCamera,
    RgbdCamera,
    StationCameraKinematics,
    load_fixed_rgb_calibration,
    validate_rgb_frame,
    validate_rgbd_frame,
)
from agp_yam_bridge.config import BridgeConfig
from agp_yam_bridge.kinematics import I2rtKinematicsBackend
from agp_yam_bridge.motion import MotionState


class YamSource(Protocol):
    @property
    def kinematics(self) -> I2rtKinematicsBackend: ...

    def read(self, *, request_id: int) -> dict[str, Any]: ...

    def read_motion_state(self) -> MotionState: ...

    def command_joint_positions(self, target: np.ndarray) -> None: ...

    def enter_safe_idle(self) -> None: ...

    def close(self) -> None: ...


class StartupHoldError(RuntimeError):
    """Retain ownership of a live i2rt hold after robot construction fails."""

    def __init__(self, motor_chain: Any, *, hold_active: bool) -> None:
        state = "confirmed" if hold_active else "not confirmed"
        super().__init__(f"YAM startup failed; emergency position hold is {state}")
        self.motor_chain = motor_chain
        self.hold_active = hold_active

    def release(self) -> None:
        """Explicitly release the retained hardware resource."""
        self.motor_chain.close()


class I2rtYamMotionSource:
    """Own the sole current-i2rt YAM instance used for feedback and motion."""

    def __init__(
        self,
        config: BridgeConfig,
        *,
        robot_factory: Callable[..., Any] | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        wall_time_ns: Callable[[], int] = time.time_ns,
        kinematics_backend: I2rtKinematicsBackend | None = None,
    ) -> None:
        from i2rt.robots.get_robot import YamStartupHoldingError, get_yam_robot
        from i2rt.robots.utils import ArmType, GripperType

        kinematics = (
            I2rtKinematicsBackend.from_config(config)
            if kinematics_backend is None
            else kinematics_backend
        )
        factory = get_yam_robot if robot_factory is None else robot_factory
        try:
            robot = factory(
                channel=config.hardware.can_channel,
                arm_type=ArmType.from_string_name(config.hardware.arm),
                gripper_type=GripperType.from_string_name(config.hardware.gripper),
                zero_gravity_mode=True,
            )
        except YamStartupHoldingError as exc:
            raise StartupHoldError(
                exc.motor_chain, hold_active=exc.hold_active
            ) from exc
        try:
            dofs = robot.num_dofs()
            if dofs != 7:
                raise RuntimeError(
                    f"P1 requires a 7 DOF YAM+gripper source, got {dofs} DOF"
                )
        except BaseException:
            robot.close()
            raise
        self._robot = robot
        self._config = config
        self._kinematics = kinematics
        self._monotonic_ns = monotonic_ns
        self._wall_time_ns = wall_time_ns
        self._sequence = -1
        self._last_motor_state: Any = None
        self._last_frame_monotonic_ns = 0
        self._last_frame_wall_time_ns = 0
        self._last_motor_errors: tuple[str, ...] = ()
        self._lock = threading.Lock()
        self._closed = False

    @property
    def kinematics(self) -> I2rtKinematicsBackend:
        return self._kinematics

    @staticmethod
    def _copy_joint_field(joint_state: Any, field: str) -> np.ndarray:
        value = np.asarray(getattr(joint_state, field, None))
        if value.shape != (7,):
            raise RuntimeError(
                f"i2rt joint-state {field} must have shape (7,), got {value.shape}"
            )
        return value.astype(np.float32, copy=True)

    def _read_joints(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, int]:
        with self._lock:
            if self._closed:
                raise RuntimeError("YAM observation source is closed")
            motor_chain = getattr(self._robot, "motor_chain", None)
            if motor_chain is None:
                raise RuntimeError("i2rt robot has no motor chain")
            if getattr(motor_chain, "running", None) is not True:
                raise RuntimeError("i2rt motor chain is not running")
            control_thread = getattr(motor_chain, "_control_thread", None)
            if control_thread is None or not control_thread.is_alive():
                raise RuntimeError("i2rt motor-chain control thread is not running")
            update_thread = getattr(self._robot, "_server_thread", None)
            if update_thread is not None and not update_thread.is_alive():
                raise RuntimeError("i2rt robot update thread is not running")
            state_lock = getattr(motor_chain, "state_lock", None)
            if state_lock is None:
                raise RuntimeError("i2rt motor chain has no state lock")
            # read_states() reuses i2rt's offset/direction conversion. Bracket it
            # with producer-token reads and retry if the CAN worker replaced the
            # underlying state during the copy, so data and generation match.
            motor_state = None
            motor_infos = None
            for _ in range(8):
                with state_lock:
                    before = getattr(motor_chain, "state", None)
                candidate = motor_chain.read_states()
                with state_lock:
                    after = getattr(motor_chain, "state", None)
                if before is after and before is not None:
                    motor_state = before
                    motor_infos = candidate
                    break
            if motor_state is None:
                raise RuntimeError("could not capture a stable i2rt motor feedback snapshot")
            converter = getattr(self._robot, "_motor_state_to_joint_state", None)
            if not callable(converter):
                raise RuntimeError("i2rt robot has no motor-state conversion")
            joint_state = converter(motor_infos)
            position = self._copy_joint_field(joint_state, "pos")
            velocity = self._copy_joint_field(joint_state, "vel")
            effort = self._copy_joint_field(joint_state, "eff")
            motor_errors = []
            for index, motor_info in enumerate(motor_infos):
                error_code = getattr(motor_info, "error_code", None)
                if not isinstance(error_code, str):
                    raise RuntimeError(
                        f"i2rt motor feedback {index} has no error code"
                    )
                if error_code != "0x1":
                    detail = str(getattr(motor_info, "error_message", error_code))
                    motor_errors.append(f"motor {index + 1}: {detail} ({error_code})")
            self._last_motor_errors = tuple(motor_errors)
            # DMChainCanInterface replaces this list only after a successful CAN
            # producer cycle. Keep the previous object alive so identity cannot
            # be recycled, and never use read_states()'s poll-time timestamp.
            is_new = motor_state is not self._last_motor_state
            if is_new:
                self._sequence += 1
                self._last_frame_monotonic_ns = self._monotonic_ns()
                self._last_frame_wall_time_ns = self._wall_time_ns()
                self._last_motor_state = motor_state
            sequence = self._sequence
            monotonic_ns = self._last_frame_monotonic_ns
            wall_time_ns = self._last_frame_wall_time_ns
        if sequence < 0 or monotonic_ns <= 0 or wall_time_ns <= 0:
            raise RuntimeError("i2rt has not produced a motor feedback frame")
        return position, velocity, effort, monotonic_ns, wall_time_ns

    def read_motion_state(self) -> MotionState:
        position, velocity, effort, monotonic_ns, _ = self._read_joints()
        return MotionState(
            position=position.astype(np.float64),
            velocity=velocity.astype(np.float64),
            effort=effort.astype(np.float64),
            monotonic_ns=monotonic_ns,
            sequence=self._sequence,
            motor_errors=self._last_motor_errors,
        )

    def command_joint_positions(self, target: np.ndarray) -> None:
        command = np.asarray(target, dtype=np.float64)
        if command.shape != (7,) or not np.isfinite(command).all():
            raise ValueError("YAM joint command must be a finite seven-vector")
        with self._lock:
            if self._closed:
                raise RuntimeError("YAM source is closed")
            self._robot.command_joint_pos(command.copy())

    def enter_safe_idle(self) -> None:
        with self._lock:
            if self._closed:
                return
            enter_idle = getattr(self._robot, "enter_gravity_comp_idle", None)
            if not callable(enter_idle):
                raise RuntimeError("i2rt robot has no verified gravity-comp idle API")
            enter_idle()

    def close(self) -> None:
        idle_error: BaseException | None = None
        with self._lock:
            if self._closed:
                return
            enter_idle = getattr(self._robot, "enter_gravity_comp_idle", None)
            if not callable(enter_idle):
                idle_error = RuntimeError(
                    "i2rt robot has no verified gravity-comp idle API"
                )
            else:
                try:
                    enter_idle()
                except BaseException as exc:
                    idle_error = exc
            self._closed = True
        try:
            self._robot.close()
        finally:
            if idle_error is not None:
                raise idle_error


class I2rtYamSource(I2rtYamMotionSource):
    """Own the current-i2rt YAM plus both production observation cameras."""

    def __init__(
        self,
        config: BridgeConfig,
        *,
        robot_factory: Callable[..., Any] | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        wall_time_ns: Callable[[], int] = time.time_ns,
        kinematics_backend: I2rtKinematicsBackend | None = None,
        camera_source: RgbdCamera | None = None,
        top_camera_source: RgbCamera | None = None,
        camera_kinematics: StationCameraKinematics | None = None,
    ) -> None:
        kinematics = (
            I2rtKinematicsBackend.from_config(config)
            if kinematics_backend is None
            else kinematics_backend
        )
        camera_fk = (
            StationCameraKinematics.from_config(config, kinematics)
            if camera_kinematics is None
            else camera_kinematics
        )
        super().__init__(
            config,
            robot_factory=robot_factory,
            monotonic_ns=monotonic_ns,
            wall_time_ns=wall_time_ns,
            kinematics_backend=kinematics,
        )
        # Production cameras are wrapped in StreamingCamera: a dedicated thread per
        # camera is the only hardware reader (continuous capture, optional run
        # recording via SIGUSR1/SIGUSR2); injected test cameras are used as-is.
        from agp_yam_bridge.record import StreamingCamera, get_recorder

        recorder = get_recorder()
        try:
            if camera_source is None:
                camera = StreamingCamera(
                    RealSenseRgbdCamera(config),
                    name="wrist",
                    recorder=recorder,
                    frame_attr="rgb",
                    timeout_s=config.camera.frame_timeout_s,
                    stale_after_s=config.camera.stale_after_s,
                    monotonic_ns=monotonic_ns,
                )
                if recorder is not None:
                    recorder.register(
                        "wrist", width=config.camera.width, height=config.camera.height,
                        pixfmt="rgb24", fps=config.camera.fps,
                    )
            else:
                camera = camera_source
        except BaseException:
            self._robot.close()
            raise
        try:
            top_calibration = load_fixed_rgb_calibration(
                config.top_camera.calibration_path
            )
            if top_camera_source is None:
                top_camera = StreamingCamera(
                    OpenCvFixedRgbCamera(config),
                    name="top",
                    recorder=recorder,
                    frame_attr="rgb",
                    timeout_s=max(1.0, 4.0 / max(1, top_calibration.fps)),
                    stale_after_s=config.top_camera.stale_after_s,
                    monotonic_ns=monotonic_ns,
                )
                if recorder is not None:
                    recorder.register(
                        "top", width=top_calibration.width, height=top_calibration.height,
                        pixfmt="rgb24", fps=top_calibration.fps, every=2,
                    )
            else:
                top_camera = top_camera_source
        except BaseException:
            camera.close()
            self._robot.close()
            raise
        self._camera = camera
        self._top_camera = top_camera
        self._top_calibration = top_calibration
        self._camera_kinematics = camera_fk
        self._observation_sequence = 0
        self._last_camera_sequence = -1
        self._last_top_camera_sequence = -1

    def read(self, *, request_id: int) -> dict[str, Any]:
        camera_frame = self._camera.read()
        validate_rgbd_frame(
            camera_frame,
            self._config.camera,
            expected_serial=self._config.hardware.camera_serial,
            # The owned camera source is responsible for freshness at capture;
            # this second validation protects the structural boundary.
            now_monotonic_ns=camera_frame.monotonic_ns,
        )
        top_frame = self._top_camera.read()
        validate_rgb_frame(
            top_frame,
            self._top_calibration,
            stale_after_s=self._config.top_camera.stale_after_s,
            now_monotonic_ns=top_frame.monotonic_ns,
        )
        position, velocity, effort, monotonic_ns, wall_time_ns = self._read_joints()
        joint_time_delta_ns = abs(camera_frame.monotonic_ns - monotonic_ns)
        max_skew_ns = int(self._config.camera.max_joint_skew_s * 1e9)
        if joint_time_delta_ns > max_skew_ns:
            raise CameraFrameError(
                f"camera/joint skew {joint_time_delta_ns / 1e6:.3f}ms exceeds "
                f"{max_skew_ns / 1e6:.3f}ms"
            )
        top_joint_time_delta_ns = abs(top_frame.monotonic_ns - monotonic_ns)
        top_max_skew_ns = int(self._config.top_camera.max_joint_skew_s * 1e9)
        if top_joint_time_delta_ns > top_max_skew_ns:
            raise CameraFrameError(
                f"top camera/joint skew {top_joint_time_delta_ns / 1e6:.3f}ms "
                f"exceeds {top_max_skew_ns / 1e6:.3f}ms"
            )
        with self._lock:
            if camera_frame.sequence <= self._last_camera_sequence:
                raise CameraFrameError(
                    f"camera sequence {camera_frame.sequence} did not advance beyond "
                    f"{self._last_camera_sequence}"
                )
            self._last_camera_sequence = camera_frame.sequence
            if top_frame.sequence <= self._last_top_camera_sequence:
                raise CameraFrameError(
                    f"top camera sequence {top_frame.sequence} did not advance beyond "
                    f"{self._last_top_camera_sequence}"
                )
            self._last_top_camera_sequence = top_frame.sequence
            observation_sequence = self._observation_sequence
            self._observation_sequence += 1
        cartesian_pose = self._kinematics.forward(position[:6])
        cartesian = np.concatenate((cartesian_pose, position[6:7])).astype(
            np.float32, copy=False
        )
        world_from_camera = self._camera_kinematics.world_from_camera(position[:6])
        return {
            "schema_version": 1,
            "message_type": "observation",
            "request_id": request_id,
            "sequence": observation_sequence,
            "monotonic_ns": monotonic_ns,
            "wall_time_ns": wall_time_ns,
            "robot_joint_pos_0": position,
            "robot_joint_vel_0": velocity,
            "robot_joint_effort_0": effort,
            "robot_cartesian_pos_0": cartesian,
            "camera_0": {
                "name": self._config.camera.name,
                "serial": camera_frame.serial,
                "frame_sequence": camera_frame.sequence,
                "observation_sequence": observation_sequence,
                "frame_monotonic_ns": camera_frame.monotonic_ns,
                "frame_wall_time_ns": camera_frame.wall_time_ns,
                "device_timestamp_ms": camera_frame.device_timestamp_ms,
                "joint_monotonic_ns": monotonic_ns,
                "joint_time_delta_ns": joint_time_delta_ns,
                "transform_translation_error_bound_m": (
                    self._config.camera.transform_translation_error_bound_m
                ),
                "color_order": "RGB",
                "depth_unit": "meter",
                "invalid_depth_value": 0.0,
                "images": {
                    "rgb": camera_frame.rgb.copy(),
                    "depth": camera_frame.depth_m.copy(),
                },
                "intrinsics": camera_frame.intrinsics.copy(),
                "distortion_model": camera_frame.distortion_model,
                "distortion_coefficients": camera_frame.distortion_coefficients.copy(),
                "camera_to_world": world_from_camera.astype(np.float64, copy=False),
            },
            "camera_1": {
                "name": self._top_calibration.name,
                "serial": top_frame.serial,
                "frame_sequence": top_frame.sequence,
                "observation_sequence": observation_sequence,
                "frame_monotonic_ns": top_frame.monotonic_ns,
                "frame_wall_time_ns": top_frame.wall_time_ns,
                "joint_monotonic_ns": monotonic_ns,
                "joint_time_delta_ns": top_joint_time_delta_ns,
                "transform_translation_error_bound_m": (
                    self._top_calibration.transform_translation_error_bound_m
                ),
                "color_order": "RGB",
                "images": {
                    "rgb": top_frame.rgb.copy(),
                },
                "intrinsics": top_frame.intrinsics.copy(),
                "distortion_model": top_frame.distortion_model,
                "distortion_coefficients": top_frame.distortion_coefficients.copy(),
                "camera_to_world": top_frame.camera_to_world.copy(),
            },
            "health": {
                "state": "ok",
                "source_connected": True,
                "motion_enabled": False,
                "safety_state": "idle",
                "active_action_request_id": None,
                "detail": "",
            },
        }

    def close(self) -> None:
        idle_error: BaseException | None = None
        with self._lock:
            if self._closed:
                return
            enter_idle = getattr(self._robot, "enter_gravity_comp_idle", None)
            if not callable(enter_idle):
                idle_error = RuntimeError(
                    "i2rt robot has no verified gravity-comp idle API"
                )
            else:
                try:
                    enter_idle()
                except BaseException as exc:
                    idle_error = exc
            self._closed = True
        try:
            try:
                self._top_camera.close()
            finally:
                self._camera.close()
        finally:
            self._robot.close()
        if idle_error is not None:
            raise idle_error


class FakeYamSource:
    """Deterministic source for contract tests and no-hardware development."""

    def __init__(self, config: BridgeConfig | None = None) -> None:
        if config is None:
            from agp_yam_bridge.config import load_config
            from agp_yam_bridge.preflight import DEFAULT_CONFIG

            config = load_config(DEFAULT_CONFIG)
        self._kinematics = I2rtKinematicsBackend.from_config(config)
        self._camera_kinematics = StationCameraKinematics.from_config(
            config, self._kinematics
        )
        self._config = config
        self.joint_pos = np.append(
            np.radians([0, 0, 0, 90, 90, 90]), 0.5
        ).astype(np.float32)
        self.joint_vel = np.zeros(7, dtype=np.float32)
        self.joint_effort = np.zeros(7, dtype=np.float32)
        self.sequence = 0
        self._motion_sequence = 0
        self.timestamp_offset_ns = 0
        self.closed = False
        self._lock = threading.Lock()
        self.idle = True

    @property
    def kinematics(self) -> I2rtKinematicsBackend:
        return self._kinematics

    def read(self, *, request_id: int) -> dict[str, Any]:
        with self._lock:
            sequence = self.sequence
            self.sequence += 1
            timestamp = time.monotonic_ns() + self.timestamp_offset_ns
        if self.joint_pos.shape == (7,):
            cartesian_pose = self._kinematics.forward(self.joint_pos[:6])
            cartesian = np.concatenate((cartesian_pose, self.joint_pos[6:7])).astype(
                np.float32, copy=False
            )
        else:
            # Let the protocol validator report the malformed joint vector;
            # do not mask it with an indexing failure in the fake source.
            cartesian = np.zeros(8, dtype=np.float32)
        camera_to_world = (
            self._camera_kinematics.world_from_camera(self.joint_pos[:6])
            if self.joint_pos.shape == (7,)
            else np.eye(4, dtype=np.float64)
        )
        rgb = np.zeros((4, 6, 3), dtype=np.uint8)
        rgb[..., 0] = 64
        rgb[..., 1] = 128
        rgb[..., 2] = 192
        top_calibration = load_fixed_rgb_calibration(
            self._config.top_camera.calibration_path
        )
        return {
            "schema_version": 1,
            "message_type": "observation",
            "request_id": request_id,
            "sequence": sequence,
            "monotonic_ns": timestamp,
            "wall_time_ns": time.time_ns(),
            "robot_joint_pos_0": self.joint_pos.copy(),
            "robot_joint_vel_0": self.joint_vel.copy(),
            "robot_joint_effort_0": self.joint_effort.copy(),
            "robot_cartesian_pos_0": cartesian,
            "camera_0": {
                "name": self._config.camera.name,
                "serial": self._config.hardware.camera_serial,
                "frame_sequence": sequence,
                "observation_sequence": sequence,
                "frame_monotonic_ns": timestamp,
                "frame_wall_time_ns": time.time_ns(),
                "device_timestamp_ms": float(sequence) * 1000.0 / self._config.camera.fps,
                "joint_monotonic_ns": timestamp,
                "joint_time_delta_ns": 0,
                "transform_translation_error_bound_m": (
                    self._config.camera.transform_translation_error_bound_m
                ),
                "color_order": "RGB",
                "depth_unit": "meter",
                "invalid_depth_value": 0.0,
                "images": {
                    "rgb": rgb,
                    "depth": np.full((4, 6), 0.25, dtype=np.float32),
                },
                "intrinsics": np.array(
                    [[100.0, 0.0, 2.5], [0.0, 100.0, 1.5], [0.0, 0.0, 1.0]],
                    dtype=np.float64,
                ),
                "distortion_model": "inverse_brown_conrady",
                "distortion_coefficients": np.array(
                    [
                        -0.05205012485384941,
                        0.06445974856615067,
                        0.0006445064209401608,
                        0.00034590926952660084,
                        -0.022210294380784035,
                    ],
                    dtype=np.float64,
                ),
                "camera_to_world": camera_to_world.astype(np.float64, copy=False),
            },
            "camera_1": {
                "name": top_calibration.name,
                "serial": top_calibration.serial,
                "frame_sequence": sequence,
                "observation_sequence": sequence,
                "frame_monotonic_ns": timestamp,
                "frame_wall_time_ns": time.time_ns(),
                "joint_monotonic_ns": timestamp,
                "joint_time_delta_ns": 0,
                "transform_translation_error_bound_m": (
                    top_calibration.transform_translation_error_bound_m
                ),
                "color_order": "RGB",
                "images": {
                    "rgb": np.full((4, 6, 3), 127, dtype=np.uint8),
                },
                "intrinsics": top_calibration.camera_matrix.copy(),
                "distortion_model": "none",
                "distortion_coefficients": np.zeros(5, dtype=np.float64),
                "camera_to_world": top_calibration.camera_to_world.copy(),
            },
            "health": {
                "state": "ok",
                "source_connected": True,
                "motion_enabled": False,
                "safety_state": "idle",
                "active_action_request_id": None,
                "detail": "fake observation source",
            },
        }

    def close(self) -> None:
        self.enter_safe_idle()
        self.closed = True

    def read_motion_state(self) -> MotionState:
        with self._lock:
            if self.closed:
                raise RuntimeError("fake YAM source is closed")
            self._motion_sequence += 1
            return MotionState(
                position=self.joint_pos.astype(np.float64, copy=True),
                velocity=self.joint_vel.astype(np.float64, copy=True),
                effort=self.joint_effort.astype(np.float64, copy=True),
                monotonic_ns=time.monotonic_ns() + self.timestamp_offset_ns,
                sequence=self._motion_sequence,
                motor_errors=(),
            )

    def command_joint_positions(self, target: np.ndarray) -> None:
        command = np.asarray(target, dtype=np.float64)
        if command.shape != (7,) or not np.isfinite(command).all():
            raise ValueError("fake YAM joint command must be a finite seven-vector")
        with self._lock:
            self.joint_pos = command.astype(np.float32)
            self.joint_vel = np.zeros(7, dtype=np.float32)
            self.idle = False

    def enter_safe_idle(self) -> None:
        self.idle = True
