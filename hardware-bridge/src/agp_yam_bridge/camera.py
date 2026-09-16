"""Aligned D405 RGB-D frames and i2rt station-model camera kinematics."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import cv2
import mujoco
import numpy as np

from agp_yam_bridge.config import BridgeConfig, CameraStreamConfig
from agp_yam_bridge.kinematics import I2rtKinematicsBackend


class CameraFrameError(RuntimeError):
    """A camera frame is missing, stale, misaligned, or violates calibration."""


@dataclass(frozen=True)
class RgbdFrame:
    """One aligned RGB-D frame in the color optical frame."""

    sequence: int
    monotonic_ns: int
    wall_time_ns: int
    device_timestamp_ms: float
    serial: str
    rgb: np.ndarray
    depth_m: np.ndarray
    intrinsics: np.ndarray
    distortion_model: str
    distortion_coefficients: np.ndarray


@dataclass(frozen=True)
class RgbFrame:
    """One undistorted RGB frame from a calibrated fixed camera."""

    sequence: int
    monotonic_ns: int
    wall_time_ns: int
    serial: str
    rgb: np.ndarray
    intrinsics: np.ndarray
    camera_to_world: np.ndarray
    distortion_model: str
    distortion_coefficients: np.ndarray


@dataclass(frozen=True)
class FixedRgbCalibration:
    """Runtime-critical fields from one accepted fixed-camera report."""

    name: str
    serial: str
    device: Path
    width: int
    height: int
    fps: int
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    camera_to_world: np.ndarray
    transform_translation_error_bound_m: float


class RgbdCamera(Protocol):
    def read(self) -> RgbdFrame: ...

    def close(self) -> None: ...


class RgbCamera(Protocol):
    def read(self) -> RgbFrame: ...

    def close(self) -> None: ...


def _rigid_transform(value: Any, *, label: str) -> np.ndarray:
    transform = np.asarray(value, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise CameraFrameError(f"{label} must be finite float64[4,4]")
    rotation = transform[:3, :3]
    if (
        not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-10)
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5)
    ):
        raise CameraFrameError(f"{label} must be a rigid transform")
    return np.ascontiguousarray(transform)


def _pinhole_matrix(value: Any, *, label: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if (
        matrix.shape != (3, 3)
        or not np.isfinite(matrix).all()
        or matrix[0, 0] <= 0
        or matrix[1, 1] <= 0
        or not np.allclose(matrix[2], [0.0, 0.0, 1.0], atol=1e-10)
    ):
        raise CameraFrameError(f"{label} must be a finite pinhole float64[3,3]")
    return np.ascontiguousarray(matrix)


def load_fixed_rgb_calibration(path: Path) -> FixedRgbCalibration:
    """Load a PASS fixed-camera artifact and reject incomplete runtime geometry."""
    calibration_path = path.resolve()
    try:
        payload = json.loads(calibration_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CameraFrameError(
            f"could not load fixed RGB calibration {calibration_path}: {exc}"
        ) from exc
    if (
        not isinstance(payload, dict)
        or payload.get("schema_version") != 1
        or payload.get("type") != "fixed_rgb_camera_calibration"
        or payload.get("status") != "PASS"
    ):
        raise CameraFrameError(
            "fixed RGB calibration must be a schema-1 PASS camera report"
        )
    camera = payload.get("camera")
    if not isinstance(camera, dict):
        raise CameraFrameError("fixed RGB calibration camera metadata must be a map")
    name = camera.get("name")
    serial = camera.get("serial")
    device_value = camera.get("device")
    if (
        not isinstance(name, str)
        or not name
        or not isinstance(serial, str)
        or not serial
        or not isinstance(device_value, str)
        or serial not in Path(device_value).name
    ):
        raise CameraFrameError(
            "fixed RGB calibration must bind a named camera and serial device path"
        )
    profile = tuple(camera.get(key) for key in ("width", "height", "fps"))
    if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in profile):
        raise CameraFrameError("fixed RGB calibration stream profile must be positive integers")
    if payload.get("distortion_model") != "opencv_radtan":
        raise CameraFrameError("fixed RGB calibration distortion model must be opencv_radtan")
    metrics = payload.get("metrics")
    transform_translation_error_bound_m = (
        metrics.get("validation_translation_max_m")
        if isinstance(metrics, dict)
        else None
    )
    if (
        isinstance(transform_translation_error_bound_m, bool)
        or not isinstance(transform_translation_error_bound_m, (int, float))
        or not np.isfinite(float(transform_translation_error_bound_m))
        or float(transform_translation_error_bound_m) <= 0.0
    ):
        raise CameraFrameError(
            "fixed RGB calibration must include a positive held-out translation error"
        )
    distortion = np.asarray(payload.get("distortion_coefficients"), dtype=np.float64)
    if distortion.shape != (5,) or not np.isfinite(distortion).all():
        raise CameraFrameError(
            "fixed RGB distortion coefficients must be a finite float64 five-vector"
        )
    width, height, fps = profile
    return FixedRgbCalibration(
        name=name,
        serial=serial,
        device=Path(device_value),
        width=width,
        height=height,
        fps=fps,
        camera_matrix=_pinhole_matrix(payload.get("camera_matrix"), label="camera_matrix"),
        distortion_coefficients=np.ascontiguousarray(distortion),
        camera_to_world=_rigid_transform(
            payload.get("camera_to_world"), label="camera_to_world"
        ),
        transform_translation_error_bound_m=float(
            transform_translation_error_bound_m
        ),
    )


def validate_rgb_frame(
    frame: RgbFrame,
    calibration: FixedRgbCalibration,
    *,
    stale_after_s: float,
    now_monotonic_ns: int | None = None,
) -> RgbFrame:
    """Fail closed unless a fixed-camera frame matches its accepted calibration."""
    if frame.serial != calibration.serial:
        raise CameraFrameError(
            f"camera serial {frame.serial!r} does not match {calibration.serial!r}"
        )
    if isinstance(frame.sequence, bool) or not isinstance(frame.sequence, int) or frame.sequence < 0:
        raise CameraFrameError("camera sequence must be a non-negative integer")
    if frame.monotonic_ns <= 0 or frame.wall_time_ns <= 0:
        raise CameraFrameError("camera timestamps must be positive")
    now = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
    age_ns = now - frame.monotonic_ns
    if age_ns < 0:
        raise CameraFrameError("camera timestamp is in the future")
    if age_ns > int(stale_after_s * 1e9):
        raise CameraFrameError(
            f"camera frame is stale: age {age_ns / 1e9:.3f}s exceeds {stale_after_s:.3f}s"
        )
    if frame.rgb.dtype != np.uint8 or frame.rgb.shape != (
        calibration.height,
        calibration.width,
        3,
    ):
        raise CameraFrameError(
            f"RGB must be uint8[{calibration.height},{calibration.width},3] in RGB order"
        )
    if not np.array_equal(frame.intrinsics, calibration.camera_matrix):
        raise CameraFrameError("camera intrinsics do not match the accepted calibration")
    if not np.array_equal(frame.camera_to_world, calibration.camera_to_world):
        raise CameraFrameError("camera_to_world does not match the accepted calibration")
    if frame.distortion_model != "none":
        raise CameraFrameError("rectified fixed RGB distortion model must be none")
    if (
        frame.distortion_coefficients.dtype != np.float64
        or frame.distortion_coefficients.shape != (5,)
        or not np.array_equal(frame.distortion_coefficients, np.zeros(5))
    ):
        raise CameraFrameError(
            "rectified fixed RGB distortion coefficients must be float64 zeros[5]"
        )
    return frame


def _frame_transform(frame: Any) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(frame.xmat, dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(frame.xpos, dtype=np.float64)
    return transform


class StationCameraKinematics:
    """Compose i2rt arm FK with the station MJCF's rigid flange-camera transform."""

    def __init__(
        self,
        arm_kinematics: I2rtKinematicsBackend,
        flange_from_camera: np.ndarray,
    ) -> None:
        transform = np.asarray(flange_from_camera, dtype=np.float64)
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise CameraFrameError("flange-to-camera transform must be finite 4x4")
        if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-10):
            raise CameraFrameError("flange-to-camera transform is not homogeneous")
        rotation = transform[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7) or not np.isclose(
            np.linalg.det(rotation), 1.0, atol=1e-7
        ):
            raise CameraFrameError("flange-to-camera rotation is not rigid")
        self._arm_kinematics = arm_kinematics
        self._flange_from_camera = transform.copy()

    @classmethod
    def from_config(
        cls,
        config: BridgeConfig,
        arm_kinematics: I2rtKinematicsBackend,
    ) -> StationCameraKinematics:
        """Read the flange-camera extrinsic from the configured i2rt station MJCF."""
        try:
            model = mujoco.MjModel.from_xml_path(str(config.hardware.station_model))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            station_from_flange = _frame_transform(
                data.body(config.camera.station_flange_body)
            )
            station_from_camera = _frame_transform(
                data.body(config.camera.station_camera_body)
            )
        except Exception as exc:
            raise CameraFrameError(
                "could not read flange-to-camera extrinsic from i2rt station model: "
                f"{exc}"
            ) from exc
        return cls(
            arm_kinematics,
            np.linalg.inv(station_from_flange) @ station_from_camera,
        )

    @property
    def flange_from_camera(self) -> np.ndarray:
        return self._flange_from_camera.copy()

    def world_from_camera(self, joints: np.ndarray) -> np.ndarray:
        chain = self._arm_kinematics.frame_chain(joints)
        return chain.world_from_base @ chain.base_from_gripper @ self._flange_from_camera


def normalize_rgbd_arrays(
    rgb: np.ndarray,
    raw_depth: np.ndarray,
    intrinsics: np.ndarray,
    *,
    depth_scale_m: float,
    width: int,
    height: int,
    min_depth_m: float,
    max_depth_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Validate aligned RGB/Z16 arrays and convert valid depth samples to meters."""
    color = np.asarray(rgb)
    depth = np.asarray(raw_depth)
    K = np.asarray(intrinsics, dtype=np.float64)
    if color.dtype != np.uint8 or color.shape != (height, width, 3):
        raise CameraFrameError(
            f"RGB must be uint8[{height},{width},3] in RGB order, got {color.dtype}{color.shape}"
        )
    if depth.dtype != np.uint16 or depth.shape != (height, width):
        raise CameraFrameError(
            f"depth must be aligned uint16[{height},{width}], got {depth.dtype}{depth.shape}"
        )
    if K.shape != (3, 3) or not np.isfinite(K).all():
        raise CameraFrameError("intrinsics must be a finite 3x3 matrix")
    if not np.allclose(K[2], [0.0, 0.0, 1.0], atol=1e-10) or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise CameraFrameError("intrinsics are not a valid pinhole matrix")
    if not np.isfinite(depth_scale_m) or depth_scale_m <= 0:
        raise CameraFrameError("depth scale must be finite and positive")

    depth_m = depth.astype(np.float32) * np.float32(depth_scale_m)
    invalid = (depth == 0) | (depth_m < min_depth_m) | (depth_m > max_depth_m)
    depth_m[invalid] = 0.0
    return (
        np.ascontiguousarray(color),
        np.ascontiguousarray(depth_m, dtype=np.float32),
        np.ascontiguousarray(K, dtype=np.float64),
    )


def validate_rgbd_frame(
    frame: RgbdFrame,
    config: CameraStreamConfig,
    *,
    expected_serial: str,
    now_monotonic_ns: int | None = None,
) -> RgbdFrame:
    """Fail closed unless a camera frame satisfies the complete P3 contract."""
    if frame.serial != expected_serial:
        raise CameraFrameError(
            f"camera serial {frame.serial!r} does not match {expected_serial!r}"
        )
    if isinstance(frame.sequence, bool) or not isinstance(frame.sequence, int) or frame.sequence < 0:
        raise CameraFrameError("camera sequence must be a non-negative integer")
    if frame.monotonic_ns <= 0 or frame.wall_time_ns <= 0:
        raise CameraFrameError("camera timestamps must be positive")
    if not np.isfinite(frame.device_timestamp_ms) or frame.device_timestamp_ms < 0:
        raise CameraFrameError("camera device timestamp must be finite and non-negative")
    now = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
    age_ns = now - frame.monotonic_ns
    if age_ns < 0:
        raise CameraFrameError("camera timestamp is in the future")
    if age_ns > int(config.stale_after_s * 1e9):
        raise CameraFrameError(
            f"camera frame is stale: age {age_ns / 1e9:.3f}s exceeds "
            f"{config.stale_after_s:.3f}s"
        )
    if frame.rgb.dtype != np.uint8 or frame.rgb.shape != (
        config.height,
        config.width,
        3,
    ):
        raise CameraFrameError(
            f"RGB must be uint8[{config.height},{config.width},3] in RGB order"
        )
    if frame.depth_m.dtype != np.float32 or frame.depth_m.shape != frame.rgb.shape[:2]:
        raise CameraFrameError("metric depth must be float32 and pixel-aligned with RGB")
    if not np.isfinite(frame.depth_m).all():
        raise CameraFrameError("metric depth must contain only finite values")
    valid_depth = frame.depth_m > 0
    if not valid_depth.any():
        raise CameraFrameError("camera frame contains no valid depth samples")
    if (
        (frame.depth_m[valid_depth] < config.min_depth_m).any()
        or (frame.depth_m[valid_depth] > config.max_depth_m).any()
    ):
        raise CameraFrameError("metric depth is outside the configured valid range")
    if frame.intrinsics.dtype != np.float64 or frame.intrinsics.shape != (3, 3):
        raise CameraFrameError("intrinsics must be float64[3,3]")
    if not np.isfinite(frame.intrinsics).all():
        raise CameraFrameError("intrinsics must contain only finite values")
    if not np.allclose(frame.intrinsics[2], [0.0, 0.0, 1.0], atol=1e-10):
        raise CameraFrameError("intrinsics bottom row must be [0,0,1]")
    if frame.intrinsics[0, 0] <= 0 or frame.intrinsics[1, 1] <= 0:
        raise CameraFrameError("intrinsics focal lengths must be positive")
    if frame.distortion_model != "inverse_brown_conrady":
        raise CameraFrameError(
            "D405 distortion model must be inverse_brown_conrady"
        )
    if (
        frame.distortion_coefficients.dtype != np.float64
        or frame.distortion_coefficients.shape != (5,)
        or not np.isfinite(frame.distortion_coefficients).all()
    ):
        raise CameraFrameError(
            "D405 distortion coefficients must be finite float64[5]"
        )
    return frame


class RealSenseRgbdCamera:
    """Thin pyrealsense2 adapter for fresh depth-to-color-aligned D405 frames."""

    def __init__(
        self,
        config: BridgeConfig,
        *,
        rs_module: Any | None = None,
        monotonic_ns: Any = time.monotonic_ns,
        wall_time_ns: Any = time.time_ns,
    ) -> None:
        if rs_module is None:
            try:
                import pyrealsense2 as rs_module
            except ImportError as exc:
                raise RuntimeError(
                    "P3 D405 capture requires pyrealsense2; run `uv sync --locked`"
                ) from exc
        self._config = config
        self._monotonic_ns = monotonic_ns
        self._wall_time_ns = wall_time_ns
        self._pipeline = rs_module.pipeline()
        rs_config = rs_module.config()
        rs_config.enable_device(config.hardware.camera_serial)
        rs_config.enable_stream(
            rs_module.stream.color,
            config.camera.width,
            config.camera.height,
            rs_module.format.rgb8,
            config.camera.fps,
        )
        rs_config.enable_stream(
            rs_module.stream.depth,
            config.camera.width,
            config.camera.height,
            rs_module.format.z16,
            config.camera.fps,
        )
        self._started = False
        try:
            profile = self._pipeline.start(rs_config)
            self._started = True
            device = profile.get_device()
            observed_serial = device.get_info(rs_module.camera_info.serial_number)
            if observed_serial != config.hardware.camera_serial:
                raise CameraFrameError(
                    f"camera serial {observed_serial!r} does not match "
                    f"{config.hardware.camera_serial!r}"
                )
            self._serial = observed_serial
            self._depth_scale_m = float(device.first_depth_sensor().get_depth_scale())
            self._align = rs_module.align(rs_module.stream.color)
        except BaseException:
            self.close()
            raise

    def read(self) -> RgbdFrame:
        timeout_ms = max(1, round(self._config.camera.frame_timeout_s * 1000))
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=timeout_ms)
            aligned = self._align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
        except Exception as exc:
            raise CameraFrameError(f"D405 frame capture failed: {exc}") from exc
        if not color_frame or not depth_frame:
            raise CameraFrameError("D405 frame is missing aligned RGB or depth")
        color_sequence = int(color_frame.get_frame_number())
        color_domain = color_frame.get_frame_timestamp_domain()
        depth_domain = depth_frame.get_frame_timestamp_domain()
        color_timestamp_ms = float(color_frame.get_timestamp())
        depth_timestamp_ms = float(depth_frame.get_timestamp())
        max_stream_skew_ms = 500.0 / self._config.camera.fps
        if color_domain != depth_domain:
            raise CameraFrameError(
                f"D405 RGB/depth timestamp domains differ: {color_domain} vs {depth_domain}"
            )
        if abs(color_timestamp_ms - depth_timestamp_ms) > max_stream_skew_ms:
            raise CameraFrameError(
                f"D405 RGB/depth timestamps differ by "
                f"{abs(color_timestamp_ms - depth_timestamp_ms):.3f}ms; "
                f"maximum is {max_stream_skew_ms:.3f}ms"
            )
        capture_monotonic_ns = self._monotonic_ns()
        capture_wall_time_ns = self._wall_time_ns()
        intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
        distortion_model = str(intrinsics.model).rsplit(".", 1)[-1]
        distortion_coefficients = np.asarray(intrinsics.coeffs, dtype=np.float64)
        K = np.array(
            [
                [intrinsics.fx, 0.0, intrinsics.ppx],
                [0.0, intrinsics.fy, intrinsics.ppy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        rgb, depth_m, K = normalize_rgbd_arrays(
            np.asanyarray(color_frame.get_data()),
            np.asanyarray(depth_frame.get_data()),
            K,
            depth_scale_m=self._depth_scale_m,
            width=self._config.camera.width,
            height=self._config.camera.height,
            min_depth_m=self._config.camera.min_depth_m,
            max_depth_m=self._config.camera.max_depth_m,
        )
        return validate_rgbd_frame(
            RgbdFrame(
                sequence=color_sequence,
                monotonic_ns=capture_monotonic_ns,
                wall_time_ns=capture_wall_time_ns,
                device_timestamp_ms=color_timestamp_ms,
                serial=self._serial,
                rgb=rgb,
                depth_m=depth_m,
                intrinsics=K,
                distortion_model=distortion_model,
                distortion_coefficients=np.ascontiguousarray(
                    distortion_coefficients
                ),
            ),
            self._config.camera,
            expected_serial=self._config.hardware.camera_serial,
            now_monotonic_ns=self._monotonic_ns(),
        )

    def close(self) -> None:
        if self._started:
            self._started = False
            self._pipeline.stop()


class OpenCvFixedRgbCamera:
    """Serial-bound V4L2 adapter for the calibrated overhead BRIO."""

    def __init__(
        self,
        config: BridgeConfig,
        *,
        capture_factory: Any = cv2.VideoCapture,
        warmup_frames: int = 5,
        monotonic_ns: Any = time.monotonic_ns,
        wall_time_ns: Any = time.time_ns,
    ) -> None:
        self._config = config.top_camera
        self._calibration = load_fixed_rgb_calibration(
            self._config.calibration_path
        )
        self._monotonic_ns = monotonic_ns
        self._wall_time_ns = wall_time_ns
        calibration = self._calibration
        capture = capture_factory(str(calibration.device), cv2.CAP_V4L2)
        self._capture = capture
        self._closed = False
        self._sequence = 0
        if not capture.isOpened():
            capture.release()
            self._closed = True
            raise CameraFrameError(
                f"could not open fixed RGB camera {calibration.device}"
            )
        try:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, calibration.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, calibration.height)
            capture.set(cv2.CAP_PROP_FPS, calibration.fps)
            actual_profile = (
                round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                round(capture.get(cv2.CAP_PROP_FPS)),
            )
            expected_profile = (
                calibration.width,
                calibration.height,
                calibration.fps,
            )
            if actual_profile != expected_profile:
                raise CameraFrameError(
                    f"fixed RGB camera profile {actual_profile} does not match "
                    f"calibrated {expected_profile}"
                )
            self._map_x, self._map_y = cv2.initUndistortRectifyMap(
                calibration.camera_matrix,
                calibration.distortion_coefficients,
                None,
                calibration.camera_matrix,
                (calibration.width, calibration.height),
                cv2.CV_32FC1,
            )
            for _ in range(warmup_frames):
                ok, image = capture.read()
                if not ok or image is None:
                    raise CameraFrameError("fixed RGB camera warmup capture failed")
        except BaseException:
            self.close()
            raise

    @property
    def calibration(self) -> FixedRgbCalibration:
        return self._calibration

    def read(self) -> RgbFrame:
        if self._closed:
            raise CameraFrameError("fixed RGB camera is closed")
        try:
            ok, bgr = self._capture.read()
        except Exception as exc:
            raise CameraFrameError(f"fixed RGB frame capture failed: {exc}") from exc
        capture_monotonic_ns = self._monotonic_ns()
        capture_wall_time_ns = self._wall_time_ns()
        if not ok or bgr is None:
            raise CameraFrameError("fixed RGB frame capture failed")
        calibration = self._calibration
        image = np.asarray(bgr)
        if image.dtype != np.uint8 or image.shape != (
            calibration.height,
            calibration.width,
            3,
        ):
            raise CameraFrameError(
                "fixed RGB camera returned an image outside its calibrated profile"
            )
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        undistorted = cv2.remap(
            rgb,
            self._map_x,
            self._map_y,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        frame = RgbFrame(
            sequence=self._sequence,
            monotonic_ns=capture_monotonic_ns,
            wall_time_ns=capture_wall_time_ns,
            serial=calibration.serial,
            rgb=np.ascontiguousarray(undistorted),
            intrinsics=calibration.camera_matrix.copy(),
            camera_to_world=calibration.camera_to_world.copy(),
            distortion_model="none",
            distortion_coefficients=np.zeros(5, dtype=np.float64),
        )
        self._sequence += 1
        return validate_rgb_frame(
            frame,
            calibration,
            stale_after_s=self._config.stale_after_s,
            now_monotonic_ns=self._monotonic_ns(),
        )

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._capture.release()
