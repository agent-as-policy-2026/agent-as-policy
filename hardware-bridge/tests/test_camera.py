from __future__ import annotations

import time

import cv2
import mujoco
import numpy as np
import pytest

from agp_yam_bridge.camera import (
    CameraFrameError,
    OpenCvFixedRgbCamera,
    RealSenseRgbdCamera,
    RgbdFrame,
    RgbFrame,
    StationCameraKinematics,
    load_fixed_rgb_calibration,
    normalize_rgbd_arrays,
    validate_rgb_frame,
    validate_rgbd_frame,
)
from agp_yam_bridge.config import load_config
from agp_yam_bridge.kinematics import I2rtKinematicsBackend
from agp_yam_bridge.preflight import DEFAULT_CONFIG


def _config():
    return load_config(DEFAULT_CONFIG)


def test_fixed_top_calibration_keeps_held_out_position_error_bound() -> None:
    """Dropping the held-out error makes a precision graph invent its budget."""
    calibration = load_fixed_rgb_calibration(_config().top_camera.calibration_path)

    # metrics.validation_translation_max_m of the shipped PASS artifact
    # acceptance/top/top_brio_calibration.json (BRIO B8C7F203, 1920x1080@30).
    # The previous constant 0.0042182384950071134 is the held-out error of the
    # superseded 640x360 calibration, which this repo does not ship.
    assert getattr(
        calibration, "transform_translation_error_bound_m", None
    ) == pytest.approx(
        0.023422225446907616
    )


def test_wrist_camera_config_keeps_independent_hand_eye_error_bound() -> None:
    """The D405 metric authority also needs a measured error budget."""
    assert getattr(
        _config().camera, "transform_translation_error_bound_m", None
    ) == pytest.approx(
        0.0016787199367799639
    )


def _transform(frame) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = frame.xmat.reshape(3, 3)
    matrix[:3, 3] = frame.xpos
    return matrix


def test_station_model_extrinsic_composes_with_i2rt_flange_fk() -> None:
    """Copying a hand-written camera offset would diverge from the station MJCF."""
    config = _config()
    arm_kinematics = I2rtKinematicsBackend.from_config(config)
    camera_kinematics = StationCameraKinematics.from_config(config, arm_kinematics)
    joints = np.deg2rad([10.0, 35.0, 60.0, -20.0, 15.0, 30.0])

    station = mujoco.MjModel.from_xml_path(str(config.hardware.station_model))
    data = mujoco.MjData(station)
    for index, value in enumerate(joints, start=1):
        address = int(station.joint(f"right_joint{index}").qposadr[0])
        data.qpos[address] = value
    mujoco.mj_forward(station, data)
    station_from_base = _transform(data.body("right_base"))
    station_from_camera = _transform(data.body(config.camera.station_camera_body))
    expected_world_from_camera = np.linalg.inv(station_from_base) @ station_from_camera

    np.testing.assert_allclose(
        camera_kinematics.world_from_camera(joints),
        expected_world_from_camera,
        atol=2e-7,
    )
    flange_from_camera = camera_kinematics.flange_from_camera
    assert flange_from_camera.shape == (4, 4)
    np.testing.assert_allclose(flange_from_camera[3], [0, 0, 0, 1], atol=1e-12)


def test_right_wrist_d405_extrinsic_matches_physical_hand_eye_calibration() -> None:
    """The station optical frame must retain the accepted serial-specific calibration."""
    config = _config()
    camera_kinematics = StationCameraKinematics.from_config(
        config,
        I2rtKinematicsBackend.from_config(config),
    )
    expected_flange_from_camera = np.array(
        [
            [
                0.018978140128509846,
                0.9056650507839406,
                0.4235689388821936,
                -0.07422281653487615,
            ],
            [
                0.9995877701854492,
                -0.008058522475136165,
                -0.02755630438209611,
                0.012702331327012938,
            ],
            [
                -0.021543441993879697,
                0.4239172985430555,
                -0.905444644416771,
                -0.07396798968575628,
            ],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        camera_kinematics.flange_from_camera,
        expected_flange_from_camera,
        atol=2e-7,
    )


def test_rgbd_normalization_preserves_rgb_and_converts_depth_to_meters() -> None:
    """Raw Z16 values or a BGR swap would corrupt every downstream 3-D point."""
    rgb = np.array(
        [
            [[255, 0, 0], [0, 255, 0], [0, 0, 255], [1, 2, 3]],
            [[4, 5, 6], [7, 8, 9], [10, 11, 12], [13, 14, 15]],
        ],
        dtype=np.uint8,
    )
    raw_depth = np.array([[0, 5, 100, 2000], [250, 500, 750, 1000]], dtype=np.uint16)
    intrinsics = np.array([[100, 0, 1.5], [0, 101, 0.5], [0, 0, 1]], dtype=np.float64)

    normalized_rgb, depth_m, K = normalize_rgbd_arrays(
        rgb,
        raw_depth,
        intrinsics,
        depth_scale_m=0.001,
        width=4,
        height=2,
        min_depth_m=0.01,
        max_depth_m=1.0,
    )

    np.testing.assert_array_equal(normalized_rgb, rgb)
    assert normalized_rgb.dtype == np.uint8
    assert normalized_rgb.flags.c_contiguous
    np.testing.assert_allclose(
        depth_m,
        [[0.0, 0.0, 0.1, 0.0], [0.25, 0.5, 0.75, 1.0]],
        atol=1e-7,
    )
    assert depth_m.dtype == np.float32
    assert K.dtype == np.float64


def _frame(**overrides) -> RgbdFrame:
    values = {
        "sequence": 4,
        "monotonic_ns": time.monotonic_ns(),
        "wall_time_ns": time.time_ns(),
        "device_timestamp_ms": 1234.5,
        "serial": "353322271910",
        "rgb": np.ones((2, 4, 3), dtype=np.uint8),
        "depth_m": np.full((2, 4), 0.2, dtype=np.float32),
        "intrinsics": np.array(
            [[100.0, 0.0, 1.5], [0.0, 101.0, 0.5], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        "distortion_model": "inverse_brown_conrady",
        "distortion_coefficients": np.array(
            [-0.05, 0.06, 0.0006, 0.0003, -0.02], dtype=np.float64
        ),
    }
    values.update(overrides)
    return RgbdFrame(**values)


@pytest.mark.parametrize(
    ("overrides", "match"),
    [
        ({"serial": "353322271204"}, "serial"),
        ({"rgb": np.ones((2, 4, 3), dtype=np.float32)}, "RGB"),
        ({"depth_m": np.ones((2, 3), dtype=np.float32)}, "aligned"),
        ({"depth_m": np.full((2, 4), np.nan, dtype=np.float32)}, "finite"),
        ({"depth_m": np.zeros((2, 4), dtype=np.float32)}, "valid depth"),
        ({"distortion_model": "brown_conrady"}, "distortion model"),
        ({"distortion_coefficients": np.zeros(4)}, "distortion coefficients"),
    ],
)
def test_rgbd_frame_validation_fails_closed(overrides, match) -> None:
    config = _config().camera.model_copy(update={"width": 4, "height": 2})
    with pytest.raises(CameraFrameError, match=match):
        validate_rgbd_frame(
            _frame(**overrides),
            config,
            expected_serial="353322271910",
            now_monotonic_ns=time.monotonic_ns(),
        )


def test_rgbd_frame_validation_rejects_stale_image() -> None:
    config = _config().camera.model_copy(update={"width": 4, "height": 2})
    now = time.monotonic_ns()
    with pytest.raises(CameraFrameError, match="stale"):
        validate_rgbd_frame(
            _frame(monotonic_ns=now - int(1e9)),
            config,
            expected_serial="353322271910",
            now_monotonic_ns=now,
        )


def test_realsense_adapter_aligns_depth_to_rgb_and_reports_device_metadata() -> None:
    """Reading separate cached color/depth frames would not prove pixel synchronization."""
    config = _config()
    height, width = config.camera.height, config.camera.width
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[0, 0] = [255, 0, 0]
    raw_depth = np.full((height, width), 250, dtype=np.uint16)

    class VideoProfile:
        def get_intrinsics(self):
            return type(
                "Intrinsics",
                (),
                {
                    "fx": 500.0,
                    "fy": 501.0,
                    "ppx": 319.5,
                    "ppy": 179.5,
                    "model": "inverse_brown_conrady",
                    "coeffs": [-0.05, 0.06, 0.0006, 0.0003, -0.02],
                },
            )()

        def as_video_stream_profile(self):
            return self

    class Frame:
        def __init__(self, data, frame_number, timestamp_ms):
            self._data = data
            self._frame_number = frame_number
            self._timestamp_ms = timestamp_ms
            self.profile = VideoProfile()

        def __bool__(self):
            return True

        def get_data(self):
            return self._data

        def get_frame_number(self):
            return self._frame_number

        def get_timestamp(self):
            return self._timestamp_ms

        def get_frame_timestamp_domain(self):
            return "global_time"

    class Frames:
        def get_color_frame(self):
            return Frame(rgb, 7, 345.25)

        def get_depth_frame(self):
            return Frame(raw_depth, 21, 345.0)

    class Device:
        def get_info(self, _key):
            return config.hardware.camera_serial

        def first_depth_sensor(self):
            return type("DepthSensor", (), {"get_depth_scale": lambda self: 0.001})()

    class Pipeline:
        def __init__(self):
            self.stopped = False

        def start(self, _config):
            return type("Profile", (), {"get_device": lambda self: Device()})()

        def wait_for_frames(self, *, timeout_ms):
            assert timeout_ms == 1000
            return Frames()

        def stop(self):
            self.stopped = True

    pipeline = Pipeline()

    class RsConfig:
        def enable_device(self, serial):
            assert serial == config.hardware.camera_serial

        def enable_stream(self, *args):
            assert args[1:3] == (width, height)

    class Aligner:
        def __init__(self):
            self.processed = False

        def process(self, frames):
            self.processed = True
            return frames

    aligner = Aligner()
    fake_rs = type(
        "FakeRs",
        (),
        {
            "pipeline": lambda: pipeline,
            "config": RsConfig,
            "align": lambda _stream: aligner,
            "stream": type("Stream", (), {"color": "color", "depth": "depth"}),
            "format": type("Format", (), {"rgb8": "rgb8", "z16": "z16"}),
            "camera_info": type("CameraInfo", (), {"serial_number": "serial"}),
        },
    )
    ticks = iter((1_000_000_000, 1_001_000_000))
    camera = RealSenseRgbdCamera(
        config,
        rs_module=fake_rs,
        monotonic_ns=lambda: next(ticks),
        wall_time_ns=lambda: 2_000_000_000,
    )
    try:
        frame = camera.read()
    finally:
        camera.close()

    assert aligner.processed is True
    assert frame.sequence == 7
    assert frame.serial == "353322271910"
    assert frame.device_timestamp_ms == 345.25
    np.testing.assert_array_equal(frame.rgb[0, 0], [255, 0, 0])
    assert frame.depth_m[0, 0] == pytest.approx(0.25)
    assert frame.distortion_model == "inverse_brown_conrady"
    assert frame.distortion_coefficients.shape == (5,)
    assert pipeline.stopped is True


def test_fixed_top_calibration_loads_the_accepted_serial_bound_artifact() -> None:
    config = _config()

    calibration = load_fixed_rgb_calibration(config.top_camera.calibration_path)

    assert calibration.name == "top_brio"
    assert calibration.serial == "B8C7F203"
    assert calibration.device.name == (
        "usb-046d_Logitech_BRIO_B8C7F203-video-index0"
    )
    assert (calibration.width, calibration.height, calibration.fps) == (1920, 1080, 30)
    assert calibration.camera_matrix.dtype == np.float64
    assert calibration.distortion_coefficients.shape == (5,)
    assert calibration.camera_to_world.dtype == np.float64


def test_fixed_top_rgb_validation_rejects_wrong_serial_or_stale_frame() -> None:
    calibration = load_fixed_rgb_calibration(_config().top_camera.calibration_path)
    now = time.monotonic_ns()
    frame = RgbFrame(
        sequence=1,
        monotonic_ns=now,
        wall_time_ns=time.time_ns(),
        serial="wrong",
        rgb=np.zeros((360, 640, 3), dtype=np.uint8),
        intrinsics=calibration.camera_matrix,
        camera_to_world=calibration.camera_to_world,
        distortion_model="none",
        distortion_coefficients=np.zeros(5, dtype=np.float64),
    )

    with pytest.raises(CameraFrameError, match="serial"):
        validate_rgb_frame(
            frame,
            calibration,
            stale_after_s=0.25,
            now_monotonic_ns=now,
        )

    with pytest.raises(CameraFrameError, match="stale"):
        validate_rgb_frame(
            RgbFrame(**{**frame.__dict__, "serial": calibration.serial, "monotonic_ns": now - int(1e9)}),
            calibration,
            stale_after_s=0.25,
            now_monotonic_ns=now,
        )


def test_opencv_top_adapter_enforces_profile_and_outputs_undistorted_rgb() -> None:
    config = _config()
    calibration = load_fixed_rgb_calibration(config.top_camera.calibration_path)
    # The shipped top calibration is 1920x1080@30; the adapter fails closed on any
    # other V4L2 profile, so the fake capture has to serve the calibrated one.
    raw_bgr = np.zeros((1080, 1920, 3), dtype=np.uint8)
    raw_bgr[..., 0] = 10
    raw_bgr[..., 1] = 20
    raw_bgr[..., 2] = 30

    class Capture:
        def __init__(self) -> None:
            self.properties = {}
            self.released = False

        def isOpened(self):
            return True

        def set(self, key, value):
            self.properties[key] = value
            return True

        def get(self, key):
            if key == cv2.CAP_PROP_FRAME_WIDTH:
                return 1920.0
            if key == cv2.CAP_PROP_FRAME_HEIGHT:
                return 1080.0
            if key == cv2.CAP_PROP_FPS:
                return 30.0
            return self.properties.get(key, 0.0)

        def read(self):
            return True, raw_bgr.copy()

        def release(self):
            self.released = True

    capture = Capture()
    ticks = iter((1_000_000_000, 1_001_000_000))
    camera = OpenCvFixedRgbCamera(
        config,
        capture_factory=lambda *_: capture,
        warmup_frames=0,
        monotonic_ns=lambda: next(ticks),
        wall_time_ns=lambda: 2_000_000_000,
    )
    try:
        frame = camera.read()
    finally:
        camera.close()

    assert frame.serial == calibration.serial
    assert frame.sequence == 0
    assert frame.rgb.shape == (1080, 1920, 3)
    assert frame.rgb.dtype == np.uint8
    assert frame.rgb.flags.c_contiguous
    assert frame.intrinsics.dtype == np.float64
    np.testing.assert_array_equal(frame.camera_to_world, calibration.camera_to_world)
    assert capture.released is True
