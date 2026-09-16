import json
import time
from pathlib import Path

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from agp_yam_bridge.camera_acceptance import (
    CheckerboardRecord,
    _detect_checkerboard,
    _save_capture,
    calibrate_checkerboard_intrinsics,
    calibrate_checkerboard_records,
    choose_checkerboard_orientation,
    deproject_pixels,
    estimate_checkerboard_pose,
    evaluate_checkerboard_pose_consistency,
    evaluate_correspondences,
    evaluate_table_plane,
    main,
    solve_fixed_camera_pose,
    solve_hand_eye,
    undistort_realsense_pixels,
)


def test_fixed_camera_detector_resolves_small_real_brio_checkerboard() -> None:
    fixture = Path(__file__).with_name("fixtures") / "top_brio_small_checkerboard.png"
    bgr = cv2.imread(str(fixture))
    assert bgr is not None

    corners = _detect_checkerboard(
        cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB),
        columns=9,
        rows=7,
        label="small BRIO checkerboard",
    )

    assert corners.shape == (63, 2)
    np.testing.assert_allclose(corners.mean(axis=0), [509.3, 258.4], atol=1.0)


def test_fixed_camera_detector_preserves_native_scale_wrist_detection() -> None:
    fixture = Path(__file__).with_name("fixtures") / "wrist_d405_large_checkerboard.png"
    bgr = cv2.imread(str(fixture))
    assert bgr is not None

    corners = _detect_checkerboard(
        cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB),
        columns=9,
        rows=7,
        label="large D405 checkerboard",
    )

    assert corners.shape == (63, 2)
    np.testing.assert_allclose(corners.mean(axis=0), [168.8, 169.0], atol=1.0)


def test_capture_archive_preserves_joint_state_for_fk_diagnostics(tmp_path) -> None:
    observations = []
    for index in range(2):
        observations.append(
            {
                "robot_joint_pos_0": np.arange(7, dtype=np.float32) + index,
                "robot_joint_vel_0": np.arange(7, dtype=np.float32) + 10 + index,
                "robot_joint_effort_0": np.arange(7, dtype=np.float32) + 20 + index,
                "camera_0": {
                    "serial": "camera",
                    "images": {
                        "rgb": np.zeros((2, 3, 3), dtype=np.uint8),
                        "depth": np.ones((2, 3), dtype=np.float32),
                    },
                    "intrinsics": np.eye(3, dtype=np.float64),
                    "camera_to_world": np.eye(4, dtype=np.float64),
                    "frame_sequence": index,
                    "frame_monotonic_ns": 100 + index,
                    "joint_time_delta_ns": index,
                },
            }
        )

    path = tmp_path / "capture.npz"
    _save_capture(path, observations)

    with np.load(path, allow_pickle=False) as capture:
        np.testing.assert_array_equal(
            capture["robot_joint_pos_0"],
            np.stack([observation["robot_joint_pos_0"] for observation in observations]),
        )
        np.testing.assert_array_equal(
            capture["robot_joint_vel_0"],
            np.stack([observation["robot_joint_vel_0"] for observation in observations]),
        )
        np.testing.assert_array_equal(
            capture["robot_joint_effort_0"],
            np.stack(
                [observation["robot_joint_effort_0"] for observation in observations]
            ),
        )


def test_deprojection_and_camera_to_world_use_pinhole_meter_convention() -> None:
    K = np.array([[100.0, 0.0, 1.0], [0.0, 100.0, 1.0], [0.0, 0.0, 1.0]])
    pixels = np.array([[1.0, 1.0], [101.0, 1.0]])
    depth_m = np.array([0.5, 0.5])

    points = deproject_pixels(pixels, depth_m, K)

    np.testing.assert_allclose(points, [[0.0, 0.0, 0.5], [0.5, 0.0, 0.5]])


def test_correspondence_evaluation_reports_world_and_reprojection_error() -> None:
    K = np.array([[100.0, 0.0, 1.0], [0.0, 100.0, 1.0], [0.0, 0.0, 1.0]])
    depth = np.full((3, 103), 0.5, dtype=np.float32)
    camera_to_world = np.eye(4)
    camera_to_world[:3, 3] = [1.0, 2.0, 3.0]
    pixels = np.array([[1.0, 1.0], [101.0, 1.0]])
    expected_world = np.array([[1.0, 2.0, 3.5], [1.5, 2.0, 3.5]])

    metrics = evaluate_correspondences(
        pixels,
        expected_world,
        depth,
        K,
        camera_to_world,
    )

    assert metrics["point_count"] == 2
    assert metrics["rms_world_error_m"] == pytest.approx(0.0, abs=1e-12)
    assert metrics["max_world_error_m"] == pytest.approx(0.0, abs=1e-12)
    assert metrics["rms_reprojection_error_px"] == pytest.approx(0.0, abs=1e-12)
    assert metrics["max_reprojection_error_px"] == pytest.approx(0.0, abs=1e-12)


def test_table_plane_evaluation_uses_world_points_and_expected_height() -> None:
    height, width = 20, 30
    K = np.array([[100.0, 0.0, 14.5], [0.0, 100.0, 9.5], [0.0, 0.0, 1.0]])
    depth = np.full((height, width), 0.5, dtype=np.float32)
    camera_to_world = np.eye(4)
    camera_to_world[2, 3] = 0.25

    metrics = evaluate_table_plane(
        depth,
        K,
        camera_to_world,
        roi=(2, 3, 28, 18),
        expected_world_z_m=0.75,
    )

    assert metrics["point_count"] == 390
    assert metrics["plane_rms_m"] == pytest.approx(0.0, abs=1e-7)
    assert metrics["normal_error_deg"] == pytest.approx(0.0, abs=1e-7)
    assert metrics["world_z_error_m"] == pytest.approx(0.0, abs=1e-7)


def _transform(rotation_xyz_deg: tuple[float, float, float], translation: tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Rotation.from_euler("xyz", rotation_xyz_deg, degrees=True).as_matrix()
    transform[:3, 3] = translation
    return transform


def test_checkerboard_intrinsics_recover_the_camera_profile() -> None:
    """Swapping image axes or checkerboard point order would corrupt the saved pinhole model."""
    columns, rows = 9, 7
    image_size = (640, 360)
    square_size_m = 0.022
    expected_K = np.array(
        [[510.0, 0.0, 318.0], [0.0, 508.0, 181.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    object_points = np.zeros((columns * rows, 3), dtype=np.float64)
    object_points[:, :2] = (
        np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    )
    poses = [
        ((-15.0, -12.0, -8.0), (-0.08, -0.06, 0.42)),
        ((12.0, -10.0, 7.0), (0.01, -0.08, 0.48)),
        ((-10.0, 14.0, 10.0), (-0.04, 0.00, 0.50)),
        ((16.0, 11.0, -12.0), (0.03, -0.02, 0.55)),
        ((-18.0, 7.0, 15.0), (-0.06, -0.04, 0.58)),
        ((8.0, -17.0, -16.0), (0.02, -0.07, 0.52)),
        ((-7.0, 18.0, 4.0), (-0.03, -0.01, 0.46)),
        ((19.0, 5.0, 18.0), (0.00, -0.05, 0.60)),
        ((-13.0, -16.0, 12.0), (-0.07, -0.02, 0.54)),
        ((14.0, 15.0, -5.0), (0.04, -0.06, 0.57)),
        ((-5.0, 9.0, -19.0), (-0.01, -0.09, 0.49)),
        ((10.0, -6.0, 20.0), (-0.05, 0.01, 0.56)),
    ]
    pixel_views = []
    for rotation_deg, translation in poses:
        camera_from_target = _transform(rotation_deg, translation)
        rotation_vector = Rotation.from_matrix(camera_from_target[:3, :3]).as_rotvec()
        pixels, _ = cv2.projectPoints(
            object_points,
            rotation_vector,
            camera_from_target[:3, 3],
            expected_K,
            np.zeros(5),
        )
        pixel_views.append(pixels.reshape(-1, 2))

    result = calibrate_checkerboard_intrinsics(
        pixel_views,
        image_size=image_size,
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
    )

    assert result.view_count == len(pixel_views)
    assert result.image_size == image_size
    assert result.rms_reprojection_error_px < 1e-3
    np.testing.assert_allclose(result.camera_matrix, expected_K, atol=0.1)
    np.testing.assert_allclose(result.distortion_coefficients, np.zeros(5), atol=1e-3)


def test_fixed_top_camera_pose_composes_wrist_board_and_top_board_transforms() -> None:
    """Inverting either board transform would install a plausible but wrong top/world pose."""
    expected_world_from_top = _transform((179.0, 2.0, -91.0), (-0.166, 0.305, 0.954))
    world_from_target = _transform((0.0, 0.0, 13.0), (0.19, -0.23, 0.04))
    wrist_from_targets = [
        _transform((4.0, -8.0, 3.0), (-0.04, -0.03, 0.30)),
        _transform((-9.0, 5.0, 17.0), (0.02, -0.04, 0.34)),
        _transform((12.0, 7.0, -11.0), (-0.01, 0.00, 0.28)),
        _transform((-6.0, -13.0, 22.0), (0.03, -0.02, 0.32)),
    ]
    world_from_wrist = [
        world_from_target @ np.linalg.inv(wrist_from_target)
        for wrist_from_target in wrist_from_targets
    ]
    wrist_from_target = [
        np.linalg.inv(world_from_wrist_pose) @ world_from_target
        for world_from_wrist_pose in world_from_wrist
    ]
    top_from_target = [
        np.linalg.inv(expected_world_from_top) @ world_from_target
        for _ in world_from_wrist
    ]

    result = solve_fixed_camera_pose(
        world_from_reference_cameras=world_from_wrist,
        reference_camera_from_targets=wrist_from_target,
        fixed_camera_from_targets=top_from_target,
    )

    assert result.sample_count == 4
    assert result.translation_rms_m == pytest.approx(0.0, abs=1e-9)
    assert result.rotation_rms_deg == pytest.approx(0.0, abs=1e-7)
    np.testing.assert_allclose(result.world_from_camera, expected_world_from_top, atol=1e-9)


def test_top_intrinsics_cli_binds_the_pass_report_to_serial_and_stream_profile(
    tmp_path, monkeypatch
) -> None:
    """A numerically valid calibration must not become an anonymous, profile-free artifact."""
    columns, rows = 3, 3
    square_size_m = 0.022
    image_size = (640, 360)
    K = np.array(
        [[510.0, 0.0, 318.0], [0.0, 508.0, 181.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    object_points = np.zeros((columns * rows, 3), dtype=np.float64)
    object_points[:, :2] = (
        np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    )
    poses = [
        ((-15.0, -12.0, -8.0), (-0.03, -0.03, 0.42)),
        ((12.0, -10.0, 7.0), (0.01, -0.04, 0.48)),
        ((-10.0, 14.0, 10.0), (-0.02, 0.00, 0.50)),
        ((16.0, 11.0, -12.0), (0.03, -0.02, 0.55)),
        ((-18.0, 7.0, 15.0), (-0.04, -0.01, 0.58)),
        ((8.0, -17.0, -16.0), (0.02, -0.05, 0.52)),
        ((-7.0, 18.0, 4.0), (-0.01, -0.01, 0.46)),
        ((19.0, 5.0, 18.0), (0.00, -0.03, 0.60)),
        ((-13.0, -16.0, 12.0), (-0.03, -0.02, 0.54)),
        ((14.0, 15.0, -5.0), (0.02, -0.04, 0.57)),
    ]
    detections = []
    for rotation_deg, translation in poses:
        camera_from_target = _transform(rotation_deg, translation)
        pixels, _ = cv2.projectPoints(
            object_points,
            Rotation.from_matrix(camera_from_target[:3, :3]).as_rotvec(),
            camera_from_target[:3, 3],
            K,
            np.zeros(5),
        )
        detections.append(pixels)
    detection_iterator = iter(detections)
    monkeypatch.setattr(
        "agp_yam_bridge.camera_acceptance.cv2.imread",
        lambda _path: np.zeros((image_size[1], image_size[0], 3), dtype=np.uint8),
    )
    monkeypatch.setattr(
        "agp_yam_bridge.camera_acceptance.cv2.findChessboardCornersSB",
        lambda *_args, **_kwargs: (True, next(detection_iterator)),
    )
    calibration_images = [tmp_path / f"calibration_{index}.png" for index in range(8)]
    validation_images = [tmp_path / f"validation_{index}.png" for index in range(2)]
    output = tmp_path / "top_intrinsics.json"

    exit_code = main(
        [
            "calibrate-top-intrinsics",
            "--images",
            *map(str, calibration_images),
            "--validation-images",
            *map(str, validation_images),
            "--device",
            "/dev/v4l/by-id/usb-046d_Logitech_BRIO_B8C7F203-video-index0",
            "--serial",
            "B8C7F203",
            "--width",
            "640",
            "--height",
            "360",
            "--fps",
            "30",
            "--columns",
            str(columns),
            "--rows",
            str(rows),
            "--square-size-m",
            str(square_size_m),
            "--output",
            str(output),
        ]
    )

    report = json.loads(output.read_text(encoding="utf-8"))
    assert exit_code == 0
    assert report["status"] == "PASS"
    assert report["camera"] == {
        "name": "top_brio",
        "model": "Logitech BRIO",
        "serial": "B8C7F203",
        "device": "/dev/v4l/by-id/usb-046d_Logitech_BRIO_B8C7F203-video-index0",
        "width": 640,
        "height": 360,
        "fps": 30,
    }
    assert report["calibration_view_count"] == 8
    assert report["validation_view_count"] == 2
    np.testing.assert_allclose(report["camera_matrix"], K, atol=0.1)


def test_top_extrinsics_cli_uses_paired_wrist_world_poses_and_held_out_pair(
    tmp_path, monkeypatch
) -> None:
    """The installed top/world transform must come from paired geometry and pass a fresh pair."""
    columns, rows = 3, 3
    square_size_m = 0.022
    K_top = np.array(
        [[510.0, 0.0, 318.0], [0.0, 508.0, 181.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    K_wrist = np.array(
        [[430.0, 0.0, 321.0], [0.0, 431.0, 179.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    expected_world_from_top = _transform(
        (179.0, 2.0, -91.0), (-0.166, 0.305, 0.954)
    )
    world_from_target = _transform((0.0, 0.0, 13.0), (0.19, -0.23, 0.04))
    wrist_from_targets = [
        _transform((4.0, -8.0, 3.0), (-0.04, -0.03, 0.30)),
        _transform((-9.0, 5.0, 17.0), (0.02, -0.04, 0.34)),
        _transform((12.0, 7.0, -11.0), (-0.01, 0.00, 0.28)),
        _transform((-6.0, -13.0, 22.0), (0.03, -0.02, 0.32)),
    ]
    world_from_wrist = [
        world_from_target @ np.linalg.inv(wrist_from_target)
        for wrist_from_target in wrist_from_targets
    ]
    object_points = np.zeros((columns * rows, 3), dtype=np.float64)
    object_points[:, :2] = (
        np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    )
    detection_sequence = []
    pair_paths = []
    for index, world_from_wrist_pose in enumerate(world_from_wrist):
        top_from_target = np.linalg.inv(expected_world_from_top) @ world_from_target
        wrist_from_target = wrist_from_targets[index]
        top_pixels, _ = cv2.projectPoints(
            object_points,
            Rotation.from_matrix(top_from_target[:3, :3]).as_rotvec(),
            top_from_target[:3, 3],
            K_top,
            np.zeros(5),
        )
        wrist_pixels, _ = cv2.projectPoints(
            object_points,
            Rotation.from_matrix(wrist_from_target[:3, :3]).as_rotvec(),
            wrist_from_target[:3, 3],
            K_wrist,
            np.zeros(5),
        )
        detection_sequence.extend((top_pixels, wrist_pixels))
        pair_path = tmp_path / f"pair_{index}.npz"
        np.savez_compressed(
            pair_path,
            top_rgb=np.zeros((360, 640, 3), dtype=np.uint8),
            wrist_rgb=np.zeros((360, 640, 3), dtype=np.uint8),
            wrist_intrinsics=K_wrist,
            wrist_camera_to_world=world_from_wrist_pose,
            wrist_serial=np.array("353322271910"),
        )
        pair_paths.append(pair_path)
    detection_iterator = iter(detection_sequence)
    monkeypatch.setattr(
        "agp_yam_bridge.camera_acceptance.cv2.findChessboardCornersSB",
        lambda *_args, **_kwargs: (True, next(detection_iterator)),
    )
    intrinsics_path = tmp_path / "top_intrinsics.json"
    intrinsics_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "type": "fixed_rgb_camera_intrinsics",
                "status": "PASS",
                "camera": {
                    "name": "top_brio",
                    "model": "Logitech BRIO",
                    "serial": "B8C7F203",
                    "device": "/dev/v4l/by-id/usb-046d_Logitech_BRIO_B8C7F203-video-index0",
                    "width": 640,
                    "height": 360,
                    "fps": 30,
                },
                "checkerboard": {
                    "columns": columns,
                    "rows": rows,
                    "square_size_m": square_size_m,
                },
                "distortion_model": "opencv_radtan",
                "camera_matrix": K_top.tolist(),
                "distortion_coefficients": [0.0] * 5,
            }
        ),
        encoding="utf-8",
    )
    output = tmp_path / "top_camera_calibration.json"

    exit_code = main(
        [
            "calibrate-top-extrinsics",
            "--captures",
            *map(str, pair_paths[:3]),
            "--validation-captures",
            str(pair_paths[3]),
            "--intrinsics",
            str(intrinsics_path),
            "--wrist-distortion-model",
            "none",
            "--wrist-distortion-coefficients",
            "0",
            "0",
            "0",
            "0",
            "0",
            "--output",
            str(output),
        ]
    )

    report = json.loads(output.read_text(encoding="utf-8"))
    assert exit_code == 0
    assert report["status"] == "PASS"
    assert report["type"] == "fixed_rgb_camera_calibration"
    assert report["camera"]["serial"] == "B8C7F203"
    assert report["reference_camera"]["serial"] == "353322271910"
    assert report["calibration_sample_count"] == 3
    assert report["validation_sample_count"] == 1
    np.testing.assert_allclose(
        report["camera_to_world"], expected_world_from_top, atol=1e-7
    )


def test_capture_top_pair_saves_rgb_and_the_calibrated_wrist_world_pose(
    tmp_path, monkeypatch
) -> None:
    """Dropping the wrist pose or using BGR would make an extrinsic pair unusable."""
    device = "/dev/v4l/by-id/usb-046d_Logitech_BRIO_B8C7F203-video-index0"
    top_intrinsics = tmp_path / "top_intrinsics.json"
    top_intrinsics.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "type": "fixed_rgb_camera_intrinsics",
                "status": "PASS",
                "camera": {
                    "name": "top_brio",
                    "model": "Logitech BRIO",
                    "serial": "B8C7F203",
                    "device": device,
                    "width": 640,
                    "height": 360,
                    "fps": 30,
                },
                "checkerboard": {"columns": 9, "rows": 7, "square_size_m": 0.022},
                "distortion_model": "opencv_radtan",
                "camera_matrix": [[510.0, 0.0, 318.0], [0.0, 508.0, 181.0], [0.0, 0.0, 1.0]],
                "distortion_coefficients": [0.0] * 5,
            }
        ),
        encoding="utf-8",
    )
    top_bgr = np.zeros((360, 640, 3), dtype=np.uint8)
    top_bgr[..., 0] = 10
    top_bgr[..., 1] = 20
    top_bgr[..., 2] = 30

    class FakeCapture:
        def isOpened(self):
            return True

        def set(self, _property, _value):
            return True

        def get(self, property_id):
            values = {
                cv2.CAP_PROP_FRAME_WIDTH: 640.0,
                cv2.CAP_PROP_FRAME_HEIGHT: 360.0,
                cv2.CAP_PROP_FPS: 30.0,
            }
            return values.get(property_id, 0.0)

        def read(self):
            return True, top_bgr.copy()

        def release(self):
            return None

    wrist_pose = _transform((5.0, -4.0, 3.0), (0.2, -0.2, 0.3))
    wrist_rgb = np.full((360, 640, 3), 40, dtype=np.uint8)
    wrist_K = np.array(
        [[430.0, 0.0, 321.0], [0.0, 431.0, 179.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )

    class FakeClient:
        def __init__(self, *_args, **_kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return None

        def get_observation(self):
            now = time.monotonic_ns()
            return {
                "camera_0": {
                    "name": "wrist_d405",
                    "serial": "353322271910",
                    "frame_monotonic_ns": now,
                    "images": {"rgb": wrist_rgb},
                    "intrinsics": wrist_K,
                    "camera_to_world": wrist_pose,
                }
            }

    monkeypatch.setattr(
        "agp_yam_bridge.camera_acceptance.cv2.VideoCapture",
        lambda *_args: FakeCapture(),
    )
    monkeypatch.setattr(
        "agp_yam_bridge.camera_acceptance.BridgeClient", FakeClient
    )
    output = tmp_path / "pair.npz"

    exit_code = main(
        [
            "capture-top-pair",
            "--intrinsics",
            str(top_intrinsics),
            "--output",
            str(output),
            "--warmup-frames",
            "1",
        ]
    )

    assert exit_code == 0
    with np.load(output, allow_pickle=False) as capture:
        np.testing.assert_array_equal(capture["top_rgb"][0, 0], [30, 20, 10])
        np.testing.assert_array_equal(capture["wrist_rgb"], wrist_rgb)
        np.testing.assert_array_equal(capture["wrist_intrinsics"], wrist_K)
        np.testing.assert_array_equal(capture["wrist_camera_to_world"], wrist_pose)
        assert str(capture["top_serial"]) == "B8C7F203"
        assert str(capture["wrist_serial"]) == "353322271910"


def test_checkerboard_pose_uses_metric_square_spacing_and_pinhole_intrinsics() -> None:
    columns, rows = 3, 3
    square_size_m = 0.02
    K = np.array([[200.0, 0.0, 100.0], [0.0, 210.0, 80.0], [0.0, 0.0, 1.0]])
    expected_camera_from_target = _transform((0.0, 0.0, 0.0), (0.01, -0.02, 0.5))
    object_points = np.zeros((columns * rows, 3), dtype=np.float64)
    object_points[:, :2] = (
        np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    )
    camera_points = object_points + expected_camera_from_target[:3, 3]
    pixels = np.column_stack(
        (
            K[0, 0] * camera_points[:, 0] / camera_points[:, 2] + K[0, 2],
            K[1, 1] * camera_points[:, 1] / camera_points[:, 2] + K[1, 2],
        )
    )

    observed = estimate_checkerboard_pose(
        pixels,
        K,
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
        distortion_coefficients=np.zeros(5),
    )

    np.testing.assert_allclose(observed, expected_camera_from_target, atol=1e-8)


def test_hand_eye_solver_recovers_gripper_from_camera_and_fixed_target() -> None:
    expected_gripper_from_camera = _transform((8.0, -12.0, 17.0), (0.03, -0.02, 0.08))
    world_from_target = _transform((0.0, 0.0, 11.0), (0.25, -0.31, 0.04))
    world_from_gripper = [
        _transform((0.0, 0.0, 0.0), (0.10, -0.20, 0.30)),
        _transform((20.0, 0.0, 0.0), (0.13, -0.24, 0.28)),
        _transform((-18.0, 7.0, 0.0), (0.08, -0.27, 0.32)),
        _transform((0.0, 24.0, 5.0), (0.16, -0.22, 0.34)),
        _transform((5.0, -21.0, -9.0), (0.06, -0.19, 0.26)),
        _transform((0.0, 0.0, 35.0), (0.14, -0.29, 0.31)),
        _transform((14.0, 16.0, -22.0), (0.09, -0.25, 0.36)),
        _transform((-11.0, -13.0, 28.0), (0.12, -0.18, 0.29)),
    ]
    camera_from_target = [
        np.linalg.inv(world_from_gripper_pose @ expected_gripper_from_camera)
        @ world_from_target
        for world_from_gripper_pose in world_from_gripper
    ]

    result = solve_hand_eye(world_from_gripper, camera_from_target)

    np.testing.assert_allclose(
        result.gripper_from_camera,
        expected_gripper_from_camera,
        atol=1e-8,
    )
    assert result.pose_count == 8
    assert result.target_translation_rms_m == pytest.approx(0.0, abs=1e-9)
    assert result.target_rotation_rms_deg == pytest.approx(0.0, abs=1e-7)


def test_checkerboard_orientation_reverses_ambiguous_detector_order() -> None:
    columns, rows = 3, 3
    square_size_m = 0.02
    K = np.array([[200.0, 0.0, 100.0], [0.0, 210.0, 80.0], [0.0, 0.0, 1.0]])
    world_from_camera = _transform((0.0, 0.0, 25.0), (0.1, -0.2, 0.3))
    world_from_target = _transform((0.0, 0.0, 5.0), (0.16, -0.22, 0.8))
    camera_from_target = np.linalg.inv(world_from_camera) @ world_from_target
    object_points = np.zeros((columns * rows, 3), dtype=np.float64)
    object_points[:, :2] = (
        np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    )
    camera_points = (
        object_points @ camera_from_target[:3, :3].T
        + camera_from_target[:3, 3]
    )
    correctly_ordered_pixels = np.column_stack(
        (
            K[0, 0] * camera_points[:, 0] / camera_points[:, 2] + K[0, 2],
            K[1, 1] * camera_points[:, 1] / camera_points[:, 2] + K[1, 2],
        )
    )

    result = choose_checkerboard_orientation(
        correctly_ordered_pixels[::-1],
        K,
        world_from_camera=world_from_camera,
        reference_world_from_target=world_from_target,
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
        distortion_coefficients=np.zeros(5),
    )

    assert result.reversed is True
    np.testing.assert_allclose(result.camera_from_target, camera_from_target, atol=1e-8)
    np.testing.assert_allclose(result.world_from_target, world_from_target, atol=1e-8)


def test_checkerboard_pose_consistency_reports_held_out_world_error() -> None:
    """A wrong camera/world composition must fail the held-out target check."""
    columns, rows = 3, 3
    square_size_m = 0.02
    K = np.array([[200.0, 0.0, 100.0], [0.0, 210.0, 80.0], [0.0, 0.0, 1.0]])
    camera_from_target = _transform((4.0, -3.0, 2.0), (0.01, -0.02, 0.5))
    reference_world_from_target = _transform((7.0, 1.0, -5.0), (0.25, -0.1, 0.7))
    expected_delta = _transform((0.0, 0.0, 1.5), (0.003, 0.004, 0.0))
    world_from_camera = (
        reference_world_from_target
        @ expected_delta
        @ np.linalg.inv(camera_from_target)
    )
    object_points = np.zeros((columns * rows, 3), dtype=np.float64)
    object_points[:, :2] = (
        np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    )
    camera_points = (
        object_points @ camera_from_target[:3, :3].T
        + camera_from_target[:3, 3]
    )
    pixels = np.column_stack(
        (
            K[0, 0] * camera_points[:, 0] / camera_points[:, 2] + K[0, 2],
            K[1, 1] * camera_points[:, 1] / camera_points[:, 2] + K[1, 2],
        )
    )

    metrics = evaluate_checkerboard_pose_consistency(
        pixels,
        K,
        world_from_camera=world_from_camera,
        reference_world_from_target=reference_world_from_target,
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
        distortion_coefficients=np.zeros(5),
    )

    assert metrics["translation_error_m"] == pytest.approx(0.005, abs=1e-8)
    assert metrics["rotation_error_deg"] == pytest.approx(1.5, abs=1e-7)
    assert metrics["reversed"] is False
    np.testing.assert_allclose(
        metrics["observed_world_from_target"],
        reference_world_from_target @ expected_delta,
        atol=1e-8,
    )


def test_held_out_checkerboard_cli_saves_pass_report(tmp_path, monkeypatch) -> None:
    columns, rows = 3, 3
    square_size_m = 0.02
    K = np.array([[200.0, 0.0, 100.0], [0.0, 210.0, 80.0], [0.0, 0.0, 1.0]])
    camera_from_target = _transform((4.0, -3.0, 2.0), (0.01, -0.02, 0.5))
    reference_world_from_target = _transform((7.0, 1.0, -5.0), (0.25, -0.1, 0.7))
    expected_delta = _transform((0.0, 0.0, 1.5), (0.003, 0.004, 0.0))
    world_from_camera = (
        reference_world_from_target
        @ expected_delta
        @ np.linalg.inv(camera_from_target)
    )
    object_points = np.zeros((columns * rows, 3), dtype=np.float64)
    object_points[:, :2] = (
        np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    )
    camera_points = (
        object_points @ camera_from_target[:3, :3].T
        + camera_from_target[:3, 3]
    )
    pixels = np.column_stack(
        (
            K[0, 0] * camera_points[:, 0] / camera_points[:, 2] + K[0, 2],
            K[1, 1] * camera_points[:, 1] / camera_points[:, 2] + K[1, 2],
        )
    )
    monkeypatch.setattr(
        "agp_yam_bridge.camera_acceptance.cv2.findChessboardCornersSB",
        lambda *_args, **_kwargs: (True, pixels.reshape(-1, 1, 2)),
    )
    capture_path = tmp_path / "held_out.npz"
    np.savez_compressed(
        capture_path,
        rgb=np.zeros((1, 180, 240, 3), dtype=np.uint8),
        intrinsics=K[None, ...],
        camera_to_world=world_from_camera[None, ...],
        serial=np.array("camera"),
    )
    reference_path = tmp_path / "reference.json"
    reference_path.write_text(
        json.dumps(
            {
                "status": "PASS",
                "captures": [{"serial": "camera"}],
                "checkerboard": {
                    "columns": columns,
                    "rows": rows,
                    "square_size_m": square_size_m,
                },
                "distortion": {"model": "none", "coefficients": [0.0] * 5},
                "solved_world_from_target": reference_world_from_target.tolist(),
            }
        ),
        encoding="utf-8",
    )
    output_path = tmp_path / "held_out_report.json"

    exit_code = main(
        [
            "evaluate-checkerboard-pose",
            "--capture",
            str(capture_path),
            "--reference-report",
            str(reference_path),
            "--output",
            str(output_path),
        ]
    )

    report = json.loads(output_path.read_text(encoding="utf-8"))
    assert exit_code == 0
    assert report["status"] == "PASS"
    assert report["capture"]["frame_index"] == 0
    assert report["metrics"]["translation_error_m"] == pytest.approx(0.005, abs=1e-7)
    assert report["metrics"]["rotation_error_deg"] == pytest.approx(1.5, abs=1e-5)


def test_checkerboard_records_recover_hand_eye_with_mixed_corner_directions() -> None:
    columns, rows = 3, 3
    square_size_m = 0.02
    K = np.array([[200.0, 0.0, 100.0], [0.0, 210.0, 80.0], [0.0, 0.0, 1.0]])
    expected_gripper_from_camera = _transform((8.0, -12.0, 17.0), (0.03, -0.02, 0.08))
    world_from_target = _transform((0.0, 0.0, 11.0), (0.25, -0.31, 0.74))
    world_from_gripper = [
        _transform((0.0, 0.0, 0.0), (0.10, -0.20, 0.30)),
        _transform((20.0, 0.0, 0.0), (0.13, -0.24, 0.28)),
        _transform((-18.0, 7.0, 0.0), (0.08, -0.27, 0.32)),
        _transform((0.0, 24.0, 5.0), (0.16, -0.22, 0.34)),
        _transform((5.0, -21.0, -9.0), (0.06, -0.19, 0.26)),
        _transform((0.0, 0.0, 35.0), (0.14, -0.29, 0.31)),
        _transform((14.0, 16.0, -22.0), (0.09, -0.25, 0.36)),
        _transform((-11.0, -13.0, 28.0), (0.12, -0.18, 0.29)),
    ]
    object_points = np.zeros((columns * rows, 3), dtype=np.float64)
    object_points[:, :2] = (
        np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    )
    records = []
    for index, world_from_gripper_pose in enumerate(world_from_gripper):
        camera_from_target = (
            np.linalg.inv(world_from_gripper_pose @ expected_gripper_from_camera)
            @ world_from_target
        )
        camera_points = (
            object_points @ camera_from_target[:3, :3].T
            + camera_from_target[:3, 3]
        )
        pixels = np.column_stack(
            (
                K[0, 0] * camera_points[:, 0] / camera_points[:, 2] + K[0, 2],
                K[1, 1] * camera_points[:, 1] / camera_points[:, 2] + K[1, 2],
            )
        )
        records.append(
            CheckerboardRecord(
                label=f"pose{index:02d}",
                pixels_uv=pixels[::-1] if index % 2 else pixels,
                intrinsics=K,
                nominal_world_from_camera=(
                    world_from_gripper_pose @ expected_gripper_from_camera
                ),
            )
        )

    result = calibrate_checkerboard_records(
        records,
        nominal_gripper_from_camera=expected_gripper_from_camera,
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
        distortion_coefficients=np.zeros(5),
    )

    np.testing.assert_allclose(
        result.hand_eye.gripper_from_camera,
        expected_gripper_from_camera,
        atol=1e-8,
    )
    assert result.reversed_pose_count == 4
    assert result.max_reprojection_error_px == pytest.approx(0.0, abs=1e-7)


def test_realsense_no_distortion_preserves_pinhole_pixels() -> None:
    K = np.array(
        [[325.0, 0.0, 321.0], [0.0, 324.0, 177.0], [0.0, 0.0, 1.0]]
    )
    pixels = np.array([[10.25, 20.5], [321.0, 177.0], [600.75, 330.25]])

    undistorted = undistort_realsense_pixels(
        pixels,
        K,
        width=640,
        height=360,
        distortion_model="none",
        distortion_coefficients=np.zeros(5),
    )

    np.testing.assert_allclose(undistorted, pixels, rtol=0.0, atol=1e-5)
