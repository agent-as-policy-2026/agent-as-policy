"""Offline camera acceptance metrics and capture CLI for P3."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from agp_yam_bridge.client import BridgeClient
from agp_yam_bridge.config import load_config
from agp_yam_bridge.preflight import DEFAULT_CONFIG


class CameraAcceptanceError(RuntimeError):
    pass


@dataclass(frozen=True)
class HandEyeResult:
    gripper_from_camera: np.ndarray
    world_from_target: np.ndarray
    pose_count: int
    target_translation_rms_m: float
    target_translation_max_m: float
    target_rotation_rms_deg: float
    target_rotation_max_deg: float


@dataclass(frozen=True)
class OrientedCheckerboardPose:
    camera_from_target: np.ndarray
    world_from_target: np.ndarray
    ordered_pixels: np.ndarray
    reversed: bool
    consistency_score: float


@dataclass(frozen=True)
class CheckerboardRecord:
    label: str
    pixels_uv: np.ndarray
    intrinsics: np.ndarray
    nominal_world_from_camera: np.ndarray


@dataclass(frozen=True)
class CheckerboardCalibrationResult:
    hand_eye: HandEyeResult
    reversed_pose_count: int
    rms_reprojection_error_px: float
    max_reprojection_error_px: float


@dataclass(frozen=True)
class CheckerboardIntrinsicsResult:
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    image_size: tuple[int, int]
    view_count: int
    rms_reprojection_error_px: float
    per_view_errors_px: np.ndarray


@dataclass(frozen=True)
class FixedCameraPoseResult:
    world_from_camera: np.ndarray
    sample_count: int
    translation_rms_m: float
    translation_max_m: float
    rotation_rms_deg: float
    rotation_max_deg: float


def _checkerboard_object_points(
    *, columns: int, rows: int, square_size_m: float
) -> np.ndarray:
    if columns < 2 or rows < 2:
        raise CameraAcceptanceError("checkerboard must have at least 2x2 internal corners")
    if not np.isfinite(square_size_m) or square_size_m <= 0:
        raise CameraAcceptanceError("checkerboard square size must be positive")
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    points[:, :2] = (
        np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    )
    return points


def calibrate_checkerboard_intrinsics(
    pixel_views: list[np.ndarray],
    *,
    image_size: tuple[int, int],
    columns: int,
    rows: int,
    square_size_m: float,
) -> CheckerboardIntrinsicsResult:
    """Calibrate one OpenCV pinhole/radtan profile from checkerboard views."""
    if len(pixel_views) < 8:
        raise CameraAcceptanceError("camera intrinsics require at least eight views")
    width, height = image_size
    if width <= 0 or height <= 0:
        raise CameraAcceptanceError("camera image size must be positive")
    object_points = _checkerboard_object_points(
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
    )
    image_points = []
    for index, value in enumerate(pixel_views):
        pixels = np.asarray(value, dtype=np.float32)
        if pixels.shape != (columns * rows, 2) or not np.isfinite(pixels).all():
            raise CameraAcceptanceError(
                f"pixel_views[{index}] must be finite with shape ({columns * rows},2)"
            )
        image_points.append(pixels.reshape(-1, 1, 2))
    rms, camera_matrix, distortion, _, _, _, _, per_view = cv2.calibrateCameraExtended(
        [object_points.copy() for _ in image_points],
        image_points,
        (width, height),
        None,
        None,
    )
    coefficients = np.asarray(distortion, dtype=np.float64).reshape(-1)
    if coefficients.size < 5:
        raise CameraAcceptanceError("OpenCV returned fewer than five distortion coefficients")
    return CheckerboardIntrinsicsResult(
        camera_matrix=_matrix(camera_matrix, (3, 3), "calibrated camera_matrix"),
        distortion_coefficients=coefficients[:5].copy(),
        image_size=(width, height),
        view_count=len(image_points),
        rms_reprojection_error_px=float(rms),
        per_view_errors_px=np.asarray(per_view, dtype=np.float64).reshape(-1),
    )


def solve_fixed_camera_pose(
    *,
    world_from_reference_cameras: list[np.ndarray],
    reference_camera_from_targets: list[np.ndarray],
    fixed_camera_from_targets: list[np.ndarray],
) -> FixedCameraPoseResult:
    """Estimate a fixed camera's world pose from paired board observations."""
    sample_count = len(world_from_reference_cameras)
    if (
        sample_count < 3
        or len(reference_camera_from_targets) != sample_count
        or len(fixed_camera_from_targets) != sample_count
    ):
        raise CameraAcceptanceError(
            "fixed-camera calibration requires at least three matching pose triples"
        )
    candidates = []
    for index, (world_from_reference, reference_from_target, fixed_from_target) in enumerate(
        zip(
            world_from_reference_cameras,
            reference_camera_from_targets,
            fixed_camera_from_targets,
            strict=True,
        )
    ):
        world_from_target = _rigid_transform(
            world_from_reference, f"world_from_reference_cameras[{index}]"
        ) @ _rigid_transform(
            reference_from_target, f"reference_camera_from_targets[{index}]"
        )
        candidates.append(
            world_from_target
            @ np.linalg.inv(
                _rigid_transform(
                    fixed_from_target, f"fixed_camera_from_targets[{index}]"
                )
            )
        )
    translations = np.stack([candidate[:3, 3] for candidate in candidates])
    mean_translation = translations.mean(axis=0)
    translation_errors = np.linalg.norm(translations - mean_translation, axis=1)
    rotations = Rotation.from_matrix(
        np.stack([candidate[:3, :3] for candidate in candidates])
    )
    mean_rotation = rotations.mean()
    rotation_errors = np.degrees((mean_rotation.inv() * rotations).magnitude())
    world_from_camera = np.eye(4, dtype=np.float64)
    world_from_camera[:3, :3] = mean_rotation.as_matrix()
    world_from_camera[:3, 3] = mean_translation
    return FixedCameraPoseResult(
        world_from_camera=_rigid_transform(world_from_camera, "world_from_camera"),
        sample_count=sample_count,
        translation_rms_m=float(np.sqrt(np.mean(translation_errors**2))),
        translation_max_m=float(translation_errors.max()),
        rotation_rms_deg=float(np.sqrt(np.mean(rotation_errors**2))),
        rotation_max_deg=float(rotation_errors.max()),
    )


def _matrix(value: Any, shape: tuple[int, int], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.isfinite(array).all():
        raise CameraAcceptanceError(f"{name} must be finite with shape {shape}")
    return array


def _rigid_transform(value: Any, name: str) -> np.ndarray:
    transform = _matrix(value, (4, 4), name)
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-10):
        raise CameraAcceptanceError(f"{name} must be homogeneous")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7) or not np.isclose(
        np.linalg.det(rotation), 1.0, atol=1e-7
    ):
        raise CameraAcceptanceError(f"{name} rotation must be rigid")
    return transform


def estimate_checkerboard_pose(
    pixels_uv: np.ndarray,
    intrinsics: np.ndarray,
    *,
    columns: int,
    rows: int,
    square_size_m: float,
    distortion_coefficients: np.ndarray,
) -> np.ndarray:
    """Estimate ``T_camera_target`` from ordered internal checkerboard corners."""
    pixels = np.asarray(pixels_uv, dtype=np.float64)
    K = _matrix(intrinsics, (3, 3), "intrinsics")
    distortion = np.asarray(distortion_coefficients, dtype=np.float64)
    if columns < 2 or rows < 2 or pixels.shape != (columns * rows, 2):
        raise CameraAcceptanceError(
            f"checkerboard pixels must have shape ({columns * rows},2)"
        )
    if not np.isfinite(pixels).all() or not np.isfinite(distortion).all():
        raise CameraAcceptanceError("checkerboard inputs must be finite")
    if not np.isfinite(square_size_m) or square_size_m <= 0:
        raise CameraAcceptanceError("checkerboard square size must be positive")
    object_points = np.zeros((columns * rows, 3), dtype=np.float64)
    object_points[:, :2] = (
        np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    )
    success, rotation_vector, translation = cv2.solvePnP(
        object_points,
        pixels,
        K,
        distortion,
        flags=cv2.SOLVEPNP_IPPE,
    )
    if not success:
        raise CameraAcceptanceError("checkerboard PnP failed")
    rotation, _ = cv2.Rodrigues(rotation_vector)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return _rigid_transform(transform, "camera_from_target")


def choose_checkerboard_orientation(
    pixels_uv: np.ndarray,
    intrinsics: np.ndarray,
    *,
    world_from_camera: np.ndarray,
    reference_world_from_target: np.ndarray,
    columns: int,
    rows: int,
    square_size_m: float,
    distortion_coefficients: np.ndarray,
) -> OrientedCheckerboardPose:
    """Resolve the checker's 180-degree corner-order ambiguity using station FK."""
    pixels = np.asarray(pixels_uv, dtype=np.float64)
    world_from_camera_matrix = _rigid_transform(
        world_from_camera, "world_from_camera"
    )
    reference = _rigid_transform(
        reference_world_from_target, "reference_world_from_target"
    )
    choices = []
    for reversed_order in (False, True):
        ordered = pixels[::-1].copy() if reversed_order else pixels.copy()
        camera_from_target = estimate_checkerboard_pose(
            ordered,
            intrinsics,
            columns=columns,
            rows=rows,
            square_size_m=square_size_m,
            distortion_coefficients=distortion_coefficients,
        )
        world_from_target = world_from_camera_matrix @ camera_from_target
        delta = np.linalg.inv(reference) @ world_from_target
        rotation_angle = Rotation.from_matrix(delta[:3, :3]).magnitude()
        score = float(np.linalg.norm(delta[:3, 3]) + 0.1 * rotation_angle)
        choices.append(
            OrientedCheckerboardPose(
                camera_from_target=camera_from_target,
                world_from_target=world_from_target,
                ordered_pixels=ordered,
                reversed=reversed_order,
                consistency_score=score,
            )
        )
    return min(choices, key=lambda choice: choice.consistency_score)


def evaluate_checkerboard_pose_consistency(
    pixels_uv: np.ndarray,
    intrinsics: np.ndarray,
    *,
    world_from_camera: np.ndarray,
    reference_world_from_target: np.ndarray,
    columns: int,
    rows: int,
    square_size_m: float,
    distortion_coefficients: np.ndarray,
) -> dict[str, Any]:
    pose = choose_checkerboard_orientation(
        pixels_uv,
        intrinsics,
        world_from_camera=world_from_camera,
        reference_world_from_target=reference_world_from_target,
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
        distortion_coefficients=distortion_coefficients,
    )
    reference = _rigid_transform(
        reference_world_from_target, "reference_world_from_target"
    )
    delta = np.linalg.inv(reference) @ pose.world_from_target
    return {
        "translation_error_m": float(np.linalg.norm(delta[:3, 3])),
        "rotation_error_deg": float(
            np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude())
        ),
        "reversed": pose.reversed,
        "observed_world_from_target": pose.world_from_target,
    }


def solve_hand_eye(
    world_from_gripper: list[np.ndarray],
    camera_from_target: list[np.ndarray],
) -> HandEyeResult:
    """Solve eye-in-hand ``T_gripper_camera`` with a fixed calibration target."""
    if len(world_from_gripper) != len(camera_from_target) or len(world_from_gripper) < 3:
        raise CameraAcceptanceError("hand-eye calibration requires at least three pose pairs")
    gripper_poses = [
        _rigid_transform(value, f"world_from_gripper[{index}]")
        for index, value in enumerate(world_from_gripper)
    ]
    target_poses = [
        _rigid_transform(value, f"camera_from_target[{index}]")
        for index, value in enumerate(camera_from_target)
    ]
    rotation, translation = cv2.calibrateHandEye(
        [pose[:3, :3] for pose in gripper_poses],
        [pose[:3, 3] for pose in gripper_poses],
        [pose[:3, :3] for pose in target_poses],
        [pose[:3, 3] for pose in target_poses],
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    gripper_from_camera = np.eye(4, dtype=np.float64)
    gripper_from_camera[:3, :3] = np.asarray(rotation, dtype=np.float64)
    gripper_from_camera[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    gripper_from_camera = _rigid_transform(
        gripper_from_camera, "solved gripper_from_camera"
    )

    world_from_targets = [
        world_from_gripper_pose @ gripper_from_camera @ camera_from_target_pose
        for world_from_gripper_pose, camera_from_target_pose in zip(
            gripper_poses, target_poses, strict=True
        )
    ]
    translations = np.stack([pose[:3, 3] for pose in world_from_targets])
    mean_translation = translations.mean(axis=0)
    translation_errors = np.linalg.norm(translations - mean_translation, axis=1)
    target_rotations = Rotation.from_matrix(
        np.stack([pose[:3, :3] for pose in world_from_targets])
    )
    mean_rotation = target_rotations.mean()
    rotation_errors = (mean_rotation.inv() * target_rotations).magnitude()
    world_from_target = np.eye(4, dtype=np.float64)
    world_from_target[:3, :3] = mean_rotation.as_matrix()
    world_from_target[:3, 3] = mean_translation
    return HandEyeResult(
        gripper_from_camera=gripper_from_camera,
        world_from_target=world_from_target,
        pose_count=len(gripper_poses),
        target_translation_rms_m=float(
            np.sqrt(np.mean(translation_errors**2))
        ),
        target_translation_max_m=float(translation_errors.max()),
        target_rotation_rms_deg=float(
            np.degrees(np.sqrt(np.mean(rotation_errors**2)))
        ),
        target_rotation_max_deg=float(np.degrees(rotation_errors.max())),
    )


def calibrate_checkerboard_records(
    records: list[CheckerboardRecord],
    *,
    nominal_gripper_from_camera: np.ndarray,
    columns: int,
    rows: int,
    square_size_m: float,
    distortion_coefficients: np.ndarray,
) -> CheckerboardCalibrationResult:
    """Orient checker detections consistently and solve one eye-in-hand transform."""
    if len(records) < 3:
        raise CameraAcceptanceError(
            "checkerboard hand-eye calibration requires at least three records"
        )
    nominal = _rigid_transform(
        nominal_gripper_from_camera, "nominal_gripper_from_camera"
    )
    first = records[0]
    first_camera_from_target = estimate_checkerboard_pose(
        first.pixels_uv,
        first.intrinsics,
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
        distortion_coefficients=distortion_coefficients,
    )
    reference_world_from_target = _rigid_transform(
        first.nominal_world_from_camera, "records[0].nominal_world_from_camera"
    ) @ first_camera_from_target

    oriented: list[OrientedCheckerboardPose] = []
    world_from_gripper: list[np.ndarray] = []
    for index, record in enumerate(records):
        world_from_camera = _rigid_transform(
            record.nominal_world_from_camera,
            f"records[{index}].nominal_world_from_camera",
        )
        oriented.append(
            choose_checkerboard_orientation(
                record.pixels_uv,
                record.intrinsics,
                world_from_camera=world_from_camera,
                reference_world_from_target=reference_world_from_target,
                columns=columns,
                rows=rows,
                square_size_m=square_size_m,
                distortion_coefficients=distortion_coefficients,
            )
        )
        world_from_gripper.append(world_from_camera @ np.linalg.inv(nominal))

    hand_eye = solve_hand_eye(
        world_from_gripper,
        [pose.camera_from_target for pose in oriented],
    )
    object_points = np.zeros((columns * rows, 3), dtype=np.float64)
    object_points[:, :2] = (
        np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    )
    reprojection_errors = []
    for record, pose in zip(records, oriented, strict=True):
        rotation_vector, _ = cv2.Rodrigues(pose.camera_from_target[:3, :3])
        projected, _ = cv2.projectPoints(
            object_points,
            rotation_vector,
            pose.camera_from_target[:3, 3],
            record.intrinsics,
            distortion_coefficients,
        )
        reprojection_errors.extend(
            np.linalg.norm(projected.reshape(-1, 2) - pose.ordered_pixels, axis=1)
        )
    errors = np.asarray(reprojection_errors, dtype=np.float64)
    return CheckerboardCalibrationResult(
        hand_eye=hand_eye,
        reversed_pose_count=sum(pose.reversed for pose in oriented),
        rms_reprojection_error_px=float(np.sqrt(np.mean(errors**2))),
        max_reprojection_error_px=float(errors.max()),
    )


def undistort_realsense_pixels(
    pixels_uv: np.ndarray,
    intrinsics: np.ndarray,
    *,
    width: int,
    height: int,
    distortion_model: str,
    distortion_coefficients: np.ndarray,
) -> np.ndarray:
    """Map RealSense pixels to an equivalent undistorted pinhole image."""
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise CameraAcceptanceError(
            "RealSense pixel undistortion requires pyrealsense2"
        ) from exc
    pixels = np.asarray(pixels_uv, dtype=np.float64)
    K = _matrix(intrinsics, (3, 3), "intrinsics")
    coefficients = np.asarray(distortion_coefficients, dtype=np.float64)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or not np.isfinite(pixels).all():
        raise CameraAcceptanceError("pixels_uv must be finite with shape (N,2)")
    if width <= 0 or height <= 0 or coefficients.shape != (5,):
        raise CameraAcceptanceError(
            "RealSense width/height must be positive and distortion requires 5 coefficients"
        )
    try:
        model = getattr(rs.distortion, distortion_model)
    except AttributeError as exc:
        raise CameraAcceptanceError(
            f"unsupported RealSense distortion model {distortion_model!r}"
        ) from exc
    rs_intrinsics = rs.intrinsics()
    rs_intrinsics.width = width
    rs_intrinsics.height = height
    rs_intrinsics.fx = float(K[0, 0])
    rs_intrinsics.fy = float(K[1, 1])
    rs_intrinsics.ppx = float(K[0, 2])
    rs_intrinsics.ppy = float(K[1, 2])
    rs_intrinsics.model = model
    rs_intrinsics.coeffs = coefficients.tolist()
    rays = np.asarray(
        [
            rs.rs2_deproject_pixel_to_point(
                rs_intrinsics, [float(pixel[0]), float(pixel[1])], 1.0
            )
            for pixel in pixels
        ],
        dtype=np.float64,
    )
    normalized = rays[:, :2] / rays[:, 2, None]
    return np.column_stack(
        (
            K[0, 0] * normalized[:, 0] + K[0, 2],
            K[1, 1] * normalized[:, 1] + K[1, 2],
        )
    )


def deproject_pixels(
    pixels_uv: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: np.ndarray,
) -> np.ndarray:
    """Deproject ``(u,v,depth_m)`` through pinhole K into camera-frame meters."""
    pixels = np.asarray(pixels_uv, dtype=np.float64)
    depths = np.asarray(depth_m, dtype=np.float64)
    K = _matrix(intrinsics, (3, 3), "intrinsics")
    if pixels.ndim != 2 or pixels.shape[1] != 2:
        raise CameraAcceptanceError("pixels_uv must have shape (N,2)")
    if depths.shape != (len(pixels),) or not np.isfinite(depths).all() or (depths <= 0).any():
        raise CameraAcceptanceError("depth_m must contain one positive finite value per pixel")
    if K[0, 0] <= 0 or K[1, 1] <= 0 or not np.allclose(K[2], [0, 0, 1]):
        raise CameraAcceptanceError("intrinsics are not a pinhole K matrix")
    x = (pixels[:, 0] - K[0, 2]) * depths / K[0, 0]
    y = (pixels[:, 1] - K[1, 2]) * depths / K[1, 1]
    return np.column_stack((x, y, depths))


def _transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    homogeneous = np.column_stack((points, np.ones(len(points), dtype=np.float64)))
    return (homogeneous @ transform.T)[:, :3]


def evaluate_correspondences(
    pixels_uv: np.ndarray,
    expected_world_xyz_m: np.ndarray,
    depth_image_m: np.ndarray,
    intrinsics: np.ndarray,
    camera_to_world: np.ndarray,
) -> dict[str, float | int]:
    """Measure deprojected world error and expected-point reprojection error."""
    pixels = np.asarray(pixels_uv, dtype=np.float64)
    expected_world = np.asarray(expected_world_xyz_m, dtype=np.float64)
    depth = np.asarray(depth_image_m, dtype=np.float32)
    K = _matrix(intrinsics, (3, 3), "intrinsics")
    world_from_camera = _matrix(camera_to_world, (4, 4), "camera_to_world")
    if pixels.ndim != 2 or pixels.shape[1] != 2 or expected_world.shape != (len(pixels), 3):
        raise CameraAcceptanceError("correspondences require pixels (N,2) and world points (N,3)")
    if len(pixels) == 0 or depth.ndim != 2 or not np.isfinite(expected_world).all():
        raise CameraAcceptanceError("correspondences and a 2-D depth image are required")
    pixel_indices = np.rint(pixels).astype(np.int64)
    u, v = pixel_indices[:, 0], pixel_indices[:, 1]
    if (u < 0).any() or (u >= depth.shape[1]).any() or (v < 0).any() or (v >= depth.shape[0]).any():
        raise CameraAcceptanceError("a correspondence pixel is outside the depth image")
    sampled_depth = depth[v, u].astype(np.float64)
    camera_points = deproject_pixels(pixels, sampled_depth, K)
    observed_world = _transform_points(camera_points, world_from_camera)
    world_errors = np.linalg.norm(observed_world - expected_world, axis=1)

    expected_camera = _transform_points(expected_world, np.linalg.inv(world_from_camera))
    if (expected_camera[:, 2] <= 0).any():
        raise CameraAcceptanceError("an expected world point is behind the camera")
    projected = np.column_stack(
        (
            K[0, 0] * expected_camera[:, 0] / expected_camera[:, 2] + K[0, 2],
            K[1, 1] * expected_camera[:, 1] / expected_camera[:, 2] + K[1, 2],
        )
    )
    reprojection_errors = np.linalg.norm(projected - pixels, axis=1)
    return {
        "point_count": len(pixels),
        "rms_world_error_m": float(np.sqrt(np.mean(world_errors**2))),
        "max_world_error_m": float(world_errors.max()),
        "rms_reprojection_error_px": float(np.sqrt(np.mean(reprojection_errors**2))),
        "max_reprojection_error_px": float(reprojection_errors.max()),
    }


def evaluate_table_plane(
    depth_image_m: np.ndarray,
    intrinsics: np.ndarray,
    camera_to_world: np.ndarray,
    *,
    roi: tuple[int, int, int, int],
    expected_world_z_m: float,
) -> dict[str, float | int]:
    """Fit a world-frame plane to a depth ROI and compare it with a horizontal table."""
    depth = np.asarray(depth_image_m, dtype=np.float32)
    K = _matrix(intrinsics, (3, 3), "intrinsics")
    world_from_camera = _matrix(camera_to_world, (4, 4), "camera_to_world")
    if depth.ndim != 2 or not np.isfinite(depth).all():
        raise CameraAcceptanceError("depth image must be a finite 2-D metric array")
    x0, y0, x1, y1 = roi
    if not (0 <= x0 < x1 <= depth.shape[1] and 0 <= y0 < y1 <= depth.shape[0]):
        raise CameraAcceptanceError("ROI must be a non-empty rectangle inside the depth image")
    yy, xx = np.mgrid[y0:y1, x0:x1]
    samples = depth[y0:y1, x0:x1].reshape(-1)
    valid = samples > 0
    if int(valid.sum()) < 3:
        raise CameraAcceptanceError("table ROI contains fewer than three valid depth samples")
    pixels = np.column_stack((xx.reshape(-1)[valid], yy.reshape(-1)[valid]))
    camera_points = deproject_pixels(pixels, samples[valid], K)
    world_points = _transform_points(camera_points, world_from_camera)
    center = world_points.mean(axis=0)
    _, _, vh = np.linalg.svd(world_points - center, full_matrices=False)
    normal = vh[-1]
    distances = (world_points - center) @ normal
    normal_cosine = float(np.clip(abs(normal[2]), 0.0, 1.0))
    return {
        "point_count": len(world_points),
        "plane_rms_m": float(np.sqrt(np.mean(distances**2))),
        "normal_error_deg": float(np.degrees(np.arccos(normal_cosine))),
        "world_z_error_m": float(abs(center[2] - expected_world_z_m)),
        "mean_world_z_m": float(center[2]),
    }


def _save_capture(path: Path, observations: list[dict[str, Any]]) -> None:
    cameras = [observation["camera_0"] for observation in observations]
    serials = {camera["serial"] for camera in cameras}
    if len(serials) != 1:
        raise CameraAcceptanceError("camera serial changed during capture")
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        rgb=np.stack([camera["images"]["rgb"] for camera in cameras]),
        depth_m=np.stack([camera["images"]["depth"] for camera in cameras]),
        intrinsics=np.stack([camera["intrinsics"] for camera in cameras]),
        camera_to_world=np.stack([camera["camera_to_world"] for camera in cameras]),
        frame_sequence=np.array([camera["frame_sequence"] for camera in cameras], dtype=np.int64),
        frame_monotonic_ns=np.array(
            [camera["frame_monotonic_ns"] for camera in cameras], dtype=np.int64
        ),
        joint_time_delta_ns=np.array(
            [camera["joint_time_delta_ns"] for camera in cameras], dtype=np.int64
        ),
        robot_joint_pos_0=np.stack(
            [observation["robot_joint_pos_0"] for observation in observations]
        ),
        robot_joint_vel_0=np.stack(
            [observation["robot_joint_vel_0"] for observation in observations]
        ),
        robot_joint_effort_0=np.stack(
            [observation["robot_joint_effort_0"] for observation in observations]
        ),
        serial=np.array(next(iter(serials))),
    )


def _pose_jitter(camera_to_world: np.ndarray) -> dict[str, float | int]:
    poses = np.asarray(camera_to_world, dtype=np.float64)
    reference = poses[0]
    translations = np.linalg.norm(poses[:, :3, 3] - reference[:3, 3], axis=1)
    angles = []
    for pose in poses:
        relative = reference[:3, :3].T @ pose[:3, :3]
        cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
        angles.append(np.degrees(np.arccos(cosine)))
    return {
        "frame_count": len(poses),
        "max_translation_jitter_m": float(translations.max()),
        "max_rotation_jitter_deg": float(max(angles)),
    }


def _load_capture(path: Path, index: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    with np.load(path, allow_pickle=False) as capture:
        frame_count = len(capture["depth_m"])
        if not 0 <= index < frame_count:
            raise CameraAcceptanceError(f"frame index {index} is outside 0..{frame_count - 1}")
        return (
            capture["depth_m"][index],
            capture["intrinsics"][index],
            capture["camera_to_world"][index],
        )


def _station_flange_from_camera(config: Any) -> np.ndarray:
    import mujoco

    try:
        model = mujoco.MjModel.from_xml_path(str(config.hardware.station_model))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        def body_transform(name: str) -> np.ndarray:
            body = data.body(name)
            transform = np.eye(4, dtype=np.float64)
            transform[:3, :3] = np.asarray(body.xmat).reshape(3, 3)
            transform[:3, 3] = body.xpos
            return transform

        station_from_flange = body_transform(config.camera.station_flange_body)
        station_from_camera = body_transform(config.camera.station_camera_body)
    except Exception as exc:
        raise CameraAcceptanceError(
            f"could not load station flange-camera transform: {exc}"
        ) from exc
    return _rigid_transform(
        np.linalg.inv(station_from_flange) @ station_from_camera,
        "station flange_from_camera",
    )


def _calibrate_checkerboard_captures(args: argparse.Namespace) -> dict[str, Any]:
    config = load_config(args.config)
    nominal = _station_flange_from_camera(config)
    records = []
    capture_reports = []
    observed_serials = set()
    for path in args.captures:
        with np.load(path, allow_pickle=False) as capture:
            frame_count = len(capture["rgb"])
            index = frame_count // 2 if args.frame_index is None else args.frame_index
            if not 0 <= index < frame_count:
                raise CameraAcceptanceError(
                    f"{path}: frame index {index} is outside 0..{frame_count - 1}"
                )
            rgb = capture["rgb"][index]
            K = capture["intrinsics"][index]
            world_from_camera = capture["camera_to_world"][index]
            serial = str(capture["serial"])
        observed_serials.add(serial)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        # retry: findChessboardCornersSB(EXHAUSTIVE) is flaky on some real 1080p
        # frames (non-deterministic misses); a miss on one image must not abort
        # a whole solve. (Local change in our copy, 2026-09-02.)
        detected, corners = False, None
        for _attempt in range(6):
            detected, corners = cv2.findChessboardCornersSB(
                gray, (args.columns, args.rows),
                flags=(cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY),
            )
            if detected and corners is not None:
                break
        if not detected or corners is None:
            raise CameraAcceptanceError(
                f"{path}: did not detect {args.columns}x{args.rows} checkerboard corners"
            )
        pixels = undistort_realsense_pixels(
            corners.reshape(-1, 2),
            K,
            width=rgb.shape[1],
            height=rgb.shape[0],
            distortion_model=args.distortion_model,
            distortion_coefficients=np.asarray(
                args.distortion_coefficients, dtype=np.float64
            ),
        )
        records.append(
            CheckerboardRecord(
                label=path.stem,
                pixels_uv=pixels,
                intrinsics=K,
                nominal_world_from_camera=world_from_camera,
            )
        )
        capture_reports.append(
            {
                "label": path.stem,
                "path": str(path.resolve()),
                "frame_count": frame_count,
                "frame_index": index,
                "corner_count": len(pixels),
                "serial": serial,
            }
        )
    if observed_serials != {config.hardware.camera_serial}:
        raise CameraAcceptanceError(
            "capture serials do not exactly match configured wrist camera: "
            f"{sorted(observed_serials)} vs {config.hardware.camera_serial!r}"
        )
    result = calibrate_checkerboard_records(
        records,
        nominal_gripper_from_camera=nominal,
        columns=args.columns,
        rows=args.rows,
        square_size_m=args.square_size_m,
        distortion_coefficients=np.zeros(5),
    )
    hand_eye = result.hand_eye
    nominal_delta = np.linalg.inv(nominal) @ hand_eye.gripper_from_camera
    nominal_delta_rotation_deg = float(
        np.degrees(Rotation.from_matrix(nominal_delta[:3, :3]).magnitude())
    )
    nominal_delta_translation_m = float(np.linalg.norm(nominal_delta[:3, 3]))
    passed = (
        result.max_reprojection_error_px <= args.max_reprojection_error_px
        and hand_eye.target_translation_rms_m
        <= args.max_target_translation_rms_m
        and hand_eye.target_rotation_rms_deg <= args.max_target_rotation_rms_deg
    )
    return {
        "status": "PASS" if passed else "FAIL",
        "pose_count": hand_eye.pose_count,
        "captures": capture_reports,
        "checkerboard": {
            "columns": args.columns,
            "rows": args.rows,
            "square_size_m": args.square_size_m,
            "reversed_pose_count": result.reversed_pose_count,
        },
        "distortion": {
            "model": args.distortion_model,
            "coefficients": args.distortion_coefficients,
        },
        "metrics": {
            "rms_reprojection_error_px": result.rms_reprojection_error_px,
            "max_reprojection_error_px": result.max_reprojection_error_px,
            "target_translation_rms_m": hand_eye.target_translation_rms_m,
            "target_translation_max_m": hand_eye.target_translation_max_m,
            "target_rotation_rms_deg": hand_eye.target_rotation_rms_deg,
            "target_rotation_max_deg": hand_eye.target_rotation_max_deg,
            "solved_from_nominal_translation_m": nominal_delta_translation_m,
            "solved_from_nominal_rotation_deg": nominal_delta_rotation_deg,
        },
        "thresholds": {
            "max_reprojection_error_px": args.max_reprojection_error_px,
            "max_target_translation_rms_m": args.max_target_translation_rms_m,
            "max_target_rotation_rms_deg": args.max_target_rotation_rms_deg,
        },
        "nominal_flange_from_camera": nominal.tolist(),
        "solved_flange_from_camera": hand_eye.gripper_from_camera.tolist(),
        "solved_world_from_target": hand_eye.world_from_target.tolist(),
        "station_model_updated": False,
    }


def _evaluate_checkerboard_pose_capture(args: argparse.Namespace) -> dict[str, Any]:
    reference = json.loads(args.reference_report.read_text(encoding="utf-8"))
    if not isinstance(reference, dict) or reference.get("status") != "PASS":
        raise CameraAcceptanceError("reference report must be a PASS calibration report")
    checkerboard = reference.get("checkerboard")
    distortion = reference.get("distortion")
    captures = reference.get("captures")
    if (
        not isinstance(checkerboard, dict)
        or not isinstance(distortion, dict)
        or not isinstance(captures, list)
    ):
        raise CameraAcceptanceError("reference report is missing calibration metadata")
    reference_serials = {
        capture.get("serial") for capture in captures if isinstance(capture, dict)
    }
    with np.load(args.capture, allow_pickle=False) as capture:
        frame_count = len(capture["rgb"])
        index = frame_count // 2 if args.frame_index is None else args.frame_index
        if not 0 <= index < frame_count:
            raise CameraAcceptanceError(
                f"{args.capture}: frame index {index} is outside 0..{frame_count - 1}"
            )
        rgb = capture["rgb"][index]
        K = capture["intrinsics"][index]
        world_from_camera = capture["camera_to_world"][index]
        serial = str(capture["serial"])
    if reference_serials != {serial}:
        raise CameraAcceptanceError(
            "held-out capture serial does not match reference captures: "
            f"{serial!r} vs {sorted(reference_serials)!r}"
        )
    columns = int(checkerboard["columns"])
    rows = int(checkerboard["rows"])
    square_size_m = float(checkerboard["square_size_m"])
    coefficients = np.asarray(distortion["coefficients"], dtype=np.float64)
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    detected, corners = cv2.findChessboardCornersSB(
        gray,
        (columns, rows),
        flags=(
            cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_EXHAUSTIVE
            | cv2.CALIB_CB_ACCURACY
        ),
    )
    if not detected or corners is None:
        raise CameraAcceptanceError(
            f"{args.capture}: did not detect {columns}x{rows} checkerboard corners"
        )
    pixels = undistort_realsense_pixels(
        corners.reshape(-1, 2),
        K,
        width=rgb.shape[1],
        height=rgb.shape[0],
        distortion_model=str(distortion["model"]),
        distortion_coefficients=coefficients,
    )
    metrics = evaluate_checkerboard_pose_consistency(
        pixels,
        K,
        world_from_camera=world_from_camera,
        reference_world_from_target=np.asarray(
            reference["solved_world_from_target"], dtype=np.float64
        ),
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
        distortion_coefficients=np.zeros(5),
    )
    observed = metrics.pop("observed_world_from_target")
    passed = (
        metrics["translation_error_m"] <= args.max_translation_error_m
        and metrics["rotation_error_deg"] <= args.max_rotation_error_deg
    )
    return {
        "status": "PASS" if passed else "FAIL",
        "capture": {
            "path": str(args.capture.resolve()),
            "frame_count": frame_count,
            "frame_index": index,
            "corner_count": len(pixels),
            "serial": serial,
        },
        "reference_report": str(args.reference_report.resolve()),
        "checkerboard": checkerboard,
        "distortion": distortion,
        "metrics": {
            **metrics,
            "observed_world_from_target": observed.tolist(),
        },
        "thresholds": {
            "max_translation_error_m": args.max_translation_error_m,
            "max_rotation_error_deg": args.max_rotation_error_deg,
        },
    }


def _detect_checkerboard_images(
    paths: list[Path],
    *,
    width: int,
    height: int,
    columns: int,
    rows: int,
) -> list[np.ndarray]:
    detections = []
    for path in paths:
        bgr = cv2.imread(str(path))
        if bgr is None:
            raise CameraAcceptanceError(f"could not read calibration image: {path}")
        if bgr.dtype != np.uint8 or bgr.shape != (height, width, 3):
            raise CameraAcceptanceError(
                f"{path}: expected uint8[{height},{width},3], got {bgr.dtype}{bgr.shape}"
            )
        detected, corners = cv2.findChessboardCornersSB(
            cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY),
            (columns, rows),
            flags=(
                cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_EXHAUSTIVE
                | cv2.CALIB_CB_ACCURACY
            ),
        )
        if not detected or corners is None:
            raise CameraAcceptanceError(
                f"{path}: did not detect {columns}x{rows} checkerboard corners"
            )
        detections.append(np.asarray(corners, dtype=np.float64).reshape(-1, 2))
    return detections


def _checkerboard_view_error(
    pixels_uv: np.ndarray,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    *,
    columns: int,
    rows: int,
    square_size_m: float,
) -> float:
    object_points = _checkerboard_object_points(
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
    ).astype(np.float64)
    camera_from_target = estimate_checkerboard_pose(
        pixels_uv,
        camera_matrix,
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
        distortion_coefficients=distortion_coefficients,
    )
    rotation_vector, _ = cv2.Rodrigues(camera_from_target[:3, :3])
    projected, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        camera_from_target[:3, 3],
        camera_matrix,
        distortion_coefficients,
    )
    errors = np.linalg.norm(projected.reshape(-1, 2) - pixels_uv, axis=1)
    return float(np.sqrt(np.mean(errors**2)))


def _calibrate_top_intrinsics_images(args: argparse.Namespace) -> dict[str, Any]:
    if not args.serial or args.serial not in Path(args.device).name:
        raise CameraAcceptanceError(
            "top-camera serial must be non-empty and present in the stable by-id device path"
        )
    calibration_pixels = _detect_checkerboard_images(
        args.images,
        width=args.width,
        height=args.height,
        columns=args.columns,
        rows=args.rows,
    )
    validation_pixels = _detect_checkerboard_images(
        args.validation_images,
        width=args.width,
        height=args.height,
        columns=args.columns,
        rows=args.rows,
    )
    result = calibrate_checkerboard_intrinsics(
        calibration_pixels,
        image_size=(args.width, args.height),
        columns=args.columns,
        rows=args.rows,
        square_size_m=args.square_size_m,
    )
    validation_errors = [
        _checkerboard_view_error(
            pixels,
            result.camera_matrix,
            result.distortion_coefficients,
            columns=args.columns,
            rows=args.rows,
            square_size_m=args.square_size_m,
        )
        for pixels in validation_pixels
    ]
    passed = (
        result.rms_reprojection_error_px <= args.max_calibration_rms_px
        and max(validation_errors) <= args.max_validation_rms_px
    )
    return {
        "schema_version": 1,
        "type": "fixed_rgb_camera_intrinsics",
        "status": "PASS" if passed else "FAIL",
        "camera": {
            "name": "top_brio",
            "model": "Logitech BRIO",
            "serial": args.serial,
            "device": args.device,
            "width": args.width,
            "height": args.height,
            "fps": args.fps,
        },
        "checkerboard": {
            "columns": args.columns,
            "rows": args.rows,
            "square_size_m": args.square_size_m,
        },
        "distortion_model": "opencv_radtan",
        "camera_matrix": result.camera_matrix.tolist(),
        "distortion_coefficients": result.distortion_coefficients.tolist(),
        "calibration_view_count": result.view_count,
        "validation_view_count": len(validation_errors),
        "metrics": {
            "calibration_rms_reprojection_error_px": result.rms_reprojection_error_px,
            "calibration_per_view_errors_px": result.per_view_errors_px.tolist(),
            "validation_rms_reprojection_errors_px": validation_errors,
            "validation_mean_rms_reprojection_error_px": float(
                np.mean(validation_errors)
            ),
            "validation_max_rms_reprojection_error_px": float(
                max(validation_errors)
            ),
        },
        "thresholds": {
            "max_calibration_rms_px": args.max_calibration_rms_px,
            "max_validation_rms_px": args.max_validation_rms_px,
        },
    }


def _station_world_from_top_camera(config: Any) -> np.ndarray:
    import mujoco

    try:
        model = mujoco.MjModel.from_xml_path(str(config.hardware.station_model))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        station_from_world = _frame_transform(data.body("right_base"))
        station_from_top = _frame_transform(data.body("top_camera"))
    except Exception as exc:
        raise CameraAcceptanceError(
            f"could not load nominal right_base-to-top_camera transform: {exc}"
        ) from exc
    return _rigid_transform(
        np.linalg.inv(station_from_world) @ station_from_top,
        "nominal world_from_top_camera",
    )


def _frame_transform(frame: Any) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(frame.xmat, dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(frame.xpos, dtype=np.float64)
    return transform


def _transform_distance(reference: np.ndarray, candidate: np.ndarray) -> tuple[float, float]:
    delta = np.linalg.inv(reference) @ candidate
    return (
        float(np.linalg.norm(delta[:3, 3])),
        float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude())),
    )


def _checkerboard_pose_candidates(
    pixels: np.ndarray,
    intrinsics: np.ndarray,
    distortion_coefficients: np.ndarray,
    *,
    columns: int,
    rows: int,
    square_size_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    return tuple(
        estimate_checkerboard_pose(
            ordered,
            intrinsics,
            columns=columns,
            rows=rows,
            square_size_m=square_size_m,
            distortion_coefficients=distortion_coefficients,
        )
        for ordered in (pixels, pixels[::-1])
    )


def _detect_checkerboard(rgb: np.ndarray, *, columns: int, rows: int, label: str) -> np.ndarray:
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise CameraAcceptanceError(f"{label} RGB must be uint8[H,W,3]")
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    for scale in (1.0, 2.0):
        detection_image = (
            gray
            if scale == 1.0
            else cv2.resize(
                gray,
                None,
                fx=scale,
                fy=scale,
                interpolation=cv2.INTER_CUBIC,
            )
        )
        detected, corners = cv2.findChessboardCornersSB(
            detection_image,
            (columns, rows),
            flags=(
                cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_EXHAUSTIVE
                | cv2.CALIB_CB_ACCURACY
            ),
        )
        if detected and corners is not None:
            return np.asarray(corners, dtype=np.float64).reshape(-1, 2) / scale
    raise CameraAcceptanceError(
        f"{label}: did not detect {columns}x{rows} checkerboard corners"
    )


def _paired_capture_candidates(
    path: Path,
    *,
    top_intrinsics: np.ndarray,
    top_distortion: np.ndarray,
    wrist_distortion_model: str,
    wrist_distortion: np.ndarray,
    columns: int,
    rows: int,
    square_size_m: float,
) -> tuple[list[np.ndarray], str]:
    with np.load(path, allow_pickle=False) as capture:
        required = {
            "top_rgb",
            "wrist_rgb",
            "wrist_intrinsics",
            "wrist_camera_to_world",
            "wrist_serial",
        }
        if not required.issubset(capture.files):
            raise CameraAcceptanceError(
                f"{path}: paired capture is missing {sorted(required - set(capture.files))}"
            )
        top_rgb = capture["top_rgb"]
        wrist_rgb = capture["wrist_rgb"]
        wrist_intrinsics = _matrix(
            capture["wrist_intrinsics"], (3, 3), f"{path} wrist_intrinsics"
        )
        world_from_wrist = _rigid_transform(
            capture["wrist_camera_to_world"], f"{path} wrist_camera_to_world"
        )
        wrist_serial = str(capture["wrist_serial"])
    top_pixels = _detect_checkerboard(
        top_rgb, columns=columns, rows=rows, label=f"{path} top"
    )
    wrist_pixels = _detect_checkerboard(
        wrist_rgb, columns=columns, rows=rows, label=f"{path} wrist"
    )
    if wrist_distortion_model == "inverse_brown_conrady":
        wrist_pixels = undistort_realsense_pixels(
            wrist_pixels,
            wrist_intrinsics,
            width=wrist_rgb.shape[1],
            height=wrist_rgb.shape[0],
            distortion_model=wrist_distortion_model,
            distortion_coefficients=wrist_distortion,
        )
        effective_wrist_distortion = np.zeros(5, dtype=np.float64)
    elif wrist_distortion_model == "none":
        effective_wrist_distortion = np.zeros(5, dtype=np.float64)
    else:
        raise CameraAcceptanceError(
            f"unsupported wrist distortion model {wrist_distortion_model!r}"
        )
    top_poses = _checkerboard_pose_candidates(
        top_pixels,
        top_intrinsics,
        top_distortion,
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
    )
    wrist_poses = _checkerboard_pose_candidates(
        wrist_pixels,
        wrist_intrinsics,
        effective_wrist_distortion,
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
    )
    candidates = [
        world_from_wrist @ wrist_from_target @ np.linalg.inv(top_from_target)
        for wrist_from_target in wrist_poses
        for top_from_target in top_poses
    ]
    return candidates, wrist_serial


def _closest_transform(candidates: list[np.ndarray], reference: np.ndarray) -> np.ndarray:
    def score(candidate: np.ndarray) -> float:
        translation_m, rotation_deg = _transform_distance(reference, candidate)
        return translation_m + 0.1 * np.radians(rotation_deg)

    return min(candidates, key=score)


def _average_transforms(transforms: list[np.ndarray]) -> FixedCameraPoseResult:
    return solve_fixed_camera_pose(
        world_from_reference_cameras=transforms,
        reference_camera_from_targets=[np.eye(4) for _ in transforms],
        fixed_camera_from_targets=[np.eye(4) for _ in transforms],
    )


def _select_consistent_transforms(
    candidate_sets: list[list[np.ndarray]], nominal: np.ndarray
) -> FixedCameraPoseResult:
    if len(candidate_sets) < 3 or any(not candidates for candidates in candidate_sets):
        raise CameraAcceptanceError(
            "fixed-camera orientation selection requires three non-empty candidate sets"
        )
    solutions = []
    for seed in candidate_sets[0]:
        reference = seed
        result = None
        for _ in range(5):
            selected = [
                _closest_transform(candidates, reference) for candidates in candidate_sets
            ]
            result = _average_transforms(selected)
            reference = result.world_from_camera
        assert result is not None
        nominal_translation_m, nominal_rotation_deg = _transform_distance(
            nominal, result.world_from_camera
        )
        consistency_score = (
            result.translation_rms_m
            + 0.1 * np.radians(result.rotation_rms_deg)
            + 1e-4 * (nominal_translation_m + np.radians(nominal_rotation_deg))
        )
        solutions.append((consistency_score, result))
    return min(solutions, key=lambda item: item[0])[1]


def _calibrate_top_extrinsics(args: argparse.Namespace) -> dict[str, Any]:
    intrinsics_payload = json.loads(args.intrinsics.read_text(encoding="utf-8"))
    if (
        not isinstance(intrinsics_payload, dict)
        or intrinsics_payload.get("type") != "fixed_rgb_camera_intrinsics"
        or intrinsics_payload.get("status") != "PASS"
    ):
        raise CameraAcceptanceError("top intrinsics must be a PASS fixed-camera report")
    camera = intrinsics_payload.get("camera")
    checkerboard = intrinsics_payload.get("checkerboard")
    if not isinstance(camera, dict) or not isinstance(checkerboard, dict):
        raise CameraAcceptanceError("top intrinsics report is missing camera/checkerboard metadata")
    columns = int(checkerboard["columns"])
    rows = int(checkerboard["rows"])
    square_size_m = float(checkerboard["square_size_m"])
    top_intrinsics = _matrix(
        intrinsics_payload.get("camera_matrix"), (3, 3), "top camera_matrix"
    )
    top_distortion = np.asarray(
        intrinsics_payload.get("distortion_coefficients"), dtype=np.float64
    )
    if top_distortion.shape != (5,) or not np.isfinite(top_distortion).all():
        raise CameraAcceptanceError("top distortion coefficients must be a finite five-vector")
    wrist_distortion = np.asarray(args.wrist_distortion_coefficients, dtype=np.float64)
    config = load_config(args.config)
    nominal = _station_world_from_top_camera(config)
    calibration_candidate_sets = []
    wrist_serials = set()
    for path in args.captures:
        candidates, wrist_serial = _paired_capture_candidates(
            path,
            top_intrinsics=top_intrinsics,
            top_distortion=top_distortion,
            wrist_distortion_model=args.wrist_distortion_model,
            wrist_distortion=wrist_distortion,
            columns=columns,
            rows=rows,
            square_size_m=square_size_m,
        )
        calibration_candidate_sets.append(candidates)
        wrist_serials.add(wrist_serial)
    if wrist_serials != {config.hardware.camera_serial}:
        raise CameraAcceptanceError(
            "paired captures do not exactly match the configured wrist camera: "
            f"{sorted(wrist_serials)} vs {config.hardware.camera_serial!r}"
        )
    result = _select_consistent_transforms(calibration_candidate_sets, nominal)
    validation_errors = []
    for path in args.validation_captures:
        candidates, wrist_serial = _paired_capture_candidates(
            path,
            top_intrinsics=top_intrinsics,
            top_distortion=top_distortion,
            wrist_distortion_model=args.wrist_distortion_model,
            wrist_distortion=wrist_distortion,
            columns=columns,
            rows=rows,
            square_size_m=square_size_m,
        )
        if wrist_serial != config.hardware.camera_serial:
            raise CameraAcceptanceError(
                f"{path}: wrist serial {wrist_serial!r} does not match "
                f"{config.hardware.camera_serial!r}"
            )
        observed = _closest_transform(candidates, result.world_from_camera)
        translation_m, rotation_deg = _transform_distance(
            result.world_from_camera, observed
        )
        validation_errors.append(
            {
                "path": str(path.resolve()),
                "translation_error_m": translation_m,
                "rotation_error_deg": rotation_deg,
            }
        )
    validation_translation_max = max(
        value["translation_error_m"] for value in validation_errors
    )
    validation_rotation_max = max(
        value["rotation_error_deg"] for value in validation_errors
    )
    passed = (
        result.translation_rms_m <= args.max_calibration_translation_rms_m
        and result.rotation_rms_deg <= args.max_calibration_rotation_rms_deg
        and validation_translation_max <= args.max_validation_translation_error_m
        and validation_rotation_max <= args.max_validation_rotation_error_deg
    )
    nominal_translation_m, nominal_rotation_deg = _transform_distance(
        nominal, result.world_from_camera
    )
    return {
        "schema_version": 1,
        "type": "fixed_rgb_camera_calibration",
        "status": "PASS" if passed else "FAIL",
        "camera": camera,
        "reference_camera": {
            "name": config.camera.name,
            "serial": config.hardware.camera_serial,
        },
        "checkerboard": checkerboard,
        "distortion_model": intrinsics_payload["distortion_model"],
        "camera_matrix": top_intrinsics.tolist(),
        "distortion_coefficients": top_distortion.tolist(),
        "camera_to_world": result.world_from_camera.tolist(),
        "nominal_camera_to_world": nominal.tolist(),
        "calibration_sample_count": result.sample_count,
        "validation_sample_count": len(validation_errors),
        "metrics": {
            "calibration_translation_rms_m": result.translation_rms_m,
            "calibration_translation_max_m": result.translation_max_m,
            "calibration_rotation_rms_deg": result.rotation_rms_deg,
            "calibration_rotation_max_deg": result.rotation_max_deg,
            "validation_translation_max_m": validation_translation_max,
            "validation_rotation_max_deg": validation_rotation_max,
            "solved_from_nominal_translation_m": nominal_translation_m,
            "solved_from_nominal_rotation_deg": nominal_rotation_deg,
        },
        "validation": validation_errors,
        "thresholds": {
            "max_calibration_translation_rms_m": args.max_calibration_translation_rms_m,
            "max_calibration_rotation_rms_deg": args.max_calibration_rotation_rms_deg,
            "max_validation_translation_error_m": args.max_validation_translation_error_m,
            "max_validation_rotation_error_deg": args.max_validation_rotation_error_deg,
        },
    }


def _capture_top_pair(args: argparse.Namespace) -> dict[str, Any]:
    intrinsics_payload = json.loads(args.intrinsics.read_text(encoding="utf-8"))
    if (
        not isinstance(intrinsics_payload, dict)
        or intrinsics_payload.get("type") != "fixed_rgb_camera_intrinsics"
        or intrinsics_payload.get("status") != "PASS"
        or not isinstance(intrinsics_payload.get("camera"), dict)
    ):
        raise CameraAcceptanceError("top intrinsics must be a PASS fixed-camera report")
    camera = intrinsics_payload["camera"]
    device = str(camera.get("device", ""))
    serial = str(camera.get("serial", ""))
    width = int(camera.get("width", 0))
    height = int(camera.get("height", 0))
    fps = int(camera.get("fps", 0))
    if not device or not serial or serial not in Path(device).name:
        raise CameraAcceptanceError("top intrinsics are not bound to a stable serial device")
    config = load_config(args.config)
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        capture.release()
        raise CameraAcceptanceError(f"could not open top camera {device}")
    try:
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, fps)
        actual_profile = (
            round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            round(capture.get(cv2.CAP_PROP_FPS)),
        )
        if actual_profile != (width, height, fps):
            raise CameraAcceptanceError(
                f"top camera profile {actual_profile} does not match calibrated "
                f"{(width, height, fps)}"
            )
        for _ in range(args.warmup_frames):
            ok, _ = capture.read()
            if not ok:
                raise CameraAcceptanceError("top camera warmup frame capture failed")
        endpoint = config.bridge
        with BridgeClient(
            args.host or endpoint.host,
            args.port or endpoint.port,
            request_timeout_s=endpoint.request_timeout_s,
            stale_after_s=endpoint.stale_after_s,
        ) as client:
            observation = client.get_observation()
        ok, top_bgr = capture.read()
        top_monotonic_ns = time.monotonic_ns()
        if not ok or top_bgr is None:
            raise CameraAcceptanceError("top camera paired frame capture failed")
    finally:
        capture.release()
    wrist = observation.get("camera_0")
    if not isinstance(wrist, dict):
        raise CameraAcceptanceError("bridge observation does not contain camera_0")
    if (
        wrist.get("name") != config.camera.name
        or wrist.get("serial") != config.hardware.camera_serial
    ):
        raise CameraAcceptanceError("bridge observation is not the configured wrist camera")
    wrist_monotonic_ns = int(wrist.get("frame_monotonic_ns", 0))
    pair_skew_ns = abs(top_monotonic_ns - wrist_monotonic_ns)
    if pair_skew_ns > int(args.max_pair_skew_s * 1e9):
        raise CameraAcceptanceError(
            f"top/wrist frame skew {pair_skew_ns / 1e6:.3f}ms exceeds "
            f"{args.max_pair_skew_s * 1000:.3f}ms"
        )
    top_rgb = cv2.cvtColor(top_bgr, cv2.COLOR_BGR2RGB)
    if top_rgb.shape != (height, width, 3):
        raise CameraAcceptanceError(
            f"top RGB shape {top_rgb.shape} does not match {(height, width, 3)}"
        )
    wrist_images = wrist.get("images")
    wrist_rgb = wrist_images.get("rgb") if isinstance(wrist_images, dict) else None
    if not isinstance(wrist_rgb, np.ndarray) or wrist_rgb.dtype != np.uint8:
        raise CameraAcceptanceError("bridge wrist RGB is missing or invalid")
    wrist_intrinsics = _matrix(
        wrist.get("intrinsics"), (3, 3), "wrist intrinsics"
    )
    wrist_camera_to_world = _rigid_transform(
        wrist.get("camera_to_world"), "wrist camera_to_world"
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output,
        top_rgb=top_rgb,
        top_serial=np.array(serial),
        top_device=np.array(device),
        top_monotonic_ns=np.array(top_monotonic_ns, dtype=np.int64),
        wrist_rgb=wrist_rgb,
        wrist_intrinsics=wrist_intrinsics,
        wrist_camera_to_world=wrist_camera_to_world,
        wrist_serial=np.array(str(wrist["serial"])),
        wrist_monotonic_ns=np.array(wrist_monotonic_ns, dtype=np.int64),
        pair_skew_ns=np.array(pair_skew_ns, dtype=np.int64),
    )
    return {
        "status": "CAPTURED",
        "path": str(args.output.resolve()),
        "top_serial": serial,
        "wrist_serial": str(wrist["serial"]),
        "pair_skew_ms": pair_skew_ns / 1e6,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Capture and evaluate P3 D405 acceptance data")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    subparsers = parser.add_subparsers(dest="command", required=True)

    capture_parser = subparsers.add_parser("capture")
    capture_parser.add_argument("--output", type=Path, required=True)
    capture_parser.add_argument("--frames", type=int, default=30)
    capture_parser.add_argument("--interval-s", type=float, default=0.1)
    capture_parser.add_argument("--host")
    capture_parser.add_argument("--port", type=int)

    points_parser = subparsers.add_parser("evaluate-points")
    points_parser.add_argument("--capture", type=Path, required=True)
    points_parser.add_argument("--points", type=Path, required=True)
    points_parser.add_argument("--frame-index", type=int, default=0)
    points_parser.add_argument("--max-world-error-m", type=float, default=0.015)
    points_parser.add_argument("--max-reprojection-error-px", type=float, default=3.0)

    plane_parser = subparsers.add_parser("evaluate-plane")
    plane_parser.add_argument("--capture", type=Path, required=True)
    plane_parser.add_argument("--frame-index", type=int, default=0)
    plane_parser.add_argument("--roi", type=int, nargs=4, required=True, metavar=("X0", "Y0", "X1", "Y1"))
    plane_parser.add_argument("--expected-world-z-m", type=float, required=True)
    plane_parser.add_argument("--max-plane-rms-m", type=float, default=0.005)
    plane_parser.add_argument("--max-normal-error-deg", type=float, default=2.0)
    plane_parser.add_argument("--max-world-z-error-m", type=float, default=0.015)

    jitter_parser = subparsers.add_parser("evaluate-jitter")
    jitter_parser.add_argument("--capture", type=Path, required=True)
    jitter_parser.add_argument("--max-translation-jitter-m", type=float, default=0.002)
    jitter_parser.add_argument("--max-rotation-jitter-deg", type=float, default=0.5)

    export_parser = subparsers.add_parser("export-rgb")
    export_parser.add_argument("--capture", type=Path, required=True)
    export_parser.add_argument("--output", type=Path, required=True)
    export_parser.add_argument("--frame-index", type=int, default=0)

    checkerboard_parser = subparsers.add_parser("calibrate-checkerboard")
    checkerboard_parser.add_argument("--captures", type=Path, nargs="+", required=True)
    checkerboard_parser.add_argument("--output", type=Path, required=True)
    checkerboard_parser.add_argument("--columns", type=int, default=9)
    checkerboard_parser.add_argument("--rows", type=int, default=7)
    checkerboard_parser.add_argument("--square-size-m", type=float, required=True)
    checkerboard_parser.add_argument("--frame-index", type=int)
    checkerboard_parser.add_argument(
        "--distortion-model", default="inverse_brown_conrady"
    )
    checkerboard_parser.add_argument(
        "--distortion-coefficients", type=float, nargs=5, required=True
    )
    checkerboard_parser.add_argument(
        "--max-reprojection-error-px", type=float, default=3.0
    )
    checkerboard_parser.add_argument(
        "--max-target-translation-rms-m", type=float, default=0.015
    )
    checkerboard_parser.add_argument(
        "--max-target-rotation-rms-deg", type=float, default=2.0
    )

    held_out_parser = subparsers.add_parser("evaluate-checkerboard-pose")
    held_out_parser.add_argument("--capture", type=Path, required=True)
    held_out_parser.add_argument("--reference-report", type=Path, required=True)
    held_out_parser.add_argument("--output", type=Path, required=True)
    held_out_parser.add_argument("--frame-index", type=int)
    held_out_parser.add_argument("--max-translation-error-m", type=float, default=0.015)
    held_out_parser.add_argument("--max-rotation-error-deg", type=float, default=2.0)

    top_intrinsics_parser = subparsers.add_parser("calibrate-top-intrinsics")
    top_intrinsics_parser.add_argument("--images", type=Path, nargs="+", required=True)
    top_intrinsics_parser.add_argument(
        "--validation-images", type=Path, nargs="+", required=True
    )
    top_intrinsics_parser.add_argument("--device", required=True)
    top_intrinsics_parser.add_argument("--serial", required=True)
    top_intrinsics_parser.add_argument("--width", type=int, required=True)
    top_intrinsics_parser.add_argument("--height", type=int, required=True)
    top_intrinsics_parser.add_argument("--fps", type=int, required=True)
    top_intrinsics_parser.add_argument("--columns", type=int, default=9)
    top_intrinsics_parser.add_argument("--rows", type=int, default=7)
    top_intrinsics_parser.add_argument("--square-size-m", type=float, required=True)
    top_intrinsics_parser.add_argument("--output", type=Path, required=True)
    top_intrinsics_parser.add_argument(
        "--max-calibration-rms-px", type=float, default=1.0
    )
    top_intrinsics_parser.add_argument(
        "--max-validation-rms-px", type=float, default=1.0
    )

    top_extrinsics_parser = subparsers.add_parser("calibrate-top-extrinsics")
    top_extrinsics_parser.add_argument(
        "--captures", type=Path, nargs="+", required=True
    )
    top_extrinsics_parser.add_argument(
        "--validation-captures", type=Path, nargs="+", required=True
    )
    top_extrinsics_parser.add_argument("--intrinsics", type=Path, required=True)
    top_extrinsics_parser.add_argument(
        "--wrist-distortion-model",
        choices=("inverse_brown_conrady", "none"),
        default="inverse_brown_conrady",
    )
    top_extrinsics_parser.add_argument(
        "--wrist-distortion-coefficients", type=float, nargs=5, required=True
    )
    top_extrinsics_parser.add_argument("--output", type=Path, required=True)
    top_extrinsics_parser.add_argument(
        "--max-calibration-translation-rms-m", type=float, default=0.015
    )
    top_extrinsics_parser.add_argument(
        "--max-calibration-rotation-rms-deg", type=float, default=2.0
    )
    top_extrinsics_parser.add_argument(
        "--max-validation-translation-error-m", type=float, default=0.015
    )
    top_extrinsics_parser.add_argument(
        "--max-validation-rotation-error-deg", type=float, default=2.0
    )

    top_pair_parser = subparsers.add_parser("capture-top-pair")
    top_pair_parser.add_argument("--intrinsics", type=Path, required=True)
    top_pair_parser.add_argument("--output", type=Path, required=True)
    top_pair_parser.add_argument("--host")
    top_pair_parser.add_argument("--port", type=int)
    top_pair_parser.add_argument("--warmup-frames", type=int, default=10)
    top_pair_parser.add_argument("--max-pair-skew-s", type=float, default=0.1)

    args = parser.parse_args(argv)
    if args.command == "capture":
        if args.frames <= 0 or args.interval_s < 0:
            parser.error("--frames must be positive and --interval-s non-negative")
        config = load_config(args.config)
        endpoint = config.bridge
        observations = []
        with BridgeClient(
            args.host or endpoint.host,
            args.port or endpoint.port,
            request_timeout_s=endpoint.request_timeout_s,
            stale_after_s=endpoint.stale_after_s,
        ) as client:
            for index in range(args.frames):
                observations.append(client.get_observation())
                if index + 1 < args.frames:
                    time.sleep(args.interval_s)
        _save_capture(args.output, observations)
        with np.load(args.output, allow_pickle=False) as saved:
            report = {
                "status": "CAPTURED",
                "path": str(args.output.resolve()),
                "serial": str(saved["serial"]),
                **_pose_jitter(saved["camera_to_world"]),
                "max_joint_time_delta_ms": float(saved["joint_time_delta_ns"].max() / 1e6),
            }
        print(json.dumps(report, sort_keys=True))
        return 0

    if args.command == "export-rgb":
        from PIL import Image

        with np.load(args.capture, allow_pickle=False) as capture:
            frame_count = len(capture["rgb"])
            if not 0 <= args.frame_index < frame_count:
                raise CameraAcceptanceError(
                    f"frame index {args.frame_index} is outside 0..{frame_count - 1}"
                )
            rgb = capture["rgb"][args.frame_index]
        args.output.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(rgb).save(args.output)
        print(
            json.dumps(
                {"status": "EXPORTED", "path": str(args.output.resolve())},
                sort_keys=True,
            )
        )
        return 0

    if args.command == "calibrate-checkerboard":
        report = _calibrate_checkerboard_captures(args)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(
            json.dumps(
                {
                    "status": report["status"],
                    "path": str(args.output.resolve()),
                    **report["metrics"],
                },
                sort_keys=True,
            )
        )
        return 0 if report["status"] == "PASS" else 1

    if args.command == "evaluate-checkerboard-pose":
        report = _evaluate_checkerboard_pose_capture(args)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(
            json.dumps(
                {
                    "status": report["status"],
                    "path": str(args.output.resolve()),
                    **report["metrics"],
                },
                sort_keys=True,
            )
        )
        return 0 if report["status"] == "PASS" else 1

    if args.command == "calibrate-top-intrinsics":
        if args.width <= 0 or args.height <= 0 or args.fps <= 0:
            parser.error("top-camera width, height, and fps must be positive")
        report = _calibrate_top_intrinsics_images(args)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(
            json.dumps(
                {
                    "status": report["status"],
                    "path": str(args.output.resolve()),
                    **report["metrics"],
                },
                sort_keys=True,
            )
        )
        return 0 if report["status"] == "PASS" else 1

    if args.command == "calibrate-top-extrinsics":
        report = _calibrate_top_extrinsics(args)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(
            json.dumps(
                {
                    "status": report["status"],
                    "path": str(args.output.resolve()),
                    **report["metrics"],
                },
                sort_keys=True,
            )
        )
        return 0 if report["status"] == "PASS" else 1

    if args.command == "capture-top-pair":
        if args.warmup_frames < 0 or args.max_pair_skew_s <= 0:
            parser.error("--warmup-frames must be non-negative and skew must be positive")
        report = _capture_top_pair(args)
        print(json.dumps(report, sort_keys=True))
        return 0

    if args.command == "evaluate-jitter":
        with np.load(args.capture, allow_pickle=False) as capture:
            metrics = _pose_jitter(capture["camera_to_world"])
        passed = (
            metrics["max_translation_jitter_m"] <= args.max_translation_jitter_m
            and metrics["max_rotation_jitter_deg"] <= args.max_rotation_jitter_deg
        )
    elif args.command == "evaluate-points":
        depth, K, pose = _load_capture(args.capture, args.frame_index)
        raw = json.loads(args.points.read_text(encoding="utf-8"))
        points = raw.get("points") if isinstance(raw, dict) else None
        if not isinstance(points, list):
            raise CameraAcceptanceError("points JSON must contain a 'points' list")
        pixels = np.array([[point["u"], point["v"]] for point in points], dtype=np.float64)
        world = np.array([point["world_xyz_m"] for point in points], dtype=np.float64)
        metrics = evaluate_correspondences(pixels, world, depth, K, pose)
        passed = (
            metrics["max_world_error_m"] <= args.max_world_error_m
            and metrics["max_reprojection_error_px"] <= args.max_reprojection_error_px
        )
    else:
        depth, K, pose = _load_capture(args.capture, args.frame_index)
        metrics = evaluate_table_plane(
            depth,
            K,
            pose,
            roi=tuple(args.roi),
            expected_world_z_m=args.expected_world_z_m,
        )
        passed = (
            metrics["plane_rms_m"] <= args.max_plane_rms_m
            and metrics["normal_error_deg"] <= args.max_normal_error_deg
            and metrics["world_z_error_m"] <= args.max_world_z_error_m
        )
    print(json.dumps({"status": "PASS" if passed else "FAIL", **metrics}, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
