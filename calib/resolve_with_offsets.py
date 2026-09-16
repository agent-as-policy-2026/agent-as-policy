"""Offline LEFT-arm hand-eye re-solve with the fitted joint-zero corrections.

The 11 captures in calib/out/left_pose*.npz were recorded with ZERO installed
joint offsets; the 2026-08-31 fit (calib/fit_joint_offsets.py) found stable
zero corrections on joints 3/4/5:

    q_true = q_measured + DELTA,  DELTA_DEG = [0, 0, -0.8135, -1.9575, +0.3592, 0]

(yam_v1.yml install convention reported = raw - offset, so the installed
can_follower_l row is -DELTA_DEG).  This script re-solves the eye-in-hand
calibration on those SAME captures with DELTA applied to the stored measured
joints, mirroring the official CLI path exactly:

  * detection/undistortion: fit_joint_offsets.load_capture_entries (identical
    to _calibrate_checkerboard_captures: findChessboardCornersSB + factory
    inverse_brown_conrady undistortion, then zero distortion downstream);
  * FK: left_handeye_common.LeftArmFK (combined YAM + linear_4310 model,
    body 'gripper'), T_world_flange = FK(q_measured + DELTA);
  * solve: agp_yam_bridge.camera_acceptance.calibrate_checkerboard_records
    with nominal_world_from_camera = T_world_flange @ CAD nominal.

The CAD nominal is built from left_handeye_common's constants (NOT re-read
from the station XML) so this script keeps solving against CAD even after the
measured transform is installed into the station model.

Run under the hardware-bridge uv project:
    cd hardware-bridge
    uv run --locked python ../calib/resolve_with_offsets.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from left_handeye_common import (  # noqa: E402
    CAD_FLANGE_CAMERA_WXYZ,
    CAD_FLANGE_CAMERA_XYZ,
    INTRINSICS_JSON,
    OUT_DIR,
    LeftArmFK,
    load_intrinsics_json,
)
from fit_joint_offsets import load_capture_entries  # noqa: E402

# Fitted correction (rad applied below): q_true = q_measured + DELTA.
# 2026-08-31 fixed-board 11-pose fit, joints 3/4/5 (calib/fit_joint_offsets.py).
DELTA_DEG = np.array([0.0, 0.0, -0.813493705088, -1.9574947249, 0.359152777503, 0.0])


def cad_flange_from_camera() -> np.ndarray:
    """CAD T_leftgripper_leftcamera from left_handeye_common's constants."""
    import mujoco

    quat = CAD_FLANGE_CAMERA_WXYZ / np.linalg.norm(CAD_FLANGE_CAMERA_WXYZ)
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, quat)
    transform = np.eye(4)
    transform[:3, :3] = rotation.reshape(3, 3)
    transform[:3, 3] = CAD_FLANGE_CAMERA_XYZ
    return transform


def matrix_to_wxyz(rotation: np.ndarray) -> np.ndarray:
    import mujoco

    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, np.asarray(rotation, dtype=np.float64).reshape(-1))
    if quat[0] < 0:
        quat = -quat
    return quat


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Re-solve the LEFT hand-eye with the fitted joint-zero "
        "corrections applied to the stored measured joints (offline)"
    )
    parser.add_argument(
        "--captures",
        type=Path,
        nargs="+",
        default=sorted(OUT_DIR.glob("left_pose*.npz")),
        help="left_pose*.npz files (default: calib/out/left_pose*.npz)",
    )
    parser.add_argument("--intrinsics-json", type=Path, default=INTRINSICS_JSON)
    parser.add_argument("--columns", type=int, default=9)
    parser.add_argument("--rows", type=int, default=7)
    parser.add_argument("--square-size-m", type=float, default=0.022)
    parser.add_argument("--frame-index", type=int, default=None)
    parser.add_argument(
        "--output",
        type=Path,
        default=OUT_DIR / "left_solved_flange_from_camera.json",
    )
    args = parser.parse_args()
    if len(args.captures) < 3:
        parser.error("the hand-eye solve needs at least 3 captures")

    from agp_yam_bridge.camera_acceptance import (
        CheckerboardRecord,
        calibrate_checkerboard_records,
    )

    payload = load_intrinsics_json(args.intrinsics_json)
    distortion_model = payload["distortion_model"]
    distortion_coefficients = np.asarray(
        payload["distortion_coefficients"], dtype=np.float64
    )
    print(f"distortion: {distortion_model} {distortion_coefficients.tolist()}")
    print(f"joint corrections DELTA (deg, q_true = q_measured + DELTA): "
          f"{DELTA_DEG.tolist()}")

    fk = LeftArmFK()
    nominal = cad_flange_from_camera()
    delta = np.radians(DELTA_DEG)

    entries = load_capture_entries(
        [Path(p) for p in args.captures],
        frame_index=args.frame_index,
        columns=args.columns,
        rows=args.rows,
        distortion_model=distortion_model,
        distortion_coefficients=distortion_coefficients,
    )
    records = []
    for entry in entries:
        world_from_flange = fk.base_from_gripper(entry["q6"] + delta)
        records.append(
            CheckerboardRecord(
                label=entry["label"],
                pixels_uv=entry["pixels"],
                intrinsics=entry["intrinsics"],
                nominal_world_from_camera=world_from_flange @ nominal,
            )
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
    solved = hand_eye.gripper_from_camera
    solved_quat = matrix_to_wxyz(solved[:3, :3])
    from_nominal = np.linalg.inv(nominal) @ solved
    from_nominal_mm = float(np.linalg.norm(from_nominal[:3, 3])) * 1000.0
    from_nominal_deg = float(
        np.degrees(Rotation.from_matrix(from_nominal[:3, :3]).magnitude())
    )

    np.set_printoptions(suppress=True, precision=9)
    print()
    print(f"captures used: {len(records)} "
          f"(reversed corner order in {result.reversed_pose_count})")
    print("solved T_leftgripper_leftcamera (at corrected joints):")
    print(str(solved))
    print(f"  translation (mm): {(solved[:3, 3] * 1000.0).tolist()}")
    print(f"  quat (wxyz):      {solved_quat.tolist()}")
    print(
        "fixed-target consistency: "
        f"{hand_eye.target_translation_rms_m * 1000:.3f} mm RMS / "
        f"{hand_eye.target_rotation_rms_deg:.4f} deg RMS "
        f"(max {hand_eye.target_translation_max_m * 1000:.3f} mm / "
        f"{hand_eye.target_rotation_max_deg:.4f} deg)"
    )
    print(
        f"reprojection: {result.rms_reprojection_error_px:.4f} px RMS / "
        f"{result.max_reprojection_error_px:.4f} px max"
    )
    print(f"solved_from_nominal(CAD): {from_nominal_mm:.3f} mm / "
          f"{from_nominal_deg:.4f} deg")

    report = {
        "delta_deg_q_true_minus_q_measured": DELTA_DEG.tolist(),
        "captures": [str(Path(p).resolve()) for p in args.captures],
        "frame_index": args.frame_index,
        "checkerboard": {
            "columns": args.columns,
            "rows": args.rows,
            "square_size_m": args.square_size_m,
            "reversed_pose_count": result.reversed_pose_count,
        },
        "distortion": {
            "model": distortion_model,
            "coefficients": distortion_coefficients.tolist(),
        },
        "nominal_flange_from_camera_cad": nominal.tolist(),
        "solved_flange_from_camera": solved.tolist(),
        "solved_translation_m": solved[:3, 3].tolist(),
        "solved_quat_wxyz": solved_quat.tolist(),
        "solved_world_from_target": hand_eye.world_from_target.tolist(),
        "metrics": {
            "pose_count": hand_eye.pose_count,
            "target_translation_rms_m": hand_eye.target_translation_rms_m,
            "target_translation_max_m": hand_eye.target_translation_max_m,
            "target_rotation_rms_deg": hand_eye.target_rotation_rms_deg,
            "target_rotation_max_deg": hand_eye.target_rotation_max_deg,
            "rms_reprojection_error_px": result.rms_reprojection_error_px,
            "max_reprojection_error_px": result.max_reprojection_error_px,
            "solved_from_nominal_translation_m": from_nominal_mm / 1000.0,
            "solved_from_nominal_rotation_deg": from_nominal_deg,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
