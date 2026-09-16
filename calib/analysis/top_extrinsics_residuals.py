#!/usr/bin/env python3
"""Per-pair residual analysis of the LEFT-station top-camera (BRIO 178B0DAE)
extrinsics solve.

ANALYSIS ONLY.  Reads
    calib/out/top_pairs/pairNN.npz
    calib/out/top_brio_178B0DAE_intrinsics.json
    calib/out/top_brio_178B0DAE_calibration.json      (the PASS solve under test)
    calib/out/left_d405_intrinsics.json               (wrist distortion)
    hardware-bridge/config/left_calib.yaml            (nominal tie-break, wrist serial)
and writes ONLY under calib/analysis/out/.  It never touches calib/out or
left_arm.yaml.

Every per-pair number is produced by the solver's OWN helpers imported from
agp_yam_bridge.camera_acceptance, so the definitions are the solver's:

  candidates_i   _paired_capture_candidates: two-scale findChessboardCornersSB on
                 both images, RealSense inverse_brown_conrady undistortion of the
                 wrist corners, IPPE PnP for both corner orders on both cameras,
                 -> 4 candidate world_from_top = world_from_wrist @ wrist_from_board
                                                 @ inv(top_from_board)
  solve          _select_consistent_transforms: for each of the 4 candidates of the
                 first calibration pair as seed, 5 iterations of {pick the closest
                 candidate per pair to the reference; average (mean translation,
                 chordal mean rotation); reference := average}; keep the seed whose
                 average has the smallest  trans_rms + 0.1*rad(rot_rms)
                 + 1e-4*(dist to MJCF nominal).
  residual r_i   _transform_distance(T_solved, _closest_transform(candidates_i,
                 T_solved)) = (|t_i - t_solved| [m], |angle(R_solved^T R_i)| [deg]).
                 For calibration pairs this is exactly the per-sample error that
                 solve_fixed_camera_pose reduces to the reported RMS/max (the
                 script asserts the RMS/max/pose reproduce the JSON to 1e-9); for
                 held-out pairs it is exactly the solver's validation error.

Run (from the hardware-bridge uv project, as solve_top.sh does):
    cd hardware-bridge && \
    uv run --locked python ../calib/analysis/top_extrinsics_residuals.py
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from scipy import stats
from scipy.spatial.transform import Rotation

import agp_yam_bridge.camera_acceptance as ca
from agp_yam_bridge.config import load_config

ROOT = Path(__file__).resolve().parents[2]   # repo root (this file is calib/analysis/)
CALIB = ROOT / "calib"
HB = ROOT / "hardware-bridge"
DEFAULT_OUT = CALIB / "analysis" / "out"

# solver defaults (camera_acceptance.main argparse)
THRESHOLDS = {
    "max_calibration_translation_rms_m": 0.015,
    "max_calibration_rotation_rms_deg": 2.0,
    "max_validation_translation_error_m": 0.015,
    "max_validation_rotation_error_deg": 2.0,
}


# --------------------------------------------------------------------------- io
def _mm(x: float) -> float:
    return 1000.0 * float(x)


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, (np.floating, float)):
        return None if not math.isfinite(float(value)) else float(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    return value


class Tee:
    def __init__(self, path: Path):
        self._lines: list[str] = []
        self._path = path

    def __call__(self, *parts: Any) -> None:
        line = " ".join(str(p) for p in parts)
        print(line)
        self._lines.append(line)

    def flush(self) -> None:
        self._path.write_text("\n".join(self._lines) + "\n", encoding="utf-8")


# ------------------------------------------------------------------- geometry
def board_centre_target(columns: int, rows: int, square: float) -> np.ndarray:
    # object points are mgrid[0:columns, 0:rows] * square (x along columns)
    return np.array([(columns - 1) * square / 2.0, (rows - 1) * square / 2.0, 0.0])


def tilt_deg(camera_from_target: np.ndarray) -> float:
    """Angle between the camera optical axis and the board normal."""
    return float(np.degrees(np.arccos(np.clip(abs(camera_from_target[2, 2]), 0.0, 1.0))))


def px_per_square(pixels: np.ndarray, columns: int, rows: int) -> float:
    grid = pixels.reshape(rows, columns, 2)
    dx = np.linalg.norm(np.diff(grid, axis=1), axis=2)
    dy = np.linalg.norm(np.diff(grid, axis=0), axis=2)
    return float(np.concatenate([dx.ravel(), dy.ravel()]).mean())


# ---------------------------------------------------------------- pair loading
def load_pair(
    path: Path,
    *,
    top_K: np.ndarray,
    top_dist: np.ndarray,
    wrist_model: str,
    wrist_dist: np.ndarray,
    columns: int,
    rows: int,
    square: float,
) -> dict[str, Any]:
    # _paired_capture_candidates unrolled step-for-step with the SAME helpers, in
    # the SAME call order (top detect, wrist detect, undistort, PnP top, PnP
    # wrist), so we can keep the intermediate detections / single-camera poses
    # for geometry.  It is unrolled rather than called because OpenCV's
    # findChessboardCornersSB(EXHAUSTIVE|ACCURACY) returns a slightly different
    # corner set (<= 0.16 px here) on the very FIRST call of a process than on
    # every later call (deterministic across processes); the solver's fresh
    # process therefore used the "cold" detection for pair01's top image and
    # "warm" detections for everything else.  Loading pairs sorted, top image
    # first, reproduces that sequence exactly (asserted against the JSON below).
    with np.load(path, allow_pickle=False) as cap:
        required = {"top_rgb", "wrist_rgb", "wrist_intrinsics", "wrist_camera_to_world", "wrist_serial"}
        if not required.issubset(cap.files):
            raise RuntimeError(f"{path}: missing {sorted(required - set(cap.files))}")
        top_rgb = cap["top_rgb"]
        wrist_rgb = cap["wrist_rgb"]
        wrist_K = ca._matrix(cap["wrist_intrinsics"], (3, 3), "wrist_intrinsics")
        world_from_wrist = ca._rigid_transform(cap["wrist_camera_to_world"], "wrist_camera_to_world")
        wrist_serial = str(cap["wrist_serial"])
        skew_ns = int(cap["pair_skew_ns"]) if "pair_skew_ns" in cap.files else -1
    top_px = ca._detect_checkerboard(top_rgb, columns=columns, rows=rows, label="top")
    wrist_px_raw = ca._detect_checkerboard(wrist_rgb, columns=columns, rows=rows, label="wrist")
    if wrist_model == "inverse_brown_conrady":
        wrist_px = ca.undistort_realsense_pixels(
            wrist_px_raw,
            wrist_K,
            width=wrist_rgb.shape[1],
            height=wrist_rgb.shape[0],
            distortion_model=wrist_model,
            distortion_coefficients=wrist_dist,
        )
    else:
        wrist_px = wrist_px_raw
    zero5 = np.zeros(5)
    top_poses = ca._checkerboard_pose_candidates(
        top_px, top_K, top_dist, columns=columns, rows=rows, square_size_m=square
    )
    wrist_poses = ca._checkerboard_pose_candidates(
        wrist_px, wrist_K, zero5, columns=columns, rows=rows, square_size_m=square
    )
    candidates = [
        world_from_wrist @ w_from_t @ np.linalg.inv(t_from_t)
        for w_from_t in wrist_poses
        for t_from_t in top_poses
    ]
    return {
        "name": path.stem,
        "path": path,
        "skew_ns": skew_ns,
        "wrist_serial": wrist_serial,
        "candidates": candidates,  # index = 2*wrist_reversed + top_reversed, as in the solver
        "top_px": top_px,
        "wrist_px_raw": wrist_px_raw,
        "wrist_px": wrist_px,
        "top_poses": top_poses,  # (normal order, reversed order)
        "wrist_poses": wrist_poses,
        "world_from_wrist": world_from_wrist,
        "wrist_K": wrist_K,
        "top_rgb": top_rgb,
    }


# ----------------------------------------------------------------------- solve
def solve_split(
    pairs: list[dict[str, Any]],
    calib_idx: list[int],
    val_idx: list[int],
    nominal: np.ndarray,
) -> dict[str, Any]:
    """Run the solver on calib_idx and score EVERY pair against the result."""
    result = ca._select_consistent_transforms([pairs[i]["candidates"] for i in calib_idx], nominal)
    T = result.world_from_camera
    per_pair = []
    for p in pairs:
        obs = ca._closest_transform(p["candidates"], T)
        k = next(i for i, c in enumerate(p["candidates"]) if c is obs)
        tr_m, rot_deg = ca._transform_distance(T, obs)
        per_pair.append(
            {"translation_m": tr_m, "rotation_deg": rot_deg, "candidate": k, "observed": obs}
        )
    cal_t = np.array([per_pair[i]["translation_m"] for i in calib_idx])
    cal_r = np.array([per_pair[i]["rotation_deg"] for i in calib_idx])
    # Consistency check: the solver's RMS/max come from the candidates selected
    # against the PREVIOUS iterate; if the fixed point converged they equal ours.
    converged = bool(
        np.isclose(np.sqrt(np.mean(cal_t**2)), result.translation_rms_m, atol=1e-12)
        and np.isclose(cal_t.max(), result.translation_max_m, atol=1e-12)
        and np.isclose(np.sqrt(np.mean(cal_r**2)), result.rotation_rms_deg, atol=1e-9)
    )
    val_t = [per_pair[i]["translation_m"] for i in val_idx]
    val_r = [per_pair[i]["rotation_deg"] for i in val_idx]
    val_t_max = max(val_t) if val_t else float("nan")
    val_r_max = max(val_r) if val_r else float("nan")
    passed = (
        result.translation_rms_m <= THRESHOLDS["max_calibration_translation_rms_m"]
        and result.rotation_rms_deg <= THRESHOLDS["max_calibration_rotation_rms_deg"]
        and (not val_t or val_t_max <= THRESHOLDS["max_validation_translation_error_m"])
        and (not val_r or val_r_max <= THRESHOLDS["max_validation_rotation_error_deg"])
    )
    nom_t, nom_r = ca._transform_distance(nominal, T)
    return {
        "result": result,
        "T": T,
        "calib_idx": list(calib_idx),
        "val_idx": list(val_idx),
        "per_pair": per_pair,
        "fixed_point_converged": converged,
        "metrics": {
            "calibration_translation_rms_m": result.translation_rms_m,
            "calibration_translation_max_m": result.translation_max_m,
            "calibration_rotation_rms_deg": result.rotation_rms_deg,
            "calibration_rotation_max_deg": result.rotation_max_deg,
            "validation_translation_max_m": val_t_max,
            "validation_rotation_max_deg": val_r_max,
            "solved_from_nominal_translation_m": nom_t,
            "solved_from_nominal_rotation_deg": nom_r,
        },
        "status": "PASS" if passed else "FAIL",
    }


def solver_style_report(
    sol: dict[str, Any],
    pairs: list[dict[str, Any]],
    template: dict[str, Any],
    nominal: np.ndarray,
    label: str,
    note: str,
) -> dict[str, Any]:
    """Same schema as the solver's fixed_rgb_camera_calibration JSON."""
    rep = {k: template[k] for k in ("schema_version", "type", "camera", "reference_camera",
                                    "checkerboard", "distortion_model", "camera_matrix",
                                    "distortion_coefficients")}
    rep["status"] = sol["status"]
    rep["camera_to_world"] = sol["T"].tolist()
    rep["nominal_camera_to_world"] = nominal.tolist()
    rep["calibration_sample_count"] = len(sol["calib_idx"])
    rep["validation_sample_count"] = len(sol["val_idx"])
    rep["metrics"] = _jsonable(sol["metrics"])
    rep["validation"] = [
        {
            "path": str(pairs[i]["path"].resolve()),
            "rotation_error_deg": sol["per_pair"][i]["rotation_deg"],
            "translation_error_m": sol["per_pair"][i]["translation_m"],
        }
        for i in sol["val_idx"]
    ]
    rep["thresholds"] = THRESHOLDS
    rep["analysis_variant"] = {
        "label": label,
        "note": "ANALYSIS VARIANT written by calib/analysis/top_extrinsics_residuals.py; "
                "NOT installed. " + note,
        "calibration_pairs": [pairs[i]["name"] for i in sol["calib_idx"]],
        "validation_pairs": [pairs[i]["name"] for i in sol["val_idx"]],
    }
    return rep


def corr(x: np.ndarray, y: np.ndarray) -> dict[str, float]:
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    pr, pp = stats.pearsonr(x, y)
    sr, sp = stats.spearmanr(x, y)
    return {"pearson_r": float(pr), "pearson_p": float(pp), "spearman_rho": float(sr), "spearman_p": float(sp)}


# ------------------------------------------------------------------------ main
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pairs-dir", type=Path, default=CALIB / "out" / "top_pairs")
    ap.add_argument("--intrinsics", type=Path, default=CALIB / "out" / "top_brio_178B0DAE_intrinsics.json")
    ap.add_argument("--calibration", type=Path, default=CALIB / "out" / "top_brio_178B0DAE_calibration.json")
    ap.add_argument("--wrist-intrinsics", type=Path, default=CALIB / "out" / "left_d405_intrinsics.json")
    ap.add_argument("--config", type=Path, default=HB / "config" / "left_calib.yaml")
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--outlier-factor", type=float, default=2.0, help="outlier if residual > factor * median")
    args = ap.parse_args(argv)

    out_dir: Path = args.out_dir
    if CALIB / "analysis" not in out_dir.parents and out_dir != CALIB / "analysis":
        raise SystemExit(f"refusing to write outside {CALIB / 'analysis'}: {out_dir}")
    out_dir.mkdir(parents=True, exist_ok=True)
    say = Tee(out_dir / "report.txt")

    # ------------------------------------------------------------- inputs
    intr = json.loads(args.intrinsics.read_text())
    orig = json.loads(args.calibration.read_text())
    wrist_json = json.loads(args.wrist_intrinsics.read_text())
    top_K = np.asarray(intr["camera_matrix"], float)
    top_dist = np.asarray(intr["distortion_coefficients"], float)
    columns = int(intr["checkerboard"]["columns"])
    rows = int(intr["checkerboard"]["rows"])
    square = float(intr["checkerboard"]["square_size_m"])
    wrist_model = wrist_json["distortion_model"]
    wrist_dist = np.asarray(wrist_json["distortion_coefficients"], float)
    T_orig = np.asarray(orig["camera_to_world"], float)
    cfg = load_config(args.config)
    nominal = ca._station_world_from_top_camera(cfg)
    if not np.allclose(nominal, np.asarray(orig["nominal_camera_to_world"]), atol=1e-9):
        say("WARNING: MJCF nominal differs from the one stored in the original JSON")

    paths = sorted(args.pairs_dir.glob("pair*.npz"))
    pairs = [
        load_pair(p, top_K=top_K, top_dist=top_dist, wrist_model=wrist_model,
                  wrist_dist=wrist_dist, columns=columns, rows=rows, square=square)
        for p in paths
    ]
    n = len(pairs)
    names = [p["name"] for p in pairs]
    serials = {p["wrist_serial"] for p in pairs}
    assert serials == {cfg.hardware.camera_serial}, serials

    # solve_top.sh split: sorted by filename, zero-based index % 4 == 3 held out
    orig_val = [i for i in range(n) if i % 4 == 3]
    orig_cal = [i for i in range(n) if i % 4 != 3]

    say("=" * 100)
    say("TOP-CAMERA EXTRINSICS RESIDUAL ANALYSIS  (BRIO 178B0DAE, world = left_base)")
    say("=" * 100)
    say(f"pairs: {n}  ({args.pairs_dir})")
    say(f"board: {columns}x{rows} inner corners, {square*1000:.1f} mm squares "
        f"-> {(columns-1)*square*1000:.0f} x {(rows-1)*square*1000:.0f} mm inner extent")
    say(f"top K: fx={top_K[0,0]:.2f} fy={top_K[1,1]:.2f} cx={top_K[0,2]:.2f} cy={top_K[1,2]:.2f}; "
        f"intrinsics RMS {intr['metrics']['calibration_rms_reprojection_error_px']:.3f} px")
    say(f"wrist distortion: {wrist_model} {np.array2string(wrist_dist, precision=5)}")
    say(f"original split: calib={[names[i] for i in orig_cal]}")
    say(f"                held-out={[names[i] for i in orig_val]}")

    # ------------------------------------------------- (c) reproduce original
    C = solve_split(pairs, orig_cal, orig_val, nominal)
    dT_m, dT_deg = ca._transform_distance(T_orig, C["T"])
    say("")
    say("-" * 100)
    say("(c) ORIGINAL SOLVE REPRODUCTION")
    say("-" * 100)
    say(f"reimplemented-vs-JSON camera_to_world: {dT_m*1e6:.3f} um, {dT_deg*3600:.4f} arcsec  "
        f"(allclose 1e-9: {np.allclose(T_orig, C['T'], atol=1e-9)})")
    for k, v in C["metrics"].items():
        say(f"  {k:42s} mine={v:.9f}  json={orig['metrics'][k]:.9f}  d={v-orig['metrics'][k]:+.2e}")
    say(f"  fixed point converged (closest-to-final == closest-to-previous): {C['fixed_point_converged']}")
    say(f"  status: {C['status']} (json {orig['status']})")
    if not np.allclose(T_orig, C["T"], atol=1e-9):
        say("ERROR: could not reproduce the original solve; per-pair numbers below are NOT the solver's")
    # Reproducibility note: OpenCV's first findChessboardCornersSB call of a process
    # differs slightly from later calls; the solver's run (and this script) detected
    # pair01's top image cold.  Quantify what an all-warm run would have produced.
    p0 = pairs[0]
    warm_px = ca._detect_checkerboard(p0["top_rgb"], columns=columns, rows=rows, label="top-warm")
    warm_top_poses = ca._checkerboard_pose_candidates(
        warm_px, top_K, top_dist, columns=columns, rows=rows, square_size_m=square)
    warm_cands = [p0["world_from_wrist"] @ w @ np.linalg.inv(t) for w in p0["wrist_poses"] for t in warm_top_poses]
    pairs_warm = [dict(p0, top_px=warm_px, top_poses=warm_top_poses, candidates=warm_cands)] + pairs[1:]
    Cw = solve_split(pairs_warm, orig_cal, orig_val, nominal)
    w_m, w_deg = ca._transform_distance(C["T"], Cw["T"])
    say(f"  reproducibility: {p0['name']} top corners cold-vs-warm detection max |dpx| = {np.abs(warm_px - p0['top_px']).max():.3f}; "
        f"all-warm re-solve moves the camera by {w_m*1000:.3f} mm / {w_deg:.4f} deg, "
        f"cal rms {_mm(Cw['metrics']['calibration_translation_rms_m']):.3f} mm (vs {_mm(C['metrics']['calibration_translation_rms_m']):.3f}), "
        f"val max {_mm(Cw['metrics']['validation_translation_max_m']):.3f} mm (vs {_mm(C['metrics']['validation_translation_max_m']):.3f})")

    # --------------------------------------- (a) all 16 + leave-one-out
    A = solve_split(pairs, list(range(n)), [], nominal)
    loo = []
    for i in range(n):
        idx = [j for j in range(n) if j != i]
        s = solve_split(pairs, idx, [i], nominal)
        loo.append(s)
    loo_t = np.array([loo[i]["per_pair"][i]["translation_m"] for i in range(n)])
    loo_r = np.array([loo[i]["per_pair"][i]["rotation_deg"] for i in range(n)])
    loo_T = np.stack([s["T"] for s in loo])
    loo_pos_spread = np.linalg.norm(loo_T[:, :3, 3] - A["T"][:3, 3], axis=1)
    loo_rot_spread = np.array([ca._transform_distance(A["T"], s["T"])[1] for s in loo])

    orig_t = np.array([C["per_pair"][i]["translation_m"] for i in range(n)])
    orig_r = np.array([C["per_pair"][i]["rotation_deg"] for i in range(n)])
    all_t = np.array([A["per_pair"][i]["translation_m"] for i in range(n)])
    all_r = np.array([A["per_pair"][i]["rotation_deg"] for i in range(n)])

    # ------------------------------------------------------- per-pair geometry
    centre_t = board_centre_target(columns, rows, square)
    geo = []
    cam_pos = C["T"][:3, 3]
    R_cam = C["T"][:3, :3]
    for i, p in enumerate(pairs):
        k = C["per_pair"][i]["candidate"]
        w_from_t = p["wrist_poses"][k // 2]
        t_from_t = p["top_poses"][k % 2]
        world_from_target = p["world_from_wrist"] @ w_from_t
        c_world = world_from_target[:3, :3] @ centre_t + world_from_target[:3, 3]
        c_top = t_from_t[:3, :3] @ centre_t + t_from_t[:3, 3]
        c_wrist = w_from_t[:3, :3] @ centre_t + w_from_t[:3, 3]
        top_c = p["top_px"].mean(axis=0)
        wr_c = p["wrist_px_raw"].mean(axis=0)
        bbox = p["top_px"].max(axis=0) - p["top_px"].min(axis=0)
        wrist_pos = p["world_from_wrist"][:3, 3]
        d_world = C["per_pair"][i]["observed"][:3, 3] - cam_pos  # residual vector (world)
        d_cam = R_cam.T @ d_world  # residual in top-camera axes (x right, y down, z optical)
        ray = c_world - cam_pos
        ray /= np.linalg.norm(ray)
        along = float(d_world @ ray)
        perp = float(np.linalg.norm(d_world - along * ray))
        normal_world = world_from_target[:3, :3] @ np.array([0, 0, 1.0])
        geo.append({
            "name": p["name"],
            "held_out": i in orig_val,
            "candidate": k,
            "wrist_reversed": bool(k // 2),
            "top_reversed": bool(k % 2),
            "skew_ms": p["skew_ns"] / 1e6,
            "top_centre_u": float(top_c[0]), "top_centre_v": float(top_c[1]),
            "top_offset_px": float(np.linalg.norm(top_c - top_K[:2, 2])),
            "top_px_per_square": px_per_square(p["top_px"], columns, rows),
            "top_bbox_w": float(bbox[0]), "top_bbox_h": float(bbox[1]),
            "top_board_dist_m": float(np.linalg.norm(c_top)),
            "top_tilt_deg": tilt_deg(t_from_t),
            "wrist_centre_u": float(wr_c[0]), "wrist_centre_v": float(wr_c[1]),
            "wrist_offset_px": float(np.linalg.norm(wr_c - p["wrist_K"][:2, 2])),
            "wrist_px_per_square": px_per_square(p["wrist_px_raw"], columns, rows),
            "wrist_board_dist_m": float(np.linalg.norm(c_wrist)),
            "wrist_tilt_deg": tilt_deg(w_from_t),
            "board_x": float(c_world[0]), "board_y": float(c_world[1]), "board_z": float(c_world[2]),
            "board_xy_from_base_m": float(np.hypot(c_world[0], c_world[1])),
            "board_3d_from_base_m": float(np.linalg.norm(c_world)),
            "board_normal_tilt_from_world_z_deg": float(np.degrees(np.arccos(np.clip(abs(normal_world[2]), 0, 1)))),
            "wrist_cam_x": float(wrist_pos[0]), "wrist_cam_y": float(wrist_pos[1]), "wrist_cam_z": float(wrist_pos[2]),
            "wrist_reach_xy_m": float(np.hypot(wrist_pos[0], wrist_pos[1])),
            "wrist_reach_3d_m": float(np.linalg.norm(wrist_pos)),
            "res_world_dx_mm": _mm(d_world[0]), "res_world_dy_mm": _mm(d_world[1]), "res_world_dz_mm": _mm(d_world[2]),
            "res_cam_dx_mm": _mm(d_cam[0]), "res_cam_dy_mm": _mm(d_cam[1]), "res_cam_dz_mm": _mm(d_cam[2]),
            "res_along_ray_mm": _mm(along), "res_perp_ray_mm": _mm(perp),
        })

    # ----------------------------------------------------- 1. residual table
    say("")
    say("-" * 100)
    say("1. PER-PAIR RESIDUALS  [mm, deg]")
    say("   orig  = vs the ORIGINAL solved average (12-pair solve): in-sample for calib pairs "
        "(exactly the solver's per-sample calibration error), out-of-sample for the 4 held-out (*) "
        "(exactly the solver's validation error)")
    say("   all16 = in-sample vs the 16-pair solve;  LOO = vs the 15-pair solve that excludes the pair "
        "(out-of-sample for every pair)")
    say("-" * 100)
    say(f"{'pair':7s}{'':2s}{'orig_t':>8s}{'orig_r':>8s}{'all16_t':>9s}{'all16_r':>8s}{'LOO_t':>8s}{'LOO_r':>8s}"
        f"{'cand':>5s}{'skew_ms':>8s}   {'res_world dx dy dz [mm]':24s}  {'cam dz':>7s}{'along':>7s}{'perp':>7s}")
    for i, g in enumerate(geo):
        say(f"{g['name']:7s}{'*' if g['held_out'] else ' ':2s}"
            f"{_mm(orig_t[i]):8.2f}{orig_r[i]:8.3f}{_mm(all_t[i]):9.2f}{all_r[i]:8.3f}"
            f"{_mm(loo_t[i]):8.2f}{loo_r[i]:8.3f}{g['candidate']:5d}{g['skew_ms']:8.1f}   "
            f"{g['res_world_dx_mm']:+7.1f} {g['res_world_dy_mm']:+7.1f} {g['res_world_dz_mm']:+7.1f}   "
            f"{g['res_cam_dz_mm']:+7.1f}{g['res_along_ray_mm']:+7.1f}{g['res_perp_ray_mm']:7.1f}")

    def summ(v: np.ndarray, idx: list[int] | None = None) -> str:
        v = v if idx is None else v[idx]
        return f"rms={np.sqrt(np.mean(v**2)):.3f} med={np.median(v):.3f} max={v.max():.3f}"

    say("")
    say(f"orig  translation mm: calib12 {summ(_mm(1)*orig_t, orig_cal)} | heldout4 {summ(_mm(1)*orig_t, orig_val)} | all16 {summ(_mm(1)*orig_t)}")
    say(f"orig  rotation   deg: calib12 {summ(orig_r, orig_cal)} | heldout4 {summ(orig_r, orig_val)} | all16 {summ(orig_r)}")
    say(f"all16 translation mm: {summ(_mm(1)*all_t)}   rotation deg: {summ(all_r)}")
    say(f"LOO   translation mm: {summ(_mm(1)*loo_t)}   rotation deg: {summ(loo_r)}")
    say(f"residual direction (orig, all 16): mean |along top-camera ray| {np.mean([abs(g['res_along_ray_mm']) for g in geo]):.2f} mm "
        f"vs mean |perpendicular| {np.mean([g['res_perp_ray_mm'] for g in geo]):.2f} mm; "
        f"rms cam-frame (dx,dy,dz) = ({np.sqrt(np.mean([g['res_cam_dx_mm']**2 for g in geo])):.2f}, "
        f"{np.sqrt(np.mean([g['res_cam_dy_mm']**2 for g in geo])):.2f}, "
        f"{np.sqrt(np.mean([g['res_cam_dz_mm']**2 for g in geo])):.2f}) mm; "
        f"rms world (dx,dy,dz) = ({np.sqrt(np.mean([g['res_world_dx_mm']**2 for g in geo])):.2f}, "
        f"{np.sqrt(np.mean([g['res_world_dy_mm']**2 for g in geo])):.2f}, "
        f"{np.sqrt(np.mean([g['res_world_dz_mm']**2 for g in geo])):.2f}) mm")
    say(f"mean residual vector (world, mm) over 16: ({np.mean([g['res_world_dx_mm'] for g in geo]):+.2f}, "
        f"{np.mean([g['res_world_dy_mm'] for g in geo]):+.2f}, {np.mean([g['res_world_dz_mm'] for g in geo]):+.2f})  "
        f"[calib-12 mean is 0 by construction]")
    cands = [g["candidate"] for g in geo]
    say(f"selected orientation candidate per pair (2*wrist_reversed + top_reversed): {cands}")

    # ------------------------------------------------------------ 2. outliers
    f = args.outlier_factor
    med_t_orig = float(np.median(orig_t))
    med_r_orig = float(np.median(orig_r))
    med_t_loo = float(np.median(loo_t))
    out_t_orig = [i for i in range(n) if orig_t[i] > f * med_t_orig]
    out_r_orig = [i for i in range(n) if orig_r[i] > f * med_r_orig]
    out_t_loo = [i for i in range(n) if loo_t[i] > f * med_t_loo]
    out_t_all = [i for i in range(n) if all_t[i] > f * float(np.median(all_t))]
    say("")
    say("-" * 100)
    say(f"2. OUTLIERS  (> {f:.1f} x median)")
    say("-" * 100)
    say(f"orig  translation: median {_mm(med_t_orig):.2f} mm, threshold {_mm(f*med_t_orig):.2f} mm -> {[names[i] for i in out_t_orig]}")
    say(f"orig  rotation   : median {med_r_orig:.3f} deg, threshold {f*med_r_orig:.3f} deg -> {[names[i] for i in out_r_orig]}")
    say(f"all16 translation: median {_mm(np.median(all_t)):.2f} mm, threshold {_mm(f*np.median(all_t)):.2f} mm -> {[names[i] for i in out_t_all]}")
    say(f"LOO   translation: median {_mm(med_t_loo):.2f} mm, threshold {_mm(f*med_t_loo):.2f} mm -> {[names[i] for i in out_t_loo]}")
    say("")
    say(f"{'pair':7s}{'':2s}{'orig_t':>7s}{'LOO_t':>7s} | {'top centre(u,v)':>16s}{'off_px':>7s}{'px/sq':>6s}{'bbox':>10s}{'dTop_m':>7s}{'tiltT':>6s}"
        f" | {'wr_dist':>7s}{'wr_tilt':>7s}{'wr_off':>7s}{'wr_px/sq':>8s} | {'board xy (m)':>16s}{'r_xy':>6s}{'z_mm':>7s}{'nTilt':>6s}"
        f" | {'wrist reach xy/3d':>18s}{'skew':>6s}")
    for i, g in enumerate(geo):
        flag = "*" if g["held_out"] else " "
        mark = "<<" if i in out_t_orig else ("<L" if i in out_t_loo else "  ")
        say(f"{g['name']:7s}{flag:2s}{_mm(orig_t[i]):7.2f}{_mm(loo_t[i]):7.2f} | "
            f"({g['top_centre_u']:5.1f},{g['top_centre_v']:5.1f}){g['top_offset_px']:7.1f}{g['top_px_per_square']:6.2f}"
            f"{g['top_bbox_w']:5.0f}x{g['top_bbox_h']:<4.0f}{g['top_board_dist_m']:7.3f}{g['top_tilt_deg']:6.1f}"
            f" | {g['wrist_board_dist_m']:7.3f}{g['wrist_tilt_deg']:7.1f}{g['wrist_offset_px']:7.1f}{g['wrist_px_per_square']:8.2f}"
            f" | ({g['board_x']:+.3f},{g['board_y']:+.3f}){g['board_xy_from_base_m']:6.3f}{_mm(g['board_z']):7.1f}{g['board_normal_tilt_from_world_z_deg']:6.2f}"
            f" | {g['wrist_reach_xy_m']:8.3f}/{g['wrist_reach_3d_m']:<8.3f}{g['skew_ms']:6.1f} {mark}")
    say("   (* held-out in the original split; << translation outlier on orig residual; <L outlier on LOO residual only)")
    say(f"   top-camera solved position (world): ({cam_pos[0]:+.4f}, {cam_pos[1]:+.4f}, {cam_pos[2]:+.4f}) m; "
        f"optical axis (world) {np.array2string(R_cam[:, 2], precision=3)}; principal point ({top_K[0,2]:.1f},{top_K[1,2]:.1f})")
    bz = np.array([g["board_z"] for g in geo])
    bn = np.array([g["board_normal_tilt_from_world_z_deg"] for g in geo])
    say(f"   wrist-chain-only diagnostic (board lies on one table; NOT camera ground truth): board centre z "
        f"mean {_mm(bz.mean()):.1f} mm, sd {_mm(bz.std(ddof=1)):.2f} mm, range {_mm(bz.min()):.1f}..{_mm(bz.max()):.1f} mm; "
        f"board normal tilt from world z mean {bn.mean():.2f} deg, sd {bn.std(ddof=1):.2f} deg")

    say("")
    say("hypothesis tests: translation residual vs geometry (n=16; Pearson r / Spearman rho with p-values)")
    factors = [
        ("board_xy_from_base_m", "board xy distance from left_base (kinematic reach of the board)"),
        ("wrist_reach_xy_m", "wrist camera xy distance from left_base (arm reach)"),
        ("wrist_reach_3d_m", "wrist camera 3d distance from left_base"),
        ("top_offset_px", "board centre offset from top principal point [px] (optics/distortion)"),
        ("top_px_per_square", "board pixel scale in top image [px/square] (PnP conditioning)"),
        ("top_board_dist_m", "top camera to board distance [m]"),
        ("top_tilt_deg", "top optical axis vs board normal [deg]"),
        ("wrist_board_dist_m", "wrist camera to board distance [m]"),
        ("wrist_tilt_deg", "wrist optical axis vs board normal [deg]"),
        ("wrist_offset_px", "board centre offset from wrist principal point [px]"),
        ("skew_ms", "pair timestamp skew [ms]"),
        ("board_z", "board centre world z (wrist-chain diagnostic)"),
    ]
    corr_table = {}
    say(f"{'factor':22s}{'range':>22s} | {'orig_t r':>9s}{'p':>7s}{'rho':>7s}{'p':>7s} | {'LOO_t r':>9s}{'p':>7s}{'rho':>7s}{'p':>7s} | {'LOO_r r':>9s}{'p':>7s}")
    for key, _desc in factors:
        x = np.array([g[key] for g in geo])
        c1 = corr(x, orig_t)
        c2 = corr(x, loo_t)
        c3 = corr(x, loo_r)
        corr_table[key] = {"vs_orig_translation": c1, "vs_loo_translation": c2, "vs_loo_rotation": c3}
        say(f"{key:22s}{x.min():10.3f}..{x.max():<10.3f} | {c1['pearson_r']:+9.3f}{c1['pearson_p']:7.3f}{c1['spearman_rho']:+7.3f}{c1['spearman_p']:7.3f}"
            f" | {c2['pearson_r']:+9.3f}{c2['pearson_p']:7.3f}{c2['spearman_rho']:+7.3f}{c2['spearman_p']:7.3f}"
            f" | {c3['pearson_r']:+9.3f}{c3['pearson_p']:7.3f}")

    # ------------------------------------------- error-mode decomposition
    say("")
    say("-" * 100)
    say("2b. ERROR-MODE DECOMPOSITION (what kind of error produces the residual?)")
    say("-" * 100)
    # (i) coupling of translation and rotation residual
    c_tr = corr(orig_t, orig_r)
    c_tr_loo = corr(loo_t, loo_r)
    say(f"translation-vs-rotation residual coupling: orig r={c_tr['pearson_r']:+.3f} (p={c_tr['pearson_p']:.3f}); "
        f"LOO r={c_tr_loo['pearson_r']:+.3f} (p={c_tr_loo['pearson_p']:.3f})")
    # (ii) how much of each pair's translation residual is a rigid rotation of the
    # candidate about the BOARD centre (a board-orientation error in either camera's
    # PnP or in the wrist chain rotates the implied camera about the board with the
    # 0.85-1.08 m lever arm), vs a genuine translation of the board (FK/hand-eye
    # position error or PnP depth error).
    say(f"{'pair':7s}{'':2s}{'res_t':>7s}{'res_r':>7s}{'lever_m':>8s}{'lever*r':>8s}{'rot-about-board part':>21s}{'left-over':>10s}{'axis|board-plane':>17s}{'axis|world-horiz':>17s}{'radial/tang cam-xy':>20s}")
    leftover = []
    rot_part = []
    axis_inplane = []
    axis_horiz = []
    radial_frac = []
    for i, (p, g) in enumerate(zip(pairs, geo)):
        obs = C["per_pair"][i]["observed"]
        delta = np.linalg.inv(C["T"]) @ obs  # in solved-camera frame
        dR = delta[:3, :3]
        dt = delta[:3, 3]
        k = g["candidate"]
        t_from_t = p["top_poses"][k % 2]
        c_cam = t_from_t[:3, :3] @ centre_t + t_from_t[:3, 3]  # board centre in top-camera frame (detected)
        # candidate = T_solved @ delta; a pure rotation dR about the board centre c
        # gives translation (I - dR) c
        t_rot = (np.eye(3) - dR) @ c_cam
        rest = dt - t_rot
        rotvec = Rotation.from_matrix(dR).as_rotvec()
        ang = np.linalg.norm(rotvec)
        axis_cam = rotvec / ang if ang > 0 else np.zeros(3)
        n_board_cam = t_from_t[:3, :3] @ np.array([0, 0, 1.0])
        inplane = float(np.degrees(np.arccos(np.clip(abs(axis_cam @ n_board_cam), 0, 1))))  # 90 => axis in board plane
        axis_world = R_cam @ axis_cam
        horiz = float(np.degrees(np.arccos(np.clip(abs(axis_world[2]), 0, 1))))  # 90 => horizontal axis
        # radial vs tangential (image) decomposition of the lateral camera-frame residual
        rad_dir = np.array([g["top_centre_u"] - top_K[0, 2], g["top_centre_v"] - top_K[1, 2]])
        rad_dir /= max(np.linalg.norm(rad_dir), 1e-9)
        lat = dt[:2]
        rad = float(lat @ rad_dir)
        tang = float(np.linalg.norm(lat - rad * rad_dir))
        leftover.append(np.linalg.norm(rest))
        rot_part.append(np.linalg.norm(t_rot))
        axis_inplane.append(inplane)
        axis_horiz.append(horiz)
        radial_frac.append(abs(rad) / max(np.hypot(rad, tang), 1e-9))
        lever = float(np.linalg.norm(c_cam))
        say(f"{g['name']:7s}{'*' if g['held_out'] else ' ':2s}{_mm(orig_t[i]):7.2f}{orig_r[i]:7.3f}{lever:8.3f}{_mm(lever*np.radians(orig_r[i])):8.2f}"
            f"{_mm(np.linalg.norm(t_rot)):21.2f}{_mm(np.linalg.norm(rest)):10.2f}{inplane:17.1f}{horiz:17.1f}"
            f"{abs(rad)*1000:10.2f}/{tang*1000:<8.2f}")
    leftover = np.array(leftover)
    rot_part = np.array(rot_part)
    say(f"rms: translation residual {_mm(np.sqrt(np.mean(orig_t**2))):.2f} mm = rotation-about-board part {_mm(np.sqrt(np.mean(rot_part**2))):.2f} mm (+) left-over {_mm(np.sqrt(np.mean(leftover**2))):.2f} mm; "
        f"lever*rot_rms = {_mm(np.mean([g['top_board_dist_m'] for g in geo]) * np.radians(np.sqrt(np.mean(orig_r**2)))):.2f} mm")
    say(f"rotation axis: mean angle to board normal {np.mean(axis_inplane):.1f} deg (90 = pure tilt error, 0 = in-plane spin); "
        f"mean angle to world z {np.mean(axis_horiz):.1f} deg (90 = horizontal axis)")
    say(f"lateral camera-frame residual: mean radial fraction {np.mean(radial_frac):.2f} (1.0 = purely radial in the top image, "
        f"as a top distortion-model error would be; ~0.71 = isotropic)")
    # (iii) board normal scatter from the two independent chains (planar-table prior, diagnostic only)
    n_w = []
    n_t = []
    for i, (p, g) in enumerate(zip(pairs, geo)):
        k = g["candidate"]
        wft = p["world_from_wrist"] @ p["wrist_poses"][k // 2]
        n_w.append(wft[:3, :3] @ np.array([0, 0, 1.0]))
        tft = C["T"] @ p["top_poses"][k % 2]
        n_t.append(tft[:3, :3] @ np.array([0, 0, 1.0]))
    n_w = np.array(n_w)
    n_t = np.array(n_t)
    n_w *= np.sign(n_w[:, 2:3])
    n_t *= np.sign(n_t[:, 2:3])

    def scatter_deg(nv: np.ndarray) -> tuple[float, float]:
        m = nv.mean(axis=0)
        m /= np.linalg.norm(m)
        a = np.degrees(np.arccos(np.clip(nv @ m, -1, 1)))
        return float(np.sqrt(np.mean(a**2))), float(a.max())

    sw = scatter_deg(n_w)
    st = scatter_deg(n_t)
    sd = scatter_deg(n_t - n_w + np.array([0, 0, 1.0]))
    ang_wt = np.degrees(np.arccos(np.clip(np.sum(n_w * n_t, axis=1), -1, 1)))
    say(f"board-normal scatter about its own mean (planar-table prior; diagnostic): wrist chain rms {sw[0]:.3f} max {sw[1]:.3f} deg | "
        f"top PnP through solved pose rms {st[0]:.3f} max {st[1]:.3f} deg | wrist-vs-top normal angle per pair rms {np.sqrt(np.mean(ang_wt**2)):.3f} max {ang_wt.max():.3f} deg")
    bz_t = np.array([(C["T"] @ p["top_poses"][g["candidate"] % 2] @ np.append(centre_t, 1.0))[2] for p, g in zip(pairs, geo)])
    say(f"board centre z (world) sd: wrist chain {_mm(bz.std(ddof=1)):.2f} mm | top PnP through solved pose {_mm(bz_t.std(ddof=1)):.2f} mm | "
        f"per-pair difference wrist-top: mean {_mm((bz - bz_t).mean()):+.2f} sd {_mm((bz - bz_t).std(ddof=1)):.2f} mm")

    # (iv) Monte-Carlo pixel-noise propagation: how much candidate scatter does corner
    # noise alone produce?  sigma_top = top intrinsics RMS, sigma_wrist = hand-eye RMS.
    rng = np.random.default_rng(0)
    sig_top = float(intr["metrics"]["calibration_rms_reprojection_error_px"])
    sig_wrist = 0.2494  # calib/out/left_hand_eye_report*.json rms_reprojection_error_px
    he_path = CALIB / "out" / "left_hand_eye_report_corrected.json"
    if he_path.exists():
        sig_wrist = float(json.loads(he_path.read_text())["metrics"]["rms_reprojection_error_px"])
    n_mc = 200
    mc = {"top": [], "wrist": [], "both": []}
    for i, (p, g) in enumerate(zip(pairs, geo)):
        k = g["candidate"]
        top_px = p["top_px"]
        wr_px = p["wrist_px"]
        base = C["per_pair"][i]["observed"]
        for mode in mc:
            ts = []
            rs = []
            for _ in range(n_mc):
                tp_ = top_px + (rng.normal(0, sig_top, top_px.shape) if mode in ("top", "both") else 0)
                wp_ = wr_px + (rng.normal(0, sig_wrist, wr_px.shape) if mode in ("wrist", "both") else 0)
                ordered_t = tp_[::-1] if k % 2 else tp_
                ordered_w = wp_[::-1] if k // 2 else wp_
                tft = ca.estimate_checkerboard_pose(ordered_t, top_K, columns=columns, rows=rows, square_size_m=square, distortion_coefficients=top_dist)
                wft = ca.estimate_checkerboard_pose(ordered_w, p["wrist_K"], columns=columns, rows=rows, square_size_m=square, distortion_coefficients=np.zeros(5))
                cand = p["world_from_wrist"] @ wft @ np.linalg.inv(tft)
                tm, rd = ca._transform_distance(base, cand)
                ts.append(tm)
                rs.append(rd)
            mc[mode].append((float(np.sqrt(np.mean(np.square(ts)))), float(np.sqrt(np.mean(np.square(rs))))))
    say(f"Monte-Carlo corner-noise propagation ({n_mc} draws/pair; sigma_top={sig_top:.3f} px, sigma_wrist={sig_wrist:.3f} px), "
        f"rms candidate deviation per pair [mm / deg]:")
    say(f"{'pair':7s}" + "".join(f"{m:>16s}" for m in mc))
    for i in range(n):
        say(f"{names[i]:7s}" + "".join(f"{_mm(mc[m][i][0]):8.2f}/{mc[m][i][1]:<7.3f}" for m in mc))
    pooled = {m: (_mm(np.sqrt(np.mean([v[0]**2 for v in mc[m]]))), float(np.sqrt(np.mean([v[1]**2 for v in mc[m]])))) for m in mc}
    say(f"pooled rms:  top-noise only {pooled['top'][0]:.2f} mm / {pooled['top'][1]:.3f} deg;  wrist-noise only {pooled['wrist'][0]:.2f} mm / {pooled['wrist'][1]:.3f} deg;  "
        f"both {pooled['both'][0]:.2f} mm / {pooled['both'][1]:.3f} deg   vs observed calibration rms {_mm(C['metrics']['calibration_translation_rms_m']):.2f} mm / {C['metrics']['calibration_rotation_rms_deg']:.3f} deg")
    unexplained = math.sqrt(max(C["metrics"]["calibration_translation_rms_m"]**2 - (pooled["both"][0] / 1000)**2, 0.0))
    say(f"=> corner noise explains {pooled['both'][0]:.2f} mm of {_mm(C['metrics']['calibration_translation_rms_m']):.2f} mm; "
        f"quadrature remainder {_mm(unexplained):.2f} mm must come from the wrist chain (FK + hand-eye) or unmodelled optics (distortion/intrinsics model error)")
    # (v) hand-eye consistency lever-arm prediction
    if he_path.exists():
        he = json.loads(he_path.read_text())["metrics"]
        lever = np.mean([g["top_board_dist_m"] for g in geo])
        pred = math.hypot(he["target_translation_rms_m"], lever * math.radians(he["target_rotation_rms_deg"]))
        say(f"hand-eye (corrected) fixed-target consistency: {_mm(he['target_translation_rms_m']):.2f} mm / {he['target_rotation_rms_deg']:.3f} deg rms over {json.loads(he_path.read_text())['pose_count']} poses; "
            f"propagated through the {lever:.2f} m top-camera lever arm: sqrt({_mm(he['target_translation_rms_m']):.2f}^2 + ({lever:.2f} m * {he['target_rotation_rms_deg']:.3f} deg)^2) = {_mm(pred):.2f} mm expected top-candidate scatter from the wrist chain alone")
    # (vi) partial correlations: image offset vs base distance
    def partial(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> float:
        rxy = stats.pearsonr(x, y)[0]
        rxz = stats.pearsonr(x, z)[0]
        ryz = stats.pearsonr(y, z)[0]
        return float((rxy - rxz * ryz) / math.sqrt((1 - rxz**2) * (1 - ryz**2)))

    off = np.array([g["top_offset_px"] for g in geo])
    rxy = np.array([g["board_xy_from_base_m"] for g in geo])
    reach = np.array([g["wrist_reach_xy_m"] for g in geo])
    dist_top = np.array([g["top_board_dist_m"] for g in geo])
    say(f"factor inter-correlation: top_offset~board_xy_from_base r={stats.pearsonr(off, rxy)[0]:+.3f}; top_offset~top_board_dist r={stats.pearsonr(off, dist_top)[0]:+.3f}; "
        f"board_xy~wrist_reach r={stats.pearsonr(rxy, reach)[0]:+.3f}")
    say(f"partial correlations with LOO translation residual: top_offset | board_xy_from_base = {partial(off, loo_t, rxy):+.3f}; "
        f"board_xy_from_base | top_offset = {partial(rxy, loo_t, off):+.3f}; wrist_reach | top_offset = {partial(reach, loo_t, off):+.3f}; "
        f"top_board_dist | board_xy = {partial(dist_top, loo_t, rxy):+.3f}")
    # left/right halves of the top image
    left = [i for i, g in enumerate(geo) if g["top_centre_u"] < top_K[0, 2]]
    right = [i for i in range(n) if i not in left]
    say(f"top-image halves: u<cx ({len(left)} pairs) LOO_t rms {_mm(np.sqrt(np.mean(loo_t[left]**2))):.2f} mm; u>cx ({len(right)} pairs) rms {_mm(np.sqrt(np.mean(loo_t[right]**2))):.2f} mm; "
        f"Mann-Whitney p={stats.mannwhitneyu(loo_t[left], loo_t[right]).pvalue:.3f}")
    near = [i for i, g in enumerate(geo) if g["top_offset_px"] < np.median(off)]
    far = [i for i in range(n) if i not in near]
    say(f"top offset below/above median ({np.median(off):.0f} px): near rms {_mm(np.sqrt(np.mean(loo_t[near]**2))):.2f} mm (n={len(near)}), far rms {_mm(np.sqrt(np.mean(loo_t[far]**2))):.2f} mm (n={len(far)}); "
        f"Mann-Whitney p={stats.mannwhitneyu(loo_t[near], loo_t[far]).pvalue:.3f}")
    nearb = [i for i, g in enumerate(geo) if g["board_xy_from_base_m"] < np.median(rxy)]
    farb = [i for i in range(n) if i not in nearb]
    say(f"board xy from base below/above median ({np.median(rxy):.3f} m): near rms {_mm(np.sqrt(np.mean(loo_t[nearb]**2))):.2f} mm, far rms {_mm(np.sqrt(np.mean(loo_t[farb]**2))):.2f} mm; "
        f"Mann-Whitney p={stats.mannwhitneyu(loo_t[nearb], loo_t[farb]).pvalue:.3f}")
    report_extra = {
        "trans_rot_coupling": {"orig": c_tr, "loo": c_tr_loo},
        "rotation_about_board_rms_m": float(np.sqrt(np.mean(rot_part**2))),
        "leftover_rms_m": float(np.sqrt(np.mean(leftover**2))),
        "axis_angle_to_board_normal_mean_deg": float(np.mean(axis_inplane)),
        "axis_angle_to_world_z_mean_deg": float(np.mean(axis_horiz)),
        "radial_fraction_mean": float(np.mean(radial_frac)),
        "board_normal_scatter_deg": {"wrist_rms": sw[0], "wrist_max": sw[1], "top_rms": st[0], "top_max": st[1],
                                     "wrist_vs_top_rms": float(np.sqrt(np.mean(ang_wt**2))), "wrist_vs_top_max": float(ang_wt.max())},
        "board_z_sd_m": {"wrist": float(bz.std(ddof=1)), "top": float(bz_t.std(ddof=1)), "diff": float((bz - bz_t).std(ddof=1))},
        "monte_carlo": {"sigma_top_px": sig_top, "sigma_wrist_px": sig_wrist, "draws": n_mc,
                        "pooled_rms_mm_deg": pooled, "per_pair": {m: mc[m] for m in mc}},
        "partial_correlations_loo_t": {"top_offset_given_board_xy": partial(off, loo_t, rxy),
                                       "board_xy_given_top_offset": partial(rxy, loo_t, off),
                                       "wrist_reach_given_top_offset": partial(reach, loo_t, off),
                                       "top_board_dist_given_board_xy": partial(dist_top, loo_t, rxy)},
    }

    # --------------------------------------------------------- 3. variants
    say("")
    say("-" * 100)
    say("3. RE-SOLVE VARIANTS  (outputs under calib/analysis/out/, nothing installed)")
    say("-" * 100)

    variants: dict[str, dict[str, Any]] = {}

    def register(tag: str, sol: dict[str, Any], label: str, note: str) -> None:
        d_m, d_deg = ca._transform_distance(T_orig, sol["T"])
        dpos = (sol["T"][:3, 3] - T_orig[:3, 3]) * 1000.0
        variants[tag] = {"sol": sol, "label": label, "note": note, "delta_vs_orig_m": d_m,
                         "delta_vs_orig_deg": d_deg, "dpos_world_mm": dpos}
        rep = solver_style_report(sol, pairs, orig, nominal, label, note)
        (out_dir / f"variant_{tag}.json").write_text(json.dumps(_jsonable(rep), indent=2, sort_keys=True))

    register("c_original", C, "(c) original every-4th split (12 calib / 4 held-out)", "Reproduction of calib/out/top_brio_178B0DAE_calibration.json.")
    register("a_all16", A, "(a) all 16 pairs calibrate; validation = leave-one-out (see report.json loo)", "In-sample metrics; LOO validation is reported in report.txt/report.json.")

    # (b) drop translation outliers of the ORIGINAL residual, same every-4th split membership
    drop = out_t_orig
    keep = [i for i in range(n) if i not in drop]
    b1_cal = [i for i in orig_cal if i not in drop]
    b1_val = [i for i in orig_val if i not in drop]
    B1 = solve_split(pairs, b1_cal, b1_val, nominal) if len(b1_cal) >= 3 else None
    if B1:
        register("b1_drop_outliers_same_split", B1,
                 f"(b) drop {[names[i] for i in drop]} (orig translation residual > {f}x median), original split membership kept",
                 "Outliers removed from whichever group they were in; every-4th membership otherwise unchanged.")
    # (b') every-4th re-applied to the remaining sorted list
    b2_val = [i for k, i in enumerate(keep) if k % 4 == 3]
    b2_cal = [i for k, i in enumerate(keep) if k % 4 != 3]
    B2 = solve_split(pairs, b2_cal, b2_val, nominal) if len(b2_cal) >= 3 and b2_val else None
    if B2:
        register("b2_drop_outliers_resplit", B2,
                 f"(b') drop {[names[i] for i in drop]}, every-4th split re-applied to the remaining {len(keep)} files",
                 "Same solve_top.sh rule (sorted, zero-based index % 4 == 3 held out) on the reduced file list.")
    # (b'') all remaining as calibration + LOO
    B3 = solve_split(pairs, keep, [], nominal) if len(keep) >= 3 else None
    b3_loo_t = b3_loo_r = None
    if B3:
        register("b3_drop_outliers_all_loo", B3,
                 f"(b'') drop {[names[i] for i in drop]}, all remaining {len(keep)} calibrate, LOO validation",
                 "LOO numbers in report.txt/report.json.")
        b3_loo_t = np.full(n, np.nan)
        b3_loo_r = np.full(n, np.nan)
        for i in keep:
            idx = [j for j in keep if j != i]
            s = solve_split(pairs, idx, [i], nominal)
            b3_loo_t[i] = s["per_pair"][i]["translation_m"]
            b3_loo_r[i] = s["per_pair"][i]["rotation_deg"]

    # Sensitivity-only variants (NOT the requested 2x-median rule; shown to bound how
    # far any defensible trimming could move the solution).
    s1_drop = [i for i in range(n) if loo_t[i] > 1.5 * med_t_loo]
    if s1_drop:
        s1_keep = [i for i in range(n) if i not in s1_drop]
        S1 = solve_split(pairs, [i for i in orig_cal if i not in s1_drop], [i for i in orig_val if i not in s1_drop], nominal)
        register("s1_drop_loo_gt_1p5x_median", S1,
                 f"(sensitivity) drop {[names[i] for i in s1_drop]} (LOO translation > 1.5x median), original split membership",
                 "Not the 2x-median rule; sensitivity bound only.")
    s2_drop = [int(i) for i in np.argsort(-loo_t)[:2]]
    S2 = solve_split(pairs, [i for i in orig_cal if i not in s2_drop], [i for i in orig_val if i not in s2_drop], nominal)
    register("s2_drop_worst2_loo", S2,
             f"(sensitivity) drop the 2 worst LOO pairs {[names[i] for i in s2_drop]}, original split membership",
             "Not the 2x-median rule; sensitivity bound only.")

    say(f"{'variant':34s}{'nC/nV':>6s}{'cal_rms':>8s}{'cal_max':>8s}{'rot_rms':>8s}{'rot_max':>8s}{'val_tmax':>9s}{'val_rmax':>9s}"
        f"{'status':>7s} | {'dpos vs orig (mm)':>20s}{'|dpos|':>7s}{'drot':>7s} | {'cam pos world (m)':>26s}")
    for tag, v in variants.items():
        s = v["sol"]
        m = s["metrics"]
        pos = s["T"][:3, 3]
        say(f"{tag:34s}{len(s['calib_idx']):3d}/{len(s['val_idx']):<2d}"
            f"{_mm(m['calibration_translation_rms_m']):8.2f}{_mm(m['calibration_translation_max_m']):8.2f}"
            f"{m['calibration_rotation_rms_deg']:8.3f}{m['calibration_rotation_max_deg']:8.3f}"
            f"{_mm(m['validation_translation_max_m']):9.2f}{m['validation_rotation_max_deg']:9.3f}"
            f"{s['status']:>7s} | ({v['dpos_world_mm'][0]:+6.2f},{v['dpos_world_mm'][1]:+6.2f},{v['dpos_world_mm'][2]:+6.2f})"
            f"{_mm(v['delta_vs_orig_m']):7.2f}{v['delta_vs_orig_deg']:7.3f} | ({pos[0]:+.4f},{pos[1]:+.4f},{pos[2]:+.4f})")
    loo_pass = (loo_t.max() <= THRESHOLDS["max_validation_translation_error_m"]
                and loo_r.max() <= THRESHOLDS["max_validation_rotation_error_deg"])
    say(f"  (a) LOO validation over 16: translation rms {_mm(np.sqrt(np.mean(loo_t**2))):.2f} max {_mm(loo_t.max()):.2f} mm ({names[int(loo_t.argmax())]}); "
        f"rotation rms {np.sqrt(np.mean(loo_r**2)):.3f} max {loo_r.max():.3f} deg ({names[int(loo_r.argmax())]}); "
        f"LOO solutions vs all-16: position spread max {_mm(loo_pos_spread.max()):.2f} mm (sd {_mm(loo_pos_spread.std()):.2f}), rotation max {loo_rot_spread.max():.3f} deg; "
        f"status with LOO as the validation set: {'PASS' if loo_pass else 'FAIL'} "
        f"({sum(loo_t > THRESHOLDS['max_validation_translation_error_m'])} of 16 LOO residuals exceed the 15 mm validation threshold)")
    # Exhaustive hold-out scan: every 4-of-16 hold-out set (C(16,4)=1820), remaining 12 calibrate.
    from itertools import combinations
    scan_status = []
    scan_valmax = []
    scan_calrms = []
    scan_pos = []
    fail_sets: dict[str, int] = {}
    for held in combinations(range(n), 4):
        cal = [i for i in range(n) if i not in held]
        s = solve_split(pairs, cal, list(held), nominal)
        scan_status.append(s["status"] == "PASS")
        scan_valmax.append(s["metrics"]["validation_translation_max_m"])
        scan_calrms.append(s["metrics"]["calibration_translation_rms_m"])
        scan_pos.append(s["T"][:3, 3])
        if s["status"] != "PASS":
            for i in held:
                if s["per_pair"][i]["translation_m"] > THRESHOLDS["max_validation_translation_error_m"] or s["per_pair"][i]["rotation_deg"] > THRESHOLDS["max_validation_rotation_error_deg"]:
                    fail_sets[names[i]] = fail_sets.get(names[i], 0) + 1
    scan_status = np.array(scan_status)
    scan_valmax = np.array(scan_valmax)
    scan_calrms = np.array(scan_calrms)
    scan_pos = np.array(scan_pos)
    say(f"  exhaustive 4-of-16 hold-out scan ({len(scan_status)} splits, 12 calibrate each): PASS in {scan_status.mean()*100:.1f}% of splits; "
        f"validation max translation: median {_mm(np.median(scan_valmax)):.2f}, 90th pct {_mm(np.percentile(scan_valmax, 90)):.2f}, max {_mm(scan_valmax.max()):.2f} mm; "
        f"calibration rms: min {_mm(scan_calrms.min()):.2f} median {_mm(np.median(scan_calrms)):.2f} max {_mm(scan_calrms.max()):.2f} mm; "
        f"solved camera position spread over splits: sd per axis ({_mm(scan_pos[:,0].std()):.2f}, {_mm(scan_pos[:,1].std()):.2f}, {_mm(scan_pos[:,2].std()):.2f}) mm, "
        f"max |dpos| vs original {_mm(np.linalg.norm(scan_pos - T_orig[:3,3], axis=1).max()):.2f} mm")
    say(f"  held-out pairs responsible for the FAIL splits (count of failing splits in which the pair exceeded a validation threshold): {dict(sorted(fail_sets.items(), key=lambda kv: -kv[1]))}")
    scan_summary = {"splits": int(len(scan_status)), "pass_fraction": float(scan_status.mean()),
                    "validation_translation_max_m": {"median": float(np.median(scan_valmax)), "p90": float(np.percentile(scan_valmax, 90)), "max": float(scan_valmax.max())},
                    "calibration_translation_rms_m": {"min": float(scan_calrms.min()), "median": float(np.median(scan_calrms)), "max": float(scan_calrms.max())},
                    "camera_position_sd_m": scan_pos.std(axis=0), "camera_position_max_delta_vs_original_m": float(np.linalg.norm(scan_pos - T_orig[:3,3], axis=1).max()),
                    "fail_pairs": fail_sets}
    if B3 is not None and b3_loo_t is not None:
        kt = b3_loo_t[keep]
        kr = b3_loo_r[keep]
        say(f"  (b'') LOO validation over {len(keep)}: translation rms {_mm(np.sqrt(np.mean(kt**2))):.2f} max {_mm(kt.max()):.2f} mm ({names[keep[int(kt.argmax())]]}); "
            f"rotation rms {np.sqrt(np.mean(kr**2)):.3f} max {kr.max():.3f} deg")
    for tag in ("b1_drop_outliers_same_split", "b2_drop_outliers_resplit"):
        if tag in variants:
            s = variants[tag]["sol"]
            say(f"  {tag}: calib={[names[i] for i in s['calib_idx']]} held-out={[names[i] for i in s['val_idx']]}; "
                f"held-out residuals mm/deg: " + ", ".join(f"{names[i]} {_mm(s['per_pair'][i]['translation_m']):.2f}/{s['per_pair'][i]['rotation_deg']:.3f}" for i in s["val_idx"]))
    say(f"  dropped pairs' pair_skew_ns: " + ", ".join(f"{names[i]}={pairs[i]['skew_ns']} ns ({pairs[i]['skew_ns']/1e6:.1f} ms)" for i in drop))
    say(f"  all pairs skew ms: " + ", ".join(f"{names[i]}={pairs[i]['skew_ns']/1e6:.1f}" for i in range(n)))
    # how do the dropped pairs score against the (b) solutions?
    for tag in ("b1_drop_outliers_same_split", "b3_drop_outliers_all_loo"):
        if tag in variants:
            s = variants[tag]["sol"]
            say(f"  dropped pairs scored against {tag}: " + ", ".join(
                f"{names[i]} {_mm(s['per_pair'][i]['translation_m']):.2f} mm / {s['per_pair'][i]['rotation_deg']:.3f} deg" for i in drop))

    # ------------------------------------------------------ pixel-space impact
    say("")
    say("overlay impact at 0.8 m (fx~384.6 px -> 1 cm world ~ 4.8 px; 1 deg ~ 6.7 px at image centre):")
    for tag, v in variants.items():
        if tag == "c_original":
            continue
        # reproject the 16 board centres through original vs variant camera pose
        d_px = []
        for g in geo:
            c_world = np.array([g["board_x"], g["board_y"], g["board_z"]])
            for T in (T_orig, v["sol"]["T"]):
                pass
            def proj(T: np.ndarray) -> np.ndarray:
                c_cam = T[:3, :3].T @ (c_world - T[:3, 3])
                uv, _ = cv2.projectPoints(c_cam.reshape(1, 1, 3), np.zeros(3), np.zeros(3), top_K, top_dist)
                return uv.reshape(2)
            d_px.append(np.linalg.norm(proj(T_orig) - proj(v["sol"]["T"])))
        d_px = np.array(d_px)
        say(f"  {tag:34s} board-centre reprojection shift orig->variant: mean {d_px.mean():.2f} px, max {d_px.max():.2f} px")
    # per-pair residual expressed as top-image reprojection error of the board centre
    say("per-pair residual as top-image reprojection of the board centre (wrist-chain board pose projected through the ORIGINAL solve vs detected centre):")
    reproj = []
    for i, g in enumerate(geo):
        c_world = np.array([g["board_x"], g["board_y"], g["board_z"]])
        c_cam = T_orig[:3, :3].T @ (c_world - T_orig[:3, 3])
        uv, _ = cv2.projectPoints(c_cam.reshape(1, 1, 3), np.zeros(3), np.zeros(3), top_K, top_dist)
        e = float(np.linalg.norm(uv.reshape(2) - np.array([g["top_centre_u"], g["top_centre_v"]])))
        g["reproj_centre_px"] = e
        reproj.append(e)
    reproj = np.array(reproj)
    say("  " + ", ".join(f"{names[i]} {reproj[i]:.2f}" for i in range(n)) + f"  | rms {np.sqrt(np.mean(reproj**2)):.2f} max {reproj.max():.2f} px")

    # ------------------------------------------------------------- 4. verdict
    say("")
    say("-" * 100)
    say("4. STATISTICS FOR THE RECOMMENDATION (see final report text)")
    say("-" * 100)
    for tag, v in variants.items():
        s = v["sol"]
        m = s["metrics"]
        margin_cal = THRESHOLDS["max_calibration_translation_rms_m"] / m["calibration_translation_rms_m"]
        margin_val = THRESHOLDS["max_validation_translation_error_m"] / m["validation_translation_max_m"] if s["val_idx"] else float("nan")
        say(f"  {tag:34s} cal_rms margin x{margin_cal:.2f}, val_max margin x{margin_val:.2f}, |dpos| vs orig {_mm(v['delta_vs_orig_m']):.2f} mm "
            f"({_mm(v['delta_vs_orig_m'])*top_K[0,0]/0.8/1000:.1f} px at 0.8 m), drot {v['delta_vs_orig_deg']:.3f} deg ({np.radians(v['delta_vs_orig_deg'])*top_K[0,0]:.1f} px)")

    # ---------------------------------------------------------------- outputs
    csv_path = out_dir / "per_pair_residuals.csv"
    cols = ["name", "held_out", "orig_t_mm", "orig_r_deg", "all16_t_mm", "all16_r_deg", "loo_t_mm", "loo_r_deg",
            "b3_loo_t_mm", "b3_loo_r_deg", "reproj_centre_px"] + [k for k in geo[0].keys() if k not in ("name", "held_out", "reproj_centre_px")]
    with csv_path.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for i, g in enumerate(geo):
            row = {**g, "orig_t_mm": _mm(orig_t[i]), "orig_r_deg": orig_r[i], "all16_t_mm": _mm(all_t[i]),
                   "all16_r_deg": all_r[i], "loo_t_mm": _mm(loo_t[i]), "loo_r_deg": loo_r[i],
                   "b3_loo_t_mm": _mm(b3_loo_t[i]) if b3_loo_t is not None else "",
                   "b3_loo_r_deg": b3_loo_r[i] if b3_loo_r is not None else ""}
            w.writerow([row[c] for c in cols])

    report = {
        "inputs": {"pairs": [str(p["path"]) for p in pairs], "intrinsics": str(args.intrinsics),
                   "calibration": str(args.calibration), "wrist_intrinsics": str(args.wrist_intrinsics),
                   "config": str(args.config)},
        "reproduction": {"allclose_1e-9": bool(np.allclose(T_orig, C["T"], atol=1e-9)),
                         "delta_m": dT_m, "delta_deg": dT_deg, "fixed_point_converged": C["fixed_point_converged"]},
        "original_split": {"calibration": [names[i] for i in orig_cal], "held_out": [names[i] for i in orig_val]},
        "per_pair": [{**geo[i], "orig_translation_m": orig_t[i], "orig_rotation_deg": orig_r[i],
                      "all16_translation_m": all_t[i], "all16_rotation_deg": all_r[i],
                      "loo_translation_m": loo_t[i], "loo_rotation_deg": loo_r[i],
                      "loo_camera_position_delta_vs_all16_m": loo_pos_spread[i],
                      "loo_camera_rotation_delta_vs_all16_deg": loo_rot_spread[i],
                      "b3_loo_translation_m": (b3_loo_t[i] if b3_loo_t is not None else None),
                      "b3_loo_rotation_deg": (b3_loo_r[i] if b3_loo_r is not None else None)}
                     for i in range(n)],
        "outliers": {"factor": f,
                     "orig_translation": {"median_m": med_t_orig, "pairs": [names[i] for i in out_t_orig]},
                     "orig_rotation": {"median_deg": med_r_orig, "pairs": [names[i] for i in out_r_orig]},
                     "all16_translation": {"median_m": float(np.median(all_t)), "pairs": [names[i] for i in out_t_all]},
                     "loo_translation": {"median_m": med_t_loo, "pairs": [names[i] for i in out_t_loo]}},
        "correlations": corr_table,
        "error_mode_decomposition": report_extra,
        "wrist_chain_diagnostic": {"board_z_mean_m": float(bz.mean()), "board_z_sd_m": float(bz.std(ddof=1)),
                                   "board_z_min_m": float(bz.min()), "board_z_max_m": float(bz.max()),
                                   "board_normal_tilt_mean_deg": float(bn.mean()), "board_normal_tilt_sd_deg": float(bn.std(ddof=1))},
        "variants": {tag: {"label": v["label"], "note": v["note"], "status": v["sol"]["status"],
                           "calibration_pairs": [names[i] for i in v["sol"]["calib_idx"]],
                           "validation_pairs": [names[i] for i in v["sol"]["val_idx"]],
                           "metrics": v["sol"]["metrics"], "camera_to_world": v["sol"]["T"],
                           "delta_vs_original_m": v["delta_vs_orig_m"], "delta_vs_original_deg": v["delta_vs_orig_deg"],
                           "dpos_world_mm": v["dpos_world_mm"],
                           "per_pair_translation_m": [v["sol"]["per_pair"][i]["translation_m"] for i in range(n)],
                           "per_pair_rotation_deg": [v["sol"]["per_pair"][i]["rotation_deg"] for i in range(n)]}
                     for tag, v in variants.items()},
        "loo_all16": {"translation_rms_m": float(np.sqrt(np.mean(loo_t**2))), "translation_max_m": float(loo_t.max()),
                      "rotation_rms_deg": float(np.sqrt(np.mean(loo_r**2))), "rotation_max_deg": float(loo_r.max()),
                      "status_with_loo_as_validation": "PASS" if loo_pass else "FAIL"},
        "holdout_scan_4_of_16": scan_summary,
        "thresholds": THRESHOLDS,
    }
    (out_dir / "report.json").write_text(json.dumps(_jsonable(report), indent=2, sort_keys=True))

    # board-position map on the top image (pair01 background), coloured by orig residual
    canvas = cv2.cvtColor(pairs[0]["top_rgb"].copy(), cv2.COLOR_RGB2BGR)
    canvas = cv2.resize(canvas, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
    vmax = max(orig_t.max(), 1e-6)
    for i, (p, g) in enumerate(zip(pairs, geo)):
        hull = cv2.convexHull((p["top_px"] * 2).astype(np.float32)).astype(np.int32)
        frac = float(orig_t[i] / vmax)
        colour = (int(255 * (1 - frac)), int(80), int(255 * frac))  # blue->red
        cv2.polylines(canvas, [hull], True, colour, 2)
        c = (int(g["top_centre_u"] * 2), int(g["top_centre_v"] * 2))
        cv2.circle(canvas, c, 4, colour, -1)
        cv2.putText(canvas, f"{g['name'][-2:]}{'*' if g['held_out'] else ''} {_mm(orig_t[i]):.1f}", (c[0] + 6, c[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)
    cv2.drawMarker(canvas, (int(top_K[0, 2] * 2), int(top_K[1, 2] * 2)), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
    cv2.imwrite(str(out_dir / "top_board_map.png"), canvas)

    say("")
    say(f"wrote: {out_dir/'report.txt'}, {out_dir/'report.json'}, {csv_path}, {out_dir/'top_board_map.png'}, "
        + ", ".join(str(out_dir / f'variant_{t}.json') for t in variants))
    say.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
