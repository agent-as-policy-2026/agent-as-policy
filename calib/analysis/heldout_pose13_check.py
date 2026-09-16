#!/usr/bin/env python3
"""Held-out cross-check of the installed LEFT wrist hand-eye chain and the
installed top-camera (BRIO 178B0DAE) extrinsics against a FRESH capture.

ANALYSIS ONLY.  Reads
    calib/out/left_pose13.npz                        (fresh held-out 30-frame capture;
                                                      camera_to_world already composed
                                                      with the INSTALLED chain)
    calib/out/top_brio_178B0DAE_intrinsics.json      (installed top K/dist)
    calib/out/top_brio_178B0DAE_calibration.json     (installed top camera_to_world)
    calib/out/left_d405_intrinsics.json              (wrist inverse_brown_conrady)
    calib/out/top_pairs/pair*.npz                    (13 pair captures, table-height ref)
plus a fresh burst of ~15 frames from the free BRIO top camera (read-only).
Writes ONLY under calib/analysis/ (report + median top frame).

Method (solver-identical helpers, as top_extrinsics_residuals.py):
  1. WRIST chain: pose13 middle frame -> _detect_checkerboard (two-scale SB)
     -> undistort_realsense_pixels -> _checkerboard_pose_candidates (IPPE PnP,
     both corner orders, zero distortion in undistorted-pinhole space)
     -> T_world_board_wrist = npz camera_to_world @ T_cam_board.
     The 180-degree corner-order ambiguity is resolved by a deterministic
     world-frame convention (board +x axis pointing toward world +x, tie-break
     +y); both candidates are reported.
  2. TOP chain: fresh BRIO burst (BrioTopCamera enforces 640x360@30 MJPG) ->
     per-pixel temporal median (capture tool's temporal_median_uint8) ->
     detect -> _checkerboard_pose_candidates with the installed top K/dist ->
     T_world_board_top = installed camera_to_world @ T_cam_board; ambiguity
     resolved by choosing the candidate closest to the WRIST result
     (choose_checkerboard_orientation's score trans + 0.1*rot_rad); both
     candidates' errors are reported so a flip would be visible.
  3. CROSS-CHECK: _transform_distance between the two world board poses,
     per-axis translation, rotation split about-normal vs tilt, centre z each.
  4. TABLE HEIGHT: per pair01..13 the wrist-chain board-plane z via the
     capture tool's OWN tracker method (capture_top_pairs.board_geometry,
     candidate 0) -> median/sd; table_z = median - 3 mm nominal plate.
  5. VERDICT block.

Run (from the hardware-bridge uv project):
    cd hardware-bridge && \
    uv run --locked python ../calib/analysis/heldout_pose13_check.py
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]   # repo root (this file is calib/analysis/)
CALIB = ROOT / "calib"
OUT_DIR = CALIB / "analysis" / "out"
sys.path.insert(0, str(CALIB))

import agp_yam_bridge.camera_acceptance as ca  # noqa: E402
from capture_top_pairs import (  # noqa: E402
    board_geometry,
    corner_drift_px,
    load_quality_context,
    temporal_median_uint8,
)

POSE13 = CALIB / "out" / "left_pose13.npz"
TOP_INTR = CALIB / "out" / "top_brio_178B0DAE_intrinsics.json"
TOP_CAL = CALIB / "out" / "top_brio_178B0DAE_calibration.json"
WRIST_INTR = CALIB / "out" / "left_d405_intrinsics.json"
PAIRS_DIR = CALIB / "out" / "top_pairs"

FRESH_TOP_FRAMES = 15
PLATE_THICKNESS_M = 0.003  # nominal board plate thickness

# expectations (verdict thresholds)
EXP_CROSS_T_M = 0.015
EXP_CROSS_R_DEG = 2.0
EXP_POSE13_DZ_M = 0.005
EXP_REPROJ_RMS_PX = 1.0


def mm(x: float) -> float:
    return 1000.0 * float(x)


class Tee:
    def __init__(self, path: Path):
        self._lines: list[str] = []
        self._path = path

    def __call__(self, *parts: Any) -> None:
        line = " ".join(str(p) for p in parts)
        print(line, flush=True)
        self._lines.append(line)

    def flush(self) -> None:
        self._path.write_text("\n".join(self._lines) + "\n", encoding="utf-8")


def board_object_points(columns: int, rows: int, square: float) -> np.ndarray:
    obj = np.zeros((columns * rows, 3), dtype=np.float64)
    obj[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square
    return obj


def board_centre(columns: int, rows: int, square: float) -> np.ndarray:
    return np.array([(columns - 1) * square / 2.0, (rows - 1) * square / 2.0, 0.0])


def reproj_rms_px(
    ordered_px: np.ndarray,
    T_cam_board: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    columns: int,
    rows: int,
    square: float,
) -> float:
    """RMS reprojection error of the PnP solution against the ORDERED corners
    it was solved from (same pixel space, same distortion coefficients)."""
    obj = board_object_points(columns, rows, square)
    rvec, _ = cv2.Rodrigues(T_cam_board[:3, :3])
    proj, _ = cv2.projectPoints(obj, rvec, T_cam_board[:3, 3], K, dist)
    d = proj.reshape(-1, 2) - ordered_px
    return float(np.sqrt(np.mean(np.sum(d**2, axis=1))))


def world_board_summary(T_world_board: np.ndarray, centre_b: np.ndarray) -> dict[str, Any]:
    c = T_world_board[:3, :3] @ centre_b + T_world_board[:3, 3]
    n = T_world_board[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return {
        "centre_world": c,
        "centre_z_m": float(c[2]),
        "normal_world": n,
        "tilt_from_horizontal_deg": float(np.degrees(np.arccos(np.clip(abs(n[2]), 0.0, 1.0)))),
        "x_axis_world": T_world_board[:3, 0],
    }


def fmt_T(T: np.ndarray) -> str:
    return "\n".join(
        "    [" + " ".join(f"{v:+.6f}" for v in row) + "]" for row in np.asarray(T)
    )


def pick_wrist_candidate(world_poses: list[np.ndarray]) -> int:
    """Deterministic convention: board +x axis toward world +x (tie-break +y)."""
    for axis in (0, 1):
        comps = [float(T[:3, 0][axis]) for T in world_poses]
        if max(abs(c) for c in comps) > 0.1:
            return int(np.argmax(comps))
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--frames", type=int, default=FRESH_TOP_FRAMES)
    ap.add_argument("--no-camera", action="store_true", help="skip the fresh BRIO grab")
    args = ap.parse_args(argv)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    say = Tee(OUT_DIR / "heldout_pose13_report.txt")

    intr = json.loads(TOP_INTR.read_text())
    cal = json.loads(TOP_CAL.read_text())
    wrist_json = json.loads(WRIST_INTR.read_text())
    top_K = np.asarray(intr["camera_matrix"], float)
    top_dist = np.asarray(intr["distortion_coefficients"], float)
    columns = int(intr["checkerboard"]["columns"])
    rows = int(intr["checkerboard"]["rows"])
    square = float(intr["checkerboard"]["square_size_m"])
    wrist_model = str(wrist_json["distortion_model"])
    wrist_dist = np.asarray(wrist_json["distortion_coefficients"], float)
    wrist_K_json = np.asarray(wrist_json["camera_matrix"], float)
    T_wc_top = ca._rigid_transform(np.asarray(cal["camera_to_world"], float), "top camera_to_world")
    centre_b = board_centre(columns, rows, square)
    obj = board_object_points(columns, rows, square)
    zero5 = np.zeros(5)

    say("=" * 100)
    say("HELD-OUT POSE13 CROSS-CHECK  (installed wrist hand-eye chain vs installed top extrinsics)")
    say("world == left_base; board: %dx%d inner corners, %.1f mm squares (%.0f x %.0f mm inner extent)"
        % (columns, rows, square * 1000, (columns - 1) * square * 1000, (rows - 1) * square * 1000))
    say("top calibration status=%s  cal rms %.2f mm / %.3f deg, val max %.2f mm / %.3f deg (13 pairs)"
        % (cal["status"], mm(cal["metrics"]["calibration_translation_rms_m"]),
           cal["metrics"]["calibration_rotation_rms_deg"],
           mm(cal["metrics"]["validation_translation_max_m"]),
           cal["metrics"]["validation_rotation_max_deg"]))
    say("=" * 100)

    # ------------------------------------------------------------------ 1. WRIST
    say("")
    say("-" * 100)
    say("1. WRIST-CHAIN BOARD POSE  (pose13 middle frame; chain installed at capture time)")
    say("-" * 100)
    with np.load(POSE13, allow_pickle=False) as z:
        n_frames = int(z["rgb"].shape[0])
        mid = n_frames // 2
        rgb13 = z["rgb"][mid]
        depth13 = z["depth_m"][mid]
        K13 = ca._matrix(z["intrinsics"][mid], (3, 3), "pose13 intrinsics")
        T_wc_wrist = ca._rigid_transform(z["camera_to_world"][mid], "pose13 camera_to_world")
        serial13 = str(z["serial"])
        joints13 = np.asarray(z["robot_joint_pos_0"][mid], float)
        # small scatter sample across the burst
        scatter_idx = sorted({0, n_frames // 4, mid, (3 * n_frames) // 4, n_frames - 1})
        scatter = [(int(i), z["rgb"][i], ca._matrix(z["intrinsics"][i], (3, 3), "K"),
                    ca._rigid_transform(z["camera_to_world"][i], "T")) for i in scatter_idx]
    assert serial13 == str(wrist_json["serial"]), (serial13, wrist_json["serial"])
    say(f"pose13: {n_frames} frames, middle frame index {mid}; wrist serial {serial13}")
    say(f"  joints (rad): {np.array2string(joints13, precision=4)}")
    say(f"  npz K vs left_d405_intrinsics.json K: max |dK| = {np.abs(K13 - wrist_K_json).max():.6f}")
    say(f"  camera_to_world (npz, middle frame): camera at "
        f"({T_wc_wrist[0,3]:+.4f}, {T_wc_wrist[1,3]:+.4f}, {T_wc_wrist[2,3]:+.4f}) m")

    wrist_px_raw = ca._detect_checkerboard(rgb13, columns=columns, rows=rows, label="pose13 wrist")
    wrist_px = ca.undistort_realsense_pixels(
        wrist_px_raw, K13, width=rgb13.shape[1], height=rgb13.shape[0],
        distortion_model=wrist_model, distortion_coefficients=wrist_dist,
    )
    say(f"  detected {len(wrist_px_raw)} corners; undistortion ({wrist_model}) moved them "
        f"mean {np.linalg.norm(wrist_px - wrist_px_raw, axis=1).mean():.3f} px, "
        f"max {np.linalg.norm(wrist_px - wrist_px_raw, axis=1).max():.3f} px; "
        f"board {ca_px_per_square(wrist_px_raw, columns, rows):.2f} px/square")

    wrist_cands = ca._checkerboard_pose_candidates(
        wrist_px, K13, zero5, columns=columns, rows=rows, square_size_m=square)
    wrist_rms = [
        reproj_rms_px(px, T, K13, zero5, columns, rows, square)
        for px, T in zip((wrist_px, wrist_px[::-1]), wrist_cands)
    ]
    wrist_world = [T_wc_wrist @ T for T in wrist_cands]
    kw = pick_wrist_candidate(wrist_world)
    flip_t, flip_r = ca._transform_distance(wrist_world[0], wrist_world[1])
    say(f"  corner-order candidates (0=as-detected, 1=reversed): reprojection RMS "
        f"{wrist_rms[0]:.3f} / {wrist_rms[1]:.3f} px (undistorted-pinhole space); "
        f"candidate1-vs-candidate0 world delta {mm(flip_t):.2f} mm / {flip_r:.2f} deg (expect ~0 mm / ~180 deg)")
    say(f"  -> selected candidate {kw} ({'as-detected' if kw == 0 else 'reversed'}) by the "
        f"board-x-toward-world-x convention")
    T_world_board_wrist = wrist_world[kw]
    wrist_reproj = wrist_rms[kw]
    sw = world_board_summary(T_world_board_wrist, centre_b)
    say("  T_world_board (wrist chain):")
    say(fmt_T(T_world_board_wrist))
    say(f"  board centre (world): ({sw['centre_world'][0]:+.4f}, {sw['centre_world'][1]:+.4f}, "
        f"{sw['centre_world'][2]:+.4f}) m;  r_xy = {np.hypot(*sw['centre_world'][:2]):.3f} m; "
        f"centre z = {mm(sw['centre_z_m']):+.2f} mm; board tilt from horizontal {sw['tilt_from_horizontal_deg']:.3f} deg")

    # depth cross-check at the detected corners (depth aligned to color)
    uv = np.rint(wrist_px_raw).astype(int)
    uv[:, 0] = uv[:, 0].clip(0, depth13.shape[1] - 1)
    uv[:, 1] = uv[:, 1].clip(0, depth13.shape[0] - 1)
    d_meas = depth13[uv[:, 1], uv[:, 0]].astype(float)
    p_cam = (wrist_cands[kw][:3, :3] @ obj.T).T + wrist_cands[kw][:3, 3]
    valid = d_meas > 0
    if valid.any():
        dz = d_meas[valid] - p_cam[valid, 2]
        say(f"  depth cross-check ({valid.sum()}/{len(d_meas)} valid corners): "
            f"depth - PnP z: median {mm(np.median(dz)):+.2f} mm, rms {mm(np.sqrt(np.mean(dz**2))):.2f} mm")

    # burst scatter of the wrist-chain board pose
    centres = []
    for i, rgb_i, K_i, T_i in scatter:
        try:
            px_i = ca._detect_checkerboard(rgb_i, columns=columns, rows=rows, label=f"pose13 f{i}")
        except ca.CameraAcceptanceError as exc:
            say(f"  scatter frame {i}: detection failed ({exc})")
            continue
        pxu_i = ca.undistort_realsense_pixels(
            px_i, K_i, width=rgb_i.shape[1], height=rgb_i.shape[0],
            distortion_model=wrist_model, distortion_coefficients=wrist_dist)
        cand_i = ca._checkerboard_pose_candidates(
            pxu_i, K_i, zero5, columns=columns, rows=rows, square_size_m=square)[0]
        Tw_i = T_i @ cand_i
        centres.append(Tw_i[:3, :3] @ centre_b + Tw_i[:3, 3])
    if len(centres) > 1:
        centres = np.array(centres)
        dev = np.linalg.norm(centres - centres.mean(axis=0), axis=1)
        say(f"  burst scatter over frames {scatter_idx}: board-centre spread max {mm(dev.max()):.2f} mm "
            f"from mean; z range {mm(centres[:,2].min()):+.2f}..{mm(centres[:,2].max()):+.2f} mm")

    # -------------------------------------------------------------------- 2. TOP
    say("")
    say("-" * 100)
    say(f"2. TOP-CHAIN BOARD POSE  (fresh BRIO burst, installed intrinsics + camera_to_world)")
    say("-" * 100)
    top_result: dict[str, Any] | None = None
    if args.no_camera:
        say("  --no-camera: skipping the fresh grab")
    else:
        try:
            from capture_top_intrinsics import (  # noqa: E402
                TOP_DEVICE, TOP_FOURCC, TOP_FPS, TOP_HEIGHT, TOP_SERIAL, TOP_WIDTH,
                BrioTopCamera,
            )
            say(f"  opening {TOP_DEVICE}")
            say(f"  (BrioTopCamera REFUSES any profile other than {TOP_WIDTH}x{TOP_HEIGHT}@{TOP_FPS} {TOP_FOURCC})")
            camera = BrioTopCamera(warmup_frames=30)
            try:
                frames = [camera.read() for _ in range(args.frames)]
            finally:
                camera.close()
            stack = np.stack([f["rgb"] for f in frames])
            span_ms = (frames[-1]["frame_monotonic_ns"] - frames[0]["frame_monotonic_ns"]) / 1e6
            say(f"  grabbed {len(frames)} frames ({stack.shape[1]}x{stack.shape[2]} verified) over {span_ms:.0f} ms")
            top_rgb = temporal_median_uint8(stack)
            cv2.imwrite(str(OUT_DIR / "heldout_top_median.png"), cv2.cvtColor(top_rgb, cv2.COLOR_RGB2BGR))
            say(f"  per-pixel temporal median -> {OUT_DIR / 'heldout_top_median.png'}")
            # per-frame detection for the board-motion check, then the median
            try:
                corner_stack = np.stack([
                    ca._detect_checkerboard(f["rgb"], columns=columns, rows=rows, label=f"top f{i}")
                    for i, f in enumerate(frames)
                ])
                drift = corner_drift_px(corner_stack)
                say(f"  board motion over the burst: {drift['drift_px']:.3f} px rms "
                    f"(worst corner {drift['max_corner_px']:.2f} px, centroid {drift['max_centroid_px']:.3f} px)")
            except ca.CameraAcceptanceError as exc:
                say(f"  per-frame detection incomplete ({exc}); continuing with the median image")
            top_px = ca._detect_checkerboard(top_rgb, columns=columns, rows=rows, label="top median")
            top_c = top_px.mean(axis=0)
            say(f"  median image: {len(top_px)} corners, board {ca_px_per_square(top_px, columns, rows):.2f} px/square, "
                f"centre ({top_c[0]:.1f},{top_c[1]:.1f}) px, offset "
                f"{np.linalg.norm(top_c - top_K[:2, 2]):.1f} px from the principal point")
            top_cands = ca._checkerboard_pose_candidates(
                top_px, top_K, top_dist, columns=columns, rows=rows, square_size_m=square)
            top_rms = [
                reproj_rms_px(px, T, top_K, top_dist, columns, rows, square)
                for px, T in zip((top_px, top_px[::-1]), top_cands)
            ]
            top_world = [T_wc_top @ T for T in top_cands]
            # disambiguation: candidate closest to the WRIST result (choose_checkerboard_orientation score)
            scores = []
            for T in top_world:
                d = np.linalg.inv(T_world_board_wrist) @ T
                scores.append(float(np.linalg.norm(d[:3, 3])
                                    + 0.1 * Rotation.from_matrix(d[:3, :3]).magnitude()))
            kt = int(np.argmin(scores))
            errs = [ca._transform_distance(T_world_board_wrist, T) for T in top_world]
            say(f"  corner-order candidates vs WRIST board pose "
                f"(score = |dt| + 0.1*rot_rad, as choose_checkerboard_orientation):")
            for j, ((t_e, r_e), s, r) in enumerate(zip(errs, scores, top_rms)):
                say(f"    candidate {j} ({'as-detected' if j == 0 else 'reversed'}): "
                    f"dt {mm(t_e):7.2f} mm, drot {r_e:8.3f} deg, score {s:.4f}, reproj RMS {r:.3f} px"
                    f"{'   <== selected' if j == kt else ''}")
            if abs(errs[1 - kt][1] - 180.0) > 90.0:
                say("  WARNING: the rejected candidate is NOT ~180 deg away - orientation disambiguation is weak here")
            T_world_board_top = top_world[kt]
            st = world_board_summary(T_world_board_top, centre_b)
            say("  T_world_board (top chain):")
            say(fmt_T(T_world_board_top))
            say(f"  board centre (world): ({st['centre_world'][0]:+.4f}, {st['centre_world'][1]:+.4f}, "
                f"{st['centre_world'][2]:+.4f}) m; centre z = {mm(st['centre_z_m']):+.2f} mm; "
                f"tilt from horizontal {st['tilt_from_horizontal_deg']:.3f} deg")
            top_result = {
                "T": T_world_board_top, "summary": st, "reproj": top_rms[kt],
                "errs": errs, "candidate": kt,
            }
        except ca.CameraAcceptanceError as exc:
            say(f"  BOARD NOT DETECTED in the fresh top frames: {exc}")
            say("  (the board was likely moved out of the top view; falling back to sections 1+4)")
        except Exception as exc:  # camera open/read failure
            say(f"  TOP CAMERA UNAVAILABLE: {type(exc).__name__}: {exc}")
            say("  (falling back to sections 1+4)")

    # ------------------------------------------------------------- 3. CROSS-CHECK
    cross: dict[str, Any] | None = None
    if top_result is not None:
        say("")
        say("-" * 100)
        say("3. CROSS-CHECK  wrist-chain vs top-chain world board pose")
        say("-" * 100)
        T_top = top_result["T"]
        t_err, r_err = ca._transform_distance(T_world_board_wrist, T_top)
        dt_world = T_top[:3, 3] - T_world_board_wrist[:3, 3]
        dc_world = top_result["summary"]["centre_world"] - sw["centre_world"]
        delta = np.linalg.inv(T_world_board_wrist) @ T_top
        rotvec = Rotation.from_matrix(delta[:3, :3]).as_rotvec()
        about_normal = float(np.degrees(abs(rotvec[2])))
        tilt_part = float(np.degrees(np.linalg.norm(rotvec[:2])))
        say(f"  translation: |dt| = {mm(t_err):.2f} mm;  origin dt (world) = "
        	f"({mm(dt_world[0]):+.2f}, {mm(dt_world[1]):+.2f}, {mm(dt_world[2]):+.2f}) mm;  "
        	f"centre dt (world) = ({mm(dc_world[0]):+.2f}, {mm(dc_world[1]):+.2f}, {mm(dc_world[2]):+.2f}) mm "
        	f"(|centre dt| = {mm(np.linalg.norm(dc_world)):.2f} mm)")
        say(f"  rotation:    {r_err:.3f} deg total = {about_normal:.3f} deg about the board normal "
            f"(in-plane) + {tilt_part:.3f} deg tilt")
        say(f"  board centre z: wrist chain {mm(sw['centre_z_m']):+.2f} mm | top chain "
            f"{mm(top_result['summary']['centre_z_m']):+.2f} mm | dz = "
            f"{mm(top_result['summary']['centre_z_m'] - sw['centre_z_m']):+.2f} mm")
        cross = {"t_m": t_err, "r_deg": r_err,
                 "dz_m": top_result["summary"]["centre_z_m"] - sw["centre_z_m"]}

    # ------------------------------------------------------------ 4. TABLE HEIGHT
    say("")
    say("-" * 100)
    say("4. TABLE HEIGHT from the 13 pair captures  (wrist-chain board-plane z, capture tool's tracker method)")
    say("-" * 100)
    ctx = load_quality_context()
    pair_paths = sorted(PAIRS_DIR.glob("pair*.npz"))
    say(f"  {len(pair_paths)} pairs in {PAIRS_DIR}")
    say(f"  {'pair':8s}{'z_mm':>9s}{'r_xy_m':>9s}{'tilt_deg':>9s}   (z = board-PATTERN plane, wrist chain)")
    zs: list[float] = []
    for p in pair_paths:
        with np.load(p, allow_pickle=False) as saved:
            g = board_geometry(saved["top_rgb"], saved["wrist_rgb"],
                               saved["wrist_intrinsics"], saved["wrist_camera_to_world"], ctx)
        zs.append(g["board_z_m"])
        say(f"  {p.stem:8s}{mm(g['board_z_m']):>9.2f}{g['board_r_xy_m']:>9.3f}"
            f"{g['board_tilt_from_horizontal_deg']:>9.2f}")
    zs_a = np.array(zs)
    z_med = float(np.median(zs_a))
    z_sd = float(zs_a.std(ddof=1))
    table_z = z_med - PLATE_THICKNESS_M
    say(f"  board-pattern z over {len(zs)} pairs: median {mm(z_med):+.2f} mm, mean {mm(zs_a.mean()):+.2f} mm, "
        f"sd {mm(z_sd):.2f} mm, range {mm(zs_a.min()):+.2f}..{mm(zs_a.max()):+.2f} mm "
        f"(spread {mm(zs_a.max() - zs_a.min()):.2f} mm)")
    say(f"  ==> board-pattern plane z (median)      = {mm(z_med):+.2f} mm")
    say(f"  ==> TABLE-SURFACE z estimate            = {mm(table_z):+.2f} mm "
        f"(median minus nominal {mm(PLATE_THICKNESS_M):.0f} mm plate thickness)")

    # ----------------------------------------------------------------- 5. VERDICT
    say("")
    say("=" * 100)
    say("5. VERDICT")
    say("=" * 100)
    flags: list[str] = []
    if cross is not None:
        ok_t = cross["t_m"] <= EXP_CROSS_T_M
        ok_r = cross["r_deg"] <= EXP_CROSS_R_DEG
        say(f"  cross-check wrist-vs-top board pose: dt {mm(cross['t_m']):.2f} mm "
            f"(expect <= {mm(EXP_CROSS_T_M):.0f}, top chain's own scatter ~9 mm) "
            f"[{'OK' if ok_t else 'FLAG'}] ; drot {cross['r_deg']:.3f} deg "
            f"(expect <= {EXP_CROSS_R_DEG:.0f}) [{'OK' if ok_r else 'FLAG'}]")
        if not ok_t:
            flags.append(f"cross-check translation {mm(cross['t_m']):.1f} mm > {mm(EXP_CROSS_T_M):.0f} mm")
        if not ok_r:
            flags.append(f"cross-check rotation {cross['r_deg']:.2f} deg > {EXP_CROSS_R_DEG:.0f} deg")
    else:
        say("  cross-check: NOT AVAILABLE (no top-chain board pose from the fresh frames); "
            "sections 1 and 4 only")
        flags.append("no fresh top-chain board pose (board not visible / camera unavailable)")
    dz13 = sw["centre_z_m"] - z_med
    ok_z = abs(dz13) <= EXP_POSE13_DZ_M
    say(f"  pose13 wrist-chain board z {mm(sw['centre_z_m']):+.2f} mm vs pairs' median {mm(z_med):+.2f} mm: "
        f"dz = {mm(dz13):+.2f} mm (expect |dz| <= {mm(EXP_POSE13_DZ_M):.0f}) [{'OK' if ok_z else 'FLAG'}]")
    if not ok_z:
        flags.append(f"pose13 board z off the table by {mm(dz13):+.1f} mm")
    ok_w = wrist_reproj <= EXP_REPROJ_RMS_PX
    say(f"  reprojection RMS: wrist chain (pose13, undistorted space) {wrist_reproj:.3f} px "
        f"[{'OK' if ok_w else 'FLAG'}]")
    if not ok_w:
        flags.append(f"wrist PnP reprojection RMS {wrist_reproj:.2f} px")
    if top_result is not None:
        ok_tp = top_result["reproj"] <= EXP_REPROJ_RMS_PX
        say(f"                    top chain (fresh median, raw+radtan space) {top_result['reproj']:.3f} px "
            f"[{'OK' if ok_tp else 'FLAG'}]")
        if not ok_tp:
            flags.append(f"top PnP reprojection RMS {top_result['reproj']:.2f} px")
        say(f"  orientation disambiguation: rejected top candidate sits "
            f"{top_result['errs'][1 - top_result['candidate']][1]:.1f} deg away (a flip would have shown here)")
    say(f"  table: pattern-plane z {mm(z_med):+.2f} mm, table-surface z {mm(table_z):+.2f} mm "
        f"(sd across pairs {mm(z_sd):.2f} mm)")
    say("")
    if flags:
        say("  OVERALL: ATTENTION - " + "; ".join(flags))
    else:
        say("  OVERALL: PASS - fresh held-out capture is consistent with BOTH installed chains")
    say(f"\nreport -> {OUT_DIR / 'heldout_pose13_report.txt'}")
    say.flush()
    return 0


def ca_px_per_square(pixels: np.ndarray, columns: int, rows: int) -> float:
    grid = np.asarray(pixels, float).reshape(rows, columns, 2)
    dx = np.linalg.norm(np.diff(grid, axis=1), axis=2)
    dy = np.linalg.norm(np.diff(grid, axis=0), axis=2)
    return float(np.concatenate([dx.ravel(), dy.ravel()]).mean())


if __name__ == "__main__":
    raise SystemExit(main())
