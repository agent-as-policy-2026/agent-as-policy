"""Standalone top-camera extrinsics PAIR capture (no bridge), per rig.

``--rig left`` (default, unchanged behaviour): the left wrist D405
(353322271204), the fixed top BRIO 178B0DAE and the i2rt robot on
can_follower_l -> calib/out/top_pairs/.  ``--rig right``: the RIGHT rig -
wrist D405 353322271910 (its OWN factory intrinsics are dumped live to
calib/out/right_d405_intrinsics.json), top BRIO B8C7F203, can_follower_r
(i2rt applies the right arm's joint-4 encoder offset by channel), station bodies
right_gripper/right_camera -> calib/out/top_pairs_right/.  world == the arm's
OWN base for both rigs (no station left/right offset).  The right-arm bridge (port 9020)
must be stopped first: the tool refuses to start while it listens, as
the left tools refuse while our left bridge listens on 9021.

The wrist D405, the fixed top BRIO (via capture_top_intrinsics.BrioTopCamera
at exactly 1920x1080@30 MJPG, focus verified/locked at start) and the i2rt
robot are opened, then the SAME g/h/c/q loop and safety invariants as
capture_left_handeye run (whose machinery is IMPORTED, not copied:
gravity/hold/limit-refusal/chain-liveness/die-without-motion/preview).

    g          gravity-comp idle: drag the arm so the WRIST sees the board
    h          hold: freeze at the CURRENT measured joint position
    c [label]  capture ONE pair (hold only) as the per-pixel TEMPORAL MEDIAN
               of --frames synchronized frames (default 20, ~0.7 s) ->
               calib/out/top_pairs/pairNN[_label].npz  (+ raw-stack sidecar
               calib/out/top_pairs/frames/pairNN[_label]_frames.npz)
    p          print checkerboard-detection status for BOTH cameras plus the
               live quality ADVICE (wrist distance/tilt, top px-per-square and
               centre offset, board reach r and plane z from the wrist chain)
    q          quit, leaving the arm in HOLD

Physical procedure: the 22 mm 9x7-inner board LIES ON THE TABLE and is moved
to a new spot between pairs (12-16 spots INSIDE the working region, r 0.25 -
0.50 m from the left base); the arm is repositioned per spot (g -> drag -> h)
so the wrist sees the board from 0.25-0.35 m at 20-35 deg.  The TOP CAMERA
NEVER MOVES.  A capture is refused (nothing written) unless BOTH cameras fully
detect the board in EVERY frame of the burst AND in the median image
(solver-identical two-scale detection), the board did not move during the
burst (corner drift <= 0.5 px rms), and every top/wrist frame pair agrees
within 0.1 s.  ADVICE lines never block a save (the operator decides).

Why the median (see calib/analysis/out/report.txt): the 9.4 mm RMS floor of
the first solve was ~5 mm single-frame corner noise on the ~85 px board in the
top image plus wrist-chain orientation scatter at far/steep poses times the
0.9 m top-camera lever arm.  Averaging 20 frames per camera removes the
temporal part of the corner noise while keeping the npz schema byte-identical
for the solver; the ADVICE/z lines steer the operator away from the far poses.

Each pairNN.npz is written in EXACTLY the schema the offline
``agp-yam-camera-acceptance calibrate-top-extrinsics`` solver consumes
(mirroring its ``capture-top-pair`` subcommand, which is not used because it
needs a running bridge):

    top_rgb               uint8[1080,1920,3] (RGB order; temporal median)
    top_serial            str scalar        "178B0DAE"
    top_device            str scalar        /dev/v4l/by-id/... path
    top_monotonic_ns      int64 scalar      (middle frame of the burst)
    wrist_rgb             uint8[360,640,3]  (RGB order, depth-aligned color;
                                            temporal median)
    wrist_intrinsics      float64[3,3]      factory color K of THIS D405
    wrist_camera_to_world float64[4,4]      FK(reported joints) @ station
                                            T_leftgripper_leftcamera (read
                                            LIVE from the measured MJCF);
                                            world == left_base
    wrist_serial          str scalar        "353322271204"
    wrist_monotonic_ns    int64 scalar      (middle frame of the burst)
    pair_skew_ns          int64 scalar      max over frames of |top - wrist|
                                            (gated at 0.1 s)

The raw stacks go to a SIDECAR one directory down
(``top_pairs/frames/pairNN_frames.npz``) so that no ``pair*.npz`` glob
(solve_top.sh, the residual analysis, this tool's numbering) ever sees them;
the solver never reads them.

Previews: wrist on port 8766, top on port 8767 (both MJPEG with detection
overlay), so the operator can confirm both cameras see the board before 'c'.

Run from the hardware-bridge uv project:
    cd hardware-bridge
    uv run --locked python ../calib/capture_top_pairs.py

Offline check (no hardware):  ... capture_top_pairs.py --dry-run
(reuses capture_left_handeye's FK check, self-checks the pair-npz schema
writer against the solver's required fields AND against the historical
pair01.npz, and exercises the median / drift-rejection / detection-rejection /
skew-rejection logic on synthetic frame stacks through the real capture path
with fake pumps and a fake robot - no command is ever sent, in any mode).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from left_handeye_common import (  # noqa: E402
    INTRINSICS_JSON,
    LEFT_CAMERA_SERIAL,
    LEFT_RIG,
    OUT_DIR,
    RIG_CHOICES,
    LeftArmFK,
    RigSpec,
    get_rig,
    load_intrinsics_json,
    rig_flange_from_camera,
    station_left_flange_from_camera,
)
from capture_left_handeye import (  # noqa: E402
    BOARD_COLUMNS,
    BOARD_ROWS,
    FramePump,
    LeftD405,
    DetectionCache,
    command_hold_at_measured,
    die_without_motion,
    enable_arm,
    ensure_chain_alive,
    print_board_status,
    read_command_space_joints,
    refuse_if_bridge_listening,
    say,
)
from capture_top_intrinsics import (  # noqa: E402
    TOP_HEIGHT,
    TOP_SERIAL,
    TOP_WIDTH,
    BrioTopCamera,
    ensure_focus_locked,
    print_rig,
    start_preview,
)

# LEFT defaults (module constants kept for existing importers); the right
# rig's paths come from left_handeye_common.RIGHT_RIG at runtime.
TOP_PAIRS_DIR = LEFT_RIG.top_pairs_dir
# Raw-stack sidecars live one level DOWN: solve_top.sh globs
# "$TOP_PAIRS_DIR"/pair*.npz (non-recursive), as do count_pairs /
# next_pair_path / the residual analysis, so nothing ever mistakes a sidecar
# for a pair.
FRAMES_SUBDIR = "frames"
TOP_INTRINSICS_JSON = LEFT_RIG.top_intrinsics_json
MAX_PAIR_SKEW_S = 0.1
PAIR_TARGET = "12-16"

# Multi-frame temporal median (item 1 of the recapture upgrade).
DEFAULT_FRAMES = 20
MIN_FRAMES = 1
# Board-motion gate over the burst: max over frames of the RMS-over-corners
# deviation from the per-corner temporal median (after corner-order
# alignment).  Measured static-scene floor on the 30-frame hand-eye stacks:
# 0.06-0.07 px (per-corner RMS 0.03 px, heavy-tailed single-corner outliers
# up to 0.37 px - which is why the gate is an RMS, not a max).
MAX_CORNER_DRIFT_PX = 0.5
BOARD_SQUARE_M = 0.022  # overridden by checkerboard.square_size_m of the top intrinsics report

# Live quality ADVICE thresholds (item 2).  Advice never blocks a save.
ADVICE_WRIST_DIST_M = (0.22, 0.38)      # wrist camera -> board centre distance
ADVICE_WRIST_TILT_DEG = (15.0, 40.0)    # board normal vs wrist optical axis
ADVICE_MIN_TOP_PX_PER_SQUARE = 30.0  # was 10 @640 wide; scaled x3 for 1920     # board scale in the top image
ADVICE_MAX_TOP_OFFSET_PX = 540.0  # was 180 @640 wide; scaled x3 for 1920        # board centre offset from the top principal point
ADVICE_MAX_BOARD_R_M = 0.50             # horizontal board reach from the left base
# Running board-plane-z consistency (item 3): |dz| vs the running median.
BOARD_Z_WARN_M = 0.005

# The exact field set calibrate-top-extrinsics requires in each capture
# (camera_acceptance._paired_capture_candidates).
SOLVER_REQUIRED_FIELDS = frozenset(
    {"top_rgb", "wrist_rgb", "wrist_intrinsics", "wrist_camera_to_world", "wrist_serial"}
)
# The full schema written, byte-compatible with the capture-top-pair CLI.
PAIR_FIELDS = SOLVER_REQUIRED_FIELDS | {
    "top_serial",
    "top_device",
    "top_monotonic_ns",
    "wrist_monotonic_ns",
    "pair_skew_ns",
}
# dtype kind / shape of every field as written by capture-top-pair and by the
# 2026-09-01 pair01.npz (strings are unicode scalars whose width follows the
# content, so only the kind 'U' is compared for them).
EXPECTED_PAIR_SCHEMA = {
    "top_rgb": ("uint8", (TOP_HEIGHT, TOP_WIDTH, 3)),
    "top_serial": ("U", ()),
    "top_device": ("U", ()),
    "top_monotonic_ns": ("int64", ()),
    "wrist_rgb": ("uint8", (360, 640, 3)),
    "wrist_intrinsics": ("float64", (3, 3)),
    "wrist_camera_to_world": ("float64", (4, 4)),
    "wrist_serial": ("U", ()),
    "wrist_monotonic_ns": ("int64", ()),
    "pair_skew_ns": ("int64", ()),
}


def next_pair_path(out_dir: Path, label: str | None) -> Path:
    """Auto-numbered ``pairNN[_label].npz`` (NN starts at 01)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    highest = 0
    for existing in out_dir.glob("pair*.npz"):
        digits = ""
        for char in existing.stem[len("pair"):]:
            if char.isdigit():
                digits += char
            else:
                break
        if digits:
            highest = max(highest, int(digits))
    suffix = ""
    if label:
        clean = "".join(c for c in label if c.isalnum() or c in "-_")
        if clean:
            suffix = f"_{clean}"
    return out_dir / f"pair{highest + 1:02d}{suffix}.npz"


def frames_sidecar_path(pair_path: Path) -> Path:
    """``<pairs dir>/frames/pairNN[_label]_frames.npz`` for one pair file."""
    return pair_path.parent / FRAMES_SUBDIR / f"{pair_path.stem}_frames.npz"


def write_pair_npz(
    path: Path,
    *,
    top_rgb: np.ndarray,
    top_serial: str,
    top_device: str,
    top_monotonic_ns: int,
    wrist_rgb: np.ndarray,
    wrist_intrinsics: np.ndarray,
    wrist_camera_to_world: np.ndarray,
    wrist_serial: str,
    wrist_monotonic_ns: int,
    pair_skew_ns: int,
) -> None:
    """Write one pair npz in exactly the solver's schema, then verify it.

    Field names, dtypes and shapes mirror camera_acceptance._capture_top_pair
    verbatim.  The written file is reloaded (allow_pickle=False) and checked
    against the solver's required-field set and rigidity gates; a file that
    fails verification is deleted, never left behind.
    """
    from agp_yam_bridge.camera_acceptance import _matrix, _rigid_transform

    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        top_rgb=np.ascontiguousarray(top_rgb),
        top_serial=np.array(top_serial),
        top_device=np.array(top_device),
        top_monotonic_ns=np.array(top_monotonic_ns, dtype=np.int64),
        wrist_rgb=np.ascontiguousarray(wrist_rgb),
        wrist_intrinsics=_matrix(wrist_intrinsics, (3, 3), "wrist_intrinsics"),
        wrist_camera_to_world=_rigid_transform(
            wrist_camera_to_world, "wrist_camera_to_world"
        ),
        wrist_serial=np.array(wrist_serial),
        wrist_monotonic_ns=np.array(wrist_monotonic_ns, dtype=np.int64),
        pair_skew_ns=np.array(pair_skew_ns, dtype=np.int64),
    )
    try:
        with np.load(path, allow_pickle=False) as saved:
            fields = set(saved.files)
            if fields != PAIR_FIELDS:
                raise RuntimeError(
                    f"pair schema mismatch: wrote {sorted(fields)}, "
                    f"expected {sorted(PAIR_FIELDS)}"
                )
            if not SOLVER_REQUIRED_FIELDS.issubset(fields):
                raise RuntimeError("pair is missing solver-required fields")
            # top frames follow the top-camera profile (1920x1080 since
            # 2026-09-01); the wrist D405 stays 640x360.
            for key, expected_shape in (
                ("top_rgb", (TOP_HEIGHT, TOP_WIDTH, 3)),
                ("wrist_rgb", (360, 640, 3)),
            ):
                image = saved[key]
                if image.dtype != np.uint8 or image.shape != expected_shape:
                    raise RuntimeError(
                        f"{key} is {image.dtype}{image.shape}, "
                        f"expected uint8[{expected_shape[0]},{expected_shape[1]},3]"
                    )
            _matrix(saved["wrist_intrinsics"], (3, 3), "saved wrist_intrinsics")
            _rigid_transform(
                saved["wrist_camera_to_world"], "saved wrist_camera_to_world"
            )
            if str(saved["wrist_serial"]) != wrist_serial:
                raise RuntimeError("saved wrist_serial mismatch")
            if str(saved["top_serial"]) != top_serial:
                raise RuntimeError("saved top_serial mismatch")
    except BaseException:
        path.unlink(missing_ok=True)  # never leave a bad npz behind
        raise


def pair_schema(path: Path) -> dict[str, tuple[str, tuple[int, ...]]]:
    """``{field: (dtype or 'U' for unicode, shape)}`` of a pair npz."""
    with np.load(path, allow_pickle=False) as saved:
        return {
            key: (
                "U" if saved[key].dtype.kind == "U" else str(saved[key].dtype),
                tuple(saved[key].shape),
            )
            for key in saved.files
        }


def count_pairs(out_dir: Path) -> int:
    return len(list(out_dir.glob("pair*.npz"))) if out_dir.is_dir() else 0


# --------------------------------------------------------------------------
# Multi-frame median + board-motion (drift) gate
# --------------------------------------------------------------------------
def temporal_median_uint8(stack: np.ndarray) -> np.ndarray:
    """Per-pixel temporal median of a uint8[N,H,W,C] stack, back to uint8.

    For even N numpy's median is the mean of the two middle values, so the
    result is rounded to the nearest integer (it is exact for odd N and for
    N == 1, where the median IS the frame).  Same shape/dtype as one frame, so
    the pair npz schema is unchanged for the solver.
    """
    frames = np.asarray(stack)
    if frames.dtype != np.uint8 or frames.ndim != 4 or frames.shape[0] < 1:
        raise ValueError(
            f"expected a uint8[N>=1,H,W,C] stack, got {frames.dtype}{frames.shape}"
        )
    if frames.shape[0] == 1:
        return np.ascontiguousarray(frames[0])
    median = np.median(frames, axis=0)
    return np.ascontiguousarray(np.rint(median).clip(0, 255).astype(np.uint8))


def align_corner_order(corners: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """``corners`` in whichever order (as detected / reversed) matches ``reference``.

    findChessboardCornersSB can return the 63 corners in the opposite order on
    some frames of a perfectly static scene (seen on the 2026-08-31 hand-eye
    stacks); the solver is immune (it tries both orders), and so must the
    drift gate be.
    """
    forward = float(np.abs(corners - reference).max())
    backward = float(np.abs(corners[::-1] - reference).max())
    return corners if forward <= backward else corners[::-1]


def corner_drift_px(corner_stack: np.ndarray) -> dict:
    """Board motion over a burst from the per-frame corner detections.

    ``corner_stack`` is float64[N, corners, 2] as detected.  After aligning the
    corner order of every frame to frame 0, deviations are taken from the
    per-corner TEMPORAL MEDIAN.  ``drift_px`` (the gated number) is the max
    over frames of the RMS over corners of that deviation: a rigid board (or
    wrist-camera) motion of d px moves all corners coherently and reads ~d,
    while independent per-corner detection noise (static floor 0.06-0.07 px
    rms on the D405 stacks) stays far below MAX_CORNER_DRIFT_PX.  The worst
    single-corner deviation and the centroid drift are returned for display.
    """
    stack = np.asarray(corner_stack, dtype=np.float64)
    if stack.ndim != 3 or stack.shape[0] < 1 or stack.shape[2] != 2:
        raise ValueError(f"expected float[N,corners,2], got {stack.shape}")
    aligned = np.stack([align_corner_order(frame, stack[0]) for frame in stack])
    median = np.median(aligned, axis=0)
    deviation = aligned - median
    magnitude = np.linalg.norm(deviation, axis=2)  # (N, corners)
    per_frame_rms = np.sqrt(np.mean(magnitude**2, axis=1))
    centroid = np.linalg.norm(deviation.mean(axis=1), axis=1)
    return {
        "drift_px": float(per_frame_rms.max()),
        "per_frame_rms_px": per_frame_rms,
        "max_corner_px": float(magnitude.max()),
        "max_centroid_px": float(centroid.max()),
        "median_corners": median,
    }


def grab_frame_stack(
    wrist_pump: FramePump, top_pump: FramePump, frames: int
) -> tuple[list[dict], list[dict]]:
    """``frames`` fresh top+wrist frame pairs at camera rate (read-only).

    Every sample waits for a STRICTLY newer pump sequence on each camera, so
    all frames are captured after the command was issued and are distinct by
    construction (a stalled pump raises instead of handing back an old frame).
    """
    if frames < MIN_FRAMES:
        raise ValueError(f"frames must be >= {MIN_FRAMES}, got {frames}")
    top_seq = top_pump.latest_sequence()
    wrist_seq = wrist_pump.latest_sequence()
    top_frames: list[dict] = []
    wrist_frames: list[dict] = []
    for _ in range(frames):
        top_frame = top_pump.wait_for_frame_after(top_seq)
        top_seq = int(top_frame["pump_sequence"])
        wrist_frame = wrist_pump.wait_for_frame_after(wrist_seq)
        wrist_seq = int(wrist_frame["pump_sequence"])
        top_frames.append(top_frame)
        wrist_frames.append(wrist_frame)
    return top_frames, wrist_frames


def write_frames_sidecar(
    path: Path,
    *,
    pair_path: Path,
    top_frames: list[dict],
    wrist_frames: list[dict],
    top_corners: np.ndarray,
    wrist_corners: np.ndarray,
    top_drift: dict,
    wrist_drift: dict,
    joint_pos: np.ndarray,
) -> None:
    """Raw per-frame stacks + detections next to (below) the pair; never read
    by the solver.  Loadable with allow_pickle=False."""
    top_ts = np.array([int(f["frame_monotonic_ns"]) for f in top_frames], dtype=np.int64)
    wrist_ts = np.array([int(f["frame_monotonic_ns"]) for f in wrist_frames], dtype=np.int64)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        pair_file=np.array(pair_path.name),
        note=np.array(
            "raw frame stacks behind ../{pair}: the solver reads ONLY the "
            "temporal-median pair npz one directory up".format(pair=pair_path.name)
        ),
        top_rgb_frames=np.stack([f["rgb"] for f in top_frames]),
        wrist_rgb_frames=np.stack([f["rgb"] for f in wrist_frames]),
        top_monotonic_ns=top_ts,
        wrist_monotonic_ns=wrist_ts,
        pair_skew_ns=np.abs(top_ts - wrist_ts),
        top_corners_px=np.asarray(top_corners, dtype=np.float64),
        wrist_corners_px=np.asarray(wrist_corners, dtype=np.float64),
        top_drift_px=np.array(top_drift["drift_px"], dtype=np.float64),
        wrist_drift_px=np.array(wrist_drift["drift_px"], dtype=np.float64),
        top_per_frame_drift_px=np.asarray(top_drift["per_frame_rms_px"], dtype=np.float64),
        wrist_per_frame_drift_px=np.asarray(wrist_drift["per_frame_rms_px"], dtype=np.float64),
        joint_pos_rad=np.asarray(joint_pos, dtype=np.float64),
    )
    with np.load(path, allow_pickle=False) as saved:  # must reload cleanly
        if saved["top_rgb_frames"].shape[0] != len(top_frames):
            raise RuntimeError("sidecar frame count mismatch")


# --------------------------------------------------------------------------
# Live quality advice (solver-identical detection / undistortion / PnP)
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class QualityContext:
    """Static inputs of the quality advice (offline: JSON files + constants)."""

    wrist_distortion_model: str
    wrist_distortion: np.ndarray          # (5,) inverse_brown_conrady coefficients
    top_intrinsics: np.ndarray | None     # (3,3) from the top intrinsics report, else None
    top_distortion: np.ndarray | None     # top radtan coefficients, else None
    top_center: tuple[float, float]       # principal point, else the nominal stream centre
    top_source: str
    square_size_m: float = BOARD_SQUARE_M
    columns: int = BOARD_COLUMNS
    rows: int = BOARD_ROWS
    base_label: str = "left base"         # operator text for the world/base frame


def load_quality_context(
    wrist_intrinsics_json: Path = INTRINSICS_JSON,
    top_intrinsics_json: Path = TOP_INTRINSICS_JSON,
    *,
    allow_missing_wrist: bool = False,
    wrist_serial: str = LEFT_CAMERA_SERIAL,
    base_label: str = "left base",
) -> QualityContext:
    """Wrist distortion from <side>_d405_intrinsics.json (validated against
    ``wrist_serial``, 5 coefficients); top K/distortion/square size from the
    top intrinsics report when present, else the stream's nominal centre (no
    top PnP then)."""
    if wrist_intrinsics_json.is_file():
        wrist = load_intrinsics_json(wrist_intrinsics_json, wrist_serial)
        wrist_model = str(wrist["distortion_model"])
        wrist_dist = np.asarray(wrist["distortion_coefficients"], dtype=np.float64)
    elif allow_missing_wrist:
        say(f"  (no {wrist_intrinsics_json}: quality advice uses an undistorted wrist model)")
        wrist_model, wrist_dist = "none", np.zeros(5)
    else:
        raise FileNotFoundError(wrist_intrinsics_json)

    top_K: np.ndarray | None = None
    top_dist: np.ndarray | None = None
    center = (TOP_WIDTH / 2.0, TOP_HEIGHT / 2.0)
    square = BOARD_SQUARE_M
    source = f"nominal stream centre (no {top_intrinsics_json.name})"
    if top_intrinsics_json.is_file():
        payload = json.loads(top_intrinsics_json.read_text(encoding="utf-8"))
        K = np.asarray(payload.get("camera_matrix", []), dtype=np.float64)
        if K.shape == (3, 3) and np.isfinite(K).all():
            top_K = K
            top_dist = np.asarray(payload.get("distortion_coefficients", np.zeros(5)), dtype=np.float64)
            center = (float(K[0, 2]), float(K[1, 2]))
            source = f"{top_intrinsics_json.name} principal point"
            board = payload.get("checkerboard", {})
            if (
                int(board.get("columns", BOARD_COLUMNS)) != BOARD_COLUMNS
                or int(board.get("rows", BOARD_ROWS)) != BOARD_ROWS
            ):
                say(
                    f"  WARNING: {top_intrinsics_json.name} checkerboard "
                    f"{board.get('columns')}x{board.get('rows')} != tool's "
                    f"{BOARD_COLUMNS}x{BOARD_ROWS}"
                )
            square = float(board.get("square_size_m", square))
        else:
            say(f"  WARNING: {top_intrinsics_json} has no valid camera_matrix; using the nominal centre")
    return QualityContext(
        wrist_distortion_model=wrist_model,
        wrist_distortion=wrist_dist,
        top_intrinsics=top_K,
        top_distortion=top_dist,
        top_center=center,
        top_source=source,
        square_size_m=square,
        base_label=base_label,
    )


def px_per_square(pixels: np.ndarray, columns: int, rows: int) -> float:
    """Mean adjacent-corner spacing (same definition as the residual analysis)."""
    grid = np.asarray(pixels, dtype=np.float64).reshape(rows, columns, 2)
    dx = np.linalg.norm(np.diff(grid, axis=1), axis=2)
    dy = np.linalg.norm(np.diff(grid, axis=0), axis=2)
    return float(np.concatenate([dx.ravel(), dy.ravel()]).mean())


def _tilt_deg(camera_from_target: np.ndarray) -> float:
    """Angle between the camera optical axis and the board normal."""
    return float(np.degrees(np.arccos(np.clip(abs(camera_from_target[2, 2]), 0.0, 1.0))))


def board_geometry(
    top_rgb: np.ndarray,
    wrist_rgb: np.ndarray,
    wrist_intrinsics: np.ndarray,
    wrist_camera_to_world: np.ndarray,
    ctx: QualityContext,
) -> dict:
    """The operator-facing pose numbers from the solver's OWN helpers.

    Detection (_detect_checkerboard, two-scale), wrist undistortion
    (undistort_realsense_pixels with the inverse_brown_conrady coefficients)
    and IPPE PnP (_checkerboard_pose_candidates) are exactly the solver's
    call sequence, so what is judged here is what the solver will see.
    Raises CameraAcceptanceError when either camera misses the full board.
    Both PnP orderings share the board centre and |normal|, so candidate 0
    is used throughout.
    """
    from agp_yam_bridge.camera_acceptance import (
        _checkerboard_pose_candidates,
        _detect_checkerboard,
        _matrix,
        _rigid_transform,
        undistort_realsense_pixels,
    )

    K = _matrix(wrist_intrinsics, (3, 3), "wrist_intrinsics")
    world_from_wrist = _rigid_transform(wrist_camera_to_world, "wrist_camera_to_world")
    columns, rows, square = ctx.columns, ctx.rows, ctx.square_size_m
    top_px = _detect_checkerboard(top_rgb, columns=columns, rows=rows, label="top")
    wrist_px_raw = _detect_checkerboard(wrist_rgb, columns=columns, rows=rows, label="wrist")
    if ctx.wrist_distortion_model == "inverse_brown_conrady":
        wrist_px = undistort_realsense_pixels(
            wrist_px_raw,
            K,
            width=int(wrist_rgb.shape[1]),
            height=int(wrist_rgb.shape[0]),
            distortion_model=ctx.wrist_distortion_model,
            distortion_coefficients=ctx.wrist_distortion,
        )
    else:
        wrist_px = wrist_px_raw
    wrist_from_board = _checkerboard_pose_candidates(
        wrist_px, K, np.zeros(5), columns=columns, rows=rows, square_size_m=square
    )[0]
    centre_board = np.array([(columns - 1) * square / 2.0, (rows - 1) * square / 2.0, 0.0])
    centre_wrist = wrist_from_board[:3, :3] @ centre_board + wrist_from_board[:3, 3]
    world_from_board = world_from_wrist @ wrist_from_board
    centre_world = world_from_board[:3, :3] @ centre_board + world_from_board[:3, 3]
    normal_world = world_from_board[:3, :3] @ np.array([0.0, 0.0, 1.0])

    top_centre = top_px.mean(axis=0)
    geometry = {
        "wrist_dist_m": float(np.linalg.norm(centre_wrist)),
        "wrist_tilt_deg": _tilt_deg(wrist_from_board),
        "wrist_px_per_square": px_per_square(wrist_px_raw, columns, rows),
        "wrist_centre_uv": tuple(float(v) for v in wrist_px_raw.mean(axis=0)),
        "top_px_per_square": px_per_square(top_px, columns, rows),
        "top_centre_uv": (float(top_centre[0]), float(top_centre[1])),
        "top_offset_px": float(np.linalg.norm(top_centre - np.asarray(ctx.top_center))),
        "top_dist_m": None,
        "top_tilt_deg": None,
        "board_centre_world_m": centre_world,
        "board_r_xy_m": float(np.hypot(centre_world[0], centre_world[1])),
        "board_z_m": float(centre_world[2]),
        "board_tilt_from_horizontal_deg": float(
            np.degrees(np.arccos(np.clip(abs(normal_world[2]), 0.0, 1.0)))
        ),
        "top_px": top_px,
        "wrist_px_raw": wrist_px_raw,
    }
    if ctx.top_intrinsics is not None and ctx.top_distortion is not None:
        top_from_board = _checkerboard_pose_candidates(
            top_px, ctx.top_intrinsics, ctx.top_distortion,
            columns=columns, rows=rows, square_size_m=square,
        )[0]
        centre_top = top_from_board[:3, :3] @ centre_board + top_from_board[:3, 3]
        geometry["top_dist_m"] = float(np.linalg.norm(centre_top))
        geometry["top_tilt_deg"] = _tilt_deg(top_from_board)
    return geometry


def advice_lines(geometry: dict, base_label: str = "left base") -> list[str]:
    """ADVICE strings for every threshold the pose violates (may be empty)."""
    lines: list[str] = []
    lo, hi = ADVICE_WRIST_DIST_M
    dist = geometry["wrist_dist_m"]
    if not lo <= dist <= hi:
        lines.append(
            f"ADVICE: wrist->board distance {dist:.3f} m is outside {lo:.2f}-{hi:.2f} m "
            f"-> move the wrist {'CLOSER' if dist > hi else 'FURTHER AWAY'} "
            "(the board should fill a good part of the wrist frame)"
        )
    lo, hi = ADVICE_WRIST_TILT_DEG
    tilt = geometry["wrist_tilt_deg"]
    if not lo <= tilt <= hi:
        lines.append(
            f"ADVICE: wrist tilt {tilt:.1f} deg is outside {lo:.0f}-{hi:.0f} deg "
            f"-> view the board {'MORE obliquely' if tilt < lo else 'LESS steeply'}"
        )
    pps = geometry["top_px_per_square"]
    if pps < ADVICE_MIN_TOP_PX_PER_SQUARE:
        lines.append(
            f"ADVICE: top-image board scale {pps:.2f} px/square < "
            f"{ADVICE_MIN_TOP_PX_PER_SQUARE:.0f} -> the board is too small/far in the "
            "top image; move it toward the top camera / image centre"
        )
    offset = geometry["top_offset_px"]
    if offset > ADVICE_MAX_TOP_OFFSET_PX:
        lines.append(
            f"ADVICE: top-image board centre is {offset:.0f} px from the principal point "
            f"(> {ADVICE_MAX_TOP_OFFSET_PX:.0f}) -> move the board toward the centre of "
            "the top image (central ~60%)"
        )
    r = geometry["board_r_xy_m"]
    if r > ADVICE_MAX_BOARD_R_M:
        lines.append(
            f"ADVICE: board centre r = {r:.3f} m from the {base_label} > "
            f"{ADVICE_MAX_BOARD_R_M:.2f} m -> far reach amplifies wrist-chain error x the "
            "0.9 m lever arm; place the board inside the working region (0.25-0.50 m)"
        )
    return lines


def print_quality(geometry: dict, heading: str, ctx: QualityContext) -> list[str]:
    """Print the pose numbers and the ADVICE lines; returns the advice."""
    say(heading)
    say(
        f"  wrist->board {geometry['wrist_dist_m']:.3f} m "
        f"(target {ADVICE_WRIST_DIST_M[0]:.2f}-{ADVICE_WRIST_DIST_M[1]:.2f}), "
        f"tilt {geometry['wrist_tilt_deg']:.1f} deg "
        f"(target {ADVICE_WRIST_TILT_DEG[0]:.0f}-{ADVICE_WRIST_TILT_DEG[1]:.0f}), "
        f"board {geometry['wrist_px_per_square']:.1f} px/square, centre "
        f"({geometry['wrist_centre_uv'][0]:.0f},{geometry['wrist_centre_uv'][1]:.0f}) px"
    )
    top_line = (
        f"  top image:   board {geometry['top_px_per_square']:.2f} px/square "
        f"(min {ADVICE_MIN_TOP_PX_PER_SQUARE:.0f}), centre "
        f"({geometry['top_centre_uv'][0]:.0f},{geometry['top_centre_uv'][1]:.0f}) px, "
        f"offset {geometry['top_offset_px']:.0f} px from the "
        f"{'principal point' if ctx.top_intrinsics is not None else 'nominal centre'} "
        f"(max {ADVICE_MAX_TOP_OFFSET_PX:.0f})"
    )
    if geometry["top_dist_m"] is not None:
        top_line += (
            f"; top->board {geometry['top_dist_m']:.3f} m, tilt "
            f"{geometry['top_tilt_deg']:.1f} deg"
        )
    say(top_line)
    c = geometry["board_centre_world_m"]
    say(
        f"  board (world, via wrist chain): centre ({c[0]:+.3f},{c[1]:+.3f},"
        f"{c[2]:+.4f}) m, r = {geometry['board_r_xy_m']:.3f} m from the {ctx.base_label} "
        f"(max {ADVICE_MAX_BOARD_R_M:.2f}), plane z {geometry['board_z_m'] * 1000:+.1f} mm, "
        f"tilt {geometry['board_tilt_from_horizontal_deg']:.2f} deg from horizontal"
    )
    advice = advice_lines(geometry, ctx.base_label)
    for line in advice:
        say("  " + line)
    if not advice:
        say("  (no ADVICE: pose within all targets)")
    return advice


class BoardZTracker:
    """Running board-plane z (world) over the saved pairs.

    The board lies on one flat table, so a tight z across pairs is the
    operator-visible consistency check of the wrist chain (FK + hand-eye);
    a pose whose z deviates > BOARD_Z_WARN_M from the running median of the
    earlier pairs is flagged as a wrist-chain warning for THAT pose.
    """

    def __init__(self) -> None:
        self._entries: list[tuple[str, float]] = []

    def __len__(self) -> int:
        return len(self._entries)

    def add(self, name: str, z_m: float) -> dict:
        previous = np.array([z for _, z in self._entries], dtype=np.float64)
        self._entries.append((name, float(z_m)))
        zs = np.array([z for _, z in self._entries], dtype=np.float64)
        report = {
            "name": name,
            "z_m": float(z_m),
            "count": int(len(zs)),
            "mean_m": float(zs.mean()),
            "sd_m": float(zs.std(ddof=1)) if len(zs) > 1 else 0.0,
            "median_m": float(np.median(zs)),
            "min_m": float(zs.min()),
            "max_m": float(zs.max()),
            "dz_vs_running_median_m": None,
            "flag": False,
        }
        if len(previous):
            dz = float(z_m - np.median(previous))
            report["dz_vs_running_median_m"] = dz
            report["running_median_m"] = float(np.median(previous))
            report["flag"] = abs(dz) > BOARD_Z_WARN_M
        return report

    def seed_from_dir(self, out_dir: Path, ctx: QualityContext) -> int:
        """Add every readable pair*.npz already in ``out_dir`` (sorted)."""
        from agp_yam_bridge.camera_acceptance import CameraAcceptanceError

        added = 0
        for path in sorted(out_dir.glob("pair*.npz")) if out_dir.is_dir() else []:
            try:
                with np.load(path, allow_pickle=False) as saved:
                    geometry = board_geometry(
                        saved["top_rgb"], saved["wrist_rgb"],
                        saved["wrist_intrinsics"], saved["wrist_camera_to_world"], ctx,
                    )
            except (CameraAcceptanceError, KeyError, ValueError, OSError) as exc:
                say(f"  (z tracker: skipping {path.name}: {type(exc).__name__}: {exc})")
                continue
            self.add(path.stem, geometry["board_z_m"])
            added += 1
        return added

    def summary(self) -> str:
        if not self._entries:
            return "board plane z: no pairs yet"
        zs = np.array([z for _, z in self._entries]) * 1000.0
        sd = zs.std(ddof=1) if len(zs) > 1 else 0.0
        return (
            f"board plane z over {len(zs)} pair(s): mean {zs.mean():+.1f} mm, "
            f"sd {sd:.1f} mm, median {np.median(zs):+.1f} mm, "
            f"range {zs.min():+.1f}..{zs.max():+.1f} mm"
        )


def print_board_z(report: dict) -> None:
    z_mm = report["z_m"] * 1000.0
    line = (
        f"board plane z (world, wrist chain) for {report['name']}: {z_mm:+.1f} mm | "
        f"running over {report['count']} pair(s): mean {report['mean_m'] * 1000:+.1f} mm, "
        f"sd {report['sd_m'] * 1000:.1f} mm, median {report['median_m'] * 1000:+.1f} mm, "
        f"range {report['min_m'] * 1000:+.1f}..{report['max_m'] * 1000:+.1f} mm"
    )
    say(line)
    if report["dz_vs_running_median_m"] is not None:
        dz_mm = report["dz_vs_running_median_m"] * 1000.0
        if report["flag"]:
            say(
                f"  WARNING (wrist chain): this pose's board z is {dz_mm:+.1f} mm from the "
                f"running median of the earlier pairs ({report['running_median_m'] * 1000:+.1f} mm), "
                f"beyond {BOARD_Z_WARN_M * 1000:.0f} mm - a flat table must give a tight z. "
                "Re-pose the arm (closer, less steep, r <= 0.50 m) and re-capture this "
                "spot; delete this pair by hand if you replace it."
            )
        else:
            say(
                f"  dz vs running median: {dz_mm:+.1f} mm "
                f"(within {BOARD_Z_WARN_M * 1000:.0f} mm)"
            )


def print_live_quality(
    wrist_pump,
    top_pump,
    robot,
    fk: LeftArmFK,
    flange_from_camera: np.ndarray,
    ctx: QualityContext,
) -> None:
    """'p': quality advice from the latest frame of each camera (read-only:
    the only robot access is get_observations)."""
    from agp_yam_bridge.camera_acceptance import CameraAcceptanceError

    wrist_frame = wrist_pump.latest()
    top_frame = top_pump.latest()
    if wrist_frame is None or top_frame is None:
        say("quality advice: no camera frames yet")
        return
    pos, _, _, _ = read_command_space_joints(robot)
    camera_to_world = fk.base_from_gripper(pos[:6]) @ flange_from_camera
    try:
        geometry = board_geometry(
            top_frame["rgb"], wrist_frame["rgb"], wrist_frame["intrinsics"],
            camera_to_world, ctx,
        )
    except CameraAcceptanceError as exc:
        say(f"quality advice unavailable: {exc}")
        return
    print_quality(geometry, "quality advice (one live frame per camera):", ctx)


# --------------------------------------------------------------------------
# Capture
# --------------------------------------------------------------------------
def capture_pair(
    wrist_pump,
    top_pump,
    robot,
    fk: LeftArmFK,
    flange_from_camera: np.ndarray,
    *,
    out_dir: Path,
    label: str | None,
    frames: int = DEFAULT_FRAMES,
    quality: QualityContext | None = None,
    z_tracker: BoardZTracker | None = None,
) -> Path | None:
    """One wrist+top pair as the temporal median of ``frames`` synchronized
    frames; refuses (returns None, nothing written) on any gate.

    Gates, in order: per-frame top/wrist skew <= 0.1 s (frame pairs beyond
    the gate are DROPPED as transient camera stalls; the capture is refused
    only when fewer than ~60% well-synced pairs remain); full-board
    detection in EVERY kept frame of both cameras; board drift over the
    burst <= MAX_CORNER_DRIFT_PX in both cameras; full-board detection in
    both MEDIAN images (what the solver will actually see).  ADVICE never
    gates.
    Read-only with respect to the robot: only get_observations is called.
    """
    from agp_yam_bridge.camera_acceptance import (
        CameraAcceptanceError,
        _detect_checkerboard,
    )

    if frames < MIN_FRAMES:
        raise ValueError(f"frames must be >= {MIN_FRAMES}, got {frames}")

    # 1. Burst: fresh frames captured AFTER the command was issued, at camera
    #    rate; the joints are read before and after so a drifting arm shows.
    pos_before, _, _, _ = read_command_space_joints(robot)
    started = time.monotonic()
    top_frames, wrist_frames = grab_frame_stack(wrist_pump, top_pump, frames)
    burst_s = time.monotonic() - started
    pos, _, _, joint_monotonic_ns = read_command_space_joints(robot)
    joint_move_deg = float(np.degrees(np.abs(pos[:6] - pos_before[:6]).max()))

    top_ts = np.array([int(f["frame_monotonic_ns"]) for f in top_frames], dtype=np.int64)
    wrist_ts = np.array([int(f["frame_monotonic_ns"]) for f in wrist_frames], dtype=np.int64)
    skews = np.abs(top_ts - wrist_ts)
    say(
        f"burst: {frames} frame pair(s) in {burst_s:.2f} s; pair skew max "
        f"{skews.max() / 1e6:.1f} ms (gate {MAX_PAIR_SKEW_S * 1e3:.0f} ms per frame), "
        f"mean {skews.mean() / 1e6:.1f} ms; wrist-joint skew "
        f"{abs(int(wrist_ts[-1]) - joint_monotonic_ns) / 1e6:.1f} ms; joints moved "
        f"{joint_move_deg:.3f} deg during the burst"
    )
    # A transient stall on one camera (a dropped or buffered frame — the BRIO
    # over V4L2/MJPG does this sporadically) gives a FEW pairings a large skew
    # while the scene is provably static: the corner-drift gate below is the
    # real static-scene guarantee.  Drop the offending frame pairs instead of
    # refusing the whole burst; refuse only when too few well-synced pairs
    # remain for a meaningful median.
    keep = np.nonzero(skews <= int(MAX_PAIR_SKEW_S * 1e9))[0]
    min_keep = frames if frames < 8 else max(8, (frames * 3) // 5)
    if len(keep) < min_keep:
        say(
            f"REFUSED: only {len(keep)}/{frames} frame pairs inside the "
            f"{MAX_PAIR_SKEW_S * 1e3:.0f} ms skew gate (need >= {min_keep}); "
            "nothing was saved."
        )
        say("(check for a stalled camera, then press 'c' again)")
        return None
    if len(keep) < frames:
        say(
            f"  dropped {frames - len(keep)} frame pair(s) with transient skew > "
            f"{MAX_PAIR_SKEW_S * 1e3:.0f} ms; continuing with {len(keep)}."
        )
        top_frames = [top_frames[i] for i in keep]
        wrist_frames = [wrist_frames[i] for i in keep]
        top_ts = top_ts[keep]
        wrist_ts = wrist_ts[keep]
    n_kept = len(keep)
    pair_skew_ns = int(np.abs(top_ts - wrist_ts).max())

    # 2. Solver-identical detection on EVERY frame of BOTH cameras.
    corner_stacks: dict[str, np.ndarray] = {}
    for label_name, frame_list in (("top", top_frames), ("wrist", wrist_frames)):
        corners = []
        for index, frame in enumerate(frame_list):
            try:
                corners.append(
                    _detect_checkerboard(
                        frame["rgb"], columns=BOARD_COLUMNS, rows=BOARD_ROWS,
                        label=f"{label_name} frame {index + 1}/{n_kept}",
                    )
                )
            except CameraAcceptanceError as exc:
                say(f"REFUSED: {exc}")
                say("adjust the board/arm so BOTH cameras see the full board in every "
                    "frame (previews: wrist :8766, top :8767); nothing was saved.")
                return None
        corner_stacks[label_name] = np.stack(corners)

    # 3. Board-motion gate.
    drift = {name: corner_drift_px(stack) for name, stack in corner_stacks.items()}
    for name, d in drift.items():
        say(
            f"  {name:5s} board drift over the burst: {d['drift_px']:.3f} px rms "
            f"(gate {MAX_CORNER_DRIFT_PX:.1f}; worst corner {d['max_corner_px']:.2f} px, "
            f"centroid {d['max_centroid_px']:.3f} px)"
        )
    moved = [name for name, d in drift.items() if d["drift_px"] > MAX_CORNER_DRIFT_PX]
    if moved:
        say(f"REFUSED: the board moved between frames in the {' and '.join(moved)} "
            "image(s); nothing was saved.")
        say("hold still (hands off the board, arm settled in HOLD) for ~1 s and press "
            "'c' again.")
        return None

    # 4. Temporal medians (uint8, same shape as one frame -> schema unchanged).
    top_rgb = temporal_median_uint8(np.stack([f["rgb"] for f in top_frames]))
    wrist_rgb = temporal_median_uint8(np.stack([f["rgb"] for f in wrist_frames]))
    wrist_intrinsics = wrist_frames[-1]["intrinsics"]

    # world == left_base: FK of the reported (offset-corrected) joints times
    # the MEASURED station T_leftgripper_leftcamera read live at startup.
    camera_to_world = fk.base_from_gripper(pos[:6]) @ flange_from_camera

    # 5. The median images must detect too (they are what the solver sees);
    #    the same detections feed the quality advice.
    geometry: dict | None = None
    if quality is not None:
        try:
            geometry = board_geometry(top_rgb, wrist_rgb, wrist_intrinsics, camera_to_world, quality)
        except CameraAcceptanceError as exc:
            say(f"REFUSED: median image: {exc}; nothing was saved.")
            return None
        print_quality(geometry, "quality of THIS capture (median images) - read before trusting it:", quality)
    else:
        for label_name, rgb in (("top", top_rgb), ("wrist", wrist_rgb)):
            try:
                _detect_checkerboard(rgb, columns=BOARD_COLUMNS, rows=BOARD_ROWS, label=f"{label_name} median")
            except CameraAcceptanceError as exc:
                say(f"REFUSED: {exc}; nothing was saved.")
                return None

    # 6. Write the pair (solver schema), then the raw-stack sidecar.
    middle = n_kept // 2
    path = next_pair_path(out_dir, label)
    write_pair_npz(
        path,
        top_rgb=top_rgb,
        top_serial=top_pump.serial,
        top_device=top_pump.device,
        top_monotonic_ns=int(top_ts[middle]),
        wrist_rgb=wrist_rgb,
        wrist_intrinsics=wrist_intrinsics,
        wrist_camera_to_world=camera_to_world,
        wrist_serial=wrist_pump.serial,
        wrist_monotonic_ns=int(wrist_ts[middle]),
        pair_skew_ns=pair_skew_ns,
    )
    say(f"saved {path}  (temporal median of {n_kept} frame(s) per camera)")
    sidecar = frames_sidecar_path(path)
    try:
        write_frames_sidecar(
            sidecar,
            pair_path=path,
            top_frames=top_frames,
            wrist_frames=wrist_frames,
            top_corners=corner_stacks["top"],
            wrist_corners=corner_stacks["wrist"],
            top_drift=drift["top"],
            wrist_drift=drift["wrist"],
            joint_pos=pos,
        )
        say(f"raw frame stacks -> {sidecar}  (sidecar; the solver never reads it)")
    except Exception as exc:  # noqa: BLE001  the pair itself is the deliverable
        sidecar.unlink(missing_ok=True)
        say(f"WARNING: raw-frame sidecar not written ({type(exc).__name__}: {exc}); "
            "the pair itself IS saved.")

    # 7. Running board-plane-z consistency indicator.
    if z_tracker is not None and geometry is not None:
        print_board_z(z_tracker.add(path.stem, geometry["board_z_m"]))
    say(f"pairs on disk: {count_pairs(out_dir)} (target {PAIR_TARGET}, inside the "
        "working region r 0.25-0.50 m)")
    return path


# --------------------------------------------------------------------------
# Dry run: FK check (reused) + pair-schema self-check + median/drift logic
# --------------------------------------------------------------------------
class _SyntheticPump:
    """Stands in for FramePump in --dry-run: serves a scripted frame list.

    ``wait_for_frame_after`` hands out the next scripted frame with a strictly
    increasing pump sequence (the last frame repeats once the list is
    exhausted); ``ts_offset_ns`` shifts its timestamps to provoke the skew
    gate.  No camera, no robot.
    """

    def __init__(
        self,
        frames_rgb: list[np.ndarray],
        *,
        serial: str,
        device: str | None = None,
        intrinsics: np.ndarray | None = None,
        ts_offset_ns: int = 0,
    ) -> None:
        self._frames = list(frames_rgb)
        self.serial = serial
        self.device = device
        self._K = intrinsics
        self._ts_offset_ns = int(ts_offset_ns)
        self._sequence = 0
        self._index = 0

    def _make(self, rgb: np.ndarray) -> dict:
        frame = {
            "rgb": rgb,
            "frame_monotonic_ns": time.monotonic_ns() + self._ts_offset_ns,
            "pump_sequence": self._sequence,
        }
        if self._K is not None:
            frame["intrinsics"] = self._K
        return frame

    def latest_sequence(self) -> int:
        return self._sequence

    def latest(self) -> dict:
        return self._make(self._frames[min(self._index, len(self._frames) - 1)])

    def wait_for_frame_after(self, sequence: int, timeout_s: float = 2.0) -> dict:
        self._sequence = max(self._sequence, sequence) + 1
        rgb = self._frames[min(self._index, len(self._frames) - 1)]
        self._index += 1
        return self._make(rgb)


class _FakeRobot:
    """get_observations() only - exactly what capture_pair may call."""

    def __init__(self, joints_rad: np.ndarray) -> None:
        self._q = np.asarray(joints_rad, dtype=np.float64)

    def get_observations(self) -> dict:
        zeros = np.zeros(6)
        return {
            "joint_pos": self._q.copy(),
            "gripper_pos": np.zeros(1),
            "joint_vel": zeros,
            "gripper_vel": np.zeros(1),
            "joint_eff": zeros,
            "gripper_eff": np.zeros(1),
        }


def _impulsive_noise(rgb: np.ndarray, fraction: float, rng: np.random.Generator) -> np.ndarray:
    """Copy of ``rgb`` with ``fraction`` of the pixels replaced by random values."""
    noisy = rgb.copy()
    mask = rng.random(rgb.shape[:2]) < fraction
    noisy[mask] = rng.integers(0, 256, size=(int(mask.sum()), 3), dtype=np.uint8)
    return noisy


def run_dry_run(rig: RigSpec = LEFT_RIG) -> int:
    from capture_left_handeye import run_dry_run as run_fk_dry_run
    from capture_left_handeye import _synthetic_board_rgb

    # Rig identity used by every check below (left == the historical defaults).
    WRIST_SERIAL = rig.wrist_serial
    RIG_TOP_SERIAL = rig.top_serial
    RIG_TOP_DEVICE = rig.top_device
    print_rig(rig)
    say(f"  wrist D405 {WRIST_SERIAL} on {rig.can_channel}; station bodies "
        f"{rig.flange_body} -> {rig.camera_body}; pairs -> {rig.top_pairs_dir}")

    say("--- 1/3: FK check (capture_left_handeye --dry-run, reused; FK is side-agnostic) ---")
    fk_rc = run_fk_dry_run()

    say("--- 2/3: pair-npz schema self-check ---")
    checks: list[tuple[str, bool]] = []
    fk = LeftArmFK()
    flange_from_camera = rig_flange_from_camera(rig)
    left_flange_from_camera = station_left_flange_from_camera()
    from agp_yam_bridge.camera_acceptance import _rigid_transform as _rigid_check

    _rigid_check(flange_from_camera, f"station T_{rig.flange_body}_{rig.camera_body}")
    np.set_printoptions(suppress=True, precision=6)
    say(f"station T_{rig.flange_body}_{rig.camera_body} translation: {flange_from_camera[:3, 3]}")
    checks.append(
        (f"station {rig.flange_body}->{rig.camera_body} transform is rigid", True)
    )
    if rig.side == "right":
        delta_mm = float(np.linalg.norm(
            flange_from_camera[:3, 3] - left_flange_from_camera[:3, 3])) * 1000.0
        say(f"  differs from the LEFT measured hand-eye by {delta_mm:.2f} mm (expected: "
            "each arm has its own measured transform)")
        checks.append(
            ("right hand-eye is the right rig's MEASURED transform (not the left one)",
             not np.allclose(flange_from_camera, left_flange_from_camera, atol=1e-9))
        )
    camera_to_world = fk.base_from_gripper(np.zeros(6)) @ flange_from_camera
    board = _synthetic_board_rgb()                                   # wrist-size (640x360)
    top_board = _synthetic_board_rgb(size=(TOP_HEIGHT, TOP_WIDTH))     # top-camera profile
    K = np.array([[430.0, 0.0, 320.0], [0.0, 430.0, 180.0], [0.0, 0.0, 1.0]])
    with tempfile.TemporaryDirectory(prefix="top_pairs_dry_") as tmp:
        out_dir = Path(tmp)
        checks.append(
            ("fresh dir numbers pair01",
             next_pair_path(out_dir, None).name == "pair01.npz")
        )
        path = out_dir / "pair01.npz"
        write_pair_npz(
            path,
            top_rgb=top_board,
            top_serial=RIG_TOP_SERIAL,
            top_device=RIG_TOP_DEVICE,
            top_monotonic_ns=time.monotonic_ns(),
            wrist_rgb=board,
            wrist_intrinsics=K,
            wrist_camera_to_world=camera_to_world,
            wrist_serial=WRIST_SERIAL,
            wrist_monotonic_ns=time.monotonic_ns(),
            pair_skew_ns=1_000_000,
        )
        with np.load(path, allow_pickle=False) as saved:
            checks.append(
                ("written schema == capture-top-pair schema",
                 set(saved.files) == PAIR_FIELDS)
            )
            checks.append(
                ("solver-required fields present",
                 SOLVER_REQUIRED_FIELDS.issubset(saved.files))
            )
            checks.append(
                ("dtypes/shapes match the discovered schema",
                 saved["top_rgb"].dtype == np.uint8
                 and saved["top_rgb"].shape == (TOP_HEIGHT, TOP_WIDTH, 3)
                 and saved["wrist_intrinsics"].shape == (3, 3)
                 and saved["wrist_camera_to_world"].shape == (4, 4)
                 and saved["top_monotonic_ns"].dtype == np.int64
                 and saved["pair_skew_ns"].dtype == np.int64
                 and str(saved["wrist_serial"]) == WRIST_SERIAL
                 and str(saved["top_serial"]) == RIG_TOP_SERIAL)
            )
        checks.append(
            ("numbering continues (pair02)",
             next_pair_path(out_dir, None).name == "pair02.npz")
        )
        # A non-rigid transform must be rejected and leave no file behind.
        bad = camera_to_world.copy()
        bad[0, 0] *= 1.5
        rejected = False
        try:
            write_pair_npz(
                out_dir / "pair_bad.npz",
                top_rgb=top_board,
                top_serial=RIG_TOP_SERIAL,
                top_device=RIG_TOP_DEVICE,
                top_monotonic_ns=1,
                wrist_rgb=board,
                wrist_intrinsics=K,
                wrist_camera_to_world=bad,
                wrist_serial=WRIST_SERIAL,
                wrist_monotonic_ns=1,
                pair_skew_ns=0,
            )
        except Exception:
            rejected = True
        checks.append(
            ("non-rigid camera_to_world rejected, no file left",
             rejected and not (out_dir / "pair_bad.npz").exists())
        )

    say("--- 3/3: temporal median + drift/detection/skew rejection on synthetic stacks ---")
    rng = np.random.default_rng(0)
    n_frames = 9
    noisy_stack = [_impulsive_noise(board, 0.01, rng) for _ in range(n_frames)]
    top_noisy_stack = [_impulsive_noise(top_board, 0.01, rng) for _ in range(n_frames)]
    top_median = temporal_median_uint8(np.stack(top_noisy_stack))
    median = temporal_median_uint8(np.stack(noisy_stack))
    differing = float(np.mean(np.any(median != board, axis=2)))
    say(f"  median of {n_frames} frames with 1% impulsive noise: "
        f"{differing * 100:.4f}% of pixels differ from the clean board")
    checks.append(
        ("temporal median recovers the clean board (uint8, same shape)",
         median.dtype == np.uint8 and median.shape == board.shape and differing < 1e-5)
    )
    checks.append(
        ("median of a single frame is the frame",
         np.array_equal(temporal_median_uint8(np.stack([noisy_stack[0]])), noisy_stack[0]))
    )
    from agp_yam_bridge.camera_acceptance import _detect_checkerboard

    corners_static = np.stack(
        [_detect_checkerboard(f, columns=BOARD_COLUMNS, rows=BOARD_ROWS, label="dry")
         for f in noisy_stack]
    )
    static_drift = corner_drift_px(corners_static)
    say(f"  static noisy stack: drift {static_drift['drift_px']:.3f} px rms, worst corner "
        f"{static_drift['max_corner_px']:.3f} px, centroid {static_drift['max_centroid_px']:.3f} px")
    checks.append(
        ("static stack passes the drift gate",
         static_drift["drift_px"] <= MAX_CORNER_DRIFT_PX)
    )
    flipped = corners_static.copy()
    flipped[1::2] = flipped[1::2, ::-1]  # corner ORDER flips on alternate frames
    flipped_drift = corner_drift_px(flipped)
    checks.append(
        ("reversed corner order on some frames is NOT drift",
         abs(flipped_drift["drift_px"] - static_drift["drift_px"]) < 1e-9)
    )
    shifted = np.roll(board, 2, axis=1)  # board moves 2 px between frames
    moved_stack = noisy_stack[:5] + [shifted] * 4
    corners_moved = np.stack(
        [_detect_checkerboard(f, columns=BOARD_COLUMNS, rows=BOARD_ROWS, label="dry")
         for f in moved_stack]
    )
    moved_drift = corner_drift_px(corners_moved)
    say(f"  board shifted 2 px in 4/9 frames: drift {moved_drift['drift_px']:.3f} px rms")
    checks.append(
        ("2 px board motion exceeds the drift gate",
         moved_drift["drift_px"] > MAX_CORNER_DRIFT_PX)
    )

    # The real capture path with fake pumps + a fake robot (get_observations only).
    quality = load_quality_context(
        rig.wrist_intrinsics_json, rig.top_intrinsics_json,
        allow_missing_wrist=True, wrist_serial=rig.wrist_serial, base_label=rig.base_label,
    )
    say(f"  quality context: wrist distortion {quality.wrist_distortion_model}, top K from "
        f"{quality.top_source}, square {quality.square_size_m * 1000:.1f} mm, "
        f"base label '{quality.base_label}'")
    checks.append(
        ("advice text names the rig's base",
         any(rig.base_label in line for line in advice_lines(
             {"wrist_dist_m": 0.30, "wrist_tilt_deg": 25.0, "top_px_per_square": 36.0,
              "top_offset_px": 270.0, "board_r_xy_m": 0.62}, quality.base_label)))
    )
    joints = np.array([0.0, 0.6, -0.9, 0.0, 0.8, 0.0])
    robot = _FakeRobot(joints)
    reference_schema: dict | None = None
    reference_name = "built-in EXPECTED_PAIR_SCHEMA"
    for candidate in (TOP_PAIRS_DIR / "pair01.npz", OUT_DIR / "top_pairs_v1" / "pair01.npz"):
        if candidate.is_file():
            reference_schema = pair_schema(candidate)
            reference_name = str(candidate)
            break
    # Historical pairs may carry the legacy 640x360 top profile: compare
    # schemas modulo the top_rgb shape (the profile is set by the calibration
    # of the day; everything else must match exactly).
    def _modulo_top_profile(schema: dict) -> dict:
        out = dict(schema)
        if "top_rgb" in out:
            out["top_rgb"] = (out["top_rgb"][0], "TOP_PROFILE")
        return out

    if reference_schema is None:
        reference_schema = EXPECTED_PAIR_SCHEMA
    else:
        checks.append(
            ("historical pair01.npz matches the built-in expected schema (modulo top profile)",
             _modulo_top_profile(reference_schema) == _modulo_top_profile(EXPECTED_PAIR_SCHEMA))
        )
    with tempfile.TemporaryDirectory(prefix="top_pairs_dry_capture_") as tmp:
        out_dir = Path(tmp)
        tracker = BoardZTracker()

        def pumps(top_list, wrist_list, *, top_offset_ns=0):
            top = _SyntheticPump(top_list, serial=RIG_TOP_SERIAL, device=RIG_TOP_DEVICE, ts_offset_ns=top_offset_ns)
            wrist = _SyntheticPump(wrist_list, serial=WRIST_SERIAL, intrinsics=K)
            return wrist, top

        wrist_pump, top_pump = pumps(top_noisy_stack, noisy_stack)
        saved_path = capture_pair(
            wrist_pump, top_pump, robot, fk, flange_from_camera,
            out_dir=out_dir, label="dry", frames=n_frames, quality=quality, z_tracker=tracker,
        )
        sidecar = frames_sidecar_path(out_dir / "pair01_dry.npz")
        checks.append(
            ("synthetic capture wrote pair01_dry.npz + frames/pair01_dry_frames.npz",
             saved_path is not None and saved_path.name == "pair01_dry.npz"
             and saved_path.is_file() and sidecar.is_file())
        )
        if saved_path is not None:
            written = pair_schema(saved_path)
            checks.append(
                (f"written key set + dtypes/shapes identical to {reference_name} (modulo top profile)",
                 _modulo_top_profile(written) == _modulo_top_profile(reference_schema))
            )
            if _modulo_top_profile(written) != _modulo_top_profile(reference_schema):
                say(f"    written:   {written}")
                say(f"    reference: {reference_schema}")
            with np.load(saved_path, allow_pickle=False) as saved:
                checks.append(
                    ("saved images are the temporal medians",
                     np.array_equal(saved["top_rgb"], top_median)
                     and np.array_equal(saved["wrist_rgb"], median))
                )
        with np.load(sidecar, allow_pickle=False) as raw:
            checks.append(
                ("sidecar loads (allow_pickle=False) with all N raw frames + corners",
                 raw["top_rgb_frames"].shape == (n_frames, TOP_HEIGHT, TOP_WIDTH, 3)
                 and raw["wrist_rgb_frames"].shape == (n_frames, 360, 640, 3)
                 and raw["top_corners_px"].shape == (n_frames, BOARD_COLUMNS * BOARD_ROWS, 2)
                 and raw["pair_skew_ns"].shape == (n_frames,))
            )
        checks.append(
            ("sidecar invisible to pair*.npz globs (count 1, next pair02)",
             count_pairs(out_dir) == 1
             and [p.name for p in sorted(out_dir.glob("pair*.npz"))] == ["pair01_dry.npz"]
             and next_pair_path(out_dir, None).name == "pair02.npz")
        )
        checks.append(("z tracker recorded the pair", len(tracker) == 1))

        # Rejections: nothing may be written for any of them.
        before = sorted(str(p) for p in out_dir.rglob("*.npz"))
        wrist_pump, top_pump = pumps(top_noisy_stack[:5] + [np.roll(top_board, 2, axis=1)] * 4, noisy_stack)
        refused_moved = capture_pair(
            wrist_pump, top_pump, robot, fk, flange_from_camera,
            out_dir=out_dir, label=None, frames=n_frames, quality=quality, z_tracker=tracker,
        )
        checks.append(
            ("moved board REFUSED, nothing written",
             refused_moved is None and sorted(str(p) for p in out_dir.rglob("*.npz")) == before)
        )
        blank = np.full_like(board, 128)
        wrist_pump, top_pump = pumps(top_noisy_stack, noisy_stack[:4] + [blank] + noisy_stack[5:])
        refused_blank = capture_pair(
            wrist_pump, top_pump, robot, fk, flange_from_camera,
            out_dir=out_dir, label=None, frames=n_frames, quality=quality, z_tracker=tracker,
        )
        checks.append(
            ("one undetectable frame REFUSED, nothing written",
             refused_blank is None and sorted(str(p) for p in out_dir.rglob("*.npz")) == before)
        )
        wrist_pump, top_pump = pumps(top_noisy_stack, noisy_stack, top_offset_ns=200_000_000)
        refused_skew = capture_pair(
            wrist_pump, top_pump, robot, fk, flange_from_camera,
            out_dir=out_dir, label=None, frames=n_frames, quality=quality, z_tracker=tracker,
        )
        checks.append(
            ("0.2 s top/wrist skew REFUSED, nothing written",
             refused_skew is None and sorted(str(p) for p in out_dir.rglob("*.npz")) == before)
        )
        checks.append(("refused captures did not enter the z tracker", len(tracker) == 1))

        # Restart behaviour: a fresh tracker re-seeds from the pairs on disk.
        reseeded = BoardZTracker()
        checks.append(
            ("z tracker re-seeds from pair*.npz on disk",
             reseeded.seed_from_dir(out_dir, quality) == 1 and len(reseeded) == 1)
        )

    # Advice + z-tracker unit checks (pure functions).
    bad_geometry = {
        "wrist_dist_m": 0.50, "wrist_tilt_deg": 8.0, "top_px_per_square": 26.7,
        "top_offset_px": 720.0, "board_r_xy_m": 0.62,
    }
    good_geometry = {
        "wrist_dist_m": 0.30, "wrist_tilt_deg": 25.0, "top_px_per_square": 36.0,
        "top_offset_px": 270.0, "board_r_xy_m": 0.40,
    }
    checks.append(("all five ADVICE thresholds fire on a far/steep/edge pose",
                   len(advice_lines(bad_geometry)) == 5))
    checks.append(("no ADVICE inside all targets", advice_lines(good_geometry) == []))
    unit_tracker = BoardZTracker()
    unit_tracker.add("a", -0.040)
    unit_tracker.add("b", -0.041)
    ok_report = unit_tracker.add("c", -0.042)
    bad_report = unit_tracker.add("d", -0.034)
    checks.append(("z tracker: 2 mm inside, 6.5 mm flagged (> 5 mm vs running median)",
                   not ok_report["flag"] and bad_report["flag"]
                   and abs(bad_report["dz_vs_running_median_m"] - 0.007) < 1e-9))

    ok = all(passed for _, passed in checks)
    for name, passed in checks:
        say(f"  [{'ok' if passed else 'FAIL'}] {name}")
    say(f"SCHEMA + MEDIAN/DRIFT SELF-CHECK {'PASS' if ok else 'FAIL'}")
    return 0 if (ok and fk_rc == 0) else 1


# --------------------------------------------------------------------------
# Main interactive loop (same safety pattern as capture_left_handeye.main)
# --------------------------------------------------------------------------
HELP_TEXT = """commands (press Enter after each):
  g           gravity-comp idle - drag the arm so the WRIST sees the board
  h           hold - freeze at the CURRENT measured joint position
  c [label]   capture ONE wrist+top pair (hold only) as the temporal median of
              --frames frames (~1 s: hold still), e.g.: c spot_front_left
              -> prints the quality ADVICE and the running board-z check
  p           print board-detection status for BOTH cameras + live quality ADVICE
  q           quit (arm stays in HOLD; then e-stop / take over)
"""


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Standalone top-camera pair capture (wrist D405 + top BRIO, no bridge)"
    )
    parser.add_argument(
        "--rig", choices=RIG_CHOICES, default="left",
        help="left = our station (default); right = the right rig "
             "(can_follower_r, D405 353322271910, BRIO B8C7F203; the right-arm bridge on "
             "9020 must be stopped)",
    )
    parser.add_argument(
        "--out-dir", type=Path, default=None,
        help=f"pair npz output directory (default: the rig's, e.g. {TOP_PAIRS_DIR})",
    )
    parser.add_argument(
        "--focus-lock", choices=("rig", "lock", "verify", "skip"), default="rig",
        help="top-BRIO focus handling at start (rig default: left verify, right lock)",
    )
    parser.add_argument(
        "--frames", type=int, default=DEFAULT_FRAMES,
        help=f"frames per camera averaged (temporal median) per pair "
             f"(default {DEFAULT_FRAMES}, min {MIN_FRAMES}; ~camera rate, so 20 = ~0.7 s)",
    )
    parser.add_argument(
        "--wrist-preview-port", type=int, default=8766,
        help="wrist D405 MJPEG preview port (default 8766)",
    )
    parser.add_argument(
        "--top-preview-port", type=int, default=8767,
        help="top BRIO MJPEG preview port (default 8767)",
    )
    parser.add_argument("--no-preview", action="store_true")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="no hardware: rerun the FK check, self-check the pair schema and "
             "exercise the median/drift-rejection logic on synthetic frame stacks",
    )
    args = parser.parse_args()
    for port in (args.wrist_preview_port, args.top_preview_port):
        if not 1 <= port <= 65535:
            parser.error("preview ports must be in 1..65535")
    if args.wrist_preview_port == args.top_preview_port:
        parser.error("wrist and top preview ports must differ")
    if args.frames < MIN_FRAMES:
        parser.error(f"--frames must be >= {MIN_FRAMES}")
    rig = get_rig(args.rig)
    out_dir: Path = args.out_dir or rig.top_pairs_dir
    focus_mode = rig.focus_lock if args.focus_lock == "rig" else args.focus_lock
    wrist_intrinsics_json = rig.wrist_intrinsics_json

    if args.dry_run:
        return run_dry_run(rig)

    say(f"== {rig.side}-station top-camera PAIR capture (wrist D405 + top BRIO) ==")
    print_rig(rig)
    say(f"  wrist D405 {rig.wrist_serial} on {rig.can_channel}; station bodies "
        f"{rig.flange_body} -> {rig.camera_body}; pairs -> {out_dir}")
    if refuse_if_bridge_listening(rig.bridge_port, rig.side):
        return 2
    for extra_port in getattr(rig, "extra_refuse_ports", ()):   # cross rig: left bridge (top BRIO owner) / our right bridge
        if refuse_if_bridge_listening(int(extra_port), f"other ({extra_port})"):
            return 2
    # Focus check BEFORE any camera is opened (v4l2-ctl touches only controls).
    try:
        ensure_focus_locked(rig.top_device, focus_mode)
    except RuntimeError as exc:
        say(f"REFUSING to start: {exc}")
        return 2
    say("loading kinematics (combined YAM+linear_4310 model, measured station XML) ...")
    fk = LeftArmFK()
    flange_from_camera = rig_flange_from_camera(rig)

    say(f"opening the {rig.side} wrist D405 {rig.wrist_serial} (640x360@30, depth aligned to color) ...")
    wrist_camera = LeftD405(rig.wrist_serial)
    payload = wrist_camera.intrinsics_payload()
    wrist_intrinsics_json.parent.mkdir(parents=True, exist_ok=True)
    wrist_intrinsics_json.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    say(f"factory color intrinsics of THIS device written to {wrist_intrinsics_json}")
    if payload["distortion_model"] != "inverse_brown_conrady":
        say("  WARNING: wrist distortion model is not inverse_brown_conrady; "
            "check solve_top.sh before solving.")

    say(f"opening the top BRIO {rig.top_serial} ({TOP_WIDTH}x{TOP_HEIGHT}@30 MJPG, profile verified) ...")
    top_camera = BrioTopCamera(rig.top_device, serial=rig.top_serial)

    quality = load_quality_context(
        wrist_intrinsics_json, rig.top_intrinsics_json,
        wrist_serial=rig.wrist_serial, base_label=rig.base_label,
    )
    say(f"quality advice: wrist distortion {quality.wrist_distortion_model} from "
        f"{wrist_intrinsics_json.name}; top centre from {quality.top_source}; "
        f"board square {quality.square_size_m * 1000:.1f} mm")
    say(f"  ADVICE thresholds: wrist distance {ADVICE_WRIST_DIST_M[0]:.2f}-"
        f"{ADVICE_WRIST_DIST_M[1]:.2f} m, tilt {ADVICE_WRIST_TILT_DEG[0]:.0f}-"
        f"{ADVICE_WRIST_TILT_DEG[1]:.0f} deg, top >= {ADVICE_MIN_TOP_PX_PER_SQUARE:.0f} "
        f"px/square, top offset <= {ADVICE_MAX_TOP_OFFSET_PX:.0f} px, board r <= "
        f"{ADVICE_MAX_BOARD_R_M:.2f} m; capture = median of {args.frames} frame(s), "
        f"drift gate {MAX_CORNER_DRIFT_PX:.1f} px rms")
    z_tracker = BoardZTracker()
    existing = count_pairs(out_dir)
    if existing:
        archive_dir = out_dir.with_name(out_dir.name + "_v1")
        say(f"NOTE: {existing} pair(s) already in {out_dir}. For a RECAPTURE move "
            f"them to {archive_dir} first (README section 8b); otherwise "
            "numbering continues and solve_top.sh will mix old and new pairs.")
        say("seeding the running board-z check from the pairs on disk ...")
        seeded = z_tracker.seed_from_dir(out_dir, quality)
        say(f"  seeded {seeded} pair(s): {z_tracker.summary()}")

    wrist_pump = FramePump(wrist_camera)
    wrist_pump.start()
    top_pump = FramePump(top_camera)
    # FramePump exposes only .serial from the camera; keep the device path
    # for the npz alongside it.
    top_pump.device = top_camera.device  # type: ignore[attr-defined]
    top_pump.start()
    wrist_pump.wait_until_ready(min_frames=10, timeout_s=10.0)
    top_pump.wait_until_ready(min_frames=5, timeout_s=10.0)

    wrist_cache: DetectionCache | None = None
    top_cache: DetectionCache | None = None
    if args.no_preview:
        say("browser previews disabled (--no-preview); 'p' still detects inline.")
    else:
        wrist_cache = start_preview(
            wrist_pump, args.wrist_preview_port, "LIVE WRIST-CAMERA PREVIEW"
        )
        top_cache = start_preview(
            top_pump, args.top_preview_port, "LIVE TOP-CAMERA (BRIO) PREVIEW"
        )

    say("")
    say("=" * 72)
    say(f"ABOUT TO ENABLE THE {rig.side.upper()} ARM ON {rig.can_channel}")
    say("  * The arm will IMMEDIATELY HOLD its current position (no motion).")
    say("  * The gripper auto-calibration wiggle is SKIPPED "
        "(gripper_limits_override).")
    say("  * Keep a hand near the e-stop.")
    if rig.side == "right":
        say("  * This is the RIGHT arm: its bridge must be stopped (checked),")
        say("    its e-stop within reach, and a second person present while it is dragged.")
    say("=" * 72)
    answer = input("type 'yes' to enable the arm (anything else aborts): ").strip()
    if answer.lower() != "yes":
        say("aborted before touching the robot.")
        wrist_pump.stop()
        top_pump.stop()
        wrist_camera.close()
        top_camera.close()
        return 1

    state = "hold"
    robot = enable_arm(rig.can_channel)

    try:
        dofs = robot.num_dofs()
        if dofs != 7:
            raise RuntimeError(f"expected a 7-DOF YAM+gripper, got {dofs}")
        pos, _, _, _ = read_command_space_joints(robot)
        say(f"arm enabled, HOLDING at joints (deg): "
            f"{np.degrees(pos[:6]).round(2).tolist()}")
        say(f"pairs on disk: {count_pairs(out_dir)} (target {PAIR_TARGET})")
        say(HELP_TEXT)

        while True:
            try:
                line = input(f"[{state}] g/h/c [label]/p/q > ").strip()
            except EOFError:
                line = "q"
            if not line:
                say(HELP_TEXT)
                continue
            parts = line.split(maxsplit=1)
            command = parts[0].lower()
            argument = parts[1] if len(parts) > 1 else None

            if command == "g":
                if state == "hold":
                    input("请先扶住手臂，按回车后释放为重力补偿模式 > ")
                robot.enter_gravity_comp_idle()
                state = "gravity"
                say("gravity-comp idle: you can drag the arm by hand now.")
            elif command == "h":
                ensure_chain_alive(robot, "before entering hold")
                held = command_hold_at_measured(robot)
                if held is None:
                    continue  # refused near a soft limit; stay in gravity mode
                state = "hold"
                say(f"holding at joints (deg): "
                    f"{np.degrees(held[:6]).round(2).tolist()}")
            elif command == "c":
                if state != "hold":
                    say("capture is only allowed in HOLD - press 'h' first.")
                    continue
                ensure_chain_alive(robot, "before capture")
                try:
                    capture_pair(
                        wrist_pump,
                        top_pump,
                        robot,
                        fk,
                        flange_from_camera,
                        out_dir=out_dir,
                        label=argument,
                        frames=args.frames,
                        quality=quality,
                        z_tracker=z_tracker,
                    )
                except Exception as exc:  # camera hiccup etc.; hold untouched
                    say(f"capture FAILED ({type(exc).__name__}: {exc})")
                    say("nothing (or a partial file) was kept; the arm HOLD is")
                    say("untouched.  Fix the camera/board and press 'c' again.")
            elif command == "p":
                # Read-only status prints: cameras only, plus ONE
                # get_observations for the wrist-chain numbers; never a command.
                say("wrist D405:")
                print_board_status(wrist_cache, wrist_pump)
                say("top BRIO:")
                print_board_status(top_cache, top_pump)
                try:
                    print_live_quality(
                        wrist_pump, top_pump, robot, fk, flange_from_camera, quality
                    )
                except Exception as exc:  # noqa: BLE001  advice must never kill the loop
                    say(f"quality advice unavailable ({type(exc).__name__}: {exc})")
                say(z_tracker.summary())
            elif command == "q":
                ensure_chain_alive(robot, "on quit")
                if state != "hold":
                    held = command_hold_at_measured(robot)
                    if held is None:
                        say("quit aborted: no hold could be installed - still in")
                        say("gravity mode; move away from the limit, then retry.")
                        continue
                    state = "hold"
                    say(f"switched to HOLD at joints (deg): "
                        f"{np.degrees(held[:6]).round(2).tolist()}")
                say("")
                say(f"pairs on disk: {count_pairs(out_dir)}")
                say(z_tracker.summary())
                say("quit: the arm REMAINS IN HOLD (motors keep executing the")
                say("last position command onboard). You may now press the")
                say("e-stop or take over the arm, then power off when done.")
                sys.stdout.flush()
                os._exit(0)
            else:
                say(HELP_TEXT)
    except BaseException as exc:  # noqa: BLE001  (includes KeyboardInterrupt)
        die_without_motion(state, exc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
