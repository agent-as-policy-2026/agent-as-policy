"""Standalone top-camera (Logitech BRIO) intrinsics capture, per rig - one or
TWO rigs in one session.

``--rig left`` (default, unchanged behaviour): OUR left station's BRIO
178B0DAE -> calib/out/top_intr/.  ``--rig right``: the RIGHT rig's BRIO
B8C7F203 -> calib/out/top_intr_right/ (the right-arm bridge on port 9020 must be
stopped first; the tool refuses to start while it listens, exactly as
the left tools refuse while our left bridge listens on 9021).  ``--rig left
right`` opens BOTH BRIOs in one process (one FramePump per camera): the focus
check runs per camera, the tool refuses to start if EITHER bridge port
listens, and every ``c``/``v`` saves INDEPENDENTLY per camera into that
camera's own directory (calib_NN / val_NN numbering per directory); a camera
whose frame has no full board is skipped ("<rig>: no board - skipped") while
the other one still saves.  The mapping lives in left_handeye_common.RIGS.

NO robot, NO bridge, NO CAN: this tool only opens the fixed overhead Logitech
BRIO via OpenCV V4L2 at EXACTLY the profile the bridge will later stream
(1920x1080@30, MJPG) and saves still checkerboard views for the offline
``agp-yam-camera-acceptance calibrate-top-intrinsics`` solver.  The negotiated
V4L2 profile (width/height/fps AND the MJPG FourCC) is verified after opening;
a silently different negotiation is refused, because intrinsics calibrated on
one profile do not transfer to another.

FOCUS: intrinsics are only valid at one focus/zoom state, so at tool start the
BRIO's V4L2 controls are checked through ``v4l2-ctl``: ``focus_automatic_
continuous`` must read 0 (autofocus OFF).  For the right rig (autofocus state
unknown) the controls are first SET (autofocus 0, focus_absolute 0, zoom 100)
and then re-read; for the left rig they are only READ (its calibration was
taken with autofocus confirmed off - nothing is changed).  Autofocus reading 1
after that refuses the tool.  ``--focus-lock lock|verify|skip`` overrides.

Interactive loop (type the command, then Enter):

    c   save a CALIBRATION view on every opened camera -> <dir>/calib_NN.png
    v   save a VALIDATION view  on every opened camera -> <dir>/val_NN.png
    p   print the detection status + the coverage tables per camera
    q   quit

A save is REFUSED (per camera) when the full 9x7 corner grid is not detected
in that exact frame (the solver hard-fails on any undetectable image, so
nothing undetectable is ever written).  Detection parameters are IDENTICAL to
the solver's ``_detect_checkerboard_images`` (scale-1 findChessboardCornersSB
with NORMALIZE|EXHAUSTIVE|ACCURACY), reused via
``capture_left_handeye.detect_board``.  Because that detector randomly misses
some real boards (see DETECT_SAMPLES) while the solver raises on ONE miss,
a frame is saved only when 3 consecutive detections all succeed ("UNSTABLE
- skipped" otherwise) and the resume scan flags on-disk views that missed.

COVERAGE STATISTICS (per camera; printed after every save and on ``p``;
persisted by RE-DETECTING the calib_NN/val_NN PNGs already on disk at start,
so a resumed session counts what is there).  The tables count CALIBRATION
views only (validation views are held out from the solve):
  * GRID: 6x4 cells over the 1920x1080 image; a cell counts a view when at
    least one detected corner falls inside it.  Empty cells are listed
    ("row4: all 6 empty; col6: rows1-3 empty").  ``--ignore-cells r4c6 c6
    left:r4c1`` declares cells the operator cannot cover (rNcM, rN or cM,
    optionally prefixed by a rig name; r1 = top row, c1 = left column).
  * SCALE: px/square of the detected board = median adjacent-corner spacing
    along the less foreshortened board axis.  Fixed bins at 1080p:
    near >= 60 px, mid 40..60 px, far < 40 px.  With the BRIO's ~1160 px
    focal length at 1080p and the 22 mm squares that is about < 0.43 m /
    0.43-0.64 m / > 0.64 m (the board flat on the table under the ~0.9 m
    high camera is ~28 px/square, i.e. "far").
  * TILT: angle between the board normal and the line of sight to the board
    centre, from the homography board-plane -> image of the 63 corners
    (cv2.findHomography, decomposed with the nominal focal length):
    flat <= 10 deg, medium 10..25 deg, steep > 25 deg.
DONE hint per camera: calib >= 30, val >= 6, no empty (non-ignored) grid
cell, and every scale and tilt bin >= 3 views.  20 calib + 5 val remain the
solver's hard MINIMUM ("minimum met, keep going for coverage" in between).
After every save a "next: ..." line names the emptiest grid cell and the
emptiest scale / tilt bin, i.e. where to hold the board next.

PREVIEW: one MJPEG page on ``--preview-port`` (default 8767; the wrist tools
use 8766) shows all opened cameras side by side (each ~960 px wide when two
are open, native 1920 px for one) with the 6x4 grid drawn on each: covered
cells shaded green, empty cells outlined red (labelled rNcM), ignored cells
grey, plus the live "board OK / NOT detected" banner per camera.

Images are saved as BGR PNG uint8[1080,1920,3], exactly what the solver's
``cv2.imread`` expects; every save is re-read and shape-checked before it is
counted.

Run from the hardware-bridge uv project:
    cd hardware-bridge
    uv run --locked python ../calib/capture_top_intrinsics.py
    ... capture_top_intrinsics.py --rig right        # the right rig
    ... capture_top_intrinsics.py --rig left right   # both BRIOs, one session

Offline check (no camera):  ... capture_top_intrinsics.py [--rig left right] --dry-run
"""

from __future__ import annotations

import argparse
import math
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from itertools import product
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from left_handeye_common import (  # noqa: E402
    CALIB_DIR,
    LEFT_RIG,
    RIG_CHOICES,
    TOP_FOURCC,
    TOP_FPS,
    TOP_HEIGHT,
    TOP_WIDTH,
    RigSpec,
    get_rig,
)
from capture_left_handeye import (  # noqa: E402
    BOARD_COLUMNS,
    BOARD_CORNER_COUNT,
    BOARD_ROWS,
    DetectionCache,
    DetectionWorker,
    EncoderWorker,
    FramePump,
    PreviewState,
    _synthetic_board_rgb,
    best_effort_host_ips,
    detect_board,
    print_board_status,
    refuse_if_bridge_listening,
    say,
    start_preview_server,
)

# The bridge/client stream contract for the fixed top camera (must match what
# agp_yam_bridge.camera.OpenCvFixedRgbCamera will negotiate at serve time).
# TOP_WIDTH/TOP_HEIGHT/TOP_FPS/TOP_FOURCC come from left_handeye_common (one
# definition shared with the solver/install scripts) and are re-exported here.
# The LEFT identity stays the module default so existing importers
# (capture_top_pairs.py, calib/analysis/) see the same names and values.
TOP_SERIAL = LEFT_RIG.top_serial
TOP_DEVICE = LEFT_RIG.top_device

TOP_INTR_DIR = LEFT_RIG.top_intr_dir
# Solver hard minimum (solve_top.sh refuses fewer) ...
CALIB_TARGET = 20
VAL_TARGET = 5
# ... and the coverage-driven per-camera goal behind the DONE hint.
CALIB_GOAL = 30
VAL_GOAL = 6
MAX_RIGS = 2   # one BRIO per rig; both rigs in one session at most

# V4L2 controls that must be pinned for a valid intrinsics calibration.
# Newer kernels name the autofocus toggle focus_automatic_continuous, older
# ones focus_auto; both are tried.  Values: autofocus OFF, focus at the far
# stop (0), no digital zoom (100 = the BRIO's minimum).
FOCUS_AUTO_CTRLS = ("focus_automatic_continuous", "focus_auto")
FOCUS_LOCK_VALUES = {"focus_absolute": 0, "zoom_absolute": 100}
FOCUS_LOCK_MODES = ("lock", "verify", "skip")

# --------------------------------------------------------------------------
# Coverage statistics: grid / scale / tilt bins (per camera)
# --------------------------------------------------------------------------
GRID_COLUMNS = 6
GRID_ROWS = 4
SQUARE_SIZE_M = 0.022
# Nominal BRIO focal length at 1920x1080 (solved left BRIO 178B0DAE:
# fx 1161.9 / fy 1162.3 px; the right BRIO is the same model at the same
# profile).  Used ONLY to (a) print the approximate metres behind the px/square
# thresholds and (b) decompose the board homography for the tilt estimate.
# The scale bins themselves are defined in pixels.
NOMINAL_FOCAL_PX = 1160.0
# SCALE rule (px/square = median adjacent-corner spacing along the less
# foreshortened board axis, at the 1080p profile):
#   near  px/square >= SCALE_NEAR_MIN_PX
#   mid   SCALE_FAR_MAX_PX <= px/square < SCALE_NEAR_MIN_PX
#   far   px/square < SCALE_FAR_MAX_PX
SCALE_NEAR_MIN_PX = 60.0
SCALE_FAR_MAX_PX = 40.0
# TILT rule (board normal vs line of sight to the board centre, degrees):
#   flat <= TILT_FLAT_MAX_DEG < medium <= TILT_STEEP_MIN_DEG < steep
TILT_FLAT_MAX_DEG = 10.0
TILT_STEEP_MIN_DEG = 25.0
MIN_VIEWS_PER_BIN = 3
SCALE_BIN_NAMES = ("near", "mid", "far")
TILT_BIN_NAMES = ("flat", "medium", "steep")
PREVIEW_TILE_WIDTH = 960   # per camera in the browser when >1 camera is open
# findChessboardCornersSB with CALIB_CB_EXHAUSTIVE (the solver's exact call)
# is NOT deterministic on some real 1080p boards: measured 2026-09-02 on the
# archived 25-view left set, 4 of the 25 images missed 6-44 % of repeated
# detections of the SAME pixels (the other 21 never missed; single-threaded
# OpenCV behaves the same, so it is not a threading race; without EXHAUSTIVE
# the misses vanish, but the solver uses EXHAUSTIVE and raises on ONE miss).
# Hence a STABILITY gate: a frame is saved only when DETECT_SAMPLES
# consecutive detections all succeed (~0.13 s each at 1080p; a miss stops
# early), and the resume scan flags on-disk views that missed any of the
# DETECT_SAMPLES attempts as "unstable" (the solver may fail on them).
DETECT_SAMPLES = 3


def parse_v4l2_ctrls(text: str) -> dict[str, int]:
    """``name: value`` lines of ``v4l2-ctl --get-ctrl`` -> {name: int}."""
    values: dict[str, int] = {}
    for line in text.splitlines():
        if ":" not in line:
            continue
        name, _, value = line.partition(":")
        name = name.strip()
        value = value.strip().split()[0] if value.strip() else ""
        if name and value.lstrip("-").isdigit():
            values[name] = int(value)
    return values


def _v4l2_ctl(device: str, *args: str) -> str:
    exe = shutil.which("v4l2-ctl")
    if exe is None:
        raise RuntimeError("v4l2-ctl not found (apt install v4l-utils); cannot verify focus")
    result = subprocess.run(
        [exe, "-d", device, *args], capture_output=True, text=True, timeout=10.0
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"v4l2-ctl {' '.join(args)} failed on {device}: "
            f"{(result.stderr or result.stdout).strip()}"
        )
    return result.stdout


def read_brio_focus_controls(device: str) -> dict[str, int]:
    """Read autofocus/focus/zoom of one BRIO (read-only; no stream is opened).

    Returns ``{"autofocus": v, "focus_absolute": v, "zoom_absolute": v,
    "autofocus_ctrl": name}``-style ints (the control name used is stored
    under ``_autofocus_ctrl`` as an index into FOCUS_AUTO_CTRLS).
    """
    last_error: Exception | None = None
    for index, auto_name in enumerate(FOCUS_AUTO_CTRLS):
        names = ",".join((auto_name, *FOCUS_LOCK_VALUES))
        try:
            values = parse_v4l2_ctrls(_v4l2_ctl(device, f"--get-ctrl={names}"))
        except RuntimeError as exc:
            last_error = exc
            continue
        if auto_name in values:
            return {
                "autofocus": values[auto_name],
                "focus_absolute": values.get("focus_absolute", -1),
                "zoom_absolute": values.get("zoom_absolute", -1),
                "_autofocus_ctrl": index,
            }
    raise RuntimeError(
        f"could not read the autofocus control of {device} "
        f"(tried {FOCUS_AUTO_CTRLS}): {last_error}"
    )


def ensure_focus_locked(device: str, mode: str) -> dict[str, int] | None:
    """Pin (``lock``) or check (``verify``) the BRIO focus state; refuse if
    autofocus still reads 1.  ``skip`` does nothing (prints a warning).

    Raises RuntimeError on refusal so the caller exits BEFORE opening the
    camera.  Read-only in ``verify`` mode.
    """
    if mode not in FOCUS_LOCK_MODES:
        raise ValueError(f"focus-lock mode must be one of {FOCUS_LOCK_MODES}, got {mode!r}")
    if mode == "skip":
        say("WARNING: --focus-lock skip: the BRIO focus state was NOT verified; the")
        say("         intrinsics are only valid if autofocus is off and focus/zoom are fixed.")
        return None
    before = read_brio_focus_controls(device)
    auto_name = FOCUS_AUTO_CTRLS[before["_autofocus_ctrl"]]
    say(
        f"BRIO focus controls before: {auto_name}={before['autofocus']} "
        f"focus_absolute={before['focus_absolute']} zoom_absolute={before['zoom_absolute']}"
    )
    if mode == "lock":
        _v4l2_ctl(device, f"--set-ctrl={auto_name}=0")
        # Focus is only settable once autofocus is off, hence two calls.
        _v4l2_ctl(
            device,
            "--set-ctrl=" + ",".join(f"{k}={v}" for k, v in FOCUS_LOCK_VALUES.items()),
        )
        after = read_brio_focus_controls(device)
        say(
            f"BRIO focus controls after lock: {auto_name}={after['autofocus']} "
            f"focus_absolute={after['focus_absolute']} zoom_absolute={after['zoom_absolute']}"
        )
    else:
        after = before
    if after["autofocus"] != 0:
        raise RuntimeError(
            f"BRIO {device}: autofocus ({auto_name}) reads {after['autofocus']} - "
            "intrinsics cannot be calibrated with autofocus on; refusing. "
            "Try: v4l2-ctl -d <device> --set-ctrl=focus_automatic_continuous=0"
        )
    if mode == "lock":
        mismatched = {
            k: after[k] for k, v in FOCUS_LOCK_VALUES.items() if after.get(k) != v
        }
        if mismatched:
            raise RuntimeError(
                f"BRIO {device}: controls did not take the locked values "
                f"{FOCUS_LOCK_VALUES}: read {mismatched}; refusing"
            )
    say(f"focus state OK (autofocus off) [{mode}].")
    return after


def _decode_fourcc(value: float) -> str:
    code = int(value)
    return "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4))


class BrioTopCamera:
    """Serial-bound V4L2 BRIO reader at exactly 1920x1080@30 MJPG.

    Mirrors the bridge's OpenCvFixedRgbCamera negotiation (FOURCC first, then
    width/height/fps, then read back and compare) and ADDITIONALLY verifies
    the negotiated FourCC is MJPG, refusing any silently different profile.
    Provides ``read() -> {"rgb", "frame_monotonic_ns"}`` and ``serial`` so it
    plugs straight into capture_left_handeye.FramePump.
    """

    def __init__(
        self,
        device: str = TOP_DEVICE,
        *,
        serial: str = TOP_SERIAL,
        warmup_frames: int = 10,
    ) -> None:
        import cv2

        self._cv2 = cv2
        if not serial or serial not in Path(device).name:
            raise RuntimeError(
                f"top-camera serial {serial!r} must appear in the stable "
                f"by-id device path {device!r}"
            )
        self.serial = serial
        self.device = device
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"could not open top camera {device}")
        self._capture = capture
        self._closed = False
        try:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*TOP_FOURCC))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, TOP_WIDTH)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, TOP_HEIGHT)
            capture.set(cv2.CAP_PROP_FPS, TOP_FPS)
            actual = (
                round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                round(capture.get(cv2.CAP_PROP_FPS)),
            )
            expected = (TOP_WIDTH, TOP_HEIGHT, TOP_FPS)
            if actual != expected:
                raise RuntimeError(
                    f"top camera negotiated profile {actual}, refusing: the "
                    f"calibration profile must be exactly {expected}"
                )
            fourcc = _decode_fourcc(capture.get(cv2.CAP_PROP_FOURCC))
            if fourcc != TOP_FOURCC:
                raise RuntimeError(
                    f"top camera negotiated FourCC {fourcc!r}, refusing: the "
                    f"bridge streams {TOP_FOURCC!r} and intrinsics must be "
                    "calibrated on the same pipeline"
                )
            for _ in range(warmup_frames):
                self.read()
        except BaseException:
            self.close()
            raise

    def read(self) -> dict:
        """One frame: rgb uint8[1080,1920,3] + capture monotonic timestamp."""
        ok, bgr = self._capture.read()
        monotonic_ns = time.monotonic_ns()
        if not ok or bgr is None:
            raise RuntimeError("top camera frame capture failed")
        image = np.asarray(bgr)
        if image.dtype != np.uint8 or image.shape != (TOP_HEIGHT, TOP_WIDTH, 3):
            raise RuntimeError(
                f"top camera returned {image.dtype}{image.shape}, expected "
                f"uint8[{TOP_HEIGHT},{TOP_WIDTH},3]"
            )
        rgb = np.ascontiguousarray(self._cv2.cvtColor(image, self._cv2.COLOR_BGR2RGB))
        return {"rgb": rgb, "frame_monotonic_ns": monotonic_ns}

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._capture.release()


# --------------------------------------------------------------------------
# Rig selection (one or two rigs per session)
# --------------------------------------------------------------------------
def parse_rigs(names) -> list[RigSpec]:
    """``--rig`` values -> RigSpecs in the given order; 1..MAX_RIGS, unique."""
    names = list(names)
    if not names:
        raise ValueError("--rig needs at least one rig")
    if len(names) > MAX_RIGS:
        raise ValueError(f"--rig accepts at most {MAX_RIGS} rigs (one BRIO each), got {names}")
    if len(set(names)) != len(names):
        raise ValueError(f"--rig lists a rig twice: {names}")
    return [get_rig(name) for name in names]


# --------------------------------------------------------------------------
# Coverage geometry: grid cells, px/square (scale), tilt from the homography
# --------------------------------------------------------------------------
Cell = tuple[int, int]   # (row, column), 1-based; r1 = top row, c1 = left column


def cell_name(cell: Cell) -> str:
    return f"r{cell[0]}c{cell[1]}"


def all_cells() -> list[Cell]:
    return [(r, c) for r in range(1, GRID_ROWS + 1) for c in range(1, GRID_COLUMNS + 1)]


def cells_hit(points) -> frozenset[Cell]:
    """Grid cells (1-based (row, col)) containing at least one image point."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    cols = np.clip(np.floor(pts[:, 0] * GRID_COLUMNS / TOP_WIDTH), 0, GRID_COLUMNS - 1)
    rows = np.clip(np.floor(pts[:, 1] * GRID_ROWS / TOP_HEIGHT), 0, GRID_ROWS - 1)
    return frozenset((int(r) + 1, int(c) + 1) for r, c in zip(rows, cols))


_CELL_RE = re.compile(r"^(?:r(\d+))?(?:c(\d+))?$")


def parse_ignore_cells(tokens, rig_names) -> dict[str, frozenset[Cell]]:
    """``--ignore-cells`` tokens -> {rig: cells}.

    Tokens: ``r4c6`` (one cell), ``r4`` (whole row), ``c6`` (whole column);
    an optional ``<rig>:`` prefix (``left:c6``) limits a token to one rig,
    otherwise it applies to every rig of the session.  Comma or space
    separated.
    """
    rig_names = list(rig_names)
    result: dict[str, set[Cell]] = {name: set() for name in rig_names}
    for raw in tokens or ():
        for token in str(raw).replace(",", " ").split():
            spec = token.lower()
            targets = rig_names
            if ":" in spec:
                rig, _, spec = spec.partition(":")
                if rig not in rig_names:
                    raise ValueError(
                        f"--ignore-cells {token!r}: rig {rig!r} is not part of this "
                        f"session ({rig_names})"
                    )
                targets = [rig]
            match = _CELL_RE.match(spec)
            if not match or (match.group(1) is None and match.group(2) is None):
                raise ValueError(
                    f"--ignore-cells {token!r}: expected rNcM, rN or cM "
                    f"(r1..r{GRID_ROWS} = top..bottom, c1..c{GRID_COLUMNS} = left..right), "
                    "optionally prefixed by 'left:' / 'right:'"
                )
            row = int(match.group(1)) if match.group(1) else None
            col = int(match.group(2)) if match.group(2) else None
            if row is not None and not 1 <= row <= GRID_ROWS:
                raise ValueError(f"--ignore-cells {token!r}: row must be 1..{GRID_ROWS}")
            if col is not None and not 1 <= col <= GRID_COLUMNS:
                raise ValueError(f"--ignore-cells {token!r}: column must be 1..{GRID_COLUMNS}")
            rows = [row] if row is not None else range(1, GRID_ROWS + 1)
            cols = [col] if col is not None else range(1, GRID_COLUMNS + 1)
            cells = {(r, c) for r in rows for c in cols}
            for name in targets:
                result[name] |= cells
    return {name: frozenset(cells) for name, cells in result.items()}


def _row_runs(rows: list[int]) -> str:
    """[1,2,3] -> 'rows1-3'; [2] -> 'row2'; [1,3] -> 'rows1,3'; [1,2,4] -> 'rows1-2,4'."""
    runs: list[str] = []
    start = prev = rows[0]
    for value in rows[1:] + [None]:
        if value is not None and value == prev + 1:
            prev = value
            continue
        runs.append(str(start) if start == prev else f"{start}-{prev}")
        if value is not None:
            start = prev = value
    return ("row" if len(rows) == 1 else "rows") + ",".join(runs)


def describe_empty_cells(empty) -> str:
    """Human summary, e.g. 'row4: all 6 empty; col6: rows1-3 empty' or 'none'."""
    remaining = set(empty)
    if not remaining:
        return "none"
    parts: list[str] = []
    for r in range(1, GRID_ROWS + 1):
        row_cells = {(r, c) for c in range(1, GRID_COLUMNS + 1)}
        if row_cells <= remaining:
            parts.append(f"row{r}: all {GRID_COLUMNS} empty")
            remaining -= row_cells
    for c in range(1, GRID_COLUMNS + 1):
        rows = sorted(r for (r, cc) in remaining if cc == c)
        if rows:
            parts.append(f"col{c}: {_row_runs(rows)} empty")
    return "; ".join(parts)


def board_object_points() -> np.ndarray:
    """(63, 2) board-plane coordinates of the inner corners in SQUARE units,
    row-major like findChessboardCornersSB (x = column index, y = row index)."""
    cols, rows = np.meshgrid(np.arange(BOARD_COLUMNS), np.arange(BOARD_ROWS))
    return np.stack([cols.ravel(), rows.ravel()], axis=1).astype(np.float64)


def measure_px_per_square(points) -> float:
    """Median adjacent-corner spacing along the LESS foreshortened board axis.

    Tilting the board shortens the spacing along one axis (cos tilt); the
    other axis stays close to focal_px * square / distance, so taking the
    larger of the two axis medians keeps the scale bin a distance proxy.
    """
    grid = np.asarray(points, dtype=np.float64).reshape(BOARD_ROWS, BOARD_COLUMNS, 2)
    along_columns = np.linalg.norm(np.diff(grid, axis=1), axis=2)   # (rows, cols-1)
    along_rows = np.linalg.norm(np.diff(grid, axis=0), axis=2)      # (rows-1, cols)
    return float(max(np.median(along_columns), np.median(along_rows)))


def estimate_tilt_deg(
    points,
    focal_px: float = NOMINAL_FOCAL_PX,
    principal: tuple[float, float] | None = None,
) -> float:
    """Board tilt (deg) from the corner homography, decomposed with a nominal K.

    H = K [r1 r2 t] (board plane z = 0, square units) -> r1, r2, t; the
    board normal is r1 x r2 and the tilt is the angle between that normal and
    the line of sight to the board CENTRE (not the optical axis, so a board at
    the image edge is not mis-read as tilted).  The nominal focal length is
    only ~5 % off any real BRIO, which moves the estimate by ~1-2 deg: fine
    for the 10 / 25 deg bins.  Returns NaN when the homography is degenerate.
    """
    import cv2

    cx, cy = principal if principal is not None else (TOP_WIDTH / 2.0, TOP_HEIGHT / 2.0)
    image_points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if image_points.shape[0] != BOARD_CORNER_COUNT:
        return float("nan")
    homography, _ = cv2.findHomography(board_object_points(), image_points, 0)
    if homography is None or not np.isfinite(homography).all():
        return float("nan")
    k_inv = np.array(
        [[1.0 / focal_px, 0.0, -cx / focal_px], [0.0, 1.0 / focal_px, -cy / focal_px], [0.0, 0.0, 1.0]]
    )
    b = k_inv @ homography
    n1, n2 = np.linalg.norm(b[:, 0]), np.linalg.norm(b[:, 1])
    if n1 <= 0.0 or n2 <= 0.0:
        return float("nan")
    scale = 2.0 / (n1 + n2)
    r1, r2, t = b[:, 0] * scale, b[:, 1] * scale, b[:, 2] * scale
    if t[2] < 0.0:   # the board is in front of the camera
        r1, r2, t = -r1, -r2, -t
    r3 = np.cross(r1, r2)
    u, _, vt = np.linalg.svd(np.column_stack([r1, r2, r3]))
    rotation = u @ vt   # nearest proper rotation
    normal = rotation[:, 2]
    centre = (
        rotation[:, 0] * (BOARD_COLUMNS - 1) / 2.0
        + rotation[:, 1] * (BOARD_ROWS - 1) / 2.0
        + t
    )
    distance = float(np.linalg.norm(centre))
    if distance <= 0.0:
        return float("nan")
    cosine = abs(float(normal @ centre)) / distance
    return math.degrees(math.acos(min(1.0, max(-1.0, cosine))))


def detect_board_stable(
    rgb: np.ndarray,
    samples: int = DETECT_SAMPLES,
    *,
    stop_at_first_miss: bool = True,
    detector=detect_board,
) -> dict:
    """Repeat the solver-identical detection on the SAME frame (see DETECT_SAMPLES).

    Returns the first ``found`` result (else the last one) with two extra
    keys: ``attempts`` (detections run) and ``hits`` (how many found the full
    board); ``stable`` is true iff every one of ``samples`` attempts found it.
    ``stop_at_first_miss`` (the save gate) returns as soon as one attempt
    misses; the resume scan passes False to run all ``samples`` so a view
    that misses sometimes is still counted, but flagged.
    """
    samples = max(1, int(samples))
    best: dict | None = None
    last: dict = {"found": False, "count": 0, "corners": None, "error": None}
    hits = attempts = 0
    for attempts in range(1, samples + 1):
        last = detector(rgb)
        if last["found"]:
            hits += 1
            if best is None:
                best = last
        elif stop_at_first_miss:
            break
    result = dict(best if best is not None else last)
    result.update({"attempts": attempts, "hits": hits, "stable": hits == samples})
    return result


def px_per_square_to_m(px_per_square: float) -> float:
    return NOMINAL_FOCAL_PX * SQUARE_SIZE_M / px_per_square


def scale_bin(px_per_square: float) -> str:
    if px_per_square >= SCALE_NEAR_MIN_PX:
        return "near"
    if px_per_square < SCALE_FAR_MAX_PX:
        return "far"
    return "mid"


def tilt_bin(tilt_deg: float) -> str:
    if not math.isfinite(tilt_deg):
        return "medium"   # degenerate homography: do not credit an extreme bin
    if tilt_deg <= TILT_FLAT_MAX_DEG:
        return "flat"
    if tilt_deg > TILT_STEEP_MIN_DEG:
        return "steep"
    return "medium"


def scale_bin_hint(name: str) -> str:
    near_m = px_per_square_to_m(SCALE_NEAR_MIN_PX)
    far_m = px_per_square_to_m(SCALE_FAR_MAX_PX)
    return {
        "near": f">={SCALE_NEAR_MIN_PX:.0f} px/square, board closer than ~{near_m:.2f} m",
        "mid": f"{SCALE_FAR_MAX_PX:.0f}-{SCALE_NEAR_MIN_PX:.0f} px/square, ~{near_m:.2f}-{far_m:.2f} m",
        "far": f"<{SCALE_FAR_MAX_PX:.0f} px/square, farther than ~{far_m:.2f} m (flat on the table)",
    }[name]


def tilt_bin_hint(name: str) -> str:
    return {
        "flat": f"<={TILT_FLAT_MAX_DEG:.0f} deg",
        "medium": f"{TILT_FLAT_MAX_DEG:.0f}-{TILT_STEEP_MIN_DEG:.0f} deg",
        "steep": f">{TILT_STEEP_MIN_DEG:.0f} deg",
    }[name]


def scale_rule_text() -> str:
    return (
        "px/square = median corner spacing along the less foreshortened axis; "
        + "; ".join(f"{name}: {scale_bin_hint(name)}" for name in SCALE_BIN_NAMES)
        + f" [22 mm squares, f~{NOMINAL_FOCAL_PX:.0f} px at 1080p]"
    )


def tilt_rule_text() -> str:
    return (
        "board normal vs line of sight, from the corner homography; "
        + "; ".join(f"{name}: {tilt_bin_hint(name)}" for name in TILT_BIN_NAMES)
    )


def analyze_corners(corners) -> dict:
    """Grid cells, px/square + scale bin, tilt + tilt bin of one detected board."""
    points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    px_per_square = measure_px_per_square(points)
    tilt_deg = estimate_tilt_deg(points)
    return {
        "cells": cells_hit(points),
        "px_per_square": px_per_square,
        "scale_bin": scale_bin(px_per_square),
        "tilt_deg": tilt_deg,
        "tilt_bin": tilt_bin(tilt_deg),
    }


class CoverageStats:
    """Per-camera coverage bookkeeping (thread-safe: the preview reads it).

    Records one entry per detectable view (calib_NN / val_NN) with its grid
    cells, px/square, tilt and bins.  The grid/scale/tilt tables count the
    CALIBRATION views only; validation views are held out from the solve.
    """

    def __init__(self, label: str, out_dir: Path, ignore_cells=frozenset()) -> None:
        self.label = label
        self.out_dir = Path(out_dir)
        self.ignore_cells: frozenset[Cell] = frozenset(ignore_cells)
        self._lock = threading.Lock()
        self._views: list[dict] = []

    # -- recording ---------------------------------------------------------
    def add_record(
        self,
        kind: str,
        cells,
        px_per_square: float,
        tilt_deg: float,
        path: Path | None = None,
    ) -> dict:
        if kind not in ("calib", "val"):
            raise ValueError(f"kind must be 'calib' or 'val', got {kind!r}")
        record = {
            "path": path,
            "kind": kind,
            "cells": frozenset(cells),
            "px_per_square": float(px_per_square),
            "scale_bin": scale_bin(float(px_per_square)),
            "tilt_deg": float(tilt_deg),
            "tilt_bin": tilt_bin(float(tilt_deg)),
        }
        with self._lock:
            self._views.append(record)
        return record

    def add_view(self, path: Path | None, kind: str, corners) -> dict:
        analysis = analyze_corners(corners)
        return self.add_record(
            kind, analysis["cells"], analysis["px_per_square"], analysis["tilt_deg"], path
        )

    def scan_disk(self) -> dict:
        """Re-detect every calib_NN/val_NN PNG in out_dir (resume support).

        Returns {"detected": n, "undetected": [paths], "bad_shape": [paths],
        "unstable": [(path, hits, attempts)]}.  Every file is detected
        DETECT_SAMPLES times: found in all -> counted; found in some ->
        counted but listed as unstable (the solver's single detection may
        miss it); found in none / wrong shape -> NOT counted (the solver
        would hard-fail) - the caller warns the operator to move them away.
        """
        import cv2

        summary: dict = {"detected": 0, "undetected": [], "bad_shape": [], "unstable": []}
        if not self.out_dir.is_dir():
            return summary
        paths = sorted(self.out_dir.glob("calib_*.png")) + sorted(self.out_dir.glob("val_*.png"))
        for path in paths:
            kind = "calib" if path.name.startswith("calib_") else "val"
            bgr = cv2.imread(str(path))
            if bgr is None or bgr.dtype != np.uint8 or bgr.shape != (TOP_HEIGHT, TOP_WIDTH, 3):
                summary["bad_shape"].append(path)
                continue
            detection = detect_board_stable(
                cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), stop_at_first_miss=False
            )
            if not detection["found"]:
                summary["undetected"].append(path)
                continue
            self.add_view(path, kind, detection["corners"])
            summary["detected"] += 1
            if not detection["stable"]:
                summary["unstable"].append((path, detection["hits"], detection["attempts"]))
        return summary

    # -- queries -----------------------------------------------------------
    def views(self, kind: str | None = None) -> list[dict]:
        with self._lock:
            return [v for v in self._views if kind is None or v["kind"] == kind]

    def counts(self) -> tuple[int, int]:
        with self._lock:
            calib = sum(1 for v in self._views if v["kind"] == "calib")
            return calib, len(self._views) - calib

    def grid_hits(self) -> np.ndarray:
        """(GRID_ROWS, GRID_COLUMNS) number of CALIB views with a corner in each cell."""
        hits = np.zeros((GRID_ROWS, GRID_COLUMNS), dtype=np.int64)
        for view in self.views("calib"):
            for r, c in view["cells"]:
                hits[r - 1, c - 1] += 1
        return hits

    def empty_cells(self) -> list[Cell]:
        hits = self.grid_hits()
        return [
            cell for cell in all_cells()
            if cell not in self.ignore_cells and hits[cell[0] - 1, cell[1] - 1] == 0
        ]

    def bin_counts(self) -> tuple[dict[str, int], dict[str, int]]:
        scale = {name: 0 for name in SCALE_BIN_NAMES}
        tilt = {name: 0 for name in TILT_BIN_NAMES}
        for view in self.views("calib"):
            scale[view["scale_bin"]] += 1
            tilt[view["tilt_bin"]] += 1
        return scale, tilt

    def next_hint(self) -> str:
        hits = self.grid_hits()
        candidates = [cell for cell in all_cells() if cell not in self.ignore_cells]
        if candidates:
            cell = min(candidates, key=lambda rc: (hits[rc[0] - 1, rc[1] - 1], rc))
            cell_text = f"board over {cell_name(cell)} ({hits[cell[0] - 1, cell[1] - 1]} views)"
        else:
            cell_text = "any cell (all ignored)"
        scale, tilt = self.bin_counts()
        scale_name = min(SCALE_BIN_NAMES, key=lambda n: (scale[n], SCALE_BIN_NAMES.index(n)))
        tilt_name = min(TILT_BIN_NAMES, key=lambda n: (tilt[n], TILT_BIN_NAMES.index(n)))
        return (
            f"next: {cell_text}, {scale_name} distance ({scale[scale_name]} views; "
            f"{scale_bin_hint(scale_name)}), {tilt_name} tilt ({tilt[tilt_name]} views; "
            f"{tilt_bin_hint(tilt_name)})"
        )

    def status(self) -> tuple[bool, str]:
        """(done, one-line status): DONE / minimum met / TODO."""
        calib, val = self.counts()
        empty = self.empty_cells()
        scale, tilt = self.bin_counts()
        short: list[str] = []
        if calib < CALIB_GOAL:
            short.append(f"calib {calib}/{CALIB_GOAL} ({CALIB_GOAL - calib} more)")
        if val < VAL_GOAL:
            short.append(f"val {val}/{VAL_GOAL} ({VAL_GOAL - val} more)")
        if empty:
            short.append(f"empty grid cells: {describe_empty_cells(empty)}")
        weak_scale = [f"{n} {scale[n]}/{MIN_VIEWS_PER_BIN}" for n in SCALE_BIN_NAMES if scale[n] < MIN_VIEWS_PER_BIN]
        if weak_scale:
            short.append("scale bins short: " + ", ".join(weak_scale))
        weak_tilt = [f"{n} {tilt[n]}/{MIN_VIEWS_PER_BIN}" for n in TILT_BIN_NAMES if tilt[n] < MIN_VIEWS_PER_BIN]
        if weak_tilt:
            short.append("tilt bins short: " + ", ".join(weak_tilt))
        covered = GRID_ROWS * GRID_COLUMNS - len(self.ignore_cells)
        ignored = (
            f" (ignored: {', '.join(cell_name(c) for c in sorted(self.ignore_cells))})"
            if self.ignore_cells else ""
        )
        if not short:
            return True, (
                f"DONE {self.label}: calib {calib} >= {CALIB_GOAL}, val {val} >= {VAL_GOAL}, "
                f"all {covered} grid cells covered{ignored}, every scale/tilt bin >= "
                f"{MIN_VIEWS_PER_BIN} views - this camera is complete."
            )
        if calib >= CALIB_TARGET and val >= VAL_TARGET:
            prefix = (
                f"{self.label}: minimum met ({CALIB_TARGET} calib + {VAL_TARGET} val), "
                "keep going for coverage - "
            )
        else:
            prefix = f"{self.label}: TODO - "
        return False, prefix + "; ".join(short)

    # -- printing ----------------------------------------------------------
    def report_lines(self) -> list[str]:
        calib, val = self.counts()
        hits = self.grid_hits()
        scale, tilt = self.bin_counts()
        lines = [
            f"--- {self.label.upper()} coverage (calib views only; val held out)  "
            f"calib {calib}/{CALIB_GOAL} (min {CALIB_TARGET})  val {val}/{VAL_GOAL} "
            f"(min {VAL_TARGET})  -> {self.out_dir}",
            f"grid {GRID_COLUMNS}x{GRID_ROWS} (calib views per cell; r1 = top, c1 = left; "
            "'.' = empty, 'x' = ignored):",
            "       " + "".join(f"{'c' + str(c):>5}" for c in range(1, GRID_COLUMNS + 1)),
        ]
        for r in range(1, GRID_ROWS + 1):
            row = f"   r{r} "
            for c in range(1, GRID_COLUMNS + 1):
                n = int(hits[r - 1, c - 1])
                text = str(n) if n > 0 else ("x" if (r, c) in self.ignore_cells else ".")
                row += f"{text:>5}"
            lines.append(row)
        ignored = (
            f"   (ignored: {', '.join(cell_name(c) for c in sorted(self.ignore_cells))})"
            if self.ignore_cells else ""
        )
        lines.append(f"   empty cells: {describe_empty_cells(self.empty_cells())}{ignored}")
        lines.append(f"scale ({scale_rule_text()}):")
        lines.append(
            "   " + "   ".join(f"{name} {scale[name]}" for name in SCALE_BIN_NAMES)
            + "   " + _weak_bins_text(scale, SCALE_BIN_NAMES)
        )
        lines.append(f"tilt ({tilt_rule_text()}):")
        lines.append(
            "   " + "   ".join(f"{name} {tilt[name]}" for name in TILT_BIN_NAMES)
            + "   " + _weak_bins_text(tilt, TILT_BIN_NAMES)
        )
        lines.append(self.next_hint())
        lines.append(self.status()[1])
        return lines

    def print_report(self) -> None:
        for line in self.report_lines():
            say(line)


def _weak_bins_text(counts: dict[str, int], names) -> str:
    empty = [n for n in names if counts[n] == 0]
    weak = [f"{n} ({counts[n]})" for n in names if 0 < counts[n] < MIN_VIEWS_PER_BIN]
    parts = []
    if empty:
        parts.append("EMPTY: " + ", ".join(empty))
    if weak:
        parts.append(f"< {MIN_VIEWS_PER_BIN} views: " + ", ".join(weak))
    return "; ".join(parts) if parts else f"all bins >= {MIN_VIEWS_PER_BIN} views"


# --------------------------------------------------------------------------
# Save-path / counter logic (exercised by --dry-run)
# --------------------------------------------------------------------------
def next_numbered_png(out_dir: Path, prefix: str) -> Path:
    """Auto-numbered ``<prefix>_NN.png`` (NN = highest existing + 1, from 01)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    highest = 0
    for existing in out_dir.glob(f"{prefix}_*.png"):
        digits = existing.stem[len(prefix) + 1 :]
        if digits.isdigit():
            highest = max(highest, int(digits))
    return out_dir / f"{prefix}_{highest + 1:02d}.png"


def count_views(out_dir: Path) -> tuple[int, int]:
    calib = len(list(out_dir.glob("calib_*.png"))) if out_dir.is_dir() else 0
    val = len(list(out_dir.glob("val_*.png"))) if out_dir.is_dir() else 0
    return calib, val


def print_counts(out_dir: Path, label: str | None = None) -> None:
    calib, val = count_views(out_dir)
    prefix = f"{label}: " if label else ""
    say(
        f"{prefix}views on disk: calibration {calib}/{CALIB_GOAL} (min {CALIB_TARGET}), "
        f"validation {val}/{VAL_GOAL} (min {VAL_TARGET})  ({out_dir})"
    )


def write_view(
    rgb: np.ndarray, kind: str, out_dir: Path, detection: dict, label: str | None = None
) -> Path | None:
    """Write one ALREADY-DETECTED view; returns the path or None on failure.

    kind: "calib" or "val".  ``detection`` must be a detect_board result with
    ``found`` true (the caller gates on it).  The written PNG is re-read
    through cv2.imread and shape-checked - the exact read path of
    ``_detect_checkerboard_images`` - before being reported as saved.
    """
    import cv2

    if not detection.get("found"):
        raise ValueError("write_view needs a detection with found=True")
    prefix = f"{label}: " if label else ""
    path = next_numbered_png(out_dir, kind)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(path), bgr):
        path.unlink(missing_ok=True)
        say(f"{prefix}REFUSED: cv2.imwrite failed for {path}; nothing was saved.")
        return None
    readback = cv2.imread(str(path))
    if (
        readback is None
        or readback.dtype != np.uint8
        or readback.shape != (TOP_HEIGHT, TOP_WIDTH, 3)
    ):
        path.unlink(missing_ok=True)
        say(f"{prefix}REFUSED: {path} did not read back as uint8[{TOP_HEIGHT},{TOP_WIDTH},3].")
        return None
    return path


def save_view(rgb: np.ndarray, kind: str, out_dir: Path) -> Path | None:
    """Detect-gate and save one view; returns the path or None when refused.

    Runs the solver-identical scale-1 detection on THIS frame and refuses to
    save when the full 9x7 grid is not found (the solver raises on any
    undetectable image).  Single-camera helper kept for existing callers;
    the interactive loop uses ``capture_views`` (per-camera, with stats).
    """
    detection = detect_board(rgb)
    if not detection["found"]:
        say(
            f"REFUSED: board not fully detected in this frame "
            f"({detection['count']}/{BOARD_CORNER_COUNT} corners) - "
            "adjust the board (tilt/glare/distance) and try again; "
            "nothing was saved."
        )
        return None
    path = write_view(rgb, kind, out_dir, detection)
    if path is not None:
        say(f"saved {path}  ({detection['count']}/{BOARD_CORNER_COUNT} corners)")
    return path


# --------------------------------------------------------------------------
# Camera sessions (one per rig) and the per-camera capture step
# --------------------------------------------------------------------------
@dataclass
class CameraSession:
    """One opened BRIO: its rig, output dir, frame source, detection, stats."""

    rig: RigSpec
    device: str
    out_dir: Path
    stats: CoverageStats
    focus_mode: str = "verify"
    camera: object | None = None      # BrioTopCamera (None in --dry-run)
    pump: object | None = None        # FramePump-like: latest/latest_sequence/wait_for_frame_after
    cache: DetectionCache | None = None

    @property
    def label(self) -> str:
        return self.rig.side


def capture_views(sessions: list[CameraSession], kind: str, detector=detect_board) -> list[dict]:
    """One ``c``/``v`` key: grab the next frame of EVERY camera, save where the
    board is STABLY detected (DETECT_SAMPLES consecutive hits), skip (and say
    so) where it is not.

    Returns one dict per session: {"session", "path" (or None), "reason"}
    with reason in {"saved", "no board", "unstable", "capture failed",
    "write failed"}.  ``detector`` is injectable for the dry run.
    """
    results: list[dict] = []
    for session in sessions:
        pump = session.pump
        try:
            frame = pump.wait_for_frame_after(pump.latest_sequence())
        except RuntimeError as exc:
            say(f"{session.label}: capture FAILED ({exc}) - skipped; check the camera and retry.")
            results.append({"session": session, "path": None, "reason": "capture failed"})
            continue
        detection = detect_board_stable(frame["rgb"], detector=detector)
        if not detection["found"]:
            say(
                f"{session.label}: no board — skipped "
                f"({detection['count']}/{BOARD_CORNER_COUNT} corners; adjust tilt/glare/distance)"
            )
            results.append({"session": session, "path": None, "reason": "no board"})
            continue
        if not detection["stable"]:
            say(
                f"{session.label}: board UNSTABLE — skipped (detected in only "
                f"{detection['hits']} of {detection['attempts']} attempts on this frame: the "
                "solver detects once and would fail on such a view; hold the board steadier/"
                "closer/sharper, avoid glare, and press again)"
            )
            results.append({"session": session, "path": None, "reason": "unstable"})
            continue
        path = write_view(frame["rgb"], kind, session.out_dir, detection, label=session.label)
        if path is None:
            results.append({"session": session, "path": None, "reason": "write failed"})
            continue
        record = session.stats.add_view(path, kind, detection["corners"])
        say(
            f"{session.label}: saved {path}  ({detection['count']}/{BOARD_CORNER_COUNT} corners, "
            f"{record['px_per_square']:.1f} px/square -> {record['scale_bin']}, "
            f"tilt {record['tilt_deg']:.1f} deg -> {record['tilt_bin']}, "
            f"{len(record['cells'])} grid cells)"
        )
        results.append({"session": session, "path": path, "reason": "saved"})
    return results


def print_session_counts(sessions: list[CameraSession]) -> None:
    for session in sessions:
        print_counts(session.out_dir, session.label)


def print_session_reports(sessions: list[CameraSession]) -> None:
    for session in sessions:
        session.stats.print_report()


def scan_sessions(sessions: list[CameraSession]) -> None:
    """Resume: re-detect the PNGs already in every session dir.

    Sequential on purpose: concurrent detections raised the detector's miss
    rate in testing (see DETECT_SAMPLES); ~0.4 s per 1080p view (3 samples).
    """
    for session in sessions:
        on_disk = count_views(session.out_dir)
        say(f"{session.label}: re-detecting {sum(on_disk)} existing views in {session.out_dir} ...")
        summary = session.stats.scan_disk()
        calib, val = session.stats.counts()
        say(f"{session.label}: resumed {summary['detected']} views (calib {calib}, val {val}).")
        for path, hits, attempts in summary["unstable"]:
            say(
                f"{session.label}: WARNING {path.name}: board detected in only {hits}/{attempts} "
                "attempts - counted, but the solver detects ONCE and may fail the whole solve on "
                "it; consider deleting it and retaking (numbering continues)."
            )
        for path in summary["undetected"]:
            say(
                f"{session.label}: WARNING {path.name}: board NOT detectable in {DETECT_SAMPLES} "
                "attempts - NOT counted; the solver hard-fails on it, move it away before solving."
            )
        for path in summary["bad_shape"]:
            say(
                f"{session.label}: WARNING {path.name}: not uint8[{TOP_HEIGHT},{TOP_WIDTH},3] "
                "- NOT counted; not a view of this profile, move it away."
            )


# --------------------------------------------------------------------------
# Preview: one MJPEG page, all cameras side by side with the coverage grid
# --------------------------------------------------------------------------
def announce_preview(title: str, port: int) -> None:
    say("")
    say("=" * 72)
    say(f"{title} (MJPEG) - open in a browser:")
    ips = best_effort_host_ips()
    for ip in ips:
        say(f"    http://{ip}:{port}/")
    if not ips:
        say(f"    http://<this-host-ip>:{port}/   (could not auto-detect host IPs)")
    say(f"    http://127.0.0.1:{port}/   (on this host)")
    say(
        f"green banner 'board OK: {BOARD_CORNER_COUNT}/{BOARD_CORNER_COUNT} "
        "corners' = the checkerboard is fully detected."
    )
    say("=" * 72)
    say("")


def start_preview(source, port: int, title: str) -> DetectionCache | None:
    """Detection+encoder workers + MJPEG server on ``port`` (best effort).

    Single-source preview kept for existing importers (capture_top_pairs.py).
    Announces the browser URLs only when the server actually bound.
    """
    cache = DetectionCache()
    state = PreviewState()
    DetectionWorker(source, cache).start()
    EncoderWorker(source, cache, state).start()
    try:
        server = start_preview_server("0.0.0.0", port, state)
    except OSError as exc:
        say(f"WARNING: preview server could not bind port {port} ({exc}).")
        say("Continuing WITHOUT the browser preview; 'p' still works.")
        return cache
    threading.Thread(
        target=server.serve_forever, name=f"preview-http-{port}", daemon=True
    ).start()
    announce_preview(title, port)
    return cache


def render_camera_tile(session: CameraSession, tile_width: int) -> np.ndarray:
    """One camera's preview tile (BGR): frame + 6x4 coverage grid + corners + banner.

    Covered cells are shaded green (alpha), empty cells outlined red and
    labelled, ignored cells outlined grey.  The frame is scaled to
    ``tile_width`` first so the overlay lines stay crisp in the browser.
    """
    import cv2

    scale = tile_width / TOP_WIDTH
    tile_height = int(round(TOP_HEIGHT * scale))
    frame = session.pump.latest() if session.pump is not None else None
    detection = session.cache.get() if session.cache is not None else None
    if frame is None:
        tile = np.full((tile_height, tile_width, 3), 40, dtype=np.uint8)
    else:
        bgr = cv2.cvtColor(frame["rgb"], cv2.COLOR_RGB2BGR)   # copy: never draw on the pump's array
        if tile_width != TOP_WIDTH:
            tile = cv2.resize(bgr, (tile_width, tile_height), interpolation=cv2.INTER_AREA)
        else:
            tile = bgr
    hits = session.stats.grid_hits()
    ignored = session.stats.ignore_cells
    cell_w = tile_width / GRID_COLUMNS
    cell_h = tile_height / GRID_ROWS

    def bounds(r: int, c: int) -> tuple[int, int, int, int]:
        return (
            int(round((c - 1) * cell_w)), int(round((r - 1) * cell_h)),
            int(round(c * cell_w)) - 1, int(round(r * cell_h)) - 1,
        )

    overlay = tile.copy()
    for r, c in all_cells():
        if hits[r - 1, c - 1] > 0:
            x0, y0, x1, y1 = bounds(r, c)
            cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 200, 0), thickness=-1)
    tile = cv2.addWeighted(overlay, 0.28, tile, 0.72, 0.0)
    for r, c in all_cells():
        x0, y0, x1, y1 = bounds(r, c)
        if hits[r - 1, c - 1] > 0:
            cv2.rectangle(tile, (x0, y0), (x1, y1), (0, 200, 0), 1)
        elif (r, c) in ignored:
            cv2.rectangle(tile, (x0, y0), (x1, y1), (150, 150, 150), 1)
        else:
            cv2.rectangle(tile, (x0, y0), (x1, y1), (0, 0, 230), 2)
            cv2.putText(
                tile, cell_name((r, c)), (x0 + 6, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 230), 1, cv2.LINE_AA,
            )
    if detection is not None and detection.get("corners") is not None:
        corners = (np.asarray(detection["corners"], dtype=np.float32) * scale).astype(np.float32)
        cv2.drawChessboardCorners(tile, (BOARD_COLUMNS, BOARD_ROWS), corners, detection["found"])
    calib, val = session.stats.counts()
    counts = f"calib {calib}/{CALIB_GOAL} val {val}/{VAL_GOAL}"
    head = f"{session.label.upper()} BRIO {session.rig.top_serial}"
    if frame is None:
        text, color = f"{head} | waiting for frames | {counts}", (0, 170, 255)
    elif detection is None:
        text, color = f"{head} | board: detecting ... | {counts}", (0, 170, 255)
    elif detection.get("error"):
        text, color = f"{head} | detection ERROR: {detection['error']}"[:80], (0, 0, 220)
    elif detection["found"]:
        text = f"{head} | board OK: {detection['count']}/{BOARD_CORNER_COUNT} corners | {counts}"
        color = (0, 160, 0)
    else:
        text = f"{head} | board NOT detected ({detection['count']}/{BOARD_CORNER_COUNT}) | {counts}"
        color = (0, 0, 220)
    cv2.rectangle(tile, (0, 0), (tile.shape[1], 26), color, thickness=-1)
    cv2.putText(tile, text, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return tile


def compose_tiles(tiles: list[np.ndarray]) -> np.ndarray:
    """Side-by-side composite (heights padded to the tallest tile)."""
    height = max(t.shape[0] for t in tiles)
    padded = []
    for tile in tiles:
        if tile.shape[0] < height:
            pad = np.zeros((height - tile.shape[0], tile.shape[1], 3), dtype=np.uint8)
            tile = np.vstack([tile, pad])
        padded.append(tile)
    return padded[0] if len(padded) == 1 else np.hstack(padded)


class CoveragePreviewWorker(threading.Thread):
    """~8 fps: render every camera's tile, compose side by side, JPEG-encode."""

    def __init__(
        self,
        sessions: list[CameraSession],
        state: PreviewState,
        tile_width: int,
        period_s: float = 0.125,
    ) -> None:
        super().__init__(name="coverage-preview", daemon=True)
        self._sessions = sessions
        self._state = state
        self._tile_width = tile_width
        self._period_s = period_s
        self._stop_event = threading.Event()

    def render_jpeg(self) -> bytes:
        import cv2

        composite = compose_tiles([render_camera_tile(s, self._tile_width) for s in self._sessions])
        encoded, buffer = cv2.imencode(".jpg", composite, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not encoded:
            raise RuntimeError("cv2.imencode failed on the preview composite")
        return buffer.tobytes()

    def run(self) -> None:
        while not self._stop_event.is_set():
            started = time.monotonic()
            try:
                self._state.update(self.render_jpeg())
            except Exception:  # noqa: BLE001  keep serving; retry next tick
                pass
            elapsed = time.monotonic() - started
            self._stop_event.wait(max(0.01, self._period_s - elapsed))

    def stop(self) -> None:
        self._stop_event.set()


def start_coverage_preview(
    sessions: list[CameraSession], port: int, title: str, tile_width: int
) -> bool:
    """Per-camera detection workers + one composite MJPEG page on ``port``.

    Sets ``session.cache`` on every session (used by 'p' and the tiles) even
    when the HTTP port cannot be bound.  Returns True iff the server bound.
    """
    for session in sessions:
        cache = DetectionCache()
        DetectionWorker(session.pump, cache).start()
        session.cache = cache
    state = PreviewState()
    CoveragePreviewWorker(sessions, state, tile_width).start()
    try:
        server = start_preview_server("0.0.0.0", port, state)
    except OSError as exc:
        say(f"WARNING: preview server could not bind port {port} ({exc}).")
        say("Continuing WITHOUT the browser preview; 'p' still works.")
        return False
    threading.Thread(
        target=server.serve_forever, name=f"preview-http-{port}", daemon=True
    ).start()
    announce_preview(title, port)
    say(
        f"grid overlay: green cells = covered by a calib view, RED outlined cells = still "
        f"empty (labelled rNcM), grey = ignored (--ignore-cells); each camera ~{tile_width} px wide."
    )
    say("")
    return True


# --------------------------------------------------------------------------
# Dry run (no camera): rig table, save path, coverage bins, dual capture, resume
# --------------------------------------------------------------------------
def print_rig(rig: RigSpec) -> None:
    say(f"rig: {rig.side.upper()}")
    say(f"  top BRIO serial {rig.top_serial}  device {rig.top_device}")
    say(f"  profile {TOP_WIDTH}x{TOP_HEIGHT}@{TOP_FPS} {TOP_FOURCC}")
    say(f"  intrinsics views -> {rig.top_intr_dir}")
    say(f"  refuses to start while a bridge listens on 127.0.0.1:{rig.bridge_port}")
    say(f"  focus: {rig.focus_lock} (autofocus must read 0)")


def render_synthetic_view(
    px_per_square: float,
    tilt_deg: float,
    centre_px: tuple[float, float] | None = None,
    inplane_deg: float = 6.0,
    focal_px: float = NOMINAL_FOCAL_PX,
) -> dict:
    """Synthetic 1080p view of the 9x7-inner board at a given scale and tilt.

    A flat board texture is warped with cv2.warpPerspective through the exact
    homography K [r1 r2 t] of a board rotated ``tilt_deg`` about its x axis
    (plus a small in-plane rotation) whose centre is placed ``focal/px`` square
    units away on the ray through ``centre_px`` (default: the principal
    point, so the true line-of-sight tilt equals ``tilt_deg``).  Returns
    {"rgb", "tilt_deg_true", "px_per_square_true"}.
    """
    import cv2

    square_px = 48
    margin = 1
    cols_sq, rows_sq = BOARD_COLUMNS + 1, BOARD_ROWS + 1
    texture = np.full(
        ((rows_sq + 2 * margin) * square_px, (cols_sq + 2 * margin) * square_px, 3),
        225, dtype=np.uint8,
    )
    for r in range(rows_sq):
        for c in range(cols_sq):
            if (r + c) % 2 == 0:
                texture[
                    (margin + r) * square_px:(margin + r + 1) * square_px,
                    (margin + c) * square_px:(margin + c + 1) * square_px,
                ] = 20
    # texture pixel -> board plane (square units; inner corner (i, j) at (i, j))
    offset = margin + 1
    to_board = np.array([[1.0 / square_px, 0.0, -offset], [0.0, 1.0 / square_px, -offset], [0.0, 0.0, 1.0]])
    cx, cy = TOP_WIDTH / 2.0, TOP_HEIGHT / 2.0
    if centre_px is None:
        centre_px = (cx, cy)
    tilt = math.radians(tilt_deg)
    spin = math.radians(inplane_deg)
    rot_z = np.array([[math.cos(spin), -math.sin(spin), 0.0], [math.sin(spin), math.cos(spin), 0.0], [0.0, 0.0, 1.0]])
    rot_x = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(tilt), -math.sin(tilt)], [0.0, math.sin(tilt), math.cos(tilt)]])
    rotation = rot_x @ rot_z
    distance = focal_px / px_per_square
    board_centre = np.array([(BOARD_COLUMNS - 1) / 2.0, (BOARD_ROWS - 1) / 2.0, 0.0])
    centre_cam = np.array(
        [(centre_px[0] - cx) / focal_px * distance, (centre_px[1] - cy) / focal_px * distance, distance]
    )
    translation = centre_cam - rotation @ board_centre
    intrinsics = np.array([[focal_px, 0.0, cx], [0.0, focal_px, cy], [0.0, 0.0, 1.0]])
    homography = intrinsics @ np.column_stack([rotation[:, 0], rotation[:, 1], translation]) @ to_board
    rgb = cv2.warpPerspective(
        texture, homography, (TOP_WIDTH, TOP_HEIGHT), flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT, borderValue=(200, 200, 200),
    )
    normal = rotation[:, 2]
    cosine = abs(float(normal @ centre_cam)) / float(np.linalg.norm(centre_cam))
    return {
        "rgb": np.ascontiguousarray(rgb),
        "tilt_deg_true": math.degrees(math.acos(min(1.0, cosine))),
        "px_per_square_true": float(px_per_square),
    }


class _StaticPump:
    """Stands in for FramePump in --dry-run: always serves one fixed frame."""

    def __init__(self, rgb: np.ndarray) -> None:
        self._rgb = rgb
        self._sequence = 0
        self.serial = "dry-run"

    def latest(self) -> dict:
        self._sequence += 1
        return {"rgb": self._rgb, "pump_sequence": self._sequence, "frame_monotonic_ns": time.monotonic_ns()}

    def latest_sequence(self) -> int:
        return self._sequence

    def wait_for_frame_after(self, sequence: int, timeout_s: float = 2.0) -> dict:
        return self.latest()


def _raises(exc_type, func, *args) -> bool:
    try:
        func(*args)
    except exc_type:
        return True
    return False


def _dry_session(rig: RigSpec, out_dir: Path, rgb: np.ndarray, ignore=frozenset()) -> CameraSession:
    return CameraSession(
        rig=rig, device=rig.top_device, out_dir=out_dir,
        stats=CoverageStats(rig.side, out_dir, ignore), pump=_StaticPump(rgb),
    )


def _fill_stats(stats: CoverageStats, calib: int, val: int, cells) -> None:
    """Inject synthetic records cycling through every scale/tilt bin."""
    scales = (SCALE_NEAR_MIN_PX + 5.0, (SCALE_NEAR_MIN_PX + SCALE_FAR_MAX_PX) / 2.0, SCALE_FAR_MAX_PX - 5.0)
    tilts = (TILT_FLAT_MAX_DEG / 2.0, (TILT_FLAT_MAX_DEG + TILT_STEEP_MIN_DEG) / 2.0, TILT_STEEP_MIN_DEG + 5.0)
    for i in range(calib):
        stats.add_record("calib", cells, scales[i % 3], tilts[(i // 3) % 3])
    for i in range(val):
        stats.add_record("val", cells, scales[i % 3], tilts[i % 3])


def run_dry_run(rigs=(LEFT_RIG,), ignore_cells: dict[str, frozenset[Cell]] | None = None) -> int:
    import cv2

    rigs = list(rigs)
    ignore_cells = ignore_cells or {}
    checks: list[tuple[str, bool]] = []
    for rig in rigs:
        print_rig(rig)
        if ignore_cells.get(rig.side):
            say(f"  ignored grid cells: {', '.join(cell_name(c) for c in sorted(ignore_cells[rig.side]))}")
        checks.append(
            (f"{rig.side}: rig top serial appears in the by-id device path",
             rig.top_serial in Path(rig.top_device).name)
        )
        checks.append(
            (f"{rig.side}: rig intrinsics dir is rig-specific",
             rig.top_intr_dir.name == ("top_intr" if rig.side == "left" else "top_intr_right"))
        )
        checks.append((f"{rig.side}: rig focus mode valid", rig.focus_lock in FOCUS_LOCK_MODES))
    if len(rigs) > 1:
        checks.append(
            ("dual rig: distinct BRIO serials, dirs and bridge ports",
             len({r.top_serial for r in rigs}) == len(rigs)
             and len({r.top_intr_dir for r in rigs}) == len(rigs)
             and len({r.bridge_port for r in rigs}) == len(rigs))
        )
    parsed = parse_v4l2_ctrls(
        "focus_automatic_continuous: 1\nfocus_absolute: 0\nzoom_absolute: 100\n"
        "brightness: 128 (default)\n"
    )
    checks.append(
        ("v4l2-ctl --get-ctrl output parses (autofocus=1 would be refused)",
         parsed == {"focus_automatic_continuous": 1, "focus_absolute": 0,
                    "zoom_absolute": 100, "brightness": 128})
    )

    # -- dual-rig argument parsing (offline, always exercised) -------------
    checks.append(("parse_rigs: one rig", [r.side for r in parse_rigs(["left"])] == ["left"]))
    checks.append(
        ("parse_rigs: two rigs keep the given order",
         [r.side for r in parse_rigs(["right", "left"])] == ["right", "left"])
    )
    checks.append(("parse_rigs: duplicate rig rejected", _raises(ValueError, parse_rigs, ["left", "left"])))
    checks.append(("parse_rigs: three rigs rejected", _raises(ValueError, parse_rigs, ["left", "right", "left"])))
    checks.append(("parse_rigs: unknown rig rejected", _raises(ValueError, parse_rigs, ["middle"])))
    checks.append(
        ("parse_ignore_cells: 'r4c6 c6 left:r1c1' -> per-rig sets",
         parse_ignore_cells(["r4c6", "c6", "left:r1c1"], ["left", "right"])
         == {"left": frozenset({(4, 6), (1, 6), (2, 6), (3, 6), (1, 1)}),
             "right": frozenset({(4, 6), (1, 6), (2, 6), (3, 6)})})
    )
    checks.append(("parse_ignore_cells: r5c1 (row out of range) rejected",
                   _raises(ValueError, parse_ignore_cells, ["r5c1"], ["left"])))
    checks.append(("parse_ignore_cells: 'right:c6' rejected in a left-only session",
                   _raises(ValueError, parse_ignore_cells, ["right:c6"], ["left"])))

    # -- grid cell mapping + empty-cell wording ----------------------------
    checks.append(
        ("cells_hit: (0,0)->r1c1 (1919,1079)->r4c6 (960,540)->r3c4 (959,539)->r2c3",
         cells_hit([[0, 0], [1919, 1079], [960, 540], [959, 539]]) == {(1, 1), (4, 6), (3, 4), (2, 3)})
    )
    empty = {(4, c) for c in range(1, 7)} | {(1, 6), (2, 6), (3, 6)}
    checks.append(
        ("describe_empty_cells -> 'row4: all 6 empty; col6: rows1-3 empty'",
         describe_empty_cells(empty) == "row4: all 6 empty; col6: rows1-3 empty")
    )
    checks.append(("describe_empty_cells: {r1c2, r3c2, r2c5} -> 'col2: rows1,3 empty; col5: row2 empty'",
                   describe_empty_cells({(1, 2), (3, 2), (2, 5)}) == "col2: rows1,3 empty; col5: row2 empty"))
    checks.append(("describe_empty_cells: none", describe_empty_cells(set()) == "none"))
    checks.append(("scale bins: 60->near, 59.9->mid, 40->mid, 39.9->far",
                   (scale_bin(60.0), scale_bin(59.9), scale_bin(40.0), scale_bin(39.9)) == ("near", "mid", "mid", "far")))
    checks.append(("tilt bins: 10->flat, 10.1->medium, 25->medium, 25.1->steep",
                   (tilt_bin(10.0), tilt_bin(10.1), tilt_bin(25.0), tilt_bin(25.1)) == ("flat", "medium", "medium", "steep")))
    say(f"scale rule: {scale_rule_text()}")
    say(f"tilt rule:  {tilt_rule_text()}")

    with tempfile.TemporaryDirectory(prefix="top_intr_dry_") as tmp:
        out_dir = Path(tmp) / rigs[0].top_intr_dir.name

        # -- single-camera save path (unchanged behaviour) ------------------
        checks.append(
            ("fresh dir numbers calib_01",
             next_numbered_png(out_dir, "calib").name == "calib_01.png")
        )
        board_rgb = _synthetic_board_rgb(size=(TOP_HEIGHT, TOP_WIDTH))
        first = save_view(board_rgb, "calib", out_dir)
        checks.append(
            ("synthetic board saved as calib_01.png",
             first is not None and first.name == "calib_01.png" and first.is_file())
        )
        # Numbering continues past gaps: fake calib_03, expect calib_04 next.
        (out_dir / "calib_03.png").write_bytes(b"")
        second = next_numbered_png(out_dir, "calib")
        checks.append(("gap in numbering -> max+1 (calib_04)", second.name == "calib_04.png"))
        (out_dir / "calib_03.png").unlink()

        blank = np.full((TOP_HEIGHT, TOP_WIDTH, 3), 128, dtype=np.uint8)
        checks.append(
            ("blank frame is refused (nothing saved)",
             save_view(blank, "calib", out_dir) is None)
        )
        stable = detect_board_stable(board_rgb)
        checks.append((f"detect_board_stable: synthetic board stable ({DETECT_SAMPLES}/{DETECT_SAMPLES} hits)",
                       stable["found"] and stable["stable"] and stable["hits"] == DETECT_SAMPLES))
        stable = detect_board_stable(blank)
        checks.append(("detect_board_stable: blank frame -> no hits, gate stops after 1 attempt",
                       not stable["found"] and stable["hits"] == 0 and stable["attempts"] == 1))
        stable = detect_board_stable(blank, stop_at_first_miss=False)
        checks.append((f"detect_board_stable: scan mode runs all {DETECT_SAMPLES} attempts on a blank",
                       not stable["found"] and stable["attempts"] == DETECT_SAMPLES))
        calls = {"n": 0}

        def flaky_detector(rgb):   # misses on its 2nd call only (a marginal real view)
            calls["n"] += 1
            result = detect_board(rgb)
            if calls["n"] == 2:
                result = {"found": False, "count": 0, "corners": None, "error": None}
            return result

        stable = detect_board_stable(board_rgb, detector=flaky_detector)
        checks.append(("detect_board_stable: flaky detector (miss on attempt 2) -> unstable, 1/2",
                       stable["found"] and not stable["stable"] and (stable["hits"], stable["attempts"]) == (1, 2)))
        calls["n"] = 0
        stable = detect_board_stable(board_rgb, stop_at_first_miss=False, detector=flaky_detector)
        checks.append((f"detect_board_stable: scan mode with flaky detector -> counted but unstable (2/{DETECT_SAMPLES})",
                       stable["found"] and not stable["stable"] and (stable["hits"], stable["attempts"]) == (2, DETECT_SAMPLES)))
        val_path = save_view(board_rgb, "val", out_dir)
        checks.append(
            ("validation counter independent (val_01.png)",
             val_path is not None and val_path.name == "val_01.png")
        )
        calib_count, val_count = count_views(out_dir)
        checks.append(("counts see 1 calib + 1 val", (calib_count, val_count) == (1, 1)))
        print_counts(out_dir)

        # The saved PNG must survive the solver's exact read gate.
        readback = cv2.imread(str(first))
        checks.append(
            ("saved PNG reads back uint8[H,W,3] at the top profile with a detectable board",
             readback is not None
             and readback.shape == (TOP_HEIGHT, TOP_WIDTH, 3)
             and detect_board(
                 cv2.cvtColor(readback, cv2.COLOR_BGR2RGB)
             )["found"])
        )

        # -- scale / tilt binning on rendered synthetic boards ----------------
        rendered: dict[tuple[float, float], dict] = {}
        for px, tilt in product((75.0, 50.0, 32.0), (0.0, 18.0, 32.0)):
            view = render_synthetic_view(px, tilt)
            detection = detect_board(view["rgb"])
            name = f"synthetic {px:.0f} px/square tilt {tilt:.0f} deg"
            if not detection["found"]:
                checks.append((f"{name}: detected", False))
                continue
            analysis = analyze_corners(detection["corners"])
            rendered[(px, tilt)] = {"view": view, "analysis": analysis}
            ok = (
                analysis["scale_bin"] == scale_bin(px)
                and analysis["tilt_bin"] == tilt_bin(tilt)
                and abs(analysis["px_per_square"] / px - 1.0) <= 0.10
                and abs(analysis["tilt_deg"] - view["tilt_deg_true"]) <= 3.0
            )
            checks.append(
                (f"{name}: measured {analysis['px_per_square']:.1f} px -> {analysis['scale_bin']}, "
                 f"tilt {analysis['tilt_deg']:.1f} deg (true {view['tilt_deg_true']:.1f}) -> "
                 f"{analysis['tilt_bin']}", ok)
            )
        off_centre = render_synthetic_view(45.0, 12.0, centre_px=(1650.0, 880.0))
        off_detection = detect_board(off_centre["rgb"])
        off_cells = cells_hit(off_detection["corners"]) if off_detection["found"] else frozenset()
        checks.append(
            (f"off-centre synthetic board (bottom-right) hits r4c6: cells {sorted(off_cells)}",
             off_detection["found"] and (4, 6) in off_cells and (1, 1) not in off_cells)
        )

        # -- per-camera independent saving (board in ONE camera only) ---------
        left_rig, right_rig = get_rig("left"), get_rig("right")
        dir_a = Path(tmp) / "dual" / left_rig.top_intr_dir.name
        dir_b = Path(tmp) / "dual" / right_rig.top_intr_dir.name
        session_a = _dry_session(left_rig, dir_a, board_rgb)
        session_b = _dry_session(right_rig, dir_b, blank)
        results = capture_views([session_a, session_b], "calib")
        checks.append(
            ("dual 'c': board camera (left) saved calib_01.png",
             results[0]["reason"] == "saved" and results[0]["path"].name == "calib_01.png"
             and (dir_a / "calib_01.png").is_file())
        )
        checks.append(
            ("dual 'c': blank camera (right) reported 'no board', nothing saved",
             results[1]["reason"] == "no board" and results[1]["path"] is None
             and count_views(dir_b) == (0, 0))
        )
        checks.append(
            ("dual 'c': stats updated only for the board camera",
             session_a.stats.counts() == (1, 0) and session_b.stats.counts() == (0, 0))
        )
        # Swap: now only the right camera sees a board; 'v' must save there only.
        far_view = rendered.get((32.0, 32.0), {}).get("view") or render_synthetic_view(32.0, 32.0)
        session_a.pump = _StaticPump(blank)
        session_b.pump = _StaticPump(far_view["rgb"])
        results = capture_views([session_a, session_b], "val")
        checks.append(
            ("dual 'v' swapped: left skipped, right saved val_01.png (own numbering)",
             results[0]["reason"] == "no board" and results[1]["reason"] == "saved"
             and results[1]["path"].name == "val_01.png"
             and count_views(dir_a) == (1, 0) and count_views(dir_b) == (0, 1))
        )
        print_session_counts([session_a, session_b])
        calls["n"] = 0
        session_b.pump = _StaticPump(board_rgb)
        results = capture_views([session_b], "calib", detector=flaky_detector)
        checks.append(
            ("dual 'c' with a flaky detection -> 'unstable', nothing saved",
             results[0]["reason"] == "unstable" and results[0]["path"] is None
             and count_views(dir_b) == (0, 1))
        )
        session_b.pump = _StaticPump(far_view["rgb"])

        # -- preview: per-camera tiles with the grid overlay, side by side ------
        session_a.cache = DetectionCache()
        session_a.cache.set({**detect_board(blank), "frame_sequence": 1, "checked_monotonic": time.monotonic()})
        session_b.cache = DetectionCache()
        session_b.cache.set({**detect_board(far_view["rgb"]), "frame_sequence": 1, "checked_monotonic": time.monotonic()})
        tile_a = render_camera_tile(session_a, PREVIEW_TILE_WIDTH)
        composite = compose_tiles([tile_a, render_camera_tile(session_b, PREVIEW_TILE_WIDTH)])
        checks.append(
            (f"preview composite: two {PREVIEW_TILE_WIDTH}-px tiles side by side -> 540x1920x3",
             composite.shape == (540, 2 * PREVIEW_TILE_WIDTH, 3))
        )
        checks.append(
            ("preview single camera: native 1080x1920 tile",
             render_camera_tile(session_a, TOP_WIDTH).shape == (TOP_HEIGHT, TOP_WIDTH, 3))
        )
        # Left tile shows a grey frame: covered cell r2c3 (test card) is tinted
        # green, empty cell r1c1 keeps the plain grey.
        covered = tile_a[202, 400].astype(int)
        plain = tile_a[67, 80].astype(int)
        checks.append(
            ("preview overlay: covered cell tinted green, empty cell untinted",
             covered[1] - covered[0] > 30 and plain[0] == plain[1] == plain[2])
        )
        jpeg = CoveragePreviewWorker([session_a, session_b], PreviewState(), PREVIEW_TILE_WIDTH).render_jpeg()
        checks.append(("preview worker encodes the composite as JPEG", len(jpeg) > 1000 and jpeg[:2] == b"\xff\xd8"))

        # -- on-disk re-detection (resume) ------------------------------------
        near_flat = rendered.get((75.0, 0.0), {}).get("view") or render_synthetic_view(75.0, 0.0)
        cv2.imwrite(str(dir_a / "calib_02.png"), cv2.cvtColor(far_view["rgb"], cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(dir_a / "val_01.png"), cv2.cvtColor(near_flat["rgb"], cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(dir_a / "val_02.png"), cv2.cvtColor(blank, cv2.COLOR_RGB2BGR))          # undetectable
        cv2.imwrite(str(dir_a / "calib_03.png"), np.zeros((360, 640, 3), dtype=np.uint8))      # wrong profile
        resumed = CoverageStats("left", dir_a)
        summary = resumed.scan_disk()
        scale_counts, tilt_counts = resumed.bin_counts()
        checks.append(
            ("resume: re-detected calib_01 + calib_02 + val_01 from disk (calib 2, val 1)",
             summary["detected"] == 3 and resumed.counts() == (2, 1))
        )
        checks.append(
            ("resume: undetectable val_02 and 640x360 calib_03 reported, NOT counted",
             [p.name for p in summary["undetected"]] == ["val_02.png"]
             and [p.name for p in summary["bad_shape"]] == ["calib_03.png"])
        )
        checks.append(
            ("resume: calib bins = 1 far/steep (calib_02) + 1 from the flat test card",
             scale_counts["far"] == 1 and tilt_counts["steep"] == 1 and sum(scale_counts.values()) == 2)
        )
        checks.append(
            ("resume: next save numbers calib_04 (after the wrong-profile calib_03)",
             next_numbered_png(dir_a, "calib").name == "calib_04.png")
        )
        resumed.print_report()

        # -- DONE hint logic --------------------------------------------------
        full = CoverageStats("left", dir_a)
        _fill_stats(full, CALIB_GOAL, VAL_GOAL, all_cells())
        done, text = full.status()
        checks.append((f"DONE: {CALIB_GOAL} calib + {VAL_GOAL} val, 24 cells, all bins >= 3 -> '{text[:40]}...'", done and text.startswith("DONE left")))
        missing = CoverageStats("left", dir_a)
        _fill_stats(missing, CALIB_GOAL, VAL_GOAL, [c for c in all_cells() if c != (4, 6)])
        done, text = missing.status()
        checks.append(("DONE withheld when r4c6 is empty; status names it",
                       not done and "col6: row4 empty" in text and text.startswith("left: minimum met")))
        ignored = CoverageStats("left", dir_a, {(4, 6)})
        _fill_stats(ignored, CALIB_GOAL, VAL_GOAL, [c for c in all_cells() if c != (4, 6)])
        done, text = ignored.status()
        checks.append(("DONE granted with --ignore-cells r4c6 (23 cells)", done and "23 grid cells" in text and "r4c6" in text))
        partial = CoverageStats("left", dir_a)
        _fill_stats(partial, 25, 5, all_cells())
        done, text = partial.status()
        checks.append(("25 calib + 5 val -> 'minimum met, keep going' with 5 calib / 1 val more",
                       not done and "minimum met" in text and "5 more" in text and "val 5/6 (1 more)" in text))
        todo = CoverageStats("left", dir_a)
        _fill_stats(todo, 10, 2, all_cells())
        done, text = todo.status()
        checks.append(("10 calib + 2 val -> TODO", not done and text.startswith("left: TODO")))
        one_bin = CoverageStats("left", dir_a)
        for _ in range(CALIB_GOAL):
            one_bin.add_record("calib", all_cells(), SCALE_NEAR_MIN_PX + 5.0, 2.0)
        for _ in range(VAL_GOAL):
            one_bin.add_record("val", all_cells(), SCALE_NEAR_MIN_PX + 5.0, 2.0)
        done, text = one_bin.status()
        checks.append(("all views near+flat -> DONE withheld, mid/far and medium/steep bins named",
                       not done and "mid 0/3" in text and "far 0/3" in text
                       and "medium 0/3" in text and "steep 0/3" in text))
        hint = one_bin.next_hint()
        checks.append((f"next hint names the emptiest scale/tilt bins: '{hint[:60]}...'",
                       "mid distance (0 views" in hint and "medium tilt (0 views" in hint))
        hint = missing.next_hint()
        checks.append(("next hint names the empty cell r4c6", "board over r4c6 (0 views)" in hint))

    ok = all(passed for _, passed in checks)
    for name, passed in checks:
        say(f"  [{'ok' if passed else 'FAIL'}] {name}")
    say(f"DRY-RUN {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


# --------------------------------------------------------------------------
# Main interactive loop
# --------------------------------------------------------------------------
HELP_TEXT = f"""commands (press Enter after each):
  c   save a CALIBRATION view on EVERY opened camera whose board is fully detected
      (goal {CALIB_GOAL} per camera, solver minimum {CALIB_TARGET}: vary tilt/distance/position)
  v   save a VALIDATION view  (goal {VAL_GOAL} per camera, minimum {VAL_TARGET}; held out from the solve)
  p   print the detection status + the coverage tables per camera
  q   quit
"""


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Standalone top-BRIO intrinsics capture (no robot, no bridge); one or two rigs"
    )
    parser.add_argument(
        "--rig", nargs="+", choices=RIG_CHOICES, default=["left"], metavar="RIG",
        help="one or two of: left = our station (BRIO 178B0DAE, default); right = the "
             "right rig (BRIO B8C7F203, the right-arm bridge on 9020 must be stopped). "
             "'--rig left right' opens both BRIOs in one session",
    )
    parser.add_argument(
        "--device", default=None,
        help=f"V4L2 by-id path (single rig only; default: the rig's, e.g. {TOP_DEVICE})",
    )
    parser.add_argument(
        "--out-dir", type=Path, default=None,
        help=f"PNG output directory (single rig only; default: the rig's, e.g. {TOP_INTR_DIR})",
    )
    parser.add_argument(
        "--focus-lock", choices=("rig", *FOCUS_LOCK_MODES), default="rig",
        help="rig = each rig's default (left: verify, right: lock); lock = set autofocus "
             "off / focus 0 / zoom 100 then verify; verify = read-only check; "
             "skip = no check (NOT recommended)",
    )
    parser.add_argument(
        "--preview-port", type=int, default=8767,
        help="MJPEG preview port (default 8767; wrist tools use 8766)",
    )
    parser.add_argument(
        "--preview-tile-width", type=int, default=None,
        help=f"browser width per camera (default: {PREVIEW_TILE_WIDTH} when two cameras are "
             f"open, native {TOP_WIDTH} for one)",
    )
    parser.add_argument("--no-preview", action="store_true")
    parser.add_argument(
        "--ignore-cells", nargs="*", default=[], metavar="CELL",
        help="grid cells the DONE hint may leave uncovered, e.g. 'r4c6', 'c6' (whole column), "
             "'r4' (whole row), 'left:c6' (one rig only); r1 = top, c1 = left",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="no camera: exercise rig parsing, save paths, coverage bins, dual capture, resume",
    )
    args = parser.parse_args()
    if not 1 <= args.preview_port <= 65535:
        parser.error("--preview-port must be in 1..65535")
    try:
        rigs = parse_rigs(args.rig)
    except ValueError as exc:
        parser.error(str(exc))
    if len(rigs) > 1 and (args.device is not None or args.out_dir is not None):
        parser.error("--device / --out-dir apply to a single --rig only")
    try:
        ignore_cells = parse_ignore_cells(args.ignore_cells, [rig.side for rig in rigs])
    except ValueError as exc:
        parser.error(str(exc))
    tile_width = args.preview_tile_width
    if tile_width is None:
        tile_width = PREVIEW_TILE_WIDTH if len(rigs) > 1 else TOP_WIDTH
    if not 160 <= tile_width <= TOP_WIDTH:
        parser.error(f"--preview-tile-width must be in 160..{TOP_WIDTH}")

    sessions: list[CameraSession] = []
    for rig in rigs:
        out_dir = args.out_dir or rig.top_intr_dir
        sessions.append(
            CameraSession(
                rig=rig,
                device=args.device or rig.top_device,
                out_dir=out_dir,
                stats=CoverageStats(rig.side, out_dir, ignore_cells.get(rig.side, frozenset())),
                focus_mode=rig.focus_lock if args.focus_lock == "rig" else args.focus_lock,
            )
        )

    if args.dry_run:
        return run_dry_run(rigs, ignore_cells)

    labels = "+".join(s.label for s in sessions)
    serials = ", ".join(f"{s.label} BRIO {s.rig.top_serial}" for s in sessions)
    say(f"== top-camera intrinsics capture: {serials} ==")
    for session in sessions:
        print_rig(session.rig)
        if session.stats.ignore_cells:
            say(f"  ignored grid cells: {', '.join(cell_name(c) for c in sorted(session.stats.ignore_cells))}")
    # ONE OWNER PER CAMERA: refuse if EITHER rig's bridge is listening (all
    # ports are checked and reported before any hardware is touched).
    refused = [refuse_if_bridge_listening(s.rig.bridge_port, s.rig.side) for s in sessions]
    if any(refused):
        return 2
    for session in sessions:
        say(f"{session.label}: focus check on {session.device} [{session.focus_mode}]")
        try:
            ensure_focus_locked(session.device, session.focus_mode)
        except RuntimeError as exc:
            say(f"REFUSING to start: {exc}")
            return 2

    scan_sessions(sessions)   # resume: coverage from the PNGs already on disk

    for session in sessions:
        say(f"{session.label}: opening {session.device}")
        say(f"  required profile: {TOP_WIDTH}x{TOP_HEIGHT}@{TOP_FPS} V4L2 {TOP_FOURCC}")
        try:
            session.camera = BrioTopCamera(session.device, serial=session.rig.top_serial)
        except RuntimeError as exc:
            say(f"REFUSING to start: {session.label}: {exc}")
            for opened in sessions:
                if opened.camera is not None:
                    opened.camera.close()
            return 2
        say(f"  {session.label}: profile verified (width/height/fps and MJPG FourCC all match).")

    for session in sessions:
        pump = FramePump(session.camera)
        pump.start()
        session.pump = pump
    try:
        for session in sessions:
            session.pump.wait_until_ready(min_frames=5, timeout_s=10.0)

        if args.no_preview:
            say("browser preview disabled (--no-preview); 'p' still detects inline.")
        else:
            start_coverage_preview(
                sessions, args.preview_port,
                "LIVE TOP-CAMERA (BRIO) PREVIEW" + (" - both cameras side by side" if len(sessions) > 1 else ""),
                tile_width,
            )

        say("Hold the 22 mm 9x7-inner checkerboard IN HAND under the camera(s).")
        say("Cover all 24 grid cells (corners and the bottom edge included), distances")
        say(f"~0.4-1.0 m (near/mid/far bins) and tilts up to ~35 deg (flat/medium/steep);")
        say("keep the board still and sharp while saving (no motion blur).")
        if len(sessions) > 1:
            say("Each 'c'/'v' saves on EVERY camera that fully sees the board; a camera")
            say("without the board is skipped and reported, the other still saves.")
        print_session_counts(sessions)
        print_session_reports(sessions)
        say(HELP_TEXT)

        while True:
            try:
                line = input(f"[top-intr {labels}] c/v/p/q > ").strip().lower()
            except EOFError:
                line = "q"
            if line in ("c", "v"):
                kind = "calib" if line == "c" else "val"
                capture_views(sessions, kind)
                print_session_counts(sessions)
                print_session_reports(sessions)
            elif line == "p":
                for session in sessions:
                    say(f"[{session.label}]")
                    print_board_status(session.cache, session.pump)
                print_session_counts(sessions)
                print_session_reports(sessions)
            elif line == "q":
                break
            else:
                say(HELP_TEXT)
    finally:
        for session in sessions:
            if session.pump is not None:
                session.pump.stop()
        for session in sessions:
            if session.camera is not None:
                session.camera.close()
    print_session_counts(sessions)
    for session in sessions:
        say(session.stats.status()[1])
    say("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
