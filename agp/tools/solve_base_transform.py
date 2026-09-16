#!/usr/bin/env python3
"""Solve T_left_right = pose of right_base in left_base (the two-arm world frame) and write
agp/config/right_base_in_left_base.json, which server_real.py --arm right loads at boot.

One simultaneous observation gives the chain (all 4x4 homogeneous, "T_a_b" = pose of b in a,
i.e. p_a = T_a_b @ p_b):

    T_lb_rb = T_lb_lt @ T_lt_rw @ inv(T_rb_rw)

  T_lb_lt  left TOP camera pose in left_base       (a) --left-calib <LEFT frames/NNNN_calib.json> [--left-cam top]
                                                       or agp/config/top_calib.json, or --left-top-pose POSE
  T_lt_rw  right WRIST camera pose in the left top (b) --cam-to-cam POSE  = the user's calibration
           camera frame ("left_top_cam -> right_wrist_cam", same arrow convention as the station
           README's "left_base -> right_base = 0 -0.61 0": the pose of the second frame in the first).
           If your calibration is the other way round (left_top_cam coordinates expressed in the
           right wrist camera), add --invert.
  T_rb_rw  right WRIST camera pose in right_base   (c) --right-calib <RIGHT frames_right/NNNN_calib.json> [--right-cam wrist]
           at the same instant                        or --right-wrist-pose POSE (right_base frame).
           A calib.json written by the RIGHT server holds WORLD-frame (left_base) poses plus
           _meta.base_in_world = the transform it used; that conversion is undone here exactly, so
           the file may come from a session that ran with the nominal offset.

POSE (for --cam-to-cam / --left-top-pose / --right-wrist-pose) is one of
  "x,y,z,qw,qx,qy,qz"          translation (m) + scalar-first unit quaternion
  path.json                    {"position":{x,y,z},"rotation":{w,x,y,z}} | {"translation":[..],"rotation_wxyz":[..]}
                               | {"pose": <either>} | {"matrix": 4x4} | a bare 4x4 nested list
  path.npz[:key]               a 4x4 array (default key camera_to_world, the calib/ tools' field)
  path.txt                     a 4x4 whitespace matrix (np.loadtxt)

Modes
  (default)   solve, print, and WRITE --out (default config/right_base_in_left_base.json;
              refuses to overwrite an existing file without --force)
  --dry-run   solve and print only
  --check     print the nominal-vs-solved difference (translation mm, rotation deg) and, if a
              config file exists, the file-vs-solved difference; never writes. With no inputs at
              all, --check compares the existing config file with the nominal offset.

Capture times: when BOTH (a) and (c) are server calib.json files (each carries _meta.wall_time_ns),
their skew is printed and stored (inputs.capture_skew_s); a write is refused above
--max-time-skew-s (default 2 s) unless --allow-time-skew (the left top camera is fixed, so an
OLDER left frame is legitimate — but the right wrist pose must be from the instant of the
--cam-to-cam observation). An existing --out is never overwritten without --force, whatever its
key shape and even when it cannot be parsed.

Offline math only (numpy). No hardware, no gap import. The system python3 has no numpy: run it
with the bridge venv python (PY below) or any python that has numpy. Examples:
  PY=hardware-bridge/.venv/bin/python
  $PY tools/solve_base_transform.py --left-calib sessions/X/frames/0003_calib.json \\
      --cam-to-cam calib/out/lefttop_to_rightwrist.json --right-calib sessions/X/frames_right/0002_calib.json --dry-run
  $PY tools/solve_base_transform.py --check
"""
import argparse
import datetime as _dt
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FA = os.path.dirname(HERE)
DEFAULT_OUT = os.path.join(FA, "config", "right_base_in_left_base.json")
DEFAULT_LEFT_TOP_CALIB = os.path.join(FA, "config", "top_calib.json")
NOMINAL_T = np.array([0.0, -0.61, 0.0])
NOMINAL_Q = np.array([1.0, 0.0, 0.0, 0.0])
WORLD_FRAME = "left_base"


# ---------------------------------------------------------------- quaternion / matrix math
def wxyz_to_mat(q):
    w, x, y, z = [float(v) for v in q]
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_wxyz(R):
    """3x3 rotation -> unit quaternion (w,x,y,z), w >= 0 (Shepperd)."""
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q, dtype=float)
    q /= np.linalg.norm(q) or 1.0
    return -q if q[0] < 0 else q


def quat_mul(a, b):
    aw, ax, ay, az = [float(v) for v in a]
    bw, bx, by, bz = [float(v) for v in b]
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw], dtype=float)


def make_T(t, q_wxyz):
    T = np.eye(4)
    T[:3, :3] = wxyz_to_mat(q_wxyz)
    T[:3, 3] = np.asarray(t, dtype=float).ravel()
    return T


def split_T(T):
    T = np.asarray(T, dtype=float)
    return T[:3, 3].copy(), mat_to_wxyz(T[:3, :3])


def inv_T(T):
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def check_rigid(T, what):
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError(f"{what}: not a finite 4x4")
    R = T[:3, :3]
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-9):
        raise ValueError(f"{what}: last row must be 0 0 0 1")
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-5) or not math.isclose(float(np.linalg.det(R)), 1.0, abs_tol=1e-5):
        raise ValueError(f"{what}: rotation block is not a proper rotation")
    return T


def rot_angle_deg(Ra, Rb):
    d = Ra.T @ Rb
    c = (np.trace(d) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, float(c)))))


# ---------------------------------------------------------------- pose parsing
def pose_dict_to_T(d, what):
    """server pose dict {position:{xyz}, rotation:{wxyz}} | {translation, rotation_wxyz} | {matrix} | 4x4 list."""
    if isinstance(d, dict) and "pose" in d and isinstance(d["pose"], (dict, list)):
        return pose_dict_to_T(d["pose"], what)
    if isinstance(d, dict) and "position" in d and "rotation" in d:
        p, r = d["position"], d["rotation"]
        t = [p["x"], p["y"], p["z"]] if isinstance(p, dict) else list(p)
        q = [r["w"], r["x"], r["y"], r["z"]] if isinstance(r, dict) else list(r)
        return make_T(t, q)
    if isinstance(d, dict) and "translation" in d:
        q = d.get("rotation_wxyz")
        if q is None and isinstance(d.get("rotation"), dict):
            q = [d["rotation"][k] for k in "wxyz"]
        if q is None:
            raise ValueError(f"{what}: missing rotation_wxyz")
        return make_T(d["translation"], q)
    if isinstance(d, dict) and "matrix" in d:
        return check_rigid(np.asarray(d["matrix"], dtype=float), what)
    if isinstance(d, dict) and "camera_to_world" in d:      # bridge / calibrate-top-extrinsics report
        return check_rigid(np.asarray(d["camera_to_world"], dtype=float), what)
    if isinstance(d, (list, tuple)):
        arr = np.asarray(d, dtype=float)
        if arr.shape == (4, 4):
            return check_rigid(arr, what)
        if arr.shape == (7,):
            return make_T(arr[:3], arr[3:])
    raise ValueError(f"{what}: unrecognised pose JSON shape (keys {list(d) if isinstance(d, dict) else type(d).__name__})")


def parse_pose(spec, what, npz_key="camera_to_world"):
    """POSE spec (see module doc) -> 4x4."""
    if spec is None:
        raise ValueError(f"{what}: missing")
    s = str(spec).strip()
    if "," in s and not os.path.exists(s):
        vals = [float(v) for v in s.replace(";", ",").split(",") if v.strip()]
        if len(vals) == 7:
            q = np.array(vals[3:])
            if abs(np.linalg.norm(q) - 1.0) > 1e-3:
                raise ValueError(f"{what}: quaternion norm {np.linalg.norm(q):.6f} is not 1")
            return make_T(vals[:3], q)
        if len(vals) == 16:
            return check_rigid(np.array(vals).reshape(4, 4), what)
        raise ValueError(f"{what}: expected 7 numbers x,y,z,qw,qx,qy,qz (got {len(vals)})")
    path, key = s, npz_key
    if ".npz:" in s:
        path, key = s.rsplit(":", 1)
    if not os.path.exists(path):
        raise ValueError(f"{what}: no such file {path}")
    if path.endswith(".npz"):
        z = np.load(path)
        if key not in z:
            raise ValueError(f"{what}: {path} has no key {key!r} (keys: {list(z.keys())})")
        return check_rigid(np.asarray(z[key], dtype=float), what)
    if path.endswith(".json"):
        with open(path) as f:
            return pose_dict_to_T(json.load(f), what)
    return check_rigid(np.loadtxt(path).reshape(4, 4), what)


def camera_pose_from_calib(path, cam, what):
    """A server calib.json -> (T_base_cam in that ARM's base frame, info dict).
    Right-server files hold world-frame poses + _meta.base_in_world; that conversion is undone."""
    with open(path) as f:
        calib = json.load(f)
    if cam not in calib:
        raise ValueError(f"{what}: {path} has no camera {cam!r} (keys: {[k for k in calib if k != '_meta']})")
    T_written = pose_dict_to_T(calib[cam], f"{what}.{cam}.pose")
    meta = calib.get("_meta", {}) or {}
    frame = calib[cam].get("frame", "robot_base")
    info = {"file": os.path.abspath(path), "cam": cam, "frame_in_file": frame, "capture_time": meta.get("capture_time"),
            "sequence": meta.get("sequence"), "wall_time_ns": meta.get("wall_time_ns")}
    biw = meta.get("base_in_world")
    if biw is not None:
        T_used = make_T(biw["translation"], biw["rotation_wxyz"])
        info["undone_base_in_world"] = biw
        return inv_T(T_used) @ T_written, info
    if frame not in ("robot_base", "right_base", "left_base_native"):
        if frame == WORLD_FRAME and what.startswith("right"):
            raise ValueError(f"{what}: {path} poses are in {frame} but _meta.base_in_world is missing — cannot undo")
    return T_written, info


def capture_skew_s(inputs):
    """|wall_time_ns(left top) - wall_time_ns(right wrist)| in seconds when BOTH inputs are server
    calib.json files carrying _meta.wall_time_ns; None otherwise (config/top_calib.json snapshot or a
    POSE input has no capture time)."""
    lt, rw = inputs.get("left_top") or {}, inputs.get("right_wrist") or {}
    a = lt.get("wall_time_ns") if isinstance(lt, dict) else None
    b = rw.get("wall_time_ns") if isinstance(rw, dict) else None
    if a is None or b is None:
        return None
    return round(abs(int(a) - int(b)) / 1e9, 3)


# ---------------------------------------------------------------- solve / report
def solve(T_lb_lt, T_lt_rw, T_rb_rw):
    """T_lb_rb = T_lb_lt @ T_lt_rw @ inv(T_rb_rw)."""
    return check_rigid(T_lb_lt @ T_lt_rw @ inv_T(T_rb_rw), "solved T_left_right")


def delta(Ta, Tb):
    return {"translation_mm": round(float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3])) * 1000.0, 2),
            "translation_diff_mm": [round(float(v) * 1000.0, 2) for v in (Ta[:3, 3] - Tb[:3, 3])],
            "rotation_deg": round(rot_angle_deg(Ta[:3, :3], Tb[:3, :3]), 3)}


def fmt_T(T):
    t, q = split_T(T)
    return f"t=({t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}) m  q_wxyz=({q[0]:+.6f}, {q[1]:+.6f}, {q[2]:+.6f}, {q[3]:+.6f})"


def result_dict(T, source, date, inputs):
    t, q = split_T(T)
    d = delta(T, make_T(NOMINAL_T, NOMINAL_Q))
    return {"translation": [float(v) for v in t], "rotation_wxyz": [float(v) for v in q],
            "source": source, "date": date, "frame": "pose of right_base in left_base (p_left = R p_right + t)",
            "delta_vs_nominal": d, "inputs": inputs}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--left-calib", help="LEFT server calib.json holding the top camera pose in left_base")
    ap.add_argument("--left-cam", default="top", help="camera key in --left-calib (default top)")
    ap.add_argument("--left-top-pose", help="POSE: left top camera in left_base (alternative to --left-calib)")
    ap.add_argument("--top-in-right-base", help="PATH/POSE: the LEFT top camera's pose in RIGHT_BASE, i.e. the "
                    "calibrate-top-extrinsics report of the 'cross' rig (calib/solve_top.sh RIG=cross -> "
                    "calib/out/top_brio_178B0DAE_in_right_base_calibration.json). With it the solve is "
                    "T_lb_rb = T_lb_lt @ inv(T_rb_lt); --cam-to-cam / --right-calib are not needed.")
    ap.add_argument("--cam-to-cam", help="POSE: T(left_top_cam -> right_wrist_cam) = right wrist camera pose in the left top camera frame")
    ap.add_argument("--invert", action="store_true", help="--cam-to-cam is given the other way round (left top cam in the right wrist cam frame)")
    ap.add_argument("--right-calib", help="RIGHT server calib.json (frames_right/NNNN_calib.json) from the same instant")
    ap.add_argument("--right-cam", default="wrist", help="camera key in --right-calib (default wrist)")
    ap.add_argument("--right-wrist-pose", help="POSE: right wrist camera in right_base (alternative to --right-calib)")
    ap.add_argument("--npz-key", default="camera_to_world", help="array key for .npz POSE files")
    ap.add_argument("--out", default=DEFAULT_OUT, help=f"output JSON (default {DEFAULT_OUT})")
    ap.add_argument("--source", default=None, help="provenance string stored in the file")
    ap.add_argument("--date", default=_dt.date.today().isoformat())
    ap.add_argument("--force", action="store_true", help="overwrite an existing --out")
    ap.add_argument("--max-time-skew-s", type=float, default=2.0,
                    help="max |left top - right wrist| capture skew (s) when both inputs are calib.json files (default 2)")
    ap.add_argument("--allow-time-skew", action="store_true",
                    help="write even when the captures are further apart than --max-time-skew-s (warning only)")
    ap.add_argument("--dry-run", action="store_true", help="solve and print, do not write")
    ap.add_argument("--check", action="store_true", help="print nominal-vs-solved (and file-vs-solved) differences; never writes")
    a = ap.parse_args(argv)

    T_nom = make_T(NOMINAL_T, NOMINAL_Q)
    have_inputs = any([a.left_calib, a.left_top_pose, a.cam_to_cam, a.right_calib, a.right_wrist_pose, a.top_in_right_base])
    existing, existing_unparsed = None, False
    if os.path.exists(a.out):
        try:
            with open(a.out) as f:
                ed = json.load(f)
            existing = pose_dict_to_T(ed, f"existing {a.out}")   # accepts every key shape the server does
        except Exception as e:  # noqa: BLE001
            existing_unparsed = True                             # still an existing file: protected by the --force guard
            print(f"WARNING: could not parse existing {a.out}: {e}", file=sys.stderr)

    if a.check and not have_inputs:
        print(f"nominal : {fmt_T(T_nom)}")
        if existing is None:
            if existing_unparsed:
                print(f"config file {a.out} exists but could not be parsed -> the right server would refuse to boot on it")
                return 2
            print(f"no config file at {a.out} -> the right server would use the NOMINAL offset")
            return 0
        print(f"file    : {fmt_T(existing)}   ({a.out})")
        print(f"file vs nominal: {json.dumps(delta(existing, T_nom))}")
        return 0

    try:
        inputs = {}
        if a.left_top_pose:
            T_lb_lt = parse_pose(a.left_top_pose, "left-top-pose", a.npz_key)
            inputs["left_top_pose"] = str(a.left_top_pose)
        else:
            lc = a.left_calib or DEFAULT_LEFT_TOP_CALIB
            if a.left_calib is None:
                print(f"NOTE: --left-calib not given, using the snapshot {lc} (camera fixed since 2026-09-02)", file=sys.stderr)
            if os.path.basename(lc) == "top_calib.json" and a.left_cam == "top":
                with open(lc) as f:
                    T_lb_lt = pose_dict_to_T(json.load(f), "left-calib.pose")
                inputs["left_top"] = {"file": os.path.abspath(lc), "cam": "top", "frame_in_file": "robot_base (left_base)"}
            else:
                T_lb_lt, info = camera_pose_from_calib(lc, a.left_cam, "left-calib")
                inputs["left_top"] = info
        T_lt_rw = T_rb_rw = None
        skew = None
        if a.top_in_right_base:
            # Same physical camera calibrated in both bases: T_lb_rb = T_lb_lt @ inv(T_rb_lt).
            if a.cam_to_cam or a.right_calib or a.right_wrist_pose:
                raise ValueError("--top-in-right-base replaces --cam-to-cam / --right-calib / --right-wrist-pose")
            T_rb_lt = parse_pose(a.top_in_right_base, "top-in-right-base", a.npz_key)
            inputs["top_in_right_base"] = {"spec": str(a.top_in_right_base), "meaning": "pose of left_top_cam in right_base (cross rig)"}
            T = T_lb_lt @ inv_T(T_rb_lt)
        else:
            if not a.cam_to_cam:
                raise ValueError("--cam-to-cam POSE (or --top-in-right-base PATH) is required")
            T_lt_rw = parse_pose(a.cam_to_cam, "cam-to-cam", a.npz_key)
            if a.invert:
                T_lt_rw = inv_T(T_lt_rw)
            inputs["cam_to_cam"] = {"spec": str(a.cam_to_cam), "inverted": bool(a.invert),
                                    "meaning": "pose of right_wrist_cam in left_top_cam"}
            if a.right_wrist_pose:
                T_rb_rw = parse_pose(a.right_wrist_pose, "right-wrist-pose", a.npz_key)
                inputs["right_wrist"] = {"spec": str(a.right_wrist_pose), "frame": "right_base"}
            elif a.right_calib:
                T_rb_rw, info = camera_pose_from_calib(a.right_calib, a.right_cam, "right-calib")
                inputs["right_wrist"] = info
            else:
                raise ValueError("--right-calib PATH or --right-wrist-pose POSE is required")
            skew = capture_skew_s(inputs)
            if skew is not None:
                inputs["capture_skew_s"] = skew
            T = solve(T_lb_lt, T_lt_rw, T_rb_rw)
    except Exception as e:  # noqa: BLE001
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    if a.top_in_right_base:
        source = a.source or ("tools/solve_base_transform.py: T_lb_lt @ inv(T_rb_lt) from "
                              f"{os.path.basename(str(a.left_calib or a.left_top_pose or DEFAULT_LEFT_TOP_CALIB))} + "
                              f"{os.path.basename(str(a.top_in_right_base))} (cross rig)")
    else:
        source = a.source or ("tools/solve_base_transform.py: T_lb_lt @ T_lt_rw @ inv(T_rb_rw) from "
                              f"{os.path.basename(str(a.left_calib or a.left_top_pose or DEFAULT_LEFT_TOP_CALIB))} + "
                              f"{os.path.basename(str(a.cam_to_cam))} + "
                              f"{os.path.basename(str(a.right_calib or a.right_wrist_pose))}")
    res = result_dict(T, source, a.date, inputs)
    print(f"left top cam in left_base    : {fmt_T(T_lb_lt)}")
    if a.top_in_right_base:
        print(f"left top cam in right_base   : {fmt_T(parse_pose(a.top_in_right_base, 'top-in-right-base', a.npz_key))}")
    else:
        print(f"right wrist in left top cam  : {fmt_T(T_lt_rw)}")
        print(f"right wrist in right_base    : {fmt_T(T_rb_rw)}")
    lt, rw = inputs.get("left_top") or {}, inputs.get("right_wrist") or {}
    if lt.get("capture_time") or rw.get("capture_time"):
        print(f"captures: left top {lt.get('capture_time')} (seq {lt.get('sequence')})  right wrist "
              f"{rw.get('capture_time')} (seq {rw.get('sequence')})" + (f"  skew {skew:.3f} s" if skew is not None else ""))
    print(f"SOLVED right_base in left_base: {fmt_T(T)}")
    print(f"nominal                       : {fmt_T(T_nom)}")
    print(f"solved vs nominal: {json.dumps(res['delta_vs_nominal'])}")
    if existing is not None:
        print(f"solved vs file   : {json.dumps(delta(T, existing))}   ({a.out})")
    d = res["delta_vs_nominal"]
    if d["translation_mm"] > 100.0 or d["rotation_deg"] > 15.0:
        print("WARNING: the solved transform is far from the nominal station offset (> 100 mm or > 15 deg); "
              "check the arrow convention of --cam-to-cam (try --invert) and that both captures are simultaneous",
              file=sys.stderr)
    skew_bad = skew is not None and skew > float(a.max_time_skew_s)
    if skew_bad:
        print(f"{'WARNING' if (a.check or a.dry_run or a.allow_time_skew) else 'ERROR'}: the left top and right wrist "
              f"captures are {skew:.1f} s apart (max {a.max_time_skew_s} s): the right wrist pose must be from the SAME "
              "instant as the --cam-to-cam observation; if the older LEFT top frame is deliberate (fixed camera), "
              "re-run with --allow-time-skew", file=sys.stderr)
    if a.check or a.dry_run:
        print(json.dumps(res, indent=1))
        return 0
    if skew_bad and not a.allow_time_skew:
        return 4
    if (existing is not None or existing_unparsed) and not a.force:
        print(f"ERROR: {a.out} exists; re-run with --force to overwrite (or --dry-run / --check)", file=sys.stderr)
        return 3
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    tmp = a.out + ".tmp"
    with open(tmp, "w") as f:
        json.dump(res, f, indent=1)
        f.write("\n")
    os.replace(tmp, a.out)
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
