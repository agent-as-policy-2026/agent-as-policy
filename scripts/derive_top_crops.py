#!/usr/bin/env python3
"""Derive one top-camera crop rectangle per (task, arms), from where the trial actually worked.

Two point sets, unioned, because neither alone is right:
  - every pixel the agent itself pointed at: the `deproject` bridge requests with cam == "top";
  - every place the gripper actually was: each capture's ee_pose projected through that session's
    top-camera calibration. The gripper sweep is typically ~3x the area of the clicks, so clicks
    alone would cut the arm out of frame; but agents sometimes point at an object they never reach,
    so the sweep alone would cut away something the trial reasoned about.

The rectangle is the p0.2/p99.8 envelope of the union plus a margin, snapped to 16 px (macroblock and
4:2:0 chroma alignment) and clamped to the frame. Dual-arm trials are not cropped: both arms use the
whole table, so their envelope is ~90% of the frame and a crop would only cost a re-encode.

Writes <out>/crops/crops-<batch>.csv, one row per released session.
"""
import argparse, csv, glob, json, os, tarfile

import numpy as np
import pyarrow.compute as pc
import pyarrow.parquet as pq

from _paths import add_fa_arg, require_sessions


def quat_to_R(w, x, y, z):
    n = (w * w + x * x + y * y + z * z) ** 0.5
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def session_calib(sid, fa, tar_index):
    """The top block of any capture's calibration for this session."""
    for p in sorted(glob.glob(os.path.join(fa, "sessions", sid, "frames", "*_calib.json")))[:1]:
        return json.load(open(p)).get("top")
    m = tar_index.get(sid)
    if m:
        tf, name = m
        return json.load(tf.extractfile(name)).get("top")
    return None


def project(top, pts):
    """Base-frame points -> pixels, through the camera's own pose and intrinsics."""
    K = np.array(top["intrinsics"], dtype=float)
    t = np.array([top["pose"]["position"][k] for k in "xyz"], dtype=float)
    r = top["pose"]["rotation"]
    R = quat_to_R(r["w"], r["x"], r["y"], r["z"])
    Xc = (R.T @ (np.asarray(pts, dtype=float) - t).T).T
    good = Xc[:, 2] > 1e-6
    uv = (K @ (Xc[good] / Xc[good, 2:3]).T).T[:, :2]
    return uv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--batch", default="b01")
    ap.add_argument("--sessions", required=True, help="release list")
    ap.add_argument("--out", required=True)
    ap.add_argument("--margin", type=int, default=200)
    ap.add_argument("--snap", type=int, default=16)
    ap.add_argument("--trim", type=float, default=0.2, help="percentile trimmed from each end")
    ap.add_argument("--no-crop-arms", default="dual", help="comma-separated arms values left uncropped")
    ap.add_argument("--no-crop-tasks", default="", help="comma-separated tasks left uncropped (a whole-table task)")
    ap.add_argument("--reuse", default=None,
                    help="an earlier crops-<batch>.csv: a (task, arms) already fixed there keeps that rectangle, "
                         "so a later batch of the same experiment frames identically")
    add_fa_arg(ap)
    a = ap.parse_args()
    D = a.dataset
    keep = [l.strip() for l in open(a.sessions) if l.strip() and not l.startswith("#")]
    keep_set = set(keep)
    require_sessions(a.fa, keep, "--sessions")

    meta = {}
    for sp in ("evaluated", "reset", "auxiliary"):
        p = os.path.join(D, "metadata", f"trials_{sp}-{a.batch}.parquet")
        if os.path.exists(p):
            for r in pq.read_table(p, columns=["session_id", "task", "arms"]).to_pylist():
                if r["session_id"] in keep_set:
                    meta[r["session_id"]] = (r["task"] or "", r["arms"] or "")

    # the agent's own top clicks
    cmd = pq.read_table(os.path.join(D, "metadata", f"commands-{a.batch}.parquet"), columns=["session_id", "cmd", "args_json"])
    cmd = cmd.filter(pc.equal(cmd["cmd"], "deproject"))
    clicks = {}
    for r in cmd.to_pylist():
        if r["session_id"] not in keep_set:
            continue
        try:
            d = json.loads(r["args_json"] or "{}")
        except json.JSONDecodeError:
            continue
        if d.get("cam") == "top" and isinstance(d.get("u"), int) and isinstance(d.get("v"), int):
            clicks.setdefault(r["session_id"], []).append((d["u"], d["v"]))

    # the gripper sweep
    cap = pq.read_table(os.path.join(D, "metadata", f"captures-{a.batch}.parquet"), columns=["session_id", "meta_json"])
    poses = {}
    for r in cap.to_pylist():
        if r["session_id"] not in keep_set:
            continue
        try:
            m = json.loads(r["meta_json"] or "{}")
        except json.JSONDecodeError:
            continue
        p = (m.get("ee_pose") or {}).get("position")
        if p:
            poses.setdefault(r["session_id"], []).append((p["x"], p["y"], p["z"]))

    tar_index = {}
    calib, serial, size = {}, {}, {}
    for sid in keep:
        top = session_calib(sid, a.fa, tar_index)
        if top:
            calib[sid] = top
            serial[sid] = top.get("serial", "")
            size[sid] = tuple(top.get("image_size") or (1920, 1080))

    # pool the points per (task, arms)
    pool = {}
    per_session = {}
    for sid in keep:
        task, arms = meta.get(sid, ("", ""))
        pts = [(float(u), float(v)) for u, v in clicks.get(sid, [])]
        n_click = len(pts)
        if sid in calib and poses.get(sid):
            uv = project(calib[sid], poses[sid])
            W, H = size.get(sid, (1920, 1080))
            uv = uv[(uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)]
            pts += [tuple(x) for x in uv]
        per_session[sid] = (n_click, len(pts) - n_click)
        if pts:
            pool.setdefault((task, arms), []).extend(pts)

    no_crop = set(x for x in a.no_crop_arms.split(",") if x)
    no_crop_tasks = set(x for x in a.no_crop_tasks.split(",") if x)
    reuse = {}
    if a.reuse and os.path.exists(a.reuse):
        for r in csv.DictReader(open(a.reuse, newline="")):
            if r["crop_w"]:
                reuse[(r["task"], r["arms"])] = (int(r["crop_x"]), int(r["crop_y"]), int(r["crop_w"]), int(r["crop_h"]))
        print(f"  复用 {a.reuse}：{len(reuse)} 个已定矩形", flush=True)
    rects, report, basis_of = {}, [], {}
    for (task, arms), pts in sorted(pool.items()):
        W, H = 1920, 1080
        p = np.asarray(pts)
        lo_u, hi_u = np.percentile(p[:, 0], [a.trim, 100 - a.trim])
        lo_v, hi_v = np.percentile(p[:, 1], [a.trim, 100 - a.trim])
        x0 = max(0, int(np.floor((lo_u - a.margin) / a.snap)) * a.snap)
        y0 = max(0, int(np.floor((lo_v - a.margin) / a.snap)) * a.snap)
        x1 = min(W, int(np.ceil((hi_u + a.margin) / a.snap)) * a.snap)
        y1 = min(H, int(np.ceil((hi_v + a.margin) / a.snap)) * a.snap)
        rect = reuse.get((task, arms)) or (x0, y0, x1 - x0, y1 - y0)
        from_reuse = (task, arms) in reuse
        out = int(((p[:, 0] < x0) | (p[:, 0] >= x1) | (p[:, 1] < y0) | (p[:, 1] >= y1)).sum())
        cropped = arms not in no_crop and task not in no_crop_tasks
        rects[(task, arms)] = rect if cropped else None
        basis_of[(task, arms)] = ("reused" if from_reuse else "task_arm_union") if cropped else (
            "not_cropped_whole_table" if task in no_crop_tasks else "not_cropped_dual")
        report.append((task, arms, len(pts), rect, 100.0 * rect[2] * rect[3] / (W * H), out, cropped, from_reuse))

    print(f"{'task':<14} {'arms':<5} {'points':>7}  {'rect (x,y,w,h)':<24} {'area%':>6} {'outside':>7}  裁剪")
    for task, arms, n, rect, area, out, cropped, from_reuse in report:
        why = "是（沿用 b01 矩形）" if cropped and from_reuse else ("是" if cropped else
              ("否（整桌任务）" if task in no_crop_tasks else "否（双臂用满整桌）"))
        print(f"{task:<14} {arms:<5} {n:>7}  {str(rect):<24} {area:>6.1f} {out:>7}  {why}")

    os.makedirs(os.path.join(a.out, "crops"), exist_ok=True)
    path = os.path.join(a.out, "crops", f"crops-{a.batch}.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["session_id", "task", "arms", "camera_serial", "source_width", "source_height",
                    "crop_x", "crop_y", "crop_w", "crop_h", "basis", "margin_px", "snap_px",
                    "n_clicks", "n_pose_points"])
        for sid in keep:
            task, arms = meta.get(sid, ("", ""))
            rect = rects.get((task, arms))
            W, H = size.get(sid, (1920, 1080))
            nc, npp = per_session.get(sid, (0, 0))
            if rect is None:
                w.writerow([sid, task, arms, serial.get(sid, ""), W, H, "", "", "", "",
                            basis_of.get((task, arms), "no_points"), a.margin, a.snap, nc, npp])
            else:
                w.writerow([sid, task, arms, serial.get(sid, ""), W, H, *rect,
                            basis_of.get((task, arms), "task_arm_union"), a.margin, a.snap, nc, npp])
    n_cropped = sum(1 for sid in keep if rects.get(meta.get(sid, ("", ""))) is not None)
    print(f"\n写出 {path}: {len(keep)} 个会话，其中 {n_cropped} 个有裁剪矩形")


if __name__ == "__main__":
    main()
