#!/usr/bin/env python3
"""Overhead sanity check of a scattered-cube scene (used between automatic trials).

    python check_scatter.py --image top.png [--calib config/top_calib.json] [--debug out.png]
                            [--expect-cyan 3 --expect-grey 3] [--min-dist 0.07]
                            [--region 0.15 0.48 -0.30 0.20]     # x_min x_max y_min y_max (base frame)

Exit code 0 = scene accepted (right number of cyan and grey cubes, all inside the
region, pairwise centre distance >= min-dist), 1 = rejected, 2 = usage/IO error.
Prints a JSON summary; --debug writes an annotated image.

Method: HSV colour masks (cyan tape faces; white/grey paper faces) -> morphological
closing so the black tape crosses do not split a face -> connected components ->
component centroid deprojected onto the cube-top plane (z = table + cube edge)
with the fixed overhead calibration -> metric size and position filters.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

TABLE_Z = -0.045
CUBE = 0.050


def quat_to_R(q):
    w, x, y, z = q["w"], q["x"], q["y"], q["z"]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class TopCam:
    def __init__(self, calib):
        self.K = np.array(calib["intrinsics"], dtype=float).reshape(3, 3)
        self.Kinv = np.linalg.inv(self.K)
        p = calib["pose"]
        self.t = np.array([p["position"][k] for k in "xyz"], dtype=float)
        self.R = quat_to_R(p["rotation"])

    def deproject(self, u, v, plane_z):
        r = self.R @ self.Kinv @ np.array([u, v, 1.0])
        s = (plane_z - self.t[2]) / r[2]
        return self.t + s * r


def find_blobs(mask, min_area_px):
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = []
    for i in range(1, n):
        a = int(stats[i, cv2.CC_STAT_AREA])
        if a < min_area_px:
            continue
        x, y, w, h = (int(stats[i, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
        # rotation-invariant extent: minimum-area rectangle of the component's outline
        # (an axis-aligned box over-estimates a cube standing at 45 deg by ~40 %)
        sub = (labels[y:y + h, x:x + w] == i).astype(np.uint8)
        cnts, _ = cv2.findContours(sub, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            (_, _), (rw, rh), _ = cv2.minAreaRect(max(cnts, key=cv2.contourArea))
            long_px, short_px = float(max(rw, rh)), float(min(rw, rh))
        else:
            long_px, short_px = float(max(w, h)), float(min(w, h))
        out.append({"area_px": a, "cx": float(cents[i][0]), "cy": float(cents[i][1]), "box": [x, y, w, h],
                    "long_px": long_px, "short_px": short_px})
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--calib", default=str(Path(__file__).resolve().parents[1] / "config" / "top_calib.json"))
    ap.add_argument("--debug")
    ap.add_argument("--expect-cyan", type=int, default=3)
    ap.add_argument("--expect-grey", type=int, default=3)
    ap.add_argument("--min-dist", type=float, default=0.07)
    # wider than the sampler's placement region: human-scattered scenes reach further out
    ap.add_argument("--region", type=float, nargs=4, default=[0.12, 0.50, -0.40, 0.20], metavar=("XMIN", "XMAX", "YMIN", "YMAX"))
    ap.add_argument("--size-range", type=float, nargs=2, default=[0.030, 0.085], help="accepted blob side (m) on the top plane")
    a = ap.parse_args()

    img = cv2.imread(a.image)
    if img is None:
        print(json.dumps({"ok": False, "error": f"cannot read {a.image}"})); return 2
    cam = TopCam(json.load(open(a.calib)))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    cyan = ((H >= 78) & (H <= 105) & (S >= 110) & (V >= 70)).astype(np.uint8) * 255
    grey = ((S <= 55) & (V >= 165)).astype(np.uint8) * 255
    k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (33, 33))
    k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    cubes = []
    plane_z = TABLE_Z + CUBE
    for colour, mask in (("cyan", cyan), ("grey", grey)):
        m = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k_close)
        for b in find_blobs(m, 1500):
            X = cam.deproject(b["cx"], b["cy"], plane_z)
            # metres per pixel on the top plane at the blob centre, then the long side of the
            # min-area rectangle: ~5-7 cm for one cube (side faces add a little), >=10 cm for
            # two cubes touching, whatever the cube's yaw
            m_per_px = float(np.linalg.norm((cam.deproject(b["cx"] + 1.0, b["cy"], plane_z) - X)[:2]))
            side = b["long_px"] * m_per_px
            b.update({"colour": colour, "x": round(float(X[0]), 4), "y": round(float(X[1]), 4), "side_m": round(side, 4),
                      "short_m": round(b["short_px"] * m_per_px, 4)})
            cubes.append(b)
    xmin, xmax, ymin, ymax = a.region
    kept, dropped, problems = [], [], []
    for b in cubes:
        inside = xmin <= b["x"] <= xmax and ymin <= b["y"] <= ymax
        if inside and a.size_range[1] < b["side_m"] <= 0.17:
            # one coloured blob the size of two or three cubes = cubes touching each other
            b["why"] = "merged/touching"; dropped.append(b)
            problems.append(f"{b['colour']} cubes touching near ({b['x']:.2f},{b['y']:.2f})"); continue
        if not (a.size_range[0] <= b["side_m"] <= a.size_range[1]):
            b["why"] = "size"; dropped.append(b); continue
        if not inside:
            b["why"] = "outside region"; dropped.append(b); continue
        kept.append(b)
    n_c = sum(1 for b in kept if b["colour"] == "cyan"); n_g = sum(1 for b in kept if b["colour"] == "grey")
    if n_c != a.expect_cyan: problems.append(f"cyan cubes: {n_c} (expected {a.expect_cyan})")
    if n_g != a.expect_grey: problems.append(f"grey cubes: {n_g} (expected {a.expect_grey})")
    dmin = None
    for i in range(len(kept)):
        for j in range(i + 1, len(kept)):
            d = float(np.hypot(kept[i]["x"] - kept[j]["x"], kept[i]["y"] - kept[j]["y"]))
            dmin = d if dmin is None else min(dmin, d)
    if dmin is not None and dmin < a.min_dist: problems.append(f"min centre distance {dmin:.3f} m < {a.min_dist} m")
    ok = not problems
    if a.debug:
        dbg = img.copy()
        for b in kept + dropped:
            col = (0, 255, 0) if "why" not in b else (0, 0, 255)
            x0, y0, w, h = b["box"]; cv2.rectangle(dbg, (x0, y0), (x0 + w, y0 + h), col, 2)
            cv2.putText(dbg, f"{b['colour']} ({b['x']:.2f},{b['y']:.2f}) {b['side_m']*100:.0f}cm" + (f" {b['why']}" if "why" in b else ""),
                        (x0, max(15, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        cv2.putText(dbg, ("OK" if ok else "REJECT: " + "; ".join(problems)), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0) if ok else (0, 0, 255), 2)
        cv2.imwrite(a.debug, dbg)
    print(json.dumps({"ok": ok, "problems": problems, "cyan": n_c, "grey": n_g, "min_dist_m": None if dmin is None else round(dmin, 3),
                      "cubes": [{k: b[k] for k in ("colour", "x", "y", "side_m")} for b in kept],
                      "dropped": [{k: b[k] for k in ("colour", "x", "y", "side_m", "why")} for b in dropped]}))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
