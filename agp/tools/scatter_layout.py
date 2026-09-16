#!/usr/bin/env python3
"""Sample a random, well-separated target layout for six cubes (scene reset between trials).

    python3 scatter_layout.py --out DIR [--seed N] [--region 0.22 0.40 -0.22 0.12]
                              [--min-dist 0.10] [--yaw-max 45]

Writes DIR/target_layout.json and DIR/target_layout.md (the table the reset agent reads).
Coordinates are robot-base metres (cube centre on the table); yaw in degrees about vertical.
Colours: 3 x light-blue/black, 3 x grey/black, assigned to positions at random.
"""
from __future__ import annotations

import argparse
import json
import os
import random
import sys


def sample(rng, region, n, min_dist, tries=20000):
    xmin, xmax, ymin, ymax = region
    pts = []
    for _ in range(tries):
        p = (rng.uniform(xmin, xmax), rng.uniform(ymin, ymax))
        if all(((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2) ** 0.5 >= min_dist for q in pts):
            pts.append(p)
            if len(pts) == n:
                return pts
    raise SystemExit(f"could not place {n} points with min distance {min_dist} in {region}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--region", type=float, nargs=4, default=[0.22, 0.40, -0.22, 0.12], metavar=("XMIN", "XMAX", "YMIN", "YMAX"))
    ap.add_argument("--min-dist", type=float, default=0.10)
    ap.add_argument("--yaw-max", type=float, default=45.0)
    a = ap.parse_args()
    seed = a.seed if a.seed is not None else random.SystemRandom().randrange(1 << 31)
    rng = random.Random(seed)
    pts = sample(rng, a.region, 6, a.min_dist)
    colours = ["light-blue/black"] * 3 + ["grey/black"] * 3
    rng.shuffle(colours)
    cubes = []
    for i, ((x, y), c) in enumerate(zip(pts, colours), 1):
        cubes.append({"id": i, "colour": c, "x": round(x, 3), "y": round(y, 3), "yaw_deg": round(rng.uniform(-a.yaw_max, a.yaw_max))})
    os.makedirs(a.out, exist_ok=True)
    meta = {"seed": seed, "region_xmin_xmax_ymin_ymax": a.region, "min_dist_m": a.min_dist, "yaw_max_deg": a.yaw_max, "cubes": cubes}
    json.dump(meta, open(os.path.join(a.out, "target_layout.json"), "w"), indent=1)
    lines = ["# Target layout for the six cubes", "",
             "Robot base frame, metres; cube centre resting on the table; yaw = rotation of the",
             "cube's faces about vertical (0 = faces parallel to the base x/y axes). Any cube of the",
             "right colour may go to any target of that colour. Tolerance: ±20 mm, ±15°.", "",
             "| target | colour | x (m) | y (m) | yaw (deg) |", "|---|---|---|---|---|"]
    for c in cubes:
        lines.append(f"| {c['id']} | {c['colour']} | {c['x']:.3f} | {c['y']:.3f} | {c['yaw_deg']:+d} |")
    lines += ["", f"(region x {a.region[0]}–{a.region[1]} m, y {a.region[2]}–{a.region[3]} m; neighbours at least {a.min_dist*100:.0f} cm apart centre-to-centre)"]
    open(os.path.join(a.out, "target_layout.md"), "w").write("\n".join(lines) + "\n")
    print(json.dumps(meta))
    return 0


if __name__ == "__main__":
    sys.exit(main())
