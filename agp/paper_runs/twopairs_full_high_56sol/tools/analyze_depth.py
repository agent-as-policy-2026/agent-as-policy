#!/usr/bin/env python3
# tool: analyze_depth.py
# category: geometry
# purpose: Summarize base-frame XYZ extents of wrist RGB-D points inside an image ROI and height band.
# usage: python3 knowledge/tools/analyze_depth.py <depth.npy> <calib.json> --roi X0 Y0 X1 Y1 --z ZMIN ZMAX
# inputs/outputs: Reads a wrist depth NPY and matching calibration JSON; prints point count, axis percentiles, extrema, and XY midpoint.
# assumptions: Depth is aligned to the wrist intrinsics; calibration uses x/y/z position, w/x/y/z quaternion, and OpenCV-compatible distortion coefficients.
# verified: used successfully in the session that wrote it
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("depth")
    parser.add_argument("calib")
    parser.add_argument(
        "--roi",
        nargs=4,
        type=int,
        required=True,
        metavar=("X0", "Y0", "X1", "Y1"),
    )
    parser.add_argument(
        "--z",
        nargs=2,
        type=float,
        required=True,
        metavar=("ZMIN", "ZMAX"),
    )
    args = parser.parse_args()

    depth = np.load(args.depth)
    calib = json.loads(Path(args.calib).read_text())["wrist"]
    intrinsics = np.asarray(calib["intrinsics"], dtype=np.float64)
    distortion = np.asarray(calib["distortion_coefficients"], dtype=np.float64)

    x0, y0, x1, y1 = args.roi
    rows, cols = np.mgrid[y0:y1, x0:x1]
    roi_depth = depth[y0:y1, x0:x1]
    valid = roi_depth > 0
    pixels = np.column_stack((cols[valid], rows[valid])).astype(np.float64).reshape(-1, 1, 2)
    rays = cv2.undistortPoints(pixels, intrinsics, distortion).reshape(-1, 2)
    points_camera = np.column_stack(
        (rays[:, 0] * roi_depth[valid], rays[:, 1] * roi_depth[valid], roi_depth[valid])
    )

    quat = calib["pose"]["rotation"]
    rotation = Rotation.from_quat(
        [quat["x"], quat["y"], quat["z"], quat["w"]]
    ).as_matrix()
    pos = calib["pose"]["position"]
    translation = np.array([pos["x"], pos["y"], pos["z"]])
    points_base = points_camera @ rotation.T + translation

    z_min, z_max = args.z
    points = points_base[
        (points_base[:, 2] >= z_min) & (points_base[:, 2] <= z_max)
    ]
    if not len(points):
        raise SystemExit("No valid points in the requested ROI and z band")

    print(f"points={len(points)}")
    for axis, name in enumerate("xyz"):
        print(
            name,
            "min",
            points[:, axis].min(),
            "p05",
            np.percentile(points[:, axis], 5),
            "median",
            np.median(points[:, axis]),
            "p95",
            np.percentile(points[:, axis], 95),
            "max",
            points[:, axis].max(),
        )
    midpoint = (points[:, :2].min(axis=0) + points[:, :2].max(axis=0)) / 2
    print("xy_mid_extrema", midpoint.tolist())


if __name__ == "__main__":
    main()
