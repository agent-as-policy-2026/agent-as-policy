# tool: measure.py
# category: geometry
# purpose: Summarize base-frame point bounds for a selected ring or post region, with optional color and height filters.
# usage: python3 knowledge/tools/measure.py CALIB DEPTH --roi X1 Y1 X2 Y2 [--image RGB --rgb-min R G B --rgb-max R G B] [--z-range LOW HIGH]    (run from the session directory)
# inputs/outputs: Reads calibration JSON, depth NPY and optional RGB image; prints count, XYZ percentiles and extrema midpoint in metres.
# assumptions: numpy/scipy/Pillow and adjacent geometry.py; wrist RGB/depth registered; half-open pixel ROI; strict RGB/height bounds; midpoint is a visible-surface estimate, not a circle fit.
# verified: used successfully in the session that wrote it
import argparse
from geometry import calibration


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('calib')
    parser.add_argument('depth')
    parser.add_argument('--roi', nargs=4, type=int, required=True)
    parser.add_argument('--image')
    parser.add_argument('--rgb-min', nargs=3, type=int)
    parser.add_argument('--rgb-max', nargs=3, type=int)
    parser.add_argument('--z-range', nargs=2, type=float)
    args = parser.parse_args()
    if (args.rgb_min is not None or args.rgb_max is not None) and not args.image:
        parser.error('RGB filters require --image')
    import numpy as np
    K, R, t = calibration(args.calib, 'wrist')
    depth = np.load(args.depth, allow_pickle=False)
    if depth.ndim != 2:
        parser.error('depth must be a two-dimensional array')
    x1, y1, x2, y2 = args.roi
    height, width = depth.shape
    if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        parser.error('ROI must be nonempty and inside the depth image')
    v, u = np.indices(depth.shape)
    mask = np.isfinite(depth) & (depth > 0) & (u >= x1) & (u < x2) & (v >= y1) & (v < y2)
    if args.image:
        from PIL import Image
        with Image.open(args.image) as image:
            rgb = np.array(image.convert('RGB'))
        if rgb.shape[:2] != depth.shape:
            parser.error('RGB and depth dimensions differ')
        if args.rgb_min is not None:
            mask &= (rgb > np.array(args.rgb_min)).all(axis=2)
        if args.rgb_max is not None:
            mask &= (rgb < np.array(args.rgb_max)).all(axis=2)
    ys, xs = np.where(mask)
    pixels = np.stack([xs, ys, np.ones(xs.size)])
    points = (R @ (np.linalg.inv(K) @ pixels * depth[ys, xs])).T + t
    if args.z_range:
        low, high = args.z_range
        if low >= high:
            parser.error('height bounds must be increasing')
        points = points[(points[:, 2] > low) & (points[:, 2] < high)]
    if not len(points):
        parser.error('no valid points remain after filtering')
    print('count', len(points))
    print('XYZ percentiles 0,10,50,90,100', np.percentile(points, [0, 10, 50, 90, 100], axis=0).tolist())
    print('center extrema', ((points.min(axis=0) + points.max(axis=0))/2).tolist())


if __name__ == '__main__':
    main()
