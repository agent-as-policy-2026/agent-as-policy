# tool: geometry.py
# category: geometry
# purpose: Convert wrist depth samples or camera rays at a supplied height to base-frame points.
# usage: python3 knowledge/tools/geometry.py CALIB {points DEPTH --uv U V [--uv U V] | plane --uv U V --z Z}    (run from the session directory)
# inputs/outputs: Reads calibration JSON and optionally depth NPY; prints pixel, depth or plane height, and base XYZ in metres.
# assumptions: numpy/scipy; calibration camera pose and 3x3 intrinsics; scalar-first quaternion keys; depth in metres; points use a positive-depth 5x5 median by default.
# verified: used successfully in the session that wrote it
import argparse
import json


def calibration(path, camera):
    import numpy as np
    from scipy.spatial.transform import Rotation
    with open(path) as source:
        c = json.load(source)[camera]
    pose = c['pose']
    rotation = Rotation.from_quat([pose['rotation'][k] for k in ('x', 'y', 'z', 'w')]).as_matrix()
    return np.asarray(c['intrinsics']), rotation, np.array([pose['position'][k] for k in ('x', 'y', 'z')])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('calib')
    commands = parser.add_subparsers(dest='command', required=True)
    points = commands.add_parser('points', help='Sample wrist depth around supplied pixels.')
    points.add_argument('depth')
    points.add_argument('--uv', nargs=2, type=int, action='append', required=True)
    points.add_argument('--radius', type=int, default=2)
    plane = commands.add_parser('plane', help='Intersect camera rays with base-frame z planes.')
    plane.add_argument('--camera', default='top')
    plane.add_argument('--uv', nargs=2, type=float, action='append', required=True)
    plane.add_argument('--z', type=float, action='append', required=True)
    args = parser.parse_args()
    import numpy as np
    K, R, t = calibration(args.calib, 'wrist' if args.command == 'points' else args.camera)
    inverse = np.linalg.inv(K)
    if args.command == 'points':
        if args.radius < 0:
            parser.error('--radius must be nonnegative')
        depth = np.load(args.depth, allow_pickle=False)
        if depth.ndim != 2:
            parser.error('depth must be a two-dimensional array')
        height, width = depth.shape
        for u, v in args.uv:
            if not (0 <= u < width and 0 <= v < height):
                parser.error('pixel outside depth image')
            r = args.radius
            patch = depth[max(0, v-r):min(height, v+r+1), max(0, u-r):min(width, u+r+1)]
            valid = patch[np.isfinite(patch) & (patch > 0)]
            if not valid.size:
                print(u, v, 'no valid depth')
                continue
            z = float(np.median(valid))
            print(u, v, z, (R @ (inverse @ np.array([u, v, 1]) * z) + t).tolist())
    else:
        for u, v in args.uv:
            ray = R @ inverse @ np.array([u, v, 1])
            if abs(ray[2]) < 1e-12:
                parser.error('ray parallel to height plane')
            for z in args.z:
                distance = (z-t[2])/ray[2]
                if distance <= 0:
                    parser.error('height plane intersects behind the camera')
                print(u, v, z, (t + ray * distance).tolist())


if __name__ == '__main__':
    main()
