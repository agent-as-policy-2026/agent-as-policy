# tool: geo.py
# category: geometry
# purpose: camera geometry helpers for frames/NNNN_calib.json: pixel->plane, wrist pixel->3D via saved depth, base point->pixel
# usage: python3 knowledge/tools/geo.py N wrist|top u v [plane_z]   |   python3 knowledge/tools/geo.py N wrist|top pix x y z    (run from the session directory; needs numpy)
# inputs/outputs: reads frames/NNNN_calib.json and frames/NNNN_wrist_depth.npy; prints a base-frame point [x y z] or a pixel [u v]; importable (cam, qmat, pix2plane, pix2depth, base2pix, calib)
# assumptions: current working directory is the session directory; wrist pixels treated as undistorted pinhole (distortion ignored, ~1px effect); depth uses a 5x5 median of valid pixels
# verified: used successfully in the session that wrote it
"""Camera geometry helpers using frames/NNNN_calib.json (robot base frame)."""
import json, os, sys, numpy as np
SESS = os.getcwd()

def qmat(q):
    w, x, y, z = q['w'], q['x'], q['y'], q['z']
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

def calib(n):
    return json.load(open(f'{SESS}/frames/{n:04d}_calib.json'))

def cam(n, c):
    d = calib(n)[c]
    K = np.array(d['intrinsics']); p = d['pose']
    t = np.array([p['position'][k] for k in 'xyz']); R = qmat(p['rotation'])
    return K, R, t

def pix2plane(n, c, u, v, z):
    K, R, t = cam(n, c)
    ray = R @ np.linalg.solve(K, [u, v, 1.0])
    s = (z - t[2]) / ray[2]
    return t + s * ray

def pix2depth(n, u, v, win=2):
    K, R, t = cam(n, 'wrist')
    D = np.load(f'{SESS}/frames/{n:04d}_wrist_depth.npy')
    u, v = int(round(u)), int(round(v))
    patch = D[max(0, v-win):v+win+1, max(0, u-win):u+win+1]
    patch = patch[patch > 0]
    if patch.size == 0: return None
    d = np.median(patch)
    Xc = np.linalg.solve(K, [u, v, 1.0]) * d
    return t + R @ Xc

def base2pix(n, c, X):
    K, R, t = cam(n, c)
    Xc = R.T @ (np.asarray(X, float) - t)
    p = K @ (Xc / Xc[2])
    return p[:2]

def wrist_points(n):
    """Per-pixel base-frame 3D points (H,W,3) and depth (H,W) of a wrist capture."""
    K, R, t = cam(n, 'wrist')
    D = np.load(f'{SESS}/frames/{n:04d}_wrist_depth.npy')
    H, W = D.shape; vv, uu = np.mgrid[0:H, 0:W]
    rays = np.stack([(uu-K[0,2])/K[0,0], (vv-K[1,2])/K[1,1], np.ones_like(D)], -1)
    return t + (rays * D[..., None]) @ R.T, D

def hsv_mask(img_rgb, spec):
    """spec 'hmin,hmax,smin,smax,vmin,vmax' (OpenCV HSV: H 0-179)."""
    import cv2
    h0, h1, s0, s1, v0, v1 = [int(a) for a in spec.split(',')]
    hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    return ((hsv[...,0] >= h0) & (hsv[...,0] <= h1) & (hsv[...,1] >= s0) & (hsv[...,1] <= s1)
            & (hsv[...,2] >= v0) & (hsv[...,2] <= v1))

if __name__ == '__main__':
    n, c = int(sys.argv[1]), sys.argv[2]
    if sys.argv[3] == 'pix':
        print(base2pix(n, c, [float(a) for a in sys.argv[4:7]]))
    elif len(sys.argv) > 5:
        print(pix2plane(n, c, float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])))
    else:
        print(pix2depth(n, float(sys.argv[3]), float(sys.argv[4])))
