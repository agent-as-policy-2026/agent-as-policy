# tool: zslice.py
# category: geometry
# purpose: locate objects by height: cluster wrist-depth points (base frame) inside a z band and print each cluster's centroid, xy extent and max z (e.g. top faces of standing parts, rims, a held object's top)
# usage: <venv-python from README_interface.md> knowledge/tools/zslice.py <capture_N> <zmin> <zmax> [min_pixels=80]    (run from the session directory; needs numpy+scipy, so use the venv interpreter, not bare python3)
# inputs/outputs: reads frames/NNNN_wrist_depth.npy and frames/NNNN_calib.json; prints one line per connected cluster: pixel count, pixel centroid, mean xyz, bbox-mid xy, x/y ranges, max z
# assumptions: pinhole deprojection ignoring the small wrist distortion; depth 0 = invalid, pixels < 0.05 m ignored; centroids are biased toward the camera-facing side of an object when the band also catches side walls (oblique wrist camera); D405 depth is unreliable closer than ~0.07 m
# verified: used successfully in the session that wrote it
import sys, json, numpy as np
from scipy import ndimage

N = int(sys.argv[1]); zmin, zmax = float(sys.argv[2]), float(sys.argv[3])
minpts = int(sys.argv[4]) if len(sys.argv) > 4 else 80
pre = f"frames/{N:04d}"
dep = np.load(pre + "_wrist_depth.npy")
cal = json.load(open(pre + "_calib.json"))["wrist"]
K = np.array(cal["intrinsics"]); p = cal["pose"]
t = np.array([p["position"][k] for k in "xyz"]); q = p["rotation"]
w, x, y, z = q["w"], q["x"], q["y"], q["z"]
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
H, W = dep.shape
uu, vv = np.meshgrid(np.arange(W), np.arange(H))
P = np.stack([(uu - K[0, 2]) / K[0, 0] * dep, (vv - K[1, 2]) / K[1, 1] * dep, dep], -1) @ R.T + t
m = (dep > 0.05) & (P[..., 2] > zmin) & (P[..., 2] < zmax)
m = ndimage.binary_opening(m, iterations=1)
lab, n = ndimage.label(m)
for i in range(1, n + 1):
    sel = lab == i
    if sel.sum() < minpts: continue
    pts = P[sel]; vs, us = np.nonzero(sel)
    lo, hi = pts.min(0), pts.max(0)
    print(f"#{i} n={sel.sum()} px=({us.mean():.0f},{vs.mean():.0f}) mean=({pts[:,0].mean():.4f},{pts[:,1].mean():.4f},{pts[:,2].mean():.4f}) "
          f"bbox_mid=({(lo[0]+hi[0])/2:.4f},{(lo[1]+hi[1])/2:.4f}) x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] zmax={hi[2]:.3f}")
