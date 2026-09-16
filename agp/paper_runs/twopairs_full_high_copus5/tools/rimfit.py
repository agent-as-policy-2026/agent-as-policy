# tool: rimfit.py
# category: geometry
# purpose: fit a circular top rim (cylinder/shaft top) at a KNOWN height from the colour outline in a wrist capture, for texture-less tops where depth is missing
# usage: python3 knowledge/tools/rimfit.py N u v z_top "hmin,hmax,smin,smax,vmin,vmax" [r_expected_m]    (run from the session directory; needs numpy, opencv, pillow)
# inputs/outputs: frames/NNNN_wrist.png + _calib.json; (u,v) = pixel inside the top face blob; prints rim centre (base x,y), radius, RANSAC inlier count; writes scratch/rimfit_NNNN.png overlay (green = inliers)
# assumptions: HSV range should select ONLY the top face (e.g. use a high V floor so the darker side wall is excluded); z_top from an earlier depth measurement; RANSAC inlier tolerance 0.6 mm; optional radius constraint +-1.5 mm
# verified: used successfully in the session that wrote it
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geo import *
from PIL import Image
import cv2
n = int(sys.argv[1]); u0, v0 = int(sys.argv[2]), int(sys.argv[3]); z0 = float(sys.argv[4]); spec = sys.argv[5]
rexp = float(sys.argv[6]) if len(sys.argv) > 6 else None
img = np.array(Image.open(f'{SESS}/frames/{n:04d}_wrist.png').convert('RGB'))
m = cv2.morphologyEx(hsv_mask(img, spec).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
nl, lab, st, _ = cv2.connectedComponentsWithStats(m)
if lab[v0, u0] == 0: sys.exit('seed pixel not in mask; adjust HSV range or seed')
m = (lab == lab[v0, u0]).astype(np.uint8)
cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
c = max(cs, key=cv2.contourArea)[:, 0, :]
pts = np.array([pix2plane(n, 'wrist', a, b, z0)[:2] for a, b in c])
rng = np.random.default_rng(0); best = None
for _ in range(3000):
    s = pts[rng.choice(len(pts), 3, replace=False)]
    A = np.c_[2*s, np.ones(3)]; b = (s**2).sum(1)
    try: cx, cy, k = np.linalg.solve(A, b)
    except np.linalg.LinAlgError: continue
    r = np.sqrt(max(k + cx*cx + cy*cy, 0))
    if rexp and abs(r - rexp) > 0.0015: continue
    inl = np.abs(np.hypot(pts[:,0]-cx, pts[:,1]-cy) - r) < 0.0006
    if best is None or inl.sum() > best[0]: best = (inl.sum(), inl)
inl = best[1]; s = pts[inl]
A = np.c_[2*s, np.ones(len(s))]; b = (s**2).sum(1)
cx, cy, k = np.linalg.lstsq(A, b, rcond=None)[0]; r = np.sqrt(k + cx*cx + cy*cy)
print(f"rim center=({cx:.4f},{cy:.4f}) r={r*1000:.1f}mm inliers={inl.sum()}/{len(pts)} (z={z0})")
ov = img.copy()
for (a, bb), f in zip(c, inl): ov[bb, a] = (0, 255, 0) if f else (255, 0, 0)
pc = base2pix(n, 'wrist', [cx, cy, z0]); cv2.circle(ov, (int(pc[0]), int(pc[1])), 2, (255, 0, 255), -1)
os.makedirs(f'{SESS}/scratch', exist_ok=True); Image.fromarray(ov).save(f'{SESS}/scratch/rimfit_{n:04d}.png')
