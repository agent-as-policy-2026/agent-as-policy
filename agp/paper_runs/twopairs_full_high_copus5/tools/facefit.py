# tool: facefit.py
# category: geometry
# purpose: locate an object's flat top face in a wrist capture (colour mask AND depth within a band of the seed's height) and report its centre, height, outline radius, polygon edge angle and inner-hole circle
# usage: python3 knowledge/tools/facefit.py N u v "hmin,hmax,smin,smax,vmin,vmax" [band_m=0.003]    (run from the session directory; needs numpy, opencv, pillow)
# inputs/outputs: frames/NNNN_wrist.png, _wrist_depth.npy, _calib.json; (u,v) = seed pixel ON the top face (not in a hole); prints z_top, region centroid, outer-contour centroid and radii, dominant edge direction mod 60 deg (hex flats) in base frame, hole centre/radius; writes scratch/facefit_NNNN.png overlay
# assumptions: the top face has stereo texture (valid depth); OpenCV HSV ranges (H 0-179); edge angle is measured from base +x; outline points are deprojected to the plane z_top
# verified: used successfully in the session that wrote it
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geo import *
from PIL import Image
import cv2
n = int(sys.argv[1]); u0, v0 = int(sys.argv[2]), int(sys.argv[3]); spec = sys.argv[4]
band = float(sys.argv[5]) if len(sys.argv) > 5 else 0.003
img = np.array(Image.open(f'{SESS}/frames/{n:04d}_wrist.png').convert('RGB'))
cm = hsv_mask(img, spec)
P, D = wrist_points(n)
seed = P[v0-2:v0+3, u0-2:u0+3, 2][D[v0-2:v0+3, u0-2:u0+3] > 0]
if seed.size == 0: sys.exit('no depth at seed pixel; pick another seed or use rimfit.py')
z0 = np.median(seed)
m = (cm & (D > 0) & (np.abs(P[..., 2] - z0) < band)).astype(np.uint8)
m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
nl, lab, st, _ = cv2.connectedComponentsWithStats(m)
if nl < 2: sys.exit('empty mask')
m = (lab == lab[v0, u0]).astype(np.uint8) if lab[v0, u0] > 0 else (lab == 1 + np.argmax(st[1:, 4])).astype(np.uint8)
zr = P[m > 0, 2]; z0 = np.median(zr)
ys, xs = np.nonzero(m)
pts = np.array([pix2plane(n, 'wrist', a, b, z0)[:2] for a, b in zip(xs, ys)])
w = D[ys, xs]**2; c = (pts * w[:, None]).sum(0) / w.sum()
print(f"z_top={z0:.4f} (p10..p90 {np.percentile(zr,10):.4f}..{np.percentile(zr,90):.4f}) region_centroid=({c[0]:.4f},{c[1]:.4f}) npx={len(xs)}")
mf = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
cs, hier = cv2.findContours(mf, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
outer = max([cc for cc, h in zip(cs, hier[0]) if h[3] < 0], key=cv2.contourArea)[:, 0, :]
po = np.array([pix2plane(n, 'wrist', a, b, z0)[:2] for a, b in outer])
x, y = po[:, 0], po[:, 1]; cr = x*np.roll(y, -1) - np.roll(x, -1)*y; A = cr.sum()/2
ocx = ((x+np.roll(x,-1))*cr).sum()/(6*A); ocy = ((y+np.roll(y,-1))*cr).sum()/(6*A)
rr = np.hypot(x-ocx, y-ocy)
print(f"outer centroid=({ocx:.4f},{ocy:.4f}) r min/med/max={rr.min()*1000:.1f}/{np.median(rr)*1000:.1f}/{rr.max()*1000:.1f}mm")
ang = np.degrees(np.arctan2(np.roll(y,-4)-y, np.roll(x,-4)-x)) % 60
h, e = np.histogram(ang, bins=30, range=(0, 60)); print('outer edge dir mod60 peak (deg from base +x):', e[np.argmax(h)] + 1)
holes = [cc for cc, h in zip(cs, hier[0]) if h[3] >= 0]
if holes:
    hc = max(holes, key=cv2.contourArea)[:, 0, :]; ph = np.array([pix2plane(n, 'wrist', a, b, z0)[:2] for a, b in hc])
    Am = np.c_[2*ph, np.ones(len(ph))]; b = (ph**2).sum(1)
    cx, cy, k = np.linalg.lstsq(Am, b, rcond=None)[0]; r = np.sqrt(k + cx*cx + cy*cy)
    print(f"hole center=({cx:.4f},{cy:.4f}) r={r*1000:.1f}mm")
ov = img.copy(); ov[m > 0] = (ov[m > 0] * 0.4 + np.array([255, 0, 0]) * 0.6).astype(np.uint8)
os.makedirs(f'{SESS}/scratch', exist_ok=True); Image.fromarray(ov).save(f'{SESS}/scratch/facefit_{n:04d}.png')
