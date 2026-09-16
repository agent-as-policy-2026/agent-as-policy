# tool: hexfit.py
# category: geometry
# purpose: fit a regular hexagon (nut, hex head, hex prism) to a colour blob in a wrist or top capture projected onto plane z; report centre, radii, vertex direction and the flat-normal directions a parallel gripper should close along
# usage: python3 knowledge/tools/hexfit.py N wrist|top u v z "hmin,hmax,smin,smax,vmin,vmax"    (run from the session directory; needs numpy, opencv, pillow)
# inputs/outputs: frames/NNNN_<cam>.png + _calib.json; (u,v) = seed pixel inside the blob; z = height of the hex face; prints base-frame centre, area, p95/p5 radius, vertex dir and flat-normal dir (deg from base +x, mod 60); writes scratch/hexfit_NNNN_<cam>.png overlay
# assumptions: OpenCV HSV ranges (H 0-179); blob = whole hex (side walls seen obliquely inflate radii by a few mm and may bias the centre; view from nearly above); orientation from the 6-fold angular moment of the outline; for a tool-down grasp with yaw psi from (w0,x1,y0,z0) the fingers close along base angle 90+psi deg, so psi = flat_normal - 90 (mod 60); uses geo.py from knowledge/tools
# verified: used successfully in the session that wrote it
"""Fit a regular hexagon to a colour blob in a wrist/top capture, projected onto plane z.
usage: hexfit.py N cam u v z "hmin,hmax,smin,smax,vmin,vmax"
prints centre, circumradius, across-flats, vertex direction (deg from base +x, mod 60),
flat-normal direction (mod 60) = directions along which a parallel gripper should close.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, os.path.join(os.getcwd(), 'knowledge/tools'))
from geo import *
from PIL import Image
import cv2
n = int(sys.argv[1]); c = sys.argv[2]; u0, v0 = int(sys.argv[3]), int(sys.argv[4]); z0 = float(sys.argv[5]); spec = sys.argv[6]
img = np.array(Image.open(f'{SESS}/frames/{n:04d}_{c}.png').convert('RGB'))
m = cv2.morphologyEx(hsv_mask(img, spec).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
nl, lab, st, _ = cv2.connectedComponentsWithStats(m)
if lab[v0, u0] == 0: sys.exit('seed not in mask')
m = (lab == lab[v0, u0]).astype(np.uint8)
cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
outer = max(cs, key=cv2.contourArea)[:, 0, :]
p = np.array([pix2plane(n, c, a, b, z0)[:2] for a, b in outer])
x, y = p[:, 0], p[:, 1]; cr = x*np.roll(y, -1) - np.roll(x, -1)*y; A = cr.sum()/2
cx = ((x+np.roll(x, -1))*cr).sum()/(6*A); cy = ((y+np.roll(y, -1))*cr).sum()/(6*A)
d = p - [cx, cy]; r = np.hypot(d[:, 0], d[:, 1]); th = np.arctan2(d[:, 1], d[:, 0])
# resample uniformly in angle to avoid contour density bias
order = np.argsort(th); ths = np.linspace(-np.pi, np.pi, 720, endpoint=False)
rs = np.interp(ths, th[order], r[order], period=2*np.pi)
z6 = (rs * np.exp(6j*ths)).sum()
vdir = (np.degrees(np.angle(z6)) / 6) % 60           # vertex direction (r max)
rmax = np.percentile(rs, 95); rmin = np.percentile(rs, 5)
print(f'centre=({cx:.4f},{cy:.4f}) area={abs(A)*1e4:.2f}cm2 r_p95={rmax*1000:.1f}mm r_p5={rmin*1000:.1f}mm')
print(f'vertex dir mod60 = {vdir:.1f} deg ; flat-normal (close-along) dir mod60 = {(vdir+30)%60:.1f} deg')
ov = img.copy(); ov[m > 0] = (ov[m > 0]*0.4 + np.array([255, 0, 0])*0.6).astype(np.uint8)
Image.fromarray(ov).save(f'{SESS}/scratch/hexfit_{n:04d}_{c}.png')
