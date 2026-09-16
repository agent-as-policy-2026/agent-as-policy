# tool: blobplane.py
# category: geometry
# purpose: take the colour blob containing a seed pixel in a wrist or top capture, project its outline onto a horizontal plane z, and report centroid, area, min-area rectangle and circle fits of the outer contour and the largest hole
# usage: python3 knowledge/tools/blobplane.py N wrist|top u v z "hmin,hmax,smin,smax,vmin,vmax"    (run from the session directory; needs numpy, opencv, pillow)
# inputs/outputs: frames/NNNN_<cam>.png + _calib.json; (u,v) = seed pixel inside the blob; z = height of the face whose outline you want (e.g. a measured top-face z); prints base-frame outer centroid/area/rect size+angle, outer circle, hole circle; writes scratch/blob_NNNN_<cam>.png overlay
# assumptions: OpenCV HSV ranges (H 0-179); restrict V/S so only the face at height z is selected (side walls seen obliquely bias the outline); uses geo.py from the same directory
# verified: used successfully in the session that wrote it
"""Colour blob in a wrist (or top) capture -> outline projected onto a horizontal plane z.
usage: blobplane.py N cam u v z "hmin,hmax,smin,smax,vmin,vmax"
prints outer-contour centroid, min-area-rect (size, angle), circle fit of outer contour and of largest hole.
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
cs, hier = cv2.findContours(m, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

def circ(p):
    A = np.c_[2*p, np.ones(len(p))]; b = (p**2).sum(1)
    cx, cy, k = np.linalg.lstsq(A, b, rcond=None)[0]
    return cx, cy, np.sqrt(k + cx*cx + cy*cy)

outer = max([cc for cc, h in zip(cs, hier[0]) if h[3] < 0], key=cv2.contourArea)[:, 0, :]
po = np.array([pix2plane(n, c, a, b, z0)[:2] for a, b in outer])
x, y = po[:, 0], po[:, 1]; cr = x*np.roll(y, -1) - np.roll(x, -1)*y; A = cr.sum()/2
ocx = ((x+np.roll(x, -1))*cr).sum()/(6*A); ocy = ((y+np.roll(y, -1))*cr).sum()/(6*A)
rect = cv2.minAreaRect((po*1e4).astype(np.float32))
print(f'outer centroid=({ocx:.4f},{ocy:.4f}) area={abs(A)*1e4:.2f}cm2 minrect size=({rect[1][0]/10:.1f},{rect[1][1]/10:.1f})mm angle={rect[2]:.1f}')
cx, cy, r = circ(po); print(f'outer circle=({cx:.4f},{cy:.4f}) r={r*1000:.1f}mm')
holes = [cc for cc, h in zip(cs, hier[0]) if h[3] >= 0]
if holes:
    hc = max(holes, key=cv2.contourArea)[:, 0, :]
    if len(hc) > 10:
        ph = np.array([pix2plane(n, c, a, b, z0)[:2] for a, b in hc])
        cx, cy, r = circ(ph); print(f'hole circle=({cx:.4f},{cy:.4f}) r={r*1000:.1f}mm')
ov = img.copy(); ov[m > 0] = (ov[m > 0]*0.4 + np.array([255, 0, 0])*0.6).astype(np.uint8)
Image.fromarray(ov).save(f'{SESS}/scratch/blob_{n:04d}_{c}.png')
