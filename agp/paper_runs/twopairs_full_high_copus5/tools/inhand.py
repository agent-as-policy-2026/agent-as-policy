# tool: inhand.py
# category: geometry
# purpose: measure where a held object sits relative to the grasp point (TOOL frame) from one wrist capture taken after lifting it, so placement can correct for in-hand offset
# usage: python3 knowledge/tools/inhand.py N "hmin,hmax,smin,smax,vmin,vmax" [v_min_px=200]    (run from the session directory; needs numpy, opencv, pillow)
# inputs/outputs: frames/NNNN_wrist.png, _wrist_depth.npy, _calib.json (uses _meta.ee_pose); prints tool-frame percentiles of the object's points, its top height (tool z is negative ABOVE the grasp point) and a circle fit (centre = offset from grasp point in tool x,y) of the top band
# assumptions: the held object appears in the lower image rows (v >= v_min_px) between the fingers; top band = points within 3 mm of the highest; circle fit suits round rims/rings, for polygons read the extents instead
# verified: used successfully in the session that wrote it
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geo import *
from PIL import Image
n = int(sys.argv[1]); spec = sys.argv[2]; vmin = int(sys.argv[3]) if len(sys.argv) > 3 else 200
img = np.array(Image.open(f'{SESS}/frames/{n:04d}_wrist.png').convert('RGB'))
P, D = wrist_points(n)
vv = np.mgrid[0:D.shape[0], 0:D.shape[1]][0]
m = hsv_mask(img, spec) & (D > 0) & (vv >= vmin)
meta = calib(n)['_meta']['ee_pose']; Re = qmat(meta['rotation']); pe = np.array([meta['position'][k] for k in 'xyz'])
loc = (P[m] - pe) @ Re
print('npts', len(loc))
if len(loc) == 0: sys.exit('no points')
for q in (1, 50, 99): print(f'pct{q}: x={np.percentile(loc[:,0],q):.4f} y={np.percentile(loc[:,1],q):.4f} z={np.percentile(loc[:,2],q):.4f}')
ztop = np.percentile(loc[:, 2], 2)
band = loc[np.abs(loc[:, 2] - ztop) < 0.003]
print(f'top z(tool)={ztop:.4f} band n={len(band)}')
if len(band) > 20:
    A = np.c_[2*band[:, :2], np.ones(len(band))]; b = (band[:, :2]**2).sum(1)
    cx, cy, k = np.linalg.lstsq(A, b, rcond=None)[0]; r = np.sqrt(k + cx*cx + cy*cy)
    print(f'top band circle (tool frame): center=({cx:.4f},{cy:.4f}) r={r*1000:.1f}mm')
