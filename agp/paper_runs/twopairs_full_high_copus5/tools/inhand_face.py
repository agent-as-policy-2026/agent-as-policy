# tool: inhand_face.py
# category: geometry
# purpose: locate a held (or about-to-be-grasped) object's flat TOP face in the TOOL frame from one wrist capture, using colour AND a depth band around the topmost level so side walls are excluded; gives the in-hand offset needed for precise placement
# usage: python3 knowledge/tools/inhand_face.py N "hmin,hmax,smin,smax,vmin,vmax" [band_m=0.002] [v_min_px=150]    (run from the session directory; needs numpy, opencv, pillow)
# inputs/outputs: frames/NNNN_wrist.png, _wrist_depth.npy, _calib.json (uses _meta.ee_pose); prints the top face's tool z (negative = above the grasp point) and the tool-frame x/y extents of its outline plus their midpoint; writes scratch/inhandface_NNNN.png overlay
# assumptions: top face has stereo texture (valid depth); largest connected face region is the object (restrict v_min_px / HSV so other same-colour objects are excluded); the lower image border may cut the face - then use the far edge/vertex and the known object size instead of the midpoint; uses geo.py from knowledge/tools
# verified: used successfully in the session that wrote it
"""Held/near object's flat top face in TOOL frame from a wrist capture: colour mask AND depth within
a band around the topmost level; reports the face tool z and outline extents (tool frame).
usage: inhand_face.py N "hmin,hmax,smin,smax,vmin,vmax" [band_m=0.002] [v_min_px=150]
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, os.path.join(os.getcwd(), 'knowledge/tools'))
from geo import *
from PIL import Image
import cv2
n = int(sys.argv[1]); spec = sys.argv[2]
band = float(sys.argv[3]) if len(sys.argv) > 3 else 0.002
vmin = int(sys.argv[4]) if len(sys.argv) > 4 else 150
img = np.array(Image.open(f'{SESS}/frames/{n:04d}_wrist.png').convert('RGB'))
P, D = wrist_points(n)
meta = calib(n)['_meta']['ee_pose']; Re = qmat(meta['rotation']); pe = np.array([meta['position'][k] for k in 'xyz'])
L = (P - pe) @ Re                                  # tool-frame points
vv = np.mgrid[0:D.shape[0], 0:D.shape[1]][0]
m0 = hsv_mask(img, spec) & (D > 0) & (vv >= vmin)
zt = np.percentile(L[m0][:, 2], 3)                 # top level (tool z is negative above grasp)
m = (m0 & (np.abs(L[..., 2] - zt) < band)).astype(np.uint8)
m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
nl, lab, st, _ = cv2.connectedComponentsWithStats(m)
m = (lab == 1 + np.argmax(st[1:, 4])).astype(np.uint8)
cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
c = max(cs, key=cv2.contourArea)[:, 0, :]
zb = pe + Re @ np.array([0, 0, zt])                # base z of the face plane
pts = np.array([(pix2plane(n, 'wrist', a, b, zb[2]) - pe) @ Re for a, b in c])[:, :2]
print(f'face tool z={zt:.4f}  outline x {pts[:,0].min():.4f}..{pts[:,0].max():.4f}  y {pts[:,1].min():.4f}..{pts[:,1].max():.4f}')
print(f'mid of extents: x={(pts[:,0].min()+pts[:,0].max())/2:.4f} y={(pts[:,1].min()+pts[:,1].max())/2:.4f}')
ov = img.copy(); ov[m > 0] = (ov[m > 0]*0.4 + np.array([255, 0, 0])*0.6).astype(np.uint8)
Image.fromarray(ov).save(f'{SESS}/scratch/inhandface_{n:04d}.png')
