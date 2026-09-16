# tool: topface.py
# category: geometry
# purpose: locate the top face of an upright bright/white part in a wrist capture: takes the bright blob containing a seed pixel, deprojects its pixels with wrist depth, keeps only the highest ones (top face) and prints their centre and height; good at close range (~0.1 m) where D405 depth on matte white is usable
# usage: python3 knowledge/tools/topface.py <capture> <u_seed> <v_seed> <z_band_m>    (run from the session directory; z_band ~0.004-0.008)
# inputs/outputs: reads frames/<capture>_wrist.png, _wrist_depth.npy, _calib.json; prints "top centre mean=(x,y) z_top=.. n=.. zmax97=.. px_mean=(u,v)"
# assumptions: bright = HSV V>=140 and S<=70 (white/light-grey parts); the 97th percentile of z is taken as the top; needs numpy + cv2 (analysis venv)
# verified: used successfully in the session that wrote it
import sys, json, numpy as np
def _cam(c):
    K = np.array(c["intrinsics"]); p = c["pose"]
    t = np.array([p["position"][k] for k in "xyz"]); q = p["rotation"]; w,x,y,z = q["w"],q["x"],q["y"],q["z"]
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return K,R,t
def _ray(c,u,v):
    K,R,t = _cam(c); d = R @ (np.linalg.inv(K) @ np.array([u,v,1.0])); return t, d/np.linalg.norm(d)
import cv2
cap = int(sys.argv[1]); u0,v0 = int(sys.argv[2]),int(sys.argv[3]); band = float(sys.argv[4])
img = cv2.imread(f'frames/{cap:04d}_wrist.png'); dep = np.load(f'frames/{cap:04d}_wrist_depth.npy')
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV); m = cv2.inRange(hsv,(0,0,140),(180,70,255))
n,lab,st,cen = cv2.connectedComponentsWithStats(m); i = lab[v0,u0]
if i == 0: print("seed pixel is not inside a bright blob"); sys.exit(1)
K,R,t = _cam(json.load(open(f'frames/{cap:04d}_calib.json'))['wrist']); Ki = np.linalg.inv(K)
mask = lab == i; vs,us = np.nonzero(mask & (dep > 0)); d = dep[vs,us]
P = (Ki @ np.vstack([us,vs,np.ones_like(us)])) * d; X = (R @ P).T + t
zmax = np.percentile(X[:,2],97); sel = X[:,2] > zmax-band; Y = X[sel]
print(f"top centre mean=({Y[:,0].mean():.4f},{Y[:,1].mean():.4f}) z_top={Y[:,2].mean():.4f} n={len(Y)} zmax97={zmax:.4f} px_mean=({us[sel].mean():.1f},{vs[sel].mean():.1f})")
