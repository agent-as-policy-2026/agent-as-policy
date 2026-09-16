# tool: ringfit.py
# category: geometry
# purpose: find the centre and radius of a coloured ring/annular part (lying flat or held) from a wrist capture: deprojects the largest colour blob with wrist depth, keeps points in a z band (the top rim) and fits a circle in base-frame xy; works with partial occlusion
# usage: python3 knowledge/tools/ringfit.py <capture> <z_min> <z_max> [hue_lo hue_hi]    (run from the session directory; z band brackets the rim height, hue in OpenCV 0-179, default 80-110 = cyan/blue)
# inputs/outputs: reads frames/<capture>_wrist.png, _wrist_depth.npy, _calib.json; prints "centre=(x,y) r=.. n=.. zmean=.."
# assumptions: colour mask = HSV hue in [hue_lo,hue_hi], S>=80, V>=60; largest blob only; algebraic least-squares circle; r is the mid-rim radius; needs numpy + cv2 (analysis venv); depth below ~0.07 m from the D405 is unreliable
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
cap = int(sys.argv[1]); zmin,zmax = float(sys.argv[2]),float(sys.argv[3])
hlo,hhi = (int(sys.argv[4]),int(sys.argv[5])) if len(sys.argv) > 5 else (80,110)
img = cv2.imread(f'frames/{cap:04d}_wrist.png'); dep = np.load(f'frames/{cap:04d}_wrist_depth.npy')
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV); m = cv2.inRange(hsv,(hlo,80,60),(hhi,255,255))
n_,lab_,st_,cen_ = cv2.connectedComponentsWithStats(m); i_ = max(range(1,n_), key=lambda k: st_[k][4]); m = lab_ == i_
K,R,t = _cam(json.load(open(f'frames/{cap:04d}_calib.json'))['wrist']); Ki = np.linalg.inv(K)
vs,us = np.nonzero(m & (dep > 0)); d = dep[vs,us]
P = (Ki @ np.vstack([us,vs,np.ones_like(us)])) * d; X = (R @ P).T + t
sel = (X[:,2] > zmin) & (X[:,2] < zmax); X = X[sel]
A = np.c_[2*X[:,0],2*X[:,1],np.ones(len(X))]; b = X[:,0]**2 + X[:,1]**2
c = np.linalg.lstsq(A,b,rcond=None)[0]; r = np.sqrt(c[2] + c[0]**2 + c[1]**2)
print(f"centre=({c[0]:.4f},{c[1]:.4f}) r={r:.4f} n={len(X)} zmean={X[:,2].mean():.4f}")
