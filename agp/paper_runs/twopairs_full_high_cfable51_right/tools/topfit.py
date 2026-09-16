# tool: topfit.py
# category: geometry
# purpose: fit a circle (centre, radius) to the wrist-depth points of a pixel ROI whose base-frame z lies in a band; use it to locate a round top face / rim / bottom edge of one part (also a part held in the gripper) without colour masks, and to compare two parts measured in the same capture
# usage: python3 knowledge/tools/topfit.py <capture> <u0> <v0> <u1> <v1> <z_lo> <z_hi>    (run from the session directory)
# inputs/outputs: reads frames/<capture>_wrist_depth.npy and _calib.json; prints "n=.. centre=(x,y) r_fit=.. rad p5/p50/p95=.. zmean=.. centroid=(x,y)" (radial percentiles show the inner/outer radius of an annulus)
# assumptions: pinhole model as in README_interface.md; depth 0 = invalid; algebraic least-squares circle; exits with an error if no points fall in the band; needs numpy (analysis venv)
# verified: used successfully in the session that wrote it
import sys, json, numpy as np
cap=int(sys.argv[1]); u0,v0,u1,v1=map(int,sys.argv[2:6]); zlo,zhi=float(sys.argv[6]),float(sys.argv[7])
c=json.load(open(f'frames/{cap:04d}_calib.json'))['wrist']
K=np.array(c['intrinsics']); p=c['pose']; t=np.array([p['position'][k] for k in 'xyz'])
q=p['rotation']; w,x,y,z=q['w'],q['x'],q['y'],q['z']
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
D=np.load(f'frames/{cap:04d}_wrist_depth.npy')
vv,uu=np.mgrid[v0:v1,u0:u1]; d=D[v0:v1,u0:u1]
m=d>0; uu=uu[m]; vv=vv[m]; d=d[m]
Xc=np.linalg.inv(K)@np.vstack([uu,vv,np.ones_like(uu)])*d
X=(R@Xc).T+t
s=(X[:,2]>=zlo)&(X[:,2]<=zhi); P=X[s]
if len(P)<5: sys.exit(f'only {len(P)} points in z band [{zlo},{zhi}] inside the ROI')
A=np.c_[2*P[:,0],2*P[:,1],np.ones(len(P))]; b=(P[:,:2]**2).sum(1)
cx,cy,cc=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(cc+cx*cx+cy*cy)
rad=np.hypot(P[:,0]-cx,P[:,1]-cy)
print(f'n={len(P)} centre=({cx:.4f},{cy:.4f}) r_fit={r:.4f} rad p5/p50/p95={np.percentile(rad,5):.4f}/{np.percentile(rad,50):.4f}/{np.percentile(rad,95):.4f} zmean={P[:,2].mean():.4f} centroid=({P[:,0].mean():.4f},{P[:,1].mean():.4f})')
