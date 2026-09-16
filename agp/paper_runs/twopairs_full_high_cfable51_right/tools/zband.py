# tool: zband.py
# category: geometry
# purpose: from a wrist capture, deproject every pixel of a colour class (blue or white) with wrist depth, keep base-frame points whose z lies in a band, cluster them in xy and for each cluster print centroid, least-squares circle (centre, radius), min/max radial distance and z stats; use it to locate top faces / rims of several parts at once and to measure diameters
# usage: python3 knowledge/tools/zband.py <capture> <blue|white> <z_min> <z_max> [cluster_gap_m=0.02]    (run from the session directory)
# inputs/outputs: reads frames/<capture>_wrist.png, _wrist_depth.npy, _calib.json; prints one line per cluster (n points, centroid, fitted circle centre and radius, radial min/max, z mean/max) plus the pixel bbox
# assumptions: blue = HSV hue 80-110,S>=80,V>=60; white = V>=150,S<=60; single-link clustering by xy gap; algebraic circle fit; needs numpy + cv2 (analysis venv)
# verified: used successfully in the session that wrote it
import sys, json, numpy as np, cv2
cap=int(sys.argv[1]); cls=sys.argv[2]; zmin,zmax=float(sys.argv[3]),float(sys.argv[4])
gap=float(sys.argv[5]) if len(sys.argv)>5 else 0.02
c=json.load(open(f'frames/{cap:04d}_calib.json'))['wrist']
K=np.array(c['intrinsics']); p=c['pose']; t=np.array([p['position'][k] for k in 'xyz'])
q=p['rotation']; w,x,y,z=q['w'],q['x'],q['y'],q['z']
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
img=cv2.imread(f'frames/{cap:04d}_wrist.png'); dep=np.load(f'frames/{cap:04d}_wrist_depth.npy')
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
m=cv2.inRange(hsv,(80,80,60),(110,255,255)) if cls=='blue' else cv2.inRange(hsv,(0,0,150),(180,60,255))
vs,us=np.nonzero((m>0)&(dep>0)); d=dep[vs,us]
P=(np.linalg.inv(K)@np.vstack([us,vs,np.ones_like(us)]))*d; X=(R@P).T+t
sel=(X[:,2]>zmin)&(X[:,2]<zmax); X=X[sel]; us=us[sel]; vs=vs[sel]
if len(X)==0: print('no points'); sys.exit()
# grid clustering
g=np.floor(X[:,:2]/gap).astype(int); keys={}
lab=-np.ones(len(X),int); cur=0
from collections import defaultdict
cells=defaultdict(list)
for i,k in enumerate(map(tuple,g)): cells[k].append(i)
seen=set()
for k in list(cells):
    if k in seen: continue
    stack=[k]; seen.add(k)
    while stack:
        kk=stack.pop()
        for i in cells[kk]: lab[i]=cur
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                nk=(kk[0]+dx,kk[1]+dy)
                if nk in cells and nk not in seen: seen.add(nk); stack.append(nk)
    cur+=1
for l in range(cur):
    S=X[lab==l]
    if len(S)<30: continue
    cen=S[:,:2].mean(0)
    A=np.c_[2*S[:,0],2*S[:,1],np.ones(len(S))]; b=S[:,0]**2+S[:,1]**2
    cc=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(max(cc[2]+cc[0]**2+cc[1]**2,0))
    rad=np.hypot(S[:,0]-cen[0],S[:,1]-cen[1])
    print(f"cluster n={len(S)} centroid=({cen[0]:.4f},{cen[1]:.4f}) circle=({cc[0]:.4f},{cc[1]:.4f}) r={r:.4f} rad_min={rad.min():.4f} rad_p95={np.percentile(rad,95):.4f} zmean={S[:,2].mean():.4f} zmax={S[:,2].max():.4f} px_u[{us[lab==l].min()}-{us[lab==l].max()}] v[{vs[lab==l].min()}-{vs[lab==l].max()}]")
