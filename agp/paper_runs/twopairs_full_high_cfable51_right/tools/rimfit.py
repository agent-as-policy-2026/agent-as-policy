# tool: rimfit.py
# category: geometry
# purpose: depth-free localisation of a short upright round part (ring, nut, cylinder) lying on a surface, from a wrist capture: takes the outer contour of a colour blob, deprojects contour pixels onto known planes (half facing away from the camera -> part top plane, half facing the camera -> bottom plane) and fits a circle in base-frame xy; complements depth-based fits where D405 depth is noisy
# usage: python3 knowledge/tools/rimfit.py <capture> <blue|white> <z_top> <z_bottom> [u0 v0 [umin vmin umax vmax]] [--far] [--r=<radius_m>]    (run from the session directory; seed pixel picks the blob containing it, pixel ROI separates touching parts; --r fixes the radius and solves only the centre; --far uses only the half away from the camera - NOT reliable for a part held in the gripper, whose side wall dominates the outline)
# inputs/outputs: reads frames/<capture>_wrist.png and _calib.json; prints centre=(x,y) r_outer=.. n=.. rms=.. bbox=..
# assumptions: blue = HSV hue 80-110,S>=80,V>=60; white = V>=150,S<=60; part axis vertical; near/far split is done geometrically from the camera's ground position; needs numpy + cv2 (analysis venv)
# verified: used successfully in the session that wrote it
import sys, json, numpy as np, cv2
FAR='--far' in sys.argv; RFIX=[float(a[4:]) for a in sys.argv if a.startswith('--r=')]; RFIX=RFIX[0] if RFIX else None
sys.argv=[a for a in sys.argv if not a.startswith('--')]
cap=int(sys.argv[1]); cls=sys.argv[2]; ztop=float(sys.argv[3]); zbot=float(sys.argv[4])
seed=(int(sys.argv[5]),int(sys.argv[6])) if len(sys.argv)>6 else None
c=json.load(open(f'frames/{cap:04d}_calib.json'))['wrist']
K=np.array(c['intrinsics']); p=c['pose']; t=np.array([p['position'][k] for k in 'xyz'])
q=p['rotation']; w,x,y,z=q['w'],q['x'],q['y'],q['z']
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
Ki=np.linalg.inv(K)
img=cv2.imread(f'frames/{cap:04d}_wrist.png'); hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
m=cv2.inRange(hsv,(80,80,60),(110,255,255)) if cls=='blue' else cv2.inRange(hsv,(0,0,150),(180,60,255))
m=cv2.morphologyEx(m,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
if len(sys.argv)>10:
    roi=np.zeros_like(m); u0,v0,u1,v1=map(int,sys.argv[7:11]); roi[v0:v1,u0:u1]=255; m=cv2.bitwise_and(m,roi)
n,lab,st,cen=cv2.connectedComponentsWithStats(m)
i=lab[seed[1],seed[0]] if seed else max(range(1,n),key=lambda k:st[k][4])
if i==0: print('seed not on blob'); sys.exit()
blob=(lab==i).astype(np.uint8)
cnts,_=cv2.findContours(blob,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE); cnt=max(cnts,key=len).reshape(-1,2)
def plane(u,v,zp):
    d=R@(Ki@np.array([u,v,1.0])); s=(zp-t[2])/d[2]; return t+s*d
Ptop=np.array([plane(u,v,ztop) for u,v in cnt]); cen0=Ptop[:,:2].mean(0)
camdir=cen0-t[:2]; camdir/=np.linalg.norm(camdir)   # from camera ground position toward part
near=((Ptop[:,:2]-cen0)@camdir)<0
P=np.array([plane(u,v,zbot) if nr else plane(u,v,ztop) for (u,v),nr in zip(cnt,near)])
if FAR:
    proj=(Ptop[:,:2]-cen0)@camdir; P=Ptop[proj>0.3*proj.max()]
for it in range(3):
    if RFIX is None:
        A=np.c_[2*P[:,0],2*P[:,1],np.ones(len(P))]; b=P[:,0]**2+P[:,1]**2
        cc=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(cc[2]+cc[0]**2+cc[1]**2)
    else:
        r=RFIX; cc=P[:,:2].mean(0)+r*camdir*(-1 if not FAR else 0)  # init
        for k in range(30):   # Gauss-Newton on centre with fixed radius
            dx=P[:,0]-cc[0]; dy=P[:,1]-cc[1]; d=np.hypot(dx,dy); J=np.c_[-dx/d,-dy/d]; f=d-r
            step=np.linalg.lstsq(J,-f,rcond=None)[0]; cc=np.array([cc[0]+step[0],cc[1]+step[1]])
            if np.linalg.norm(step)<1e-6: break
    res=np.hypot(P[:,0]-cc[0],P[:,1]-cc[1])-r; keep=np.abs(res)<max(2.5*res.std(),0.002); P=P[keep]
print(f"centre=({cc[0]:.4f},{cc[1]:.4f}) r_outer={r:.4f} n={len(P)} rms={np.sqrt((res[keep]**2).mean()):.4f} bbox={st[i][:4].tolist()}")
