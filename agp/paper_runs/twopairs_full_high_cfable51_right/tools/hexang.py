# tool: hexang.py
# category: geometry
# purpose: orientation of a hexagonal part's flats in the robot base xy plane from a wrist capture (rotating-calipers minimum width of the colour blob's outline deprojected onto a horizontal plane); use it to choose the wrist yaw so the fingers close on two flats
# usage: python3 knowledge/tools/hexang.py <capture> <blue|white> <u_seed> <v_seed> <z_plane>    (run from the session directory)
# inputs/outputs: reads frames/<capture>_wrist.png and _calib.json; prints flat-normal angle (deg, base xy, valid mod 60), width across flats, perpendicular width and outline centre on the plane
# assumptions: blue = HSV hue 80-110,S>=80,V>=60; white = V>=150,S<=60; seed pixel must lie on the part body (not in its hole); z_plane = height of the part's top face; side walls bias the width upward by a few mm; needs numpy + cv2 (analysis venv)
# verified: used successfully in the session that wrote it
import sys, json, numpy as np, cv2
cap=int(sys.argv[1]); cls=sys.argv[2]; us,vs=int(sys.argv[3]),int(sys.argv[4]); zp=float(sys.argv[5])
c=json.load(open(f'frames/{cap:04d}_calib.json'))['wrist']
K=np.array(c['intrinsics']); p=c['pose']; t=np.array([p['position'][k] for k in 'xyz'])
q=p['rotation']; w,x,y,z=q['w'],q['x'],q['y'],q['z']
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
img=cv2.imread(f'frames/{cap:04d}_wrist.png'); hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
m=cv2.inRange(hsv,(80,80,60),(110,255,255)) if cls=='blue' else cv2.inRange(hsv,(0,0,150),(180,60,255))
n,lab,st,cen=cv2.connectedComponentsWithStats(m)
l=lab[vs,us]; assert l>0, 'seed not on blob'
mm=(lab==l).astype(np.uint8)
cs,_=cv2.findContours(mm,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE); cnt=max(cs,key=len)[:,0,:]
rays=(R@np.linalg.inv(K)@np.vstack([cnt[:,0],cnt[:,1],np.ones(len(cnt))]))
s=(zp-t[2])/rays[2]; P=(t[:,None]+rays*s).T[:,:2]
best=None
for a in np.arange(0,180,0.5):
    d=np.array([np.cos(np.radians(a)),np.sin(np.radians(a))]); pr=P@d; wdt=pr.max()-pr.min()
    if best is None or wdt<best[1]: best=(a,wdt)
a,wdt=best
d=np.array([np.cos(np.radians(a)),np.sin(np.radians(a))]); pr=P@d
d2=np.array([-d[1],d[0]]); pr2=P@d2
print(f'flat-normal angle={a:.1f} deg  width_across_flats={wdt:.4f}  perp_width={pr2.max()-pr2.min():.4f}  centre=({P[:,0].mean():.4f},{P[:,1].mean():.4f})')
