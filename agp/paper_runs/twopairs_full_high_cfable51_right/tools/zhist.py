# tool: zhist.py
# category: geometry
# purpose: deproject the wrist depth inside a pixel ROI to the base frame and print a 5 mm z-histogram plus the centroid of the highest layer; use it to read part heights, step/shoulder levels and top-face centres from one wrist capture
# usage: python3 knowledge/tools/zhist.py <capture> <u0> <v0> <u1> <v1>    (run from the session directory)
# inputs/outputs: reads frames/<capture>_wrist_depth.npy and _calib.json; prints one line per 5 mm z bin (count) and "top97 z=.. n=.. centroid=(x,y)" for points within 6 mm of the 97th z percentile
# assumptions: pinhole model as in README_interface.md; depth 0 = invalid; histogram spans z -0.06..0.08 m; needs numpy (analysis venv)
# verified: used successfully in the session that wrote it
import sys, json, numpy as np
cap=int(sys.argv[1]); u0,v0,u1,v1=map(int,sys.argv[2:6])
c=json.load(open(f'frames/{cap:04d}_calib.json'))['wrist']
K=np.array(c['intrinsics']); p=c['pose']; t=np.array([p['position'][k] for k in 'xyz'])
q=p['rotation']; w,x,y,z=q['w'],q['x'],q['y'],q['z']
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
D=np.load(f'frames/{cap:04d}_wrist_depth.npy')
vv,uu=np.mgrid[v0:v1,u0:u1]; d=D[v0:v1,u0:u1]
m=d>0; uu=uu[m]; vv=vv[m]; d=d[m]
Xc=np.linalg.inv(K)@np.vstack([uu,vv,np.ones_like(uu)])*d
X=(R@Xc).T+t
h,e=np.histogram(X[:,2],bins=np.arange(-0.06,0.08,0.005))
for hh,ee in zip(h,e): print(f'z>={ee:+.3f}: {hh}')
zt=np.percentile(X[:,2],97)
top=X[X[:,2]>zt-0.006]
print('top97 z=%.4f n=%d centroid=(%.4f,%.4f)'%(zt,len(top),top[:,0].mean(),top[:,1].mean()))
