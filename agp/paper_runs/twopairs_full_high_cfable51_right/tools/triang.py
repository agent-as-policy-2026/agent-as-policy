# tool: triang.py
# category: geometry
# purpose: triangulate the base-frame 3D position of one feature (e.g. a part's top-face centre) seen in BOTH the wrist and top images of the same capture, by intersecting the two pixel rays; works where wrist depth is unreliable (white/featureless surfaces)
# usage: python3 knowledge/tools/triang.py <capture> <u_wrist> <v_wrist> <u_top> <v_top>    (run from the session directory)
# inputs/outputs: reads frames/<capture>_calib.json; prints "x y z gap=..mm" (midpoint of the closest approach of the two rays; gap = ray miss distance, large gap = inconsistent pixel picks)
# assumptions: both cameras captured at the same instant (true for one frames command); pinhole model; needs numpy
# verified: used successfully in the session that wrote it
import sys, json, numpy as np
def _cam(c):
    K = np.array(c["intrinsics"]); p = c["pose"]
    t = np.array([p["position"][k] for k in "xyz"]); q = p["rotation"]; w,x,y,z = q["w"],q["x"],q["y"],q["z"]
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return K,R,t
def _ray(c,u,v):
    K,R,t = _cam(c); d = R @ (np.linalg.inv(K) @ np.array([u,v,1.0])); return t, d/np.linalg.norm(d)
cap = int(sys.argv[1]); uw,vw,ut,vt = map(float, sys.argv[2:6])
C = json.load(open(f'frames/{cap:04d}_calib.json'))
p1,d1 = _ray(C['wrist'],uw,vw); p2,d2 = _ray(C['top'],ut,vt)
n = np.cross(d1,d2); A = np.array([d1,-d2,n]).T; s,t_,_ = np.linalg.solve(A, p2-p1)
q1 = p1+s*d1; q2 = p2+t_*d2; X = (q1+q2)/2
print(f"{X[0]:.4f} {X[1]:.4f} {X[2]:.4f}  gap={np.linalg.norm(q1-q2)*1000:.1f}mm")
