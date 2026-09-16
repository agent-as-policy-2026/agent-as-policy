# tool: proj.py
# category: geometry
# purpose: project a base-frame 3D point into a camera image of a capture (wrist or top) to predict where an object/feature should appear; useful to check alignment of a held object against a target without depth
# usage: python3 knowledge/tools/proj.py <capture> <x> <y> <z> [wrist|top]    (run from the session directory)
# inputs/outputs: reads frames/<capture>_calib.json; prints "u=.. v=.. depth=.." (pixel and camera-z depth in metres)
# assumptions: pinhole model as documented in README_interface.md; defaults to the wrist camera; needs numpy
# verified: used successfully in the session that wrote it
import sys, json, numpy as np
def _cam(c):
    K = np.array(c["intrinsics"]); p = c["pose"]
    t = np.array([p["position"][k] for k in "xyz"]); q = p["rotation"]; w,x,y,z = q["w"],q["x"],q["y"],q["z"]
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return K,R,t
def _ray(c,u,v):
    K,R,t = _cam(c); d = R @ (np.linalg.inv(K) @ np.array([u,v,1.0])); return t, d/np.linalg.norm(d)
cap = int(sys.argv[1]); X = np.array(list(map(float, sys.argv[2:5]))); which = sys.argv[5] if len(sys.argv) > 5 else 'wrist'
K,R,t = _cam(json.load(open(f'frames/{cap:04d}_calib.json'))[which]); Xc = R.T @ (X - t); p = K @ (Xc/Xc[2])
print(f"u={p[0]:.1f} v={p[1]:.1f} depth={Xc[2]:.3f}")
