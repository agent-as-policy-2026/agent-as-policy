# tool: px2base.py
# category: geometry
# purpose: deproject a wrist-camera pixel at a chosen camera depth (metres) into the robot base frame using frames/NNNN_calib.json; lets you use a depth you trust (e.g. the median over a coloured rim) instead of the single-pixel depth
# usage: python3 knowledge/tools/px2base.py <capture> <u> <v> <depth_m>    (run from the session directory)
# inputs/outputs: reads frames/<capture>_calib.json (wrist intrinsics + pose); prints "x y z" in metres, base frame
# assumptions: pinhole model, calib pose = camera pose in base frame, pixel = K @ (R^T (X - t)) normalised; needs numpy (use the analysis venv interpreter if plain python3 lacks it)
# verified: used successfully in the session that wrote it
import sys, json, numpy as np
def _cam(c):
    K = np.array(c["intrinsics"]); p = c["pose"]
    t = np.array([p["position"][k] for k in "xyz"]); q = p["rotation"]; w,x,y,z = q["w"],q["x"],q["y"],q["z"]
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return K,R,t
def _ray(c,u,v):
    K,R,t = _cam(c); d = R @ (np.linalg.inv(K) @ np.array([u,v,1.0])); return t, d/np.linalg.norm(d)
cap,u,v,d = int(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
c = json.load(open(f'frames/{cap:04d}_calib.json'))['wrist']
K,R,t = _cam(c); Xc = np.linalg.inv(K) @ np.array([u,v,1.0]); Xc = Xc/Xc[2]*d; X = R @ Xc + t
print(f"{X[0]:.4f} {X[1]:.4f} {X[2]:.4f}")
