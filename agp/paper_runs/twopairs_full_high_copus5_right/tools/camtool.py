# tool: camtool.py
# category: geometry
# purpose: print the wrist camera pose relative to the tool (grasp-point) frame and the tool/camera axes in the base frame for a capture, to predict where the grasp point and fingers appear in the wrist image
# usage: <venv-python from README_interface.md> knowledge/tools/camtool.py <capture_N>    (run from the session directory; needs numpy)
# inputs/outputs: reads frames/NNNN_calib.json (_meta.ee_pose and wrist pose); prints tool axes in base, camera position and rotation in the tool frame, camera optical/image axes in base
# assumptions: none
# verified: used successfully in the session that wrote it
import sys, json, numpy as np
def qR(q):
    w,x,y,z=q["w"],q["x"],q["y"],q["z"]
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
N=int(sys.argv[1]); c=json.load(open(f"frames/{N:04d}_calib.json"))
print(c["_meta"].keys())
ee=c["_meta"]["ee_pose"]; Re=qR(ee["rotation"]); te=np.array([ee["position"][k] for k in "xyz"])
p=c["wrist"]["pose"]; Rc=qR(p["rotation"]); tc=np.array([p["position"][k] for k in "xyz"])
print("tool z in base", Re[:,2], "tool x", Re[:,0], "tool y", Re[:,1])
print("cam pos in tool", Re.T@(tc-te)); print("cam R in tool\n", Re.T@Rc)
print("cam optical axis base", Rc[:,2], "cam x(image right) base", Rc[:,0], "cam y(image down)", Rc[:,1])
