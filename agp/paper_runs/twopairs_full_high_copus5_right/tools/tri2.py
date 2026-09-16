# tool: tri2.py
# category: geometry
# purpose: triangulate one 3D point (base frame) from the same feature seen in two captures/cameras (e.g. two wrist views after a sideways move, or wrist + top)
# usage: <venv-python from README_interface.md> knowledge/tools/tri2.py <N1> <top|wrist> <u1> <v1> <N2> <top|wrist> <u2> <v2>    (run from the session directory; needs numpy)
# inputs/outputs: reads frames/NNNN_calib.json of both captures; prints the closest point on each ray, their midpoint and the ray gap in mm
# assumptions: pinhole model from README_interface.md, wrist distortion ignored; height (z) is sensitive to pixel errors when the baseline is short (a 5 cm baseline at 15 cm range gave a z error above 1 cm with blob-centre pixels) - prefer a contact stop or plane_z for height
# verified: used successfully in the session that wrote it
import sys, json, numpy as np
def qR(q):
    w,x,y,z=q["w"],q["x"],q["y"],q["z"]
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def ray(N,cam,u,v):
    c=json.load(open(f"frames/{int(N):04d}_calib.json"))[cam]
    K=np.array(c["intrinsics"]); p=c["pose"]; R=qR(p["rotation"]); t=np.array([p["position"][k] for k in "xyz"])
    d=R@np.array([(float(u)-K[0,2])/K[0,0],(float(v)-K[1,2])/K[1,1],1.0]); return t,d/np.linalg.norm(d)
a=sys.argv[1:]
o1,d1=ray(*a[0:4]); o2,d2=ray(*a[4:8])
s=np.linalg.lstsq(np.c_[d1,-d2],o2-o1,rcond=None)[0]; p1=o1+s[0]*d1; p2=o2+s[1]*d2
print("p1",p1.round(4),"p2",p2.round(4),"mid",((p1+p2)/2).round(4),"gap mm",round(np.linalg.norm(p1-p2)*1000,1))
