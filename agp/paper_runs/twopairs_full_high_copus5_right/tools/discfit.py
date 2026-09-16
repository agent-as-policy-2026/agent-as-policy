# tool: discfit.py
# category: geometry
# purpose: locate the axis of an upright light-coloured cylinder/peg (top face at a known height) by fitting a projected horizontal circle to the far-side silhouette of its bright blob in the top or wrist image
# usage: <venv-python from README_interface.md> knowledge/tools/discfit.py <top|wrist> <capture_N> <u0> <v0> <u1> <v1> <z_top> <x_guess> <y_guess> [brightness_thr=170]    (run from the session directory; needs numpy+PIL+scipy)
# inputs/outputs: reads frames/NNNN_<cam>.png and frames/NNNN_calib.json; crops to the pixel box u0..u1, v0..v1 (should contain only the object); prints the fitted circle centre (base-frame x,y), radius, point count and rms pixel residual
# assumptions: object pixels have min(R,G,B) > thr and the largest bright blob in the box is the object; table z = -0.045 (from README) for the image-vertical direction; initial radius guess 0.019 m; z_top must be known (e.g. from a contact stop); wrist distortion ignored; in one test the top- and wrist-camera fits of the same object disagreed by ~6 mm, so treat the result as +-3-5 mm and cross-check both cameras
# verified: used successfully in the session that wrote it
import sys, json, numpy as np
from PIL import Image
from scipy import ndimage, optimize
def qR(q):
    w,x,y,z=q["w"],q["x"],q["y"],q["z"]
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
CAM=sys.argv.pop(1); N=int(sys.argv[1]); u0,v0,u1,v1=map(int,sys.argv[2:6]); Z=float(sys.argv[6]); x0,y0=float(sys.argv[7]),float(sys.argv[8])
thr=int(sys.argv[9]) if len(sys.argv)>9 else 170
c=json.load(open(f"frames/{N:04d}_calib.json"))[CAM]; K=np.array(c["intrinsics"]); R=qR(c["pose"]["rotation"]); t=np.array([c["pose"]["position"][k] for k in "xyz"])
def proj(P): Pc=(P-t)@R; return (Pc[:,:2]/Pc[:,2:3])*[K[0,0],K[1,1]]+[K[0,2],K[1,2]]
im=np.asarray(Image.open(f"frames/{N:04d}_"+CAM+".png").convert("RGB")).astype(float)[v0:v1,u0:u1]
m=im.min(-1)>thr; m=ndimage.binary_opening(m,iterations=1); lab,n=ndimage.label(m); sz=ndimage.sum(m,lab,range(1,n+1)); m=lab==(1+np.argmax(sz))
edge=m&~ndimage.binary_erosion(m); vs,us=np.nonzero(edge); pts=np.c_[us+u0,vs+v0].astype(float)
# vertical direction in image at the object
b=proj(np.array([[x0,y0,-0.045],[x0,y0,Z]])); vd=(b[1]-b[0])/np.linalg.norm(b[1]-b[0]); ctr=b[1]
sel=pts[(pts-ctr)@vd> -2]   # far-side silhouette (top-disc side)
th=np.linspace(0,2*np.pi,360,endpoint=False)
def cost(p):
    cx,cy,r=p; C=proj(np.c_[cx+r*np.cos(th),cy+r*np.sin(th),np.full_like(th,Z)])
    d=np.sqrt(((sel[:,None,:]-C[None])**2).sum(-1)).min(1); return np.sum(np.minimum(d,6)**2)
res=optimize.minimize(cost,[x0,y0,0.019],method="Nelder-Mead",options={"xatol":1e-5,"fatol":1e-3,"maxiter":2000})
cx,cy,r=res.x; C=proj(np.c_[cx+r*np.cos(th),cy+r*np.sin(th),np.full_like(th,Z)])
d=np.sqrt(((sel[:,None,:]-C[None])**2).sum(-1)).min(1)
print(f"center=({cx:.4f},{cy:.4f}) radius={r:.4f} at z={Z}  npts={len(sel)} rms_px={np.sqrt(np.mean(np.minimum(d,6)**2)):.2f}")
