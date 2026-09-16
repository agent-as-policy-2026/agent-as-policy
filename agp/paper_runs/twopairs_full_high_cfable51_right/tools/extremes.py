# tool: extremes.py
# category: geometry
# purpose: in a wrist-image pixel ROI, mask one colour class (blue, white or brown table) and print the leftmost/rightmost/topmost/bottommost pixels and centroid of the largest blob; feed the left/right pixels to deproject with plane_z to get a rim's centre line without depth (useful for parts in the gripper or featureless white parts)
# usage: python3 knowledge/tools/extremes.py <capture> <u0> <v0> <u1> <v1> <blue|white|brown>    (run from the session directory)
# inputs/outputs: reads frames/<capture>_wrist.png; prints "left: u v", "right: u v", "top: u v", "bottom: u v", "area N centroid u v"
# assumptions: blue = HSV hue 80-110,S>=80,V>=60; white = V>=150,S<=60; brown = hue<=30,S>=40,V<=200 (wood table seen through a hole); largest connected component only; needs numpy + cv2 (analysis venv)
# verified: used successfully in the session that wrote it
import sys, numpy as np, cv2
cap=int(sys.argv[1]); u0,v0,u1,v1=map(int,sys.argv[2:6]); cls=sys.argv[6]
im=cv2.imread(f'frames/{cap:04d}_wrist.png'); hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
H,S,V=[hsv[:,:,i].astype(int) for i in range(3)]
if cls=='blue': m=(H>=80)&(H<=110)&(S>=80)&(V>=60)
elif cls=='white': m=(V>=150)&(S<=60)
else: m=(H<=30)&(S>=40)&(V<=200)
roi=np.zeros_like(m); roi[v0:v1,u0:u1]=True; m&=roi
m=m.astype(np.uint8); n,lab,st,cen=cv2.connectedComponentsWithStats(m)
if n<2: sys.exit('no pixels of that class in the ROI')
k=1+np.argmax(st[1:,cv2.CC_STAT_AREA]); m=lab==k
vs,us=np.nonzero(m)
for name,i in [('left',np.argmin(us)),('right',np.argmax(us)),('top',np.argmin(vs)),('bottom',np.argmax(vs))]:
    print(f'{name}: u={us[i]} v={vs[i]}')
print('area',len(us),'centroid u=%.1f v=%.1f'%(us.mean(),vs.mean()))
