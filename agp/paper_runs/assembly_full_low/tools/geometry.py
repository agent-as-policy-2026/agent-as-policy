# tool: geometry.py
# category: geometry
# purpose: Batch-project base-frame points or intersect pixel rays with horizontal planes using saved camera calibration.
# usage: python3 knowledge/tools/geometry.py <calibration.json> <camera> <ray|project> '<JSON list of triples>'    (run from the session directory)
# inputs/outputs: Reads returned calibration JSON; ray triples are [u,v,plane_z], project triples are [x,y,z]; prints one result per line.
# assumptions: Requires numpy and scipy; metres, pinhole intrinsics, scalar-first pose quaternion fields; rectified images only; rays must intersect the plane and projected points must be in front of the camera.
# verified: used successfully in the session that wrote it
import json, sys, numpy as np
from scipy.spatial.transform import Rotation
c=json.load(open(sys.argv[1]))[sys.argv[2]]
q=c['pose']['rotation']; r=Rotation.from_quat([q[k] for k in ['x','y','z','w']]).as_matrix()
t=np.array([c['pose']['position'][k] for k in ['x','y','z']]); K=np.array(c['intrinsics'])
for item in json.loads(sys.argv[4]):
 if sys.argv[3]=='ray':
  u,v,z=item; d=r@np.linalg.solve(K,[u,v,1]); p=t+d*(z-t[2])/d[2]; print(np.round(p,5).tolist())
 else:
  p=r.T@(np.array(item)-t); uv=K@(p/p[2]); print(np.round(uv[:2],1).tolist())
