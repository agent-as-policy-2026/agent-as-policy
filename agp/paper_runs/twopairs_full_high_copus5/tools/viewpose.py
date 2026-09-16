# tool: viewpose.py
# category: geometry
# purpose: compute a tool-down grasp-point pose (any yaw) that makes the wrist camera look at a given base-frame point from a given distance
# usage: python3 knowledge/tools/viewpose.py x y z [dist_m=0.15] [yaw_deg=0]    (run from the session directory; needs numpy)
# inputs/outputs: target point in base frame; prints move_ee JSON args {"position":...,"rotation":...} to pass to robot_client.py move_ee
# assumptions: wrist camera sits at (0.0728, 0.0021, -0.0613) m in the tool frame with optical axis (-0.428, -0.021, 0.903) in the tool frame (measured from calib pose vs ee_pose on this robot; re-check against frames/NNNN_calib.json if the camera mount changed); yaw is about base z from the canonical down rotation (w0,x1,y0,z0)
# verified: used successfully in the session that wrote it
"""Grasp-point pose (tool down, yaw psi about base z) so the wrist camera looks at target from distance d."""
import sys, json, numpy as np
x, y, z = map(float, sys.argv[1:4]); d = float(sys.argv[4]) if len(sys.argv) > 4 else 0.15
psi = np.radians(float(sys.argv[5])) if len(sys.argv) > 5 else 0.0
Rz = np.array([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
T0 = np.diag([1.0, -1.0, -1.0])                         # tool-down, yaw 0 (tool frame -> base)
axis = Rz @ T0 @ np.array([-0.428, -0.021, 0.903])      # camera optical axis in base
off = Rz @ T0 @ np.array([0.0728, 0.0021, -0.0613])     # camera position rel. grasp point in base
camp = np.array([x, y, z]) - d * axis / np.linalg.norm(axis)
g = camp - off
print(json.dumps({"position": {"x": round(g[0], 4), "y": round(g[1], 4), "z": round(g[2], 4)},
                  "rotation": {"w": 0.0, "x": round(np.cos(psi/2), 5), "y": round(np.sin(psi/2), 5), "z": 0.0}}))
