# Tools left by previous sessions (index generated from the file headers)

| file | category | purpose | usage | assumptions |
|---|---|---|---|---|
| `geometry.py` | geometry | Batch-project base-frame points or intersect pixel rays with horizontal planes using saved camera calibration. | `python3 knowledge/tools/geometry.py <calibration.json> <camera> <ray|project> '<JSON list of triples>'    (run from the session directory)` | Requires numpy and scipy; metres, pinhole intrinsics, scalar-first pose quaternion fields; rectified images only; rays must intersect the plane and projected points must be in front of the camera. |
| `grip_capture.py` | process | Set the gripper and immediately capture both camera views for verification. | `python3 knowledge/tools/grip_capture.py open|close|fraction (run from the session directory)` | Session contains robot_client.py supporting gripper and frames commands; fractions range from zero to one. |
| `pose_and_capture.py` | process | Command a downward grasp pose with a yaw angle and capture the resulting scene. | `python3 knowledge/tools/pose_and_capture.py x y z yaw_deg [linear|plan]    (run from the session directory)` | Robot interface uses scalar-first quaternions and metres; yaw is in base-frame degrees. |
| `tilted_pose_capture.py` | process | Move to a grasp pose with yaw and tilt, then capture both cameras. | `python3 knowledge/tools/tilted_pose_capture.py x y z yaw_deg tilt_deg [linear|plan] (run from the session directory)` | Metres, base-frame yaw, tilt from downward toward positive local x, scalar-first quaternion. |
