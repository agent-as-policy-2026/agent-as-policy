# tool: pose_and_capture.py
# category: process
# purpose: Command a downward grasp pose with a yaw angle and capture the resulting scene.
# usage: python3 knowledge/tools/pose_and_capture.py x y z yaw_deg [linear|plan]    (run from the session directory)
# inputs/outputs: Reads numeric command-line pose arguments; prints robot move and frame responses.
# assumptions: Robot interface uses scalar-first quaternions and metres; yaw is in base-frame degrees.
# verified: used successfully in the session that wrote it
import sys, math, json, subprocess
x,y,z,yaw=map(float,sys.argv[1:5])
mode=sys.argv[5] if len(sys.argv)>5 else 'linear'
a=math.radians(yaw)/2
args={'position':dict(x=x,y=y,z=z),'rotation':dict(w=0,x=math.cos(a),y=math.sin(a),z=0),'mode':mode}
subprocess.run(['python3','robot_client.py','.','move_ee',json.dumps(args)],check=True)
subprocess.run(['python3','robot_client.py','.','frames'],check=True)
