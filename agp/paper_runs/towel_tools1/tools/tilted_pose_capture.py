# tool: tilted_pose_capture.py
# category: process
# purpose: Move to a grasp pose with yaw and tilt, then capture both cameras.
# usage: python3 knowledge/tools/tilted_pose_capture.py x y z yaw_deg tilt_deg [linear|plan] (run from the session directory)
# inputs/outputs: Reads pose arguments; prints robot movement and capture responses.
# assumptions: Metres, base-frame yaw, tilt from downward toward positive local x, scalar-first quaternion.
# verified: used successfully in the session that wrote it
import sys, math, json, subprocess
x,y,z,yaw,tilt=map(float,sys.argv[1:6])
a,b=math.radians(yaw)/2,math.radians(tilt)/2
args={'position':dict(x=x,y=y,z=z),'rotation':dict(w=-math.sin(a)*math.sin(b),x=math.cos(a)*math.cos(b),y=math.sin(a)*math.cos(b),z=math.cos(a)*math.sin(b)),'mode':sys.argv[6] if len(sys.argv)>6 else 'linear'}
p=subprocess.run(['python3','robot_client.py','.','move_ee',json.dumps(args)],capture_output=True,text=True,check=True)
print(p.stdout,flush=True)
subprocess.run(['python3','robot_client.py','.','frames'],check=True)
