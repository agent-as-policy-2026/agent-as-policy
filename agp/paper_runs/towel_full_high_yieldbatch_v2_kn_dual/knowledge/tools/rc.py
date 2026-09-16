# tool: rc.py
# category: process
# purpose: Call the robot CLI and print compact responses while retaining full replies in a scratch log.
# usage: python3 knowledge/tools/rc.py left|right command '[JSON arguments]'    (run from the session directory)
# inputs/outputs: CLI arm, command and optional JSON; prints compact JSON and appends scratch/robot_calls.jsonl
# assumptions: robot_client.py exists in the current session; calls remain sequential per arm
# verified: used successfully in the session that wrote it
import json, subprocess, sys
from pathlib import Path
arm, command = sys.argv[1:3]
if arm not in ('left', 'right'): raise SystemExit('arm must be left or right')
args=sys.argv[3:]
if args: json.loads(args[0])
r=subprocess.run(['python3','robot_client.py','.', '--arm',arm,command]+args,capture_output=True,text=True)
try: reply=json.loads(r.stdout)
except Exception:
 print(r.stdout); print(r.stderr); raise SystemExit(r.returncode or 1)
Path('scratch').mkdir(exist_ok=True)
with open('scratch/robot_calls.jsonl','a') as f: f.write(json.dumps({'arm':arm,'command':command,'args':args,'reply':reply})+'\n')
if command=='frames' and reply.get('ok'):
 reply={'ok':True,'arm':arm,'capture':reply['capture'],'files':{k: str(Path(v['rgb']).relative_to(Path.cwd())) for k,v in reply['files'].items()}}
elif command=='state':
 reply={k:v for k,v in reply.items() if k in ('ok','ee_pose','gripper_fraction','gripper_width_m','health')}
print(json.dumps(reply))
