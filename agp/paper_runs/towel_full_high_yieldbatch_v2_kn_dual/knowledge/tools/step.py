# tool: step.py
# category: process
# purpose: Execute one explicit job per arm with state snapshots, command logging, and post-action images.
# usage: python3 knowledge/tools/step.py '<JSON jobs>' (run from the session directory)
# inputs/outputs: Reads argument jobs [[arm,command,args],...] or [arm,"move",[x,y,z],[w,x,y,z]]; prints results and frame paths; appends scratch/robot_calls.jsonl.
# assumptions: scratch exists; move defaults to down quaternion (0,1,0,0); caller verifies prior frames, clearance, target safety, errors, and all resulting images; no automatic collision checking; invoke with 90000 ms command yield.
# verified: used successfully in the session that wrote it
import json,subprocess,sys,concurrent.futures
from pathlib import Path
def call(arm,cmd,args=None):
 p=['python3','robot_client.py','.', '--arm',arm,cmd]
 if args is not None:p.append(json.dumps(args))
 r=json.loads(subprocess.check_output(p,text=True))
 with open('scratch/robot_calls.jsonl','a') as f:f.write(json.dumps(dict(arm=arm,command=cmd,args=args,reply=r))+'\n')
 if cmd=='frames' and r.get('ok'):r={'ok':True,'capture':r['capture'],'files':{k:str(Path(v['rgb']).relative_to(Path.cwd())) for k,v in r['files'].items()}}
 if cmd=='state':r={k:v for k,v in r.items() if k in ('ee_pose','gripper_fraction','gripper_width_m','ok')}
 print(arm,cmd,json.dumps(r),flush=True)
 return r
jobs=json.loads(sys.argv[1])
for a in ['left','right']:call(a,'state')
def run(j):
 a,c,*v=j
 if c=='move':
  xyz=v[0];q=v[1] if len(v)>1 else [0,1,0,0]
  return call(a,'move_ee',{'position':dict(zip('xyz',xyz)),'rotation':dict(zip(['w','x','y','z'],q))})
 return call(a,c,v[0] if v else None)
assert len(set(j[0] for j in jobs))==len(jobs),'one command per arm'
with concurrent.futures.ThreadPoolExecutor() as ex:list(ex.map(run,jobs))
call('left','frames')
if any(j[0]=='right' for j in jobs):call('right','frames')
