# tool: summarize_program.py
# category: process
# purpose: Summarize measured joint and jaw response from a buffered execution report.
# usage: python3 knowledge/tools/summarize_program.py REPORT [--step SECONDS] (run from the session directory)
# inputs/outputs: Reads a returned program report JSON; prints sampled commands, measurements, velocities, and maximum errors.
# assumptions: Seven-element command, feedback and velocity arrays; arm units radians and rad/s, jaw fraction; trace clocks follow README_interface.md.
# verified: used successfully in the session that wrote it
import argparse,json
p=argparse.ArgumentParser();p.add_argument('report');p.add_argument('--step',type=float,default=.1);a=p.parse_args()
d=json.load(open(a.report)); rows=d['trace'];nxt=0
print('status',d.get('status'),'events',d.get('events'))
print('time J4_command J4_measured J4_velocity jaw_command jaw_measured')
for r in rows:
 t=r['scheduled_s']
 if t+1e-8<nxt:continue
 print(' '.join(f'{v:.4f}' for v in [t,r['command'][3],r['feedback'][3],r['velocity'][3],r['command'][6],r['feedback'][6]]))
 nxt=t+a.step
print('max_joint_tracking_errors', [round(max(abs(r['command'][j]-r['feedback'][j]) for r in rows),5) for j in range(6)])
print('max_joint_measured_speeds', [round(max(abs(r['velocity'][j]) for r in rows),5) for j in range(6)])
print('other_target_ranges', {j:round(max(r['command'][j] for r in rows)-min(r['command'][j] for r in rows),9) for j in [0,1,2,4,5]})
