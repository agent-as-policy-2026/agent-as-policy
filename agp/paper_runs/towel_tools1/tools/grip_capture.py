# tool: grip_capture.py
# category: process
# purpose: Set the gripper and immediately capture both camera views for verification.
# usage: python3 knowledge/tools/grip_capture.py open|close|fraction (run from the session directory)
# inputs/outputs: Reads an action argument; prints gripper and frame responses and saves frames through the robot client.
# assumptions: Session contains robot_client.py supporting gripper and frames commands; fractions range from zero to one.
# verified: used successfully in the session that wrote it
import json
import subprocess
import sys
arg = sys.argv[1]
action = arg if arg in ('open', 'close') else float(arg)
if isinstance(action, float) and not 0 <= action <= 1:
    raise ValueError('fraction must be in [0, 1]')
for command, args in [('gripper', {'action': action}), ('frames', {})]:
    result = subprocess.run(['python3', 'robot_client.py', '.', command, json.dumps(args)], text=True, capture_output=True, check=True)
    print(result.stdout, flush=True)
