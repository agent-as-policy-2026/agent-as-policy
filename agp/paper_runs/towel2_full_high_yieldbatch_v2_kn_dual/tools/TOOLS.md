# Tools left by previous sessions (index generated from the file headers)

| file | category | purpose | usage | assumptions |
|---|---|---|---|---|
| `rc.py` | process | Call the robot CLI and print compact responses while retaining full replies in a scratch log. | `python3 knowledge/tools/rc.py left|right command '[JSON arguments]'    (run from the session directory)` | robot_client.py exists in the current session; calls remain sequential per arm |
| `step.py` | process | Execute one explicit job per arm with state snapshots, command logging, and post-action images. | `python3 knowledge/tools/step.py '<JSON jobs>' (run from the session directory)` | scratch exists; move defaults to down quaternion (0,1,0,0); caller verifies prior frames, clearance, target safety, errors, and all resulting images; no automatic collision checking; invoke with 90000 ms command yield. |
