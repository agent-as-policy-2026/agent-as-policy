# Real-robot perception rehearsal — OBSERVATION ONLY

You are an autonomous robot operator connected to a REAL 6-DOF arm with a
wrist RGB-D camera and a fixed overhead RGB camera. In THIS session the arm
is observation-only: every motion command (`move_ee`, `move_delta`,
`move_joints`, `home`, `gripper`) is refused with `READ_ONLY`. Do not try a
motion command more than once. The arm may be parked in any posture; you
work with whatever the two cameras currently see.

## Task

On the table in front of the robot there is ONE **light-blue cube**. Report,
in the robot base frame:

1. the cube's centre (x, y, z) in metres,
2. its edge length in metres,
3. its rotation about the vertical axis (yaw, degrees, any consistent convention),

and do it TWICE, independently:

- **(a) wrist camera** — use `frames` + `deproject` with the wrist depth on
  the cube's top face and edges (several pixels; the top face is at
  z ≈ table z + edge).
- **(b) top camera** — the overhead camera has no depth: use `deproject` with
  `"plane_z"`: the table plane (z = -0.045) for the cube's footprint outline
  seen from above, and `plane_z = -0.045 + your edge estimate` for the top
  face centre.

Then state the disagreement between (a) and (b) in metres and which one you
trust more and why. If the cube is not visible in the wrist image, say so
and deliver the top-camera estimate only.

## Interface

Read `README_interface.md` in this directory, then interact with the robot
only through:

```
python3 robot_client.py . <command> ['<json-args>']
```

## Rules

1. Modify files ONLY inside this session directory. Put every script, note
   and extracted frame you create under `scratch/`.
2. Interact with the robot ONLY via `robot_client.py` and the files it
   returns (`frames/…`). Do NOT read the robot software's source code or any
   file outside this directory — everything you legitimately need is in here.
3. Budget: at most 40 counted commands and 30 minutes of wall clock. Look at
   every image you capture with your image tool before using its pixels.

## Deliverable

Write `scratch/RESULT.md`: the captures and pixels you used, both estimates,
their disagreement, and your judgement. End it with a JSON block:

```json
{"cube_center_base": [x, y, z], "edge_m": e, "yaw_deg": d,
 "wrist_estimate": {"center": [x, y, z], "edge_m": e},
 "top_estimate": {"center": [x, y, z], "edge_m": e},
 "disagreement_m": r, "trusted": "wrist" | "top"}
```

Then stop.
