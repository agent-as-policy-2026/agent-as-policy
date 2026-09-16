# Robot manipulation — reset the scene between experiments (REAL robot)

You are an autonomous robot operator. A REAL 6-DOF arm with a parallel-jaw
gripper stands at a table in front of you, with a wrist RGB-D camera and a
fixed overhead RGB camera. It is live and waiting for your commands. A human
supervisor is next to the robot holding an emergency stop.

## Task

On the table there are six small cubes (about 50 mm) in two colours: three
light-blue/black and three grey/black. They are currently arranged in a
structure left by a previous experiment (some may already be loose). Your task:

> **Take the arrangement apart and lay the six cubes out individually on the
> table, each upright, none touching, at approximately the target positions
> listed in `goal/target_layout.md`.**

`goal/scatter_example.png` shows what a scattered layout looks like from this
robot's overhead camera (the positions in your target table are different).
Target coordinates are in the robot base frame (metres, cube centre resting on
the table; yaw = rotation of the cube's faces about vertical). Tolerance:
±20 mm in position, ±15° in yaw. Which physical cube goes to which target of
its colour is your choice. Study the target table and the example image FIRST.

Method constraints:

- Dismantle from the top down: always take the highest cube first; never pull a
  cube out from under another one.
- Move cubes only by lifting them with the gripper and setting them down; do not
  push, drag or sweep them.
- Every cube stays on the table and inside the region x 0.15–0.48 m,
  y −0.30–0.20 m (base frame). Nothing may fall off the table.
- Look (frames) before and after every consequential action; the gripper's
  returned opening after `close` tells you whether you are actually holding
  something.

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
   file outside this directory.
3. Budget: at most 240 counted commands and 45 minutes of wall clock.
4. If an approach fails repeatedly, change strategy rather than repeating the
   same command. There is NO way to reset the scene — this is a real robot. If
   a cube slips or ends up somewhere unexpected, re-perceive and continue from
   the cubes' new state. Never command the grasp point below the table surface.

## Deliverable

When every cube is placed — or you conclude you cannot finish — leave the
gripper open and the arm withdrawn to the observation posture, take one fresh
overhead frame, and write `scratch/SCATTER_DONE.md`: for each of the six cubes
its colour and observed (x, y), plus anything you could not do. Then stop.
