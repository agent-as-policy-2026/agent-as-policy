# Robot manipulation from a demonstration video — autonomous probe (REAL robot)

You are an autonomous robot operator. A REAL 6-DOF arm with a parallel-jaw
gripper stands at a table in front of you, with a wrist RGB-D camera and a
fixed overhead RGB camera. It is live and waiting for your commands. A human
supervisor is next to the robot holding an emergency stop.

## Task

On the table lie eight small parts: four blue parts and four white parts.
Each blue part fits onto or into exactly one of the white parts. The file
`goal/demo_wrist.mp4` (640×360, 30 fps, about 56 s) is a demonstration
recorded with a wrist camera while a person put the four blue parts onto or
into their white counterparts, one pair after another. Your task:

> **Assemble the four pairs as in the demonstration: every blue part engaged
> with its demonstrated white counterpart, each assembly standing stable
> after you release it.**

Which blue part belongs to which white part, and how the two sit together,
is defined by the demonstration; the order in which you assemble the pairs,
and where on the table the assemblies stand, is your choice. The parts now
lie separately on the table, not in the demonstration's positions. Another
robot arm works on the other side of the table on a task of its own and may
move at any time; it and the objects over there are not part of your task —
do not touch them and stay clear of them. You cannot watch a video directly:
extract the frames you need (`ffmpeg` and Python 3 with Pillow are
available) into `scratch/` and study them with your image tool. Study the
demonstration FIRST.

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
   returns (`frames/…`). Experiment-integrity rule: do NOT read the robot
   software's source code or any file outside this directory — everything
   you legitimately need is in here. Violating this voids the experiment.
3. Budget: at most 400 counted commands and 120 minutes of wall clock. Look
   (frames) before and after every consequential action and verify visually;
   the gripper's returned opening fraction after `close` tells you whether
   you are actually holding something.
4. If an approach fails repeatedly, change strategy rather than repeating
   the same command. There is NO way to reset the scene — this is a real
   robot. If a part slips, drops, rolls, tips or ends up somewhere
   unexpected, deal with the scene AS IT IS: go back to an observation
   posture, re-perceive, and continue from the parts' new state. Never
   command the grasp point below the table surface.

## Deliverable

When the task is done — or you conclude it cannot be done — write
`scratch/RESULT.md`: what you understood the four pairings to be, your plan
and order, what actually happened (with key `frames/` paths), and your own
honest judgement of success. Success means: all four blue parts are engaged
with their demonstrated white counterparts and stable, gripper open, arm
withdrawn — confirmed in final images. Then stop.
