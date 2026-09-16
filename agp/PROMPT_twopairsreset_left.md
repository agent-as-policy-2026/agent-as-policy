# Robot manipulation from a demonstration video — autonomous probe (REAL robot)

You are an autonomous robot operator. A REAL 6-DOF arm with a parallel-jaw
gripper — one of two arms at this table — stands in front of you, with a wrist RGB-D camera and a
fixed overhead RGB camera. It is live and waiting for your commands. A human
supervisor is next to the robot holding an emergency stop.

## Task

Two assembled pairs — each a blue part engaged with a white part — stand on
the table, one set for each of two robot arms. **You are the LEFT arm**;
your set is the one on the left half of the overhead image, nearest your own
base. The files in `goal/` are a demonstration recorded with this robot's
own overhead camera while a person took the two pairs of one set apart and
laid the four parts out separately:

- `demo_top.mp4` — the overhead camera during the demonstration (15 fps);
- `demo_start.png` — the overhead view just before the demonstration;
- `top_camera.png` — the overhead view after the demonstration: the four parts
  lying separately (an example of the target state).

Your task:

> **Take both pairs of your set apart and lay the four parts out separately
> on your half of the table: every blue part fully disengaged from its white
> part, no part touching another, no two parts closer than 8 cm, each part
> resting stably on the table in the orientation shown in `top_camera.png`,
> all within your own half.**

How you separate a pair and where exactly on your half the parts end up is
your choice; the positions in `top_camera.png` are an example, not a
requirement. The right arm works on the other set at the same time; it and
that set are not part of your task — never touch them, stay clear of the
right arm, and keep everything within your own half of the table. You cannot
watch a video directly: extract the frames you need (`ffmpeg` and Python 3
with Pillow are available) into `scratch/` and study them with your image
tool. Study the demonstration FIRST.

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
`scratch/RESULT.md`: how you took each pair apart, where you laid the
parts, what actually happened (with key `frames/` paths), and your own
honest judgement of success. Success means: all four parts of your set lie
separately and stably on your half, none engaged with or touching another,
no two closer than 8 cm, gripper open, arm withdrawn — confirmed in final
images. Then stop.
