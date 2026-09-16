# Robot manipulation from a demonstration video — autonomous probe (REAL robot)

You are an autonomous robot operator. A REAL 6-DOF arm with a parallel-jaw
gripper stands at a table in front of you, with a wrist RGB-D camera and a
fixed overhead RGB camera. It is live and waiting for your commands. A human
supervisor is next to the robot holding an emergency stop.

## Task

On your side of the table lie two small parts: one blue part and one white
part. The blue part fits onto or into the white part. The files in `goal/`
are a demonstration recorded with this robot's own overhead camera while a
person put the blue part onto or into the white part:

- `demo_top.mp4` — the overhead camera during the demonstration (15 fps);
- `demo_start.png` — the overhead view just before the demonstration;
- `top_camera.png` — the overhead view of the finished assembly (the goal
  state).

Your task:

> **Assemble the pair as in the demonstration: the blue part engaged with
> the white part, stable after you release it, as shown in
> `top_camera.png`.**

How the two parts sit together is defined by the demonstration; where on
the table the assembly ends up is your choice. The parts now lie separately
on the table, not in the demonstration's positions. Another robot arm works
at the far side of the table; it and the objects over there are not part of
your task — do not touch them and stay clear of them. You cannot watch a
video directly: extract the frames you need (`ffmpeg` and Python 3 with
Pillow are available) into `scratch/` and study them with your image tool.
Study the demonstration FIRST.

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
`scratch/RESULT.md`: what you understood the pairing to be, your plan, what
actually happened (with key `frames/` paths), and your own honest judgement
of success. Success means: the blue part is engaged with the white part and
stable, gripper open, arm withdrawn — confirmed in final images. Then stop.
