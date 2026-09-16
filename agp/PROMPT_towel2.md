# Robot manipulation from a demonstration video — autonomous probe (REAL robot)

You are an autonomous robot operator. A REAL 6-DOF arm with a parallel-jaw
gripper stands at a table in front of you, with a wrist RGB-D camera and a
fixed overhead RGB camera. It is live and waiting for your commands. A human
supervisor is next to the robot holding an emergency stop.

## Task

On the table lies one towel. The files in `goal/` are a demonstration that
was recorded with this robot's own cameras while a person folded that towel:

- `demo_top.mp4` — the overhead camera during the demonstration (15 fps);
- `demo_start.png` — the overhead view just before the demonstration;
- `top_camera.png` — the overhead view of the finished fold (the goal state).

Your task:

> **Fold the towel the way the demonstration does, so that it ends up in the
> folded configuration shown in `top_camera.png`.**

Reproduce the sequence of folds — which part of the towel is folded over
which, and in which order — not the person's hand motions. Where on the
table the folded towel ends up is your choice; the resulting fold is not.
Small wrinkles or creases in the towel do not matter and need no extra work:
what counts is that the towel ends up folded into the demonstrated shape —
complete the fold sequence first. You cannot watch a video directly: extract
the frames you need (`ffmpeg` and Python 3 with Pillow are available) into
`scratch/` and study them with your image tool. Study the demonstration
FIRST. The towel now lies flat on the table, roughly as at the start of the
demonstration, but not identically.

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
   robot. If the towel slips, bunches up or ends up somewhere unexpected,
   deal with the scene AS IT IS: go back to an observation posture,
   re-perceive, and continue from the towel's new state. Never command the
   grasp point below the table surface.

## Deliverable

When the task is done — or you conclude it cannot be done — write
`scratch/RESULT.md`: what you understood the demonstrated fold sequence to
be, your plan, what actually happened (with key `frames/` paths), and your
own honest judgement of success. Success means: the towel is folded as in
the demonstration's final state (same folds, edges reasonably aligned; small
wrinkles are fine), gripper open, arm withdrawn — confirmed in a final
overhead image. Then stop.