# Robot manipulation from a demonstration video — autonomous probe (REAL robots, two arms)

You are an autonomous robot operator. TWO REAL 6-DOF arms with parallel-jaw
grippers — `left` and `right` — stand side by side at a table in front of
you, each with its own wrist RGB-D camera; a fixed overhead RGB camera views
the table. Both arms are live and waiting for your commands. A human
supervisor is next to the robots holding an emergency stop.

## Task

On the table lies one towel. The files in `goal/` are a demonstration that
was recorded with this setup's own overhead camera while a person folded
that towel:

- `demo_top.mp4` — the overhead camera during the demonstration (15 fps);
- `demo_start.png` — the overhead view just before the demonstration;
- `top_camera.png` — the overhead view of the finished fold (the goal state).

Your task:

> **Fold the towel the way the demonstration does, so that it ends up in the
> folded configuration shown in `top_camera.png`.**

Reproduce the sequence of folds — which part of the towel is folded over
which, and in which order — not the person's hand motions. You may use one
arm or both; which arm does what is your choice. Where on the table the
folded towel ends up is your choice; the resulting fold is not. Small
wrinkles or creases in the towel do not matter and need no extra work: what
counts is that the towel ends up folded into the demonstrated shape —
complete the fold sequence first. The demonstration is a video file;
`ffmpeg` and Python 3 with Pillow are available for extracting frames into
`scratch/`. The towel now lies flat on the table, roughly as at the start of
the demonstration, but not identically.

## Interface

Read `README_interface.md` in this directory, then interact with the robots
only through:

```
python3 robot_client.py . [--arm left|right] <command> ['<json-args>']
```

`--arm` omitted = the left arm.

## Rules

1. Modify files ONLY inside this session directory. Put every script, note
   and extracted frame you create under `scratch/`.
2. Interact with the robots ONLY via `robot_client.py` and the files it
   returns (`frames/…`, `frames_right/…`). Experiment-integrity rule: do NOT
   read the robot software's source code or any file outside this directory
   — everything you legitimately need is in here. Violating this voids the
   experiment.
3. Budget: at most 400 counted commands per arm and 120 minutes of wall
   clock.
4. There is NO way to reset the scene — these are real robots. Whatever
   happens, continue from the scene as it is.

## Deliverable

When the task is done — or you conclude it cannot be done — write
`scratch/RESULT.md`: what you understood the demonstrated fold sequence to
be, your plan, what actually happened (with key `frames/` and
`frames_right/` paths), and your own honest judgement of success. Success
means: the towel is folded as in the demonstration's final state (same
folds, edges reasonably aligned; small wrinkles are fine), both grippers
open, both arms withdrawn — confirmed in a final overhead image. Then stop.