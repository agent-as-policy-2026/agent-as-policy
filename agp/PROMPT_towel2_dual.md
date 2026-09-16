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
which, and in which order — not the person's hand motions. You have two arms
and may use one or both for any step; which arm does what is your choice.
Where on the table the folded towel ends up is your choice; the resulting
fold is not. Small wrinkles or creases in the towel do not matter and need
no extra work: what counts is that the towel ends up folded into the
demonstrated shape — complete the fold sequence first. You cannot watch a
video directly: extract the frames you need (`ffmpeg` and Python 3 with
Pillow are available) into `scratch/` and study them with your image tool.
Study the demonstration FIRST.

Two arms are yours. Use them the way the demonstration uses two hands where
that helps: both can move at the same time (see `README_interface.md`), one
arm can hold an edge while the other folds, and two corners can be grasped and
carried together so the cloth stays taut. Each arm only reaches its own side of
the table (its envelope is measured from its own base): when a point is beyond
one arm's reach, let the other arm do that part, or have the holding arm set the
cloth down where the other arm can grasp it and continue from there. Keep the
two arms clear of each other at all times — nothing stops them from colliding. The towel now lies
flat on the table, roughly as at the start of the demonstration, but not
identically.

## Interface

Read `README_interface.md` in this directory, then interact with the robots
only through:

```
python3 robot_client.py . [--arm left|right] <command> ['<json-args>']
```

`--arm` omitted = the left arm. Both arms use one world frame (the left arm's
base); the overhead camera is served by the left arm.

## Rules

1. Modify files ONLY inside this session directory. Put every script, note
   and extracted frame you create under `scratch/`.
2. Interact with the robots ONLY via `robot_client.py` and the files it
   returns (`frames/…`, `frames_right/…`). Experiment-integrity rule: do NOT
   read the robot software's source code or any file outside this directory
   — everything you legitimately need is in here. Violating this voids the
   experiment.
3. Budget: at most 400 counted commands per arm and 120 minutes of wall
   clock. Look (frames) before and after every consequential action and
   verify visually; the gripper's returned opening fraction after `close`
   tells you whether you are actually holding something.
4. If an approach fails repeatedly, change strategy rather than repeating
   the same command. There is NO way to reset the scene — these are real
   robots. If the towel slips, bunches up or ends up somewhere unexpected,
   deal with the scene AS IT IS: go back to an observation posture,
   re-perceive, and continue from the towel's new state. Never command a
   grasp point below the table surface.
5. The two arms share the workspace and nothing in the system keeps them
   apart: before moving an arm, know where the other one is (`state`), and
   never let the arms or their grippers touch each other or what the other
   arm is holding.

## Deliverable

When the task is done — or you conclude it cannot be done — write
`scratch/RESULT.md`: what you understood the demonstrated fold sequence to
be, your plan (including which arm did what), what actually happened (with
key `frames/` and `frames_right/` paths), and your own honest judgement of
success. Success means: the towel is folded as in the demonstration's final
state (same folds, edges reasonably aligned; small wrinkles are fine), both
grippers open, both arms withdrawn — confirmed in a final overhead image.
Then stop.