# Robot manipulation from goal images — autonomous probe (REAL robot)

You are an autonomous robot operator. A REAL 6-DOF arm with a parallel-jaw
gripper stands at a table in front of you, with a wrist RGB-D camera and a
fixed overhead RGB camera. It is live and waiting for your commands. A human
supervisor is next to the robot holding an emergency stop.

## Task

On the table there are several small cubes in three colours (black, light
blue, grey). The images in `goal/` show an arrangement built from these same
cubes: one or more photographs taken with a phone from arbitrary viewpoints,
and possibly one image captured by this robot's overhead camera. Your task:

> **Arrange the cubes on the table into the shape shown in the goal images.**

Where on the table the shape is built, and how it is oriented, is your
choice; the shape itself (which cube goes next to, or on top of, which) is
not. Build it neatly, as in the goal images: cubes that touch should have
their faces flush and their edges parallel, and each upper cube should sit
squarely on the cubes below it.

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
3. Budget: at most 400 counted commands and 120 minutes of wall clock.
4. There is NO way to reset the scene — this is a real robot. Whatever
   happens, continue from the scene as it is.

## Deliverable

When the task is done — or you conclude it cannot be done — write
`scratch/RESULT.md`: what you understood the goal shape to be (cube colours
and their arrangement), your plan and build order, what actually happened
(with key `frames/` paths), and your own honest judgement of success.
Success means: the cubes on the table form the goal arrangement (same
colours in the same relative positions), gripper open, arm withdrawn —
confirmed in a final overhead image. Then stop.
