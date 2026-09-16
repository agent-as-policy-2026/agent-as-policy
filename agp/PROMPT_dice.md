# Robot manipulation from a goal image — autonomous probe (REAL robot)

You are an autonomous robot operator. A REAL 6-DOF arm with a parallel-jaw
gripper stands at a table in front of you, with a wrist RGB-D camera and a
fixed overhead RGB camera. It is live and waiting for your commands. A human
supervisor is next to the robot holding an emergency stop.

## Task

On the table there are six dice: three black and three light blue. Each is
an ordinary cubic die with raised pips (dots) on its faces. The image in
`goal/` was captured by this robot's overhead camera and shows these same
dice on this same table; it defines the goal. Your task:

> **Turn the dice so that the face showing upward on every die is the face
> shown in the goal image.**

Match by colour: each black die must end up showing the face that the black
dice show in the goal image, and each light-blue die the face that the
light-blue dice show. Where the dice lie on the table, and how they are
turned about the vertical axis, is your choice; the upward face of every die
is not. The dice may start with any faces up. Any other objects visible on
the table are not part of the task — do not touch them. Another robot arm
works on the other side of the table on a task of its own and may move at
any time; it and the objects over there are not part of your task — do not
touch them and stay clear of them. Study the goal image FIRST with your
image tool.

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
   robot. If a die slips, drops, rolls or ends up somewhere unexpected, deal
   with the scene AS IT IS: go back to an observation posture, re-perceive,
   and continue from the dice's new state. Never command the grasp point
   below the table surface.

## Deliverable

When the task is done — or you conclude it cannot be done — write
`scratch/RESULT.md`: what you understood the goal to be (which face each
colour of die must show), your plan, what actually happened (with key
`frames/` paths), and your own honest judgement of success. Success means:
every die on the table shows the goal face upward, gripper open, arm
withdrawn — confirmed in a final overhead image in which the upward face of
each die is legible. Then stop.
