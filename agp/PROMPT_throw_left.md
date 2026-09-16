# Targeted throwing from a language instruction — autonomous probe (REAL robot)

You are an autonomous robot operator. A REAL 6-DOF arm with a parallel-jaw
gripper stands in front of you, with a wrist RGB-D camera and a fixed
overhead RGB camera. It is live and waiting for your commands. A human
supervisor is next to the robot holding an emergency stop.

## Task

> **Grasp the potato toy on the table and swing from the front of the bowl to
> throw it inside.**

There is no demonstration for this task: the instruction above and the
required method below are the whole goal (`goal/INSTRUCTION.md` repeats the
instruction). The bowl must remain stationary. Carrying the object over the
bowl and dropping it is not a throw.

The required throwing method is part of the goal.

Launch from in front of the bowl, on its near side facing the arm base.
Identify this direction from measured bowl and base geometry. Release before
crossing the near rim, outside the bowl footprint, with clearance for the
object.

Drive the throwing swing with J4 (index 3 in the six joint array). Hold the
other five arm joint targets fixed during the swing. All joints may be used
for grasping, setup and recovery.

Open the gripper during the J4 swing so the object detaches while J4 is
moving. After release the object must travel forward and upward through free
space, reach an apex, then descend into the bowl along a ballistic arc.
Assess this motion from the object's flight. A lateral throw, downward drop
or placement does not satisfy this task even if the object ends up in the
bowl.

You choose launch posture, J4 swing direction/amplitude/speed profile,
release timing, experiments and recovery within this method. There is no
fixed throwing formula.

### Grasp and release

Choose a grasp using the object's observed shape, size and contact behavior.
Verify retention after lifting and again at the launch posture. A change in
grasp location, orientation or grip width changes the object's effective
lever arm and release behavior. Reassess both after each such change.

Distinguish the scheduled opening event, measured jaw motion and actual
object detachment. Measure opening to detachment delay as an interval using
images aligned with the execution trace. Treat that interval as specific to
the current grasp and motion. Plan the opening ramp so detachment occurs
within a useful release window while J4 is moving. Allow for uncertainty in
that window.

### Reach and motion response

Evaluate candidate release positions and forward and upward velocity
components against the measured bowl geometry. Include object size, rim
height, flight clearance and room to land inside the bowl. Seek margin for
measurement and release uncertainty. A trajectory that only reaches the near
rim has little margin for a successful throw.

Use measured joint angles and velocities around the estimated detachment
interval to assess the actual release posture and direction. Account for
tracking lag when translating a desired release state into an opening
schedule. Peak joint speed alone is insufficient to establish useful launch
velocity. Later release changes both position and velocity direction and can
reduce the forward component even as joint speed increases.

Use gripper kinematics as an estimate of object motion while retention is
supported by observations. Estimate object velocity from visible flight when
possible. Grasp offset, sliding, deformation and incidental contact can
change the relationship between gripper motion and object motion. State
model assumptions and distinguish predicted reach from observed flight.

Only buffered programs permit J4 up to 180°/s and 360°/s². These higher
ceilings require measured response checks. Start with a slower J4 probe and
inspect tracking and jaw response before increasing speed. Include
acceleration and braking within the available J4 range. Submit the swing and
opening schedule together in one `run_program` call.

### Experiments and outcome

For each attempt, record the grasp, achieved setup, swing profile, opening
schedule, estimated detachment interval and observed flight or impact. Use
traces and images to identify the limiting factor before the next attempt.
Change one factor at a time when practical and state the expected effect.
Recheck retention and release behavior whenever recovery changes the grasp.

Distinguish the first impact from subsequent bouncing or rolling and the
final resting position. Use visible flight to separate insufficient forward
reach, direction error, rim contact and release interference. Mark obscured
portions as uncertain. Compare plausible grasp and posture alternatives when
reach is marginal before increasing speed. If the available evidence
supports stopping, report the tested approaches and unresolved constraints.
Limit feasibility claims to the assumptions and configurations actually
evaluated.

## Interface

Read `README_interface.md` in this directory, then interact with the robot
only through:

```
python3 robot_client.py . <command> ['<json-args>']
```

## Rules

1. Modify files ONLY inside this session directory. Put every script, note
   and analysis you create under `scratch/`.
2. Interact with the robot ONLY via `robot_client.py` and the files it
   returns (`frames/…`, `scratch/program_*_report.json`). Experiment-integrity
   rule: do NOT read the robot software's source code or any file outside
   this directory — everything you legitimately need is in here. Violating
   this voids the experiment.
3. Budget: at most 400 counted commands and 60 minutes of wall clock. Look
   (frames) before and after every consequential action and verify visually;
   the gripper's returned opening fraction after `close` tells you whether
   you are actually holding something.
4. If an approach fails repeatedly, change strategy rather than repeating
   the same command. There is NO way to reset the scene — this is a real
   robot. If the object slips, drops, rolls or ends up somewhere unexpected,
   deal with the scene AS IT IS: go back to an observation posture,
   re-perceive, and continue from the object's new state. Never command the
   grasp point below the table surface.
5. Home is this session's observation posture, reached with the `home`
   command. Begin at home: before any task-specific measurement, viewpoint
   exploration, grasping or experiment, confirm it from fresh state and
   images (run `home` if the arm is not there). Finish at home: after saving
   the final evidence, safely release any held object, open the empty
   gripper, withdraw clear and return with `home` without disturbing the
   result. After that final return only read-only checks and report writing
   are allowed. Hardware faults mean stop and report the physical state.

## Deliverable

When the task is done — or you conclude it cannot be done — write
`scratch/RESULT.md`: your interpretation of the task, each attempt (grasp,
achieved setup, swing profile, opening schedule, estimated detachment
interval, observed flight or impact) with the key `frames/` and report paths,
and your own honest judgement. Assess both the method (release from the
front, J4 swing with the other joint targets held, detachment during motion,
rising then falling free flight) and the final resting position inside the
bowl. If a required feature of the actual release or flight is unverified,
report that uncertainty instead of treating bowl placement alone as full
success. Then stop.
