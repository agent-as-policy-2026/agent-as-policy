
## Buffered joint programs (this session)

Three extra commands exist in this session:

| command | args (JSON) | effect / returns |
|---|---|---|
| `preview_program` | `{"program":{...}}` | validates a program against the current arm state and returns the compiled 50 Hz commands, TCP poses and velocities, the scheduled jaw events and the peak joint rates; no motion (free) |
| `run_program` | `{"program":{...}}` | executes exactly ONE authored joint/gripper program; returns `program_id`, `status`, the achieved state and `report_file` |
| `program_report` | `{"program_id":"..."}` | the full/partial per-tick report of an executed program, including faults (free) |

`program` is `{"times_s": [...], "joint_deltas_rad": [[6 floats], ...],
"gripper_events": [{"time_s": t, "fraction": f}, ...]}`:

- `times_s`: strictly increasing finite knot times, starting at 0; positive
  duration. There is no separate program cap; the session wall-clock budget
  still applies.
- `joint_deltas_rad`: one six-vector per knot; the first vector must be all
  zeros. Deltas are relative to the freshly measured arm position at
  execution. Use ordinary moves for absolute setup.
- `gripper_events`: ordered `{"time_s", "fraction"}` entries, optional
  (fraction 0 = closed … 1 = open, as for `gripper`). Before the first event
  the existing grip target is held. Each event starts movement toward its
  fraction at 0.25 fraction/s; a later event can change that target before
  the previous ramp finishes.

The executor linearly interpolates joint knots onto a uniform 50 Hz grid.
Each jaw event takes effect at the first tick at or after its requested time
(less than 20 ms quantization). Two events may not occupy the same tick.
Preview shows requested and scheduled times. Choose multiples of 0.02 s when
exact tick placement matters. The final arm knot is held while the last jaw
ramp finishes, if necessary; preview includes that extension and the actual
ramped seven-vector commands.

Rate limits: ordinary moves use 20°/s and 40°/s² for all joints. Inside a
program, J4 (index 3 of the six-joint array) may reach **180°/s and
360°/s²**; the other five joints keep 20°/s and 40°/s². Validation checks
every tick INCLUDING the transitions from and back to a held position, so
include acceleration and braking. Limits are ceilings, not target speeds.
For coordinated joint/gripper motion submit one buffered `run_program`; do
not synchronize separate CLI commands with shell sleeps.

At each tick the executor submits ONE seven-vector `[J1,...,J6, gripper]`,
reads the feedback from the motor loop, checks it, and only then waits for
the next tick. Motor messages are sequential: a seven-vector is one
coordinated software target, not a physically simultaneous motor response or
a measured object detachment. Feedback older than 100 ms, motor errors,
excess gripper effort, cancellation and loss of the heartbeat stop execution.
A program has a hard execution timeout of max(10 s, 2 × compiled duration +
5 s), including settling. Once the trajectory has finished, failing to reach
the final joint/jaw tolerance returns `status: "settle_miss"` with measured
feedback and a monitored hold.

Read and use the execution evidence. Each report
(`scratch/program_<id>_report.json`) contains the submitted program, the
measured initial position, the requested and scheduled jaw events, and a
`trace` with, per tick: command, command_accepted, dispatch/return/feedback
times, and the measured position, velocity, effort and feedback timestamp.
`command_accepted` means the actuator API accepted the buffer update; the
feedback shows the measured response. Neither command acceptance nor an
opening fraction proves physical task success.

- `completed` (`ok:true`): the program finished and the measured endpoint and
  jaw settled. Verify the task outcome visually.
- `settle_miss` (`ok:false`, `error_code:"SETTLE_MISS"`, retryable): the
  bounded program ended but the measured endpoint or jaw did not converge
  within tolerance (0.03 rad per joint, 0.02 jaw fraction) before the hard
  timeout. The whole program was dispatched and executed; only the final
  resting position missed. After a fast swing the joint typically stops a
  degree or two from its last target, so this is the expected outcome of a
  throw and says nothing about the release. The arm holds its last command.
  Inspect the report and fresh images, then decide your next action.
- `rejected` (`ok:false`, retryable): the program failed validation before
  any motion target was sent; revise it. The arm stays where it was.
- `fault` (`ok:false`, `motion_stopped:true`): execution, feedback or
  ownership failed; partial evidence is retained. Stop and report; never
  infer physical task success.

The client timeout of `run_program` grows with the program duration.

### Continuous recording of both cameras (this session)

Independently of `frames`, the robot records both cameras and the arm state
for the whole session into `bridge_rec/` in this directory (this is separate
from the third camera mentioned above, which you cannot access):

- `bridge_rec/top.mp4` (overhead, about 15 fps) and `bridge_rec/wrist.mp4`
  (wrist RGB, about 30 fps): H.264 in fragmented MP4, readable while still
  being written — `ffmpeg -i bridge_rec/wrist.mp4 ...` or `ffprobe` work at
  any time; a captured frame is on disk within about a second, so wait a
  moment after a program before reading its frames.
- `bridge_rec/top_frames.csv` and `bridge_rec/wrist_frames.csv`: one row per
  recorded frame, `frame_index,wall_time_ns,monotonic_ns` (capture time).
  Frame k of the CSV is frame k of the video.
- `bridge_rec/joints.csv`: the measured arm state at 50 Hz with the same two
  clocks (see its header).

Alignment: a program report's `trace` rows carry `dispatch_monotonic_s` and
`feedback_monotonic_ns` on the SAME monotonic clock as `monotonic_ns` in the
CSVs (`started_wall_time_ns` is on the wall clock of `wall_time_ns`). So the
video frames captured during a swing can be identified from the CSVs and
extracted with ffmpeg (e.g. by frame index, `select='eq(n\,K)'`) into
`scratch/` — that is how detachment and flight can be observed even though
`run_program` blocks and `frames` cannot run during it. Capture and encoder
timing add a few tens of milliseconds of uncertainty.
