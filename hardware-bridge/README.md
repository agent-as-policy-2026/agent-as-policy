# AgP YAM hardware bridge

This project is the independent Python/uv environment for the YAM hardware
bridge. It is a separate process and the only process that owns the i2rt robot
and the two cameras of an arm station. It starts read-only by default; motion
is available only with the explicit motion flag, and real hardware additionally
requires acknowledgement of the physical first-motion checklist.

Its `i2rt` dependency is editable and resolves to `../i2rt`, i.e. the patched
checkout next to this directory in the same repository (see
`third_party/i2rt/UPSTREAM.md` for how to recreate it). Never repoint it at
another i2rt on the machine: a `graph-as-policy` clone with its submodules
carries a second copy at `third_party/robots_realtime/dependencies/i2rt`, which
is not our patched tree, so its `git diff HEAD` will not match the config's
`tracked_diff_sha256` and preflight will refuse to serve. The connector
vendored here under `third_party/graph-as-policy/` contains no i2rt at all —
only the `gap` / `gap_core` modules.

Clients talk to the bridge over loopback TCP with the msgpack wire contract at
the end of this file:

* the experiment harness in `agp/` — its session server reaches the bridge
  through the connector package vendored at `third_party/graph-as-policy/`,
  which this project declares as an editable path dependency, so a plain
  `uv sync` here installs it into `.venv` and the harness runs on that
  interpreter;
* the calibration tools in `calib/`, which use the bridge's own client and its
  `agp-yam-camera-acceptance` command;
* the bridge's own test suite, against `--source fake`.

P2 reuses i2rt's own `Kinematics` class and
`combine_arm_and_gripper_xml(ArmType.YAM, GripperType.LINEAR_4310)`. The
target is the combined model's real `grasp_site`; no motor IDs, joint order,
`link_6` TCP offset, or IK implementation is copied into this project.

## Environment and preflight

```bash
cd hardware-bridge
uv sync --locked
uv run --locked agp-yam-preflight --config config/left_arm.yaml
```

The preflight imports `get_yam_robot` to verify provenance but never calls it.
It does not construct a robot, open a CAN socket, enable a motor, or start the
bridge. It reads git/package/model metadata, `ip -json -details link`, and
`rs-enumerate-devices`, compares them with the configuration, and loads the
top-camera calibration artifact. It prints `status: READY` plus the complete
startup report (i2rt revision and tracked-diff sha, CAN channel and bitrate,
motor offsets, both camera streams, station model, home joints, workspace,
speed limits and safety thresholds) only when every observed fact matches. Any
missing probe or mismatch prints `status: BLOCKED` to stderr and exits nonzero.

Without `--config` the preflight and every other `agp-yam-*` command load
`config/first_acceptance.yaml` (see below), which is not this rig's
configuration — always pass `--config`.

## Configurations

| file | role |
|---|---|
| `config/left_arm.yaml` | **runtime**, left arm: `127.0.0.1:9021`, `can_follower_l`, wrist D405 `353322271204` at 640x360, overhead BRIO `178B0DAE` at 1920x1080 through `acceptance/top_left/top_brio_calibration.json`, poses in `left_base`, action log `../logs/actions.jsonl` |
| `config/left_arm_throw.yaml` | **runtime**, left arm, throwing sessions: `left_arm.yaml` plus a three-line preamble and the two keys `acceptance.speed_limits.program_j4_velocity_deg_s: 180` / `program_j4_acceleration_deg_s2: 360`, which is what enables buffered joint programs |
| `config/right_arm.yaml` | **runtime**, right arm: `127.0.0.1:9022`, `can_follower_r`, wrist D405 `353322271910` at 640x360, overhead BRIO `B8C7F203` at 1920x1080 through `acceptance/top/top_brio_calibration.json`, poses in `right_base`, action log `../logs/actions_right.jsonl` |
| `config/first_acceptance.yaml` | **package default / reference**: what the console scripts load without `--config`, what `FakeYamSource()` builds from, and the schema fixture of the test suite. It describes a single right-hand station on port 9020 and pins the P4 first-motion speed and safety envelope; its workspace is kept in step with the runtime configs |
| `config/left_calib.yaml`, `config/right_calib.yaml` | **offline only**: the hardware identity the `agp-yam-camera-acceptance` solvers need for captures of either rig. Never serve them; port 9020 in `right_calib.yaml` belongs to another bridge |

The two calibration artifacts under `acceptance/` are the only ones a runtime
config loads, and bridge startup verifies each one's schema, `PASS` status,
serial-bound by-id device path, stream profile, pinhole matrix, distortion
coefficients and rigid world pose before serving. They are produced and
installed by the procedures in `calib/README_runbook_zh.md`.

## Starting the bridge

The harness starts both arms from the repository root:

```bash
bash agp/start_bridges.sh start left          # left arm, 9021
bash agp/start_bridges.sh start both          # left and right
bash agp/start_bridges.sh start left throw    # left arm with config/left_arm_throw.yaml
bash agp/start_bridges.sh status
bash agp/start_bridges.sh stop left           # SIGINT: gravity comp, then motors off
```

`start` restores the overhead BRIO's calibrated focus state, refuses an arm
whose port is already taken, launches `uv run --locked agp-yam-bridge` with
`cwd=hardware-bridge` in its own session with SIGINT restored to default, logs
to `agp/bridge_logs/bridge_<arm>_<ts>.log`, and waits up to 150 s for
`status: SERVING`. `stop` refuses while a session is running on that arm's
bridge.

The equivalent command by hand (from `hardware-bridge`, relative paths work
because the bridge keeps that working directory):

```bash
uv run --locked agp-yam-preflight --config config/left_arm.yaml
uv run --locked agp-yam-bridge \
  --config config/left_arm.yaml \
  --source i2rt \
  --acknowledge-i2rt-startup-motion \
  --enable-motion \
  --acknowledge-first-motion-checklist \
  --record-dir ../agp/bridge_recordings
```

> Hardware warning: the network API can be read-only, but i2rt
> `get_yam_robot(sim=False)` enables the motor chain during construction,
> sends its startup hold, and may calibrate/move `linear_4310`. Clear the
> workspace, prepare the emergency stop, and make sure no other process owns
> the CAN channel before acknowledging startup. Leaving out `--enable-motion`
> and `--acknowledge-first-motion-checklist` gives a read-only bridge, which
> still performs that startup motion.

Drop `--config` only if you intend `config/first_acceptance.yaml`. `--host` and
`--port` override the configured endpoint; the TCP listener is bound before
hardware construction.

On success the bridge prints one banner:

```text
status: SERVING
endpoint: 127.0.0.1:9021
mode: motion-enabled
recording: armed (SIGUSR1/SIGUSR2, pid N, videos + joints.csv @50 Hz)
joint programs: enabled (J4 up to 180 deg/s, 360 deg/s^2; other joints and ordinary moves 20/40)
```

`mode` is `read-only` without `--enable-motion`, `recording` is `off` without
`--record-dir`, and the `joint programs` line appears only for a config that
names program limits. `start_bridges.sh` and the harness both grep this banner,
so the strings are part of the interface.

If i2rt fails after installing its startup hold, the bridge reports
`status: STARTUP_HOLD` and retains the live chain until an operator supports
the arm and presses Ctrl-C to release it; it does not abandon that hardware
resource while unwinding. A normal Ctrl-C cancels/idles the controller, then
closes the cameras and the robot.

## No-hardware contract run

The production server runs against a deterministic fake source, without CAN or
cameras:

```bash
uv run --locked agp-yam-bridge --config config/left_arm.yaml --source fake
uv run --locked agp-yam-bridge --config config/left_arm.yaml --source fake --enable-motion \
  --host 127.0.0.1 --port 9041
```

This is the same path the test suite exercises, and how the harness's dual-arm
mode was validated before it ever ran on hardware. Motion mode against the fake
source exercises the complete action/watchdog path; its gripper reports stalls
and most IK solves fail, which is a limitation of the fake data, not the gate.

## Run recording

`--record-dir DIR` arms the recorder. A dedicated thread per camera becomes the
only hardware reader: it captures continuously, keeps the latest frame for
observations (freshness gates unchanged) and, while a recording is active,
pipes every frame to ffmpeg.

* SIGUSR1 starts a recording, SIGUSR2 stops it. Both handlers are installed
  even without `--record-dir`, because the default action of either signal
  would terminate the bridge mid-session.
* The output directory is the single absolute path in `<DIR>/target.txt` when
  that file exists, else `<DIR>/<timestamp>/`. The harness writes `target.txt`
  so a recording lands inside the session directory.
* Each recording writes one mp4 per camera, `joints.csv` (50 Hz: wall and
  monotonic time, sequence, q0-q6, v0-v6, eff0-eff6, with q6 the gripper
  fraction) and `<stream>_frames.csv` (wall/monotonic capture time of every
  frame handed to ffmpeg), which is what aligns video frames with joint rows.
* `--record-live-readable` writes fragmented MP4 with a keyframe about every
  second, so the file can be decoded while it is still being written. Used by
  throwing sessions, where the agent has to find the release moment in the
  growing recording.

## Motion mode

Motion mode adds a bridge-owned 50 Hz dispatcher. Every arm trajectory is
checked against model joint limits, the configured 0.5 rad waypoint-jump limit,
the configured velocity and acceleration caps, and the complete
`yam_base`/`grasp_site` workspace path before the first motor command. Absolute
joint targets are converted to minimum-jerk trajectories using the same
configured joint limits. Cartesian targets are world-frame `grasp_site` poses,
interpolated within the configured Cartesian limits, solved with the current
i2rt model, and then passed through the same joint trajectory gate.

Joint/home motion uses the available timeout to settle and records any residual
joint-position error, but a final joint tolerance miss alone is not a motion
fault. Cartesian completion is independent: measured FK must reach the
configured translation and orientation tolerance, so a small joint error cannot
falsely pass a small TCP move. To remove the measured PD steady-state error
without changing the established joint/home behavior, Cartesian settling uses a
bounded outer correction with the configured gain and command-bias cap. Every
correction tick remains subject to the configured velocity, acceleration,
joint-limit and workspace gates. Gripper commands preserve the six measured arm
joints and are limited by the configured fraction rate.

The application watchdog requires a matching action-lease heartbeat every
`safety.command_heartbeat_timeout_s`. A timeout, explicit cancellation, stale
motor feedback, a dead i2rt control/update thread, motor error, gripper effort
above the configured maximum, or a detected gripper stall cancels the
trajectory and calls i2rt's verified `enter_gravity_comp_idle()`. If the client
process crashes or disconnects, its heartbeat stops and the same watchdog path
runs.

Two reactions are deliberately configured, not hard-coded (`config/*.yaml`
carries the rationale): `cartesian_settle_miss: hold` keeps the last command as
a monitored hold and completes with a `SETTLE_MISS:` detail instead of dropping
into gravity-comp idle (which released held objects), and a confirmed gripper
contact closes a further `gripper_contact_squeeze_fraction` of the stroke,
bounded by `gripper_max_effort_nm`.

Every action writes `start`, per-control-tick `feedback`, and terminal `result`
JSONL records to the configured action log (`logs/actions.jsonl`,
`logs/actions_right.jsonl`); the exact path and all safety thresholds are
printed by preflight.

### Physical first-motion gate

Real motion will not start unless both startup and first-motion checks are
acknowledged. Before using `--enable-motion` on hardware, all of the following
must be true:

1. Two people are present: one remains at the physical emergency stop and
   one operates the terminal. Both agree on the spoken stop command.
2. The table and full configured workspace are empty, the arm and cables are
   unobstructed, and nobody is inside the motion envelope.
3. The CAN channel has one owner, the D405 serial is correct, preflight is
   `READY`, and the bridge first ran read-only long enough to confirm fresh
   joints, camera, and `health.state == "ok"`.
4. The action log is writable and a second terminal is ready to inspect it.
5. Acceptance is performed strictly in this order, restarting/reviewing after
   any fault: one-joint 1-degree micro-move; six-joint home; a short
   multi-waypoint joint trajectory; a 2 mm Cartesian micro-move; empty-space
   gripper motion. Never skip ahead after a failed or unexplained step.
6. After every level, cancel/close the client, confirm the arm entered the
   documented safe state, review the action log for command/feedback,
   convergence, tracking error, and result, then obtain agreement from both
   operators before proceeding.

The repository tests and fake-source run prove software behavior only. They
do not authorize or substitute for this physical sequence.

## Buffered joint programs

A config that names `program_j4_velocity_deg_s` / `program_j4_acceleration_deg_s2`
(only `config/left_arm_throw.yaml`) gets `ProgramController` instead of the
plain `MotionController`; every other config keeps the unchanged code path.

A program is one buffered sequence of joint increments plus a timed gripper
release, compiled to 50 Hz commands. The raised limits apply to J4 inside a
program only: every other joint, and every ordinary move, keeps the config's
20 deg/s / 40 deg/s². Clients get two non-motion request methods,
`preview_program` (compile and return the commands, TCP pose speeds and peak
joint rates) and `program_report` (the per-tick record of an executed program,
also written under the action log's `programs/` directory), and one action kind,
`joint_program`.

On a bridge without program limits both request methods and the action are
refused with `READ_ONLY` ("joint programs are not enabled on this bridge").
On a program-enabled bridge, `program_report` for an id that has no report
fails with `PROGRAM_NOT_FOUND` — which is exactly how the harness tells the two
apart: it probes `program_report("boot-probe")` at startup and refuses to boot
a `--programs` session unless the answer is that error.

## Requests

Non-motion requests: `get_observation`, `solve_ik` (full pose) and
`solve_position_ik` (xyz, orientation free). Both IK methods reuse the same
current i2rt `grasp_site` model as FK and motion, and neither moves the arm.

Actions: `absolute_joints`, `cartesian_pose`, `joint_trajectory`, `gripper`,
`cancel_trajectory`, `heartbeat`, and `joint_program` on a program-enabled
bridge. A bridge started without `--enable-motion` rejects every valid action
with `READ_ONLY`.

Every observation carries six arm joints plus gripper fraction, velocities,
efforts, validated `grasp_site` FK, bridge health, sequence, source timestamps
and measured age, plus both cameras: `camera_0` is the RGB-D wrist
`wrist_d405`, `camera_1` the RGB-only calibrated overhead `top_brio`. The
wrist stream is requested as RGB8 + Z16, depth aligned to color and converted
to `float32` meters with `0.0` for invalid/out-of-range depth, published with
the color intrinsics and the runtime `inverse_brown_conrady` model and its five
coefficients — consumers must apply that model during deprojection. The
overhead images are undistorted before publication, so `camera_1.intrinsics`
is directly usable as a pinhole matrix and its distortion contract is `none`
with five zero coefficients; its fixed pose is copied from the calibration
artifact and never derived from the current robot pose. Stale, malformed,
missing-depth, wrong-distortion or wrong-serial frames fail closed.

The flange-to-camera transform is read from the current i2rt station MJCF
(`station_flange_body -> station_camera_body`) instead of being copied into
bridge code. For every observation the bridge evaluates
`T_world_camera = T_world_gripper @ T_gripper_camera` with the same joint
feedback used for the robot pose, and enforces the configured maximum host-time
difference between image capture and joint feedback.

## Camera acceptance and calibration CLI

`agp-yam-camera-acceptance` is the offline/solver side used by `calib/`
(`calib/solve_left.sh`, `calib/solve_top.sh`, the capture and analysis scripts)
and documented as a procedure in `calib/README_runbook_zh.md`:

| subcommand | what it does |
|---|---|
| `capture` | record N observation frames from a running bridge into an npz |
| `evaluate-jitter` | static pose jitter of a capture |
| `evaluate-plane` | plane fit over an ROI against an expected world height |
| `evaluate-points` | reprojection of measured world points |
| `export-rgb` | write a capture's RGB frame to PNG |
| `calibrate-checkerboard` | multi-pose wrist hand-eye solve, `T_flange_camera` |
| `evaluate-checkerboard-pose` | verify a held-out pose against a reference report |
| `calibrate-top-intrinsics` | overhead-camera intrinsics for one serial and stream profile |
| `capture-top-pair` | one synchronized wrist + overhead pair |
| `calibrate-top-extrinsics` | fixed overhead `camera_to_world` from those pairs |

Every report is serial- and profile-bound, is written even on failure, and
returns nonzero when a threshold is exceeded. A `FAIL` report must not be
installed.

## P2 offline kinematics and trajectory gate

`I2rtKinematicsBackend` loads the current i2rt arm+gripper MJCF without
constructing a hardware robot. It makes the
`world -> yam_base -> gripper -> grasp_site` chain and `wxyz` convention
explicit, delegates FK/IK to i2rt, seeds neighboring IK solves with the prior
solution, and post-checks IK through FK and the model's joint limits.
For P2, `world` is explicitly defined to equal `yam_base`, so
`T_world_yam_base` is identity. A later station/table world frame must provide
a measured transform; it must not be inferred or silently substituted here.

`validate_trajectory()` rejects non-monotonic timing, a joint jump above
0.5 rad, velocity above 20 deg/s, acceleration above 40 deg/s², model
joint-limit violations, or a `grasp_site` outside the configured `yam_base`
workspace at either a waypoint or any interior 50 Hz joint-interpolation
sample. P4 is the only layer that may dispatch a trajectory after this gate.

## Wire contract

Messages use a 4-byte big-endian length followed by msgpack-numpy. Schema 1
requires `float32[7]` position, velocity, and effort arrays plus a
`float32[8]` Cartesian `xyz+wxyz+gripper` array; positive monotonic and wall
timestamps; a non-negative sequence; and explicit source/motion/safety health.
It also requires `camera_0` with aligned RGB `uint8[H,W,3]`, metric depth
`float32[H,W]`, `float64[3,3]` intrinsics, `float64[4,4]`
`camera_to_world`, serial, frame timestamps/sequence, joint timestamp/skew,
the shared observation sequence, `inverse_brown_conrady` plus five finite
coefficients, the hand-eye `transform_translation_error_bound_m`, and explicit
RGB/meter/invalid-depth semantics. That transform-only check is not a complete
depth/segmentation position uncertainty. `camera_1` is the calibrated
fixed `top_brio`: RGB `uint8[H,W,3]` only, `float64[3,3]` pinhole intrinsics,
the measured rigid `float64[4,4]` `camera_to_world`, serial, timestamps, and
joint skew, with `none`/zero distortion and its held-out
`transform_translation_error_bound_m`. Both camera entries are mandatory;
RGB-only `camera_1` rejects a fabricated depth field.
Action schema 1 keeps `absolute_joints` (`float32[6]`) and `gripper`
(`open_fraction` in `[0,1]`) as independent payloads. Both include a hard
timeout and measured convergence tolerance. It also fixes
`joint_trajectory` (`float32[N,6]`, exactly one of per-waypoint times or a
control rate, timeout, and convergence tolerance), `cancel_trajectory`, and
world-frame `grasp_site` `cartesian_pose` (`float32[7]`, `xyz+wxyz`). Every
action also carries monotonic/wall timestamps, sequence, and request ID.
`heartbeat` names the active action lease. `action_result` reports measured
final `float32[7]` feedback, start/finish monotonic times, maximum tracking
error, terminal status, and detail. A bridge started without `--enable-motion`
rejects every valid action with `READ_ONLY`.

Run tests and lint with:

```bash
uv run --locked pytest -q
uv run --locked ruff check src tests
```

When the local i2rt implementation changes intentionally, review the diff and
update both `git_revision` (if HEAD moved) and `tracked_diff_sha256` in the
configuration. `../make_config_sha.sh hardware-bridge/config/<name>.yaml` (or
`--check` for a report only) recomputes and patches it for one config; the
fingerprint it writes is exactly

```bash
git -C ../i2rt diff --binary HEAD -- i2rt pyproject.toml | sha256sum
```

Every runtime config pins the same i2rt working tree, so run it once per
config. The bridge preflight refuses to serve on a mismatch: the sha is what
blesses the local i2rt modifications as reviewed.
