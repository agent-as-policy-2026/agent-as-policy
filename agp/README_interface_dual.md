# Robot interface (TWO REAL 6-DOF arms)

You control TWO REAL 6-DOF robot arms — `left` and `right`, standing side by
side at one real table — each with a two-finger parallel gripper (max inner
opening **0.0955 m**). The ONLY way to interact with them is the CLI below.
There is no simulation, no reset and no undo: whatever you move stays moved.
A human supervisor holds an emergency stop and may end the session at any
time. The servers hold state between commands (persistent scene, persistent
robots).

## Calling convention

```
python3 robot_client.py . [--arm left|right] <command> ['<json-args>']
```

Run from the session directory (`.` = session dir). `--arm` selects the arm
and goes directly after the session dir; when omitted the command goes to the
**left** arm. One command at a time PER ARM — each call blocks until that arm
finishes it (a command for the other arm may run concurrently, see "Two
arms"). The arms are slow by design (≤ 0.03 m/s straight-line, ≤ 10°/s tool
rotation, ≤ 20°/s joints): a 0.25 m straight move takes about 10 s. Client
timeout 240 s. The response is a single JSON object on stdout, always with an
`"ok"` field (and an `"error_code"` when `ok` is false). If a call times out,
check server liveness with `tail -3 server.log` (left) / `tail -3
server_right.log` (right) and retry once.

## Frames & units

- All poses — for BOTH arms — are in ONE world frame: the **left arm's base
  frame**: origin at the left arm's mount, z up, metres. The table surface is
  at z ≈ **-0.045** (measured); the arm bases are mounted at table level (no
  pedestal). The right arm's base sits at **(0, -0.61, 0)** in this frame
  (nominal; may be refined by calibration — the right arm's `status` reports
  the transform actually used as `base_in_world`).
- Quaternions are scalar-first `{"w","x","y","z"}`.
- Tool frame: **+z points out of the fingers** toward the workspace; the
  fingers close along tool y. Rotation `{"w":0,"x":1,"y":0,"z":0}` points the
  tool straight DOWN (a top-down approach with a canonical wrist roll). Poses
  refer to the **grasp point** between the finger pads.
- Finger geometry: the finger tips reach **10 mm beyond the grasp point**
  along tool +z. With the tool pointing straight down the tips touch the
  table when the grasp point is at `table_z + 0.010`; a lower grasp point
  presses the tips into the table and the move ends with `SETTLE_MISS`. A
  thin object lying on the table is between the pads only when the grasp
  point is within those 10 mm above the table.
- Workspace clamp (checked before anything moves): 0.12 ≤ r ≤ 0.65 where r is
  the horizontal distance from THAT arm's own base (left: √(x²+y²); right:
  √(x²+(y+0.61)²) with the nominal offset), -0.050 ≤ z ≤ 0.60, and a single
  move must not travel more than 0.25 m from the current pose (split long
  moves). The robots additionally enforce joint limits and reachability: an
  unreachable target returns `ok:false` with `IK_FAILED: ...` or
  `JOINT_LIMIT: ...` and the arm stays put.
- If a move's path completes but the arm cannot settle onto the target within
  tolerance (typically the object or the fingers are pressing against
  something, or the target is a few mm inside a surface), the response is
  `ok:false` with `error_code: "SETTLE_MISS"` plus the achieved `ee_pose` and
  `target_error_mm`. The arm stays stiff where it stopped and the gripper keeps
  its grip. Decide from fresh frames: lift/retreat, open the gripper there, or
  retry with a corrected target — do not repeat the identical command.
- `gripper` can likewise return `ok:false` with `error_code: "GRIPPER_STALL"`
  when the fingers stop more than 0.02 of the full stroke short of the
  commanded opening for 0.5 s. Nothing is released: the gripper holds the
  position it stalled at. That response carries no `fraction` — read `state`
  (`gripper_fraction`, `gripper_width_m`) or the `_meta` of the next `frames`.
  After `close`, a residual opening of a few hundredths means the fingers
  stopped on something thin or compliant (the hold may or may not survive a
  lift); a large residual means they are blocked.
- Physics hint: with the tool pointing straight down the highest reachable z
  falls with distance from the arm's own base — roughly 0.21 m at r = 0.30,
  0.13 m at r = 0.45, 0.05 m at r = 0.50. Tilt the tool to reach higher or
  farther.

## Commands

| command | args (JSON) | effect / returns |
|---|---|---|
| `status` | — | server, bridge and budget info of that arm, whether motion is enabled, `arm`, `world_frame` and (right arm) `base_in_world` (free) |
| `help` | — | this command list (free) |
| `state` | — | joints (rad), ee_pose (world frame), gripper_fraction, gripper_width_m, robot health of that arm (free) |
| `frames` | `{"cams":["wrist","top"],"depth":true}` (both optional) | left arm: captures BOTH cameras from the same instant; saves `frames/NNNN_<cam>.png`, `frames/NNNN_wrist_depth.npy` (float32 metres, wrist only), `frames/NNNN_calib.json`; returns the paths. Right arm: its own `wrist` only (the default there), saved as `frames_right/NNNN_*`; asking it for `top`/`side` returns `ok:false` (the overhead camera is served by the left arm) |
| `deproject` | `{"capture":N,"cam":"wrist","u":<px>,"v":<px>}` | 3D point (world frame) of that pixel using the saved wrist depth (5×5 median) of that arm's capture N (free). Add `"plane_z":<m>` to intersect the pixel ray with the horizontal plane z = plane_z instead of using depth — the only option for `top` (e.g. `plane_z` = table z for a footprint, table z + height for a top face) |
| `move_ee` | `{"position":{"x","y","z"},"rotation":{"w","x","y","z"},"mode":"linear"\|"plan"}` | target in the world frame. `linear` = straight Cartesian line holding orientation (default); `plan` = joint-space move to the IK solution. Returns achieved `ee_pose`, `target_error_mm`, and `ok:false` + error if the move was refused or failed (arm stays where it stopped) |
| `move_delta` | `{"dpos":[dx,dy,dz],"drot_deg":[rx,ry,rz]}` (either optional) | relative straight-line move from that arm's current pose; deltas in the world frame, rotation deltas about the WORLD axes, applied before the current rotation |
| `move_joints` | `{"joints":[6 floats]}` | joint-space move (radians); the robot chooses the timing |
| `home` | — | move THAT arm to its OBSERVE posture (wrist camera overlooking the table); this is not a folded park |
| `gripper` | `{"action":"open"\|"close"\|<fraction 0..1>}` | returns `fraction` (0 closed … 1 open) and `width_m` (= fraction × 0.0955). `close` stops on contact: a clearly nonzero fraction after `close` means something is held between the pads; ≈0 means the grasp is empty; a `GRIPPER_STALL` error (see above) also means the fingers stopped on something — read `state` |

In an **observation-only** session (see `status`), every motion command
(`move_ee`, `move_delta`, `move_joints`, `home`, `gripper`) is refused with
`error_code: "READ_ONLY"` without moving anything. Do not retry it.

## Cameras

- `wrist` — one per arm: 640×360 RGB-D (Intel RealSense D405) mounted on that
  arm's hand, moves with it. Depth is float32 metres, 0 = invalid, nominal
  range roughly 0.07–1.0 m. It is a passive stereo camera: depth exists only
  where the surface has visible texture, and pixels without it are 0 or
  unreliable. Its pose (world frame) comes from that arm's kinematics at
  capture time.
- `top` — 1920×1080 RGB only (Logitech BRIO), fixed above the table, served
  by the LEFT arm only (`frames` without `--arm`, or `--arm left`). It is
  already undistorted: the pinhole `intrinsics` apply directly to the saved
  PNG. No depth — use `deproject` with `plane_z`. `side` is accepted as an
  alias for `top`.

`frames/NNNN_calib.json` (left) / `frames_right/NNNN_calib.json` (right;
`wrist` and `_meta` only) per capture: for each camera `intrinsics` (3×3 K),
`pose` (camera position + orientation **in the world frame** at capture
time), `image_size` [w,h], `distortion_model`, `distortion_coefficients`,
plus a `_meta` entry with that arm's joints, `ee_pose` and gripper fraction
at that instant. Projection convention: for a world-frame point X,
`Xc = R^T (X - t)` (R = rotation matrix of `pose.rotation`, t = position),
pixel = `K @ (Xc / Xc[2])`. `deproject` is the exact inverse of this — you
rarely need to do the math yourself.

## Two arms

- `--arm left` (the default when omitted) / `--arm right` selects the arm:
  `python3 robot_client.py . --arm right move_ee '{...}'`. The two arms are
  served by two independent servers: left → `bridge/`, `frames/`,
  `server.log`; right → `bridge_right/`, `frames_right/`, `server_right.log`.
- One world frame: both arms report AND accept every pose (`ee_pose`,
  `move_ee` targets, `move_delta` deltas, camera poses, `deproject` points)
  in the left arm's base frame. The right arm's base sits at (0, -0.61, 0) in
  that frame (nominal; may be refined by calibration — its `status` shows
  `base_in_world`, the translation and rotation actually used).
- Reach: each arm's radial envelope 0.12 ≤ r ≤ 0.65 m is measured from ITS
  OWN base, not from the world origin (right arm, nominal:
  r = √(x²+(y+0.61)²)). The z limits are WORLD-frame heights (the table
  plane), identical for both arms; the 0.25 m single-move limit and the
  joint/IK limits apply per arm in the same way.
- Cameras: each arm has its own `wrist` camera, reached through that arm's
  `frames`. The overhead camera is available only through the left arm:
  `--arm right frames` captures `wrist` only (its default cams), and asking it
  for `top`/`side` returns `ok:false` with `error: "the overhead camera is
  served by the left arm (use --arm left frames)"`. `deproject` works on the
  captures of the arm you address (`frames/` vs `frames_right/`).
- Budgets are counted per arm; each arm's `status` shows its own count and
  cap.
- No coordination between the arms is enforced by the system. Both may move
  at the same time — a command to one arm does not wait for the other, and
  nothing checks the arms against each other. Avoiding contact between the
  arms (and between one arm and what the other holds) is your responsibility.
  Commands to different arms may be issued from the same exec call; commands
  to the SAME arm still execute strictly one at a time, in order.
- `home`, `state` and the observation-only refusal apply per arm:
  `--arm right home` moves only the right arm.

## Budget

`status`/`help`/`state`/`deproject` are free. Everything else counts,
separately for each arm (the cap is shown by that arm's `status`). The
session also has a wall-clock limit set in your task prompt. Be deliberate:
look (frames), think, then act.

## Safety

- Real forces, real objects. Never command a grasp point below the table.
- The two arms share the table and can reach each other; nothing in the
  system keeps them apart. Know where the other arm is (`state`) before you
  move one, and never let the arms or their grippers touch each other.
- Every move is slow and a human watches; the human may stop the session at
  any moment, and the session may end without warning if anything looks unsafe.
- If a move fails part-way the response still reports the achieved `ee_pose`;
  re-observe before the next action.

## Local tooling

- `python3` (stdlib) is enough for `robot_client.py`.
- numpy / PIL / scipy / cv2 are available in the interpreter
  `<CONNECTOR_PY>`
  (read-only use, for your own analysis scripts in `scratch/`).
- `ffmpeg` is on PATH.
- View any local image with your image-viewing tool.
- The run is video-recorded from a third camera for the human record; you
  have no access to that stream.

### Moving both arms at the same time

The two arms are served by two independent processes, so two robot commands
only overlap in time if you START them concurrently. Inside one exec call, run
the two clients in the background and wait for both, e.g.

```
python3 robot_client.py . move_ee '<left pose json>' > scratch/left.json &
python3 robot_client.py . --arm right move_ee '<right pose json>' > scratch/right.json &
wait; cat scratch/left.json scratch/right.json
```

Each client still blocks until its own arm has finished; commands written one
after another in the same call run one after another. Nothing in the system
keeps the two arms apart: when both move, or when one arm works near the other,
their poses and paths must not intersect — that is your responsibility.
