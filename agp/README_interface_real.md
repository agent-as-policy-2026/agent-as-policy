# Robot interface (REAL 6-DOF arm)

You control a REAL 6-DOF robot arm with a two-finger parallel gripper (max
inner opening **0.0955 m**) standing at a real table. The ONLY way to
interact with it is the CLI below. There is no simulation, no reset and no
undo: whatever you move stays moved. A human supervisor holds an emergency
stop and may end the session at any time. The server holds state between
commands (persistent scene, persistent robot).

## Calling convention

```
python3 robot_client.py . <command> ['<json-args>']
```

Run from the session directory (`.` = session dir). One command at a time —
each call blocks until the robot finishes it. The arm is slow by design
(≤ 0.03 m/s straight-line, ≤ 10°/s tool rotation, ≤ 20°/s joints): a 0.25 m
straight move takes about 10 s. Client timeout 240 s. The response is a
single JSON object on stdout, always with an `"ok"` field (and an
`"error_code"` when `ok` is false). If a call times out, check server
liveness with `tail -3 server.log` and retry once.

## Frames & units

- All poses are in the **robot base frame**: origin at the arm mount, z up,
  metres. The table surface is at z ≈ **-0.045** (measured); the arm base is
  mounted at table level (no pedestal).
- Quaternions are scalar-first `{"w","x","y","z"}`.
- Tool frame: **+z points out of the fingers** toward the workspace; the
  fingers close along tool y. Rotation `{"w":0,"x":1,"y":0,"z":0}` points the
  tool straight DOWN (a top-down approach with a canonical wrist roll). Poses
  refer to the **grasp point** between the finger pads.
- Workspace clamp (checked before anything moves): 0.12 ≤ √(x²+y²) ≤ 0.65,
  -0.050 ≤ z ≤ 0.60, and a single move must not travel more than 0.25 m from
  the current pose (split long moves). The robot additionally enforces joint
  limits and reachability: an unreachable target returns `ok:false` with
  `IK_FAILED: ...` or `JOINT_LIMIT: ...` and the arm stays put.
- If a move's path completes but the arm cannot settle onto the target within
  tolerance (typically the object or the fingers are pressing against
  something, or the target is a few mm inside a surface), the response is
  `ok:false` with `error_code: "SETTLE_MISS"` plus the achieved `ee_pose` and
  `target_error_mm`. The arm stays stiff where it stopped and the gripper keeps
  its grip. Decide from fresh frames: lift/retreat, open the gripper there, or
  retry with a corrected target — do not repeat the identical command.
- Physics hint: with the tool pointing straight down the highest reachable z
  falls with distance — roughly 0.21 m at r = 0.30, 0.13 m at r = 0.45,
  0.05 m at r = 0.50. Tilt the tool to reach higher or farther.

## Commands

| command | args (JSON) | effect / returns |
|---|---|---|
| `status` | — | server, bridge and budget info, whether motion is enabled (free) |
| `help` | — | this command list (free) |
| `state` | — | joints (rad), ee_pose, gripper_fraction, gripper_width_m, robot health (free) |
| `frames` | `{"cams":["wrist","top"],"depth":true}` (both optional) | captures BOTH cameras from the same instant; saves `frames/NNNN_<cam>.png`, `frames/NNNN_wrist_depth.npy` (float32 metres, wrist only), `frames/NNNN_calib.json`; returns the paths |
| `deproject` | `{"capture":N,"cam":"wrist","u":<px>,"v":<px>}` | 3D point (base frame) of that pixel using the saved wrist depth (5×5 median) (free). Add `"plane_z":<m>` to intersect the pixel ray with the horizontal plane z = plane_z instead of using depth — the only option for `top` (e.g. `plane_z` = table z for a footprint, table z + height for a top face) |
| `move_ee` | `{"position":{"x","y","z"},"rotation":{"w","x","y","z"},"mode":"linear"\|"plan"}` | `linear` = straight Cartesian line holding orientation (default); `plan` = joint-space move to the IK solution. Returns achieved `ee_pose`, `target_error_mm`, and `ok:false` + error if the move was refused or failed (arm stays where it stopped) |
| `move_delta` | `{"dpos":[dx,dy,dz],"drot_deg":[rx,ry,rz]}` (either optional) | relative straight-line move from the current pose; rotation deltas about the BASE axes, applied before the current rotation |
| `move_joints` | `{"joints":[6 floats]}` | joint-space move (radians); the robot chooses the timing |
| `home` | — | move to the OBSERVE posture (wrist camera overlooking the table); this is not a folded park |
| `gripper` | `{"action":"open"\|"close"\|<fraction 0..1>}` | returns `fraction` (0 closed … 1 open) and `width_m` (= fraction × 0.0955). `close` stops on contact: a clearly nonzero fraction after `close` means something is held between the pads; ≈0 means the grasp is empty |

In an **observation-only** session (see `status`), every motion command
(`move_ee`, `move_delta`, `move_joints`, `home`, `gripper`) is refused with
`error_code: "READ_ONLY"` without moving anything. Do not retry it.

## Cameras

- `wrist` — 640×360 RGB-D (Intel RealSense D405) mounted on the hand, moves
  with the arm. Depth is float32 metres, 0 = invalid, nominal range roughly
  0.07–1.0 m. It is a passive stereo camera: depth exists only where the
  surface has visible texture, and pixels without it are 0 or unreliable.
  Its pose comes from the arm's kinematics at capture time.
- `top` — 1920×1080 RGB only (Logitech BRIO), fixed above the table. It is
  already undistorted: the pinhole `intrinsics` apply directly to the saved
  PNG. No depth — use `deproject` with `plane_z`. `side` is accepted as an
  alias for `top`.

`frames/NNNN_calib.json` per capture: for each camera `intrinsics` (3×3 K),
`pose` (camera position + orientation **in the robot base frame** at capture
time), `image_size` [w,h], `distortion_model`, `distortion_coefficients`,
plus a `_meta` entry with the robot joints, `ee_pose` and gripper fraction at
that instant. Projection convention: for a base-frame point X,
`Xc = R^T (X - t)` (R = rotation matrix of `pose.rotation`, t = position),
pixel = `K @ (Xc / Xc[2])`. `deproject` is the exact inverse of this — you
rarely need to do the math yourself.

## Budget

`status`/`help`/`state`/`deproject` are free. Everything else counts (the
cap is shown by `status`). The session also has a wall-clock limit set in
your task prompt. Be deliberate: look (frames), think, then act.

## Safety

- Real forces, real objects. Never command the grasp point below the table.
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
