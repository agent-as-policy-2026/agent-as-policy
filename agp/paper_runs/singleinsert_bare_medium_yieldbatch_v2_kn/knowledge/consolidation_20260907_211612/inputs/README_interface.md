# Robot interface — free-agent probe (REAL 6-DOF arm)

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
each call blocks until the robot finishes it. Speed caps: ≤ 0.03 m/s
straight-line, ≤ 10°/s tool rotation, ≤ 20°/s joints. Client timeout 240 s.
The response is a single JSON object on stdout, always with an `"ok"` field
(and an `"error_code"` when `ok` is false). If a call times out, check server
liveness with `tail -3 server.log` and retry once.

## Frames & units

- All poses are in the **robot base frame**: origin at the arm mount, z up,
  metres.
- Quaternions are scalar-first `{"w","x","y","z"}`.
- Tool frame: **+z points out of the fingers**; the fingers close along
  tool y. Poses refer to the **grasp point** between the finger pads.
- Workspace envelope (checked before anything moves): 0.12 ≤ √(x²+y²) ≤ 0.65,
  -0.050 ≤ z ≤ 0.60, and a single move may not travel more than 0.25 m from
  the current pose. A target outside it is refused with `error_code:
  "CLAMP"` and nothing moves. The robot additionally enforces joint limits
  and reachability: `ok:false` with `IK_FAILED: ...` or `JOINT_LIMIT: ...`,
  and the arm stays put.
- `error_code: "SETTLE_MISS"`: the move's path completed but the arm did not
  reach the target within tolerance. The response carries the achieved
  `ee_pose` and `target_error_mm`; the arm holds where it stopped.

## Commands

| command | args (JSON) | returns |
|---|---|---|
| `status` | — | server, bridge and budget info (free) |
| `help` | — | this command list (free) |
| `state` | — | joints (rad), ee_pose, gripper_fraction, gripper_width_m, robot health (free) |
| `frames` | `{"cams":["wrist","top"],"depth":true}` (both optional) | captures BOTH cameras from the same instant; saves `frames/NNNN_<cam>.png`, `frames/NNNN_wrist_depth.npy` (float32 metres, wrist only), `frames/NNNN_calib.json`; returns the paths |
| `move_ee` | `{"position":{"x","y","z"},"rotation":{"w","x","y","z"},"mode":"linear"\|"plan"}` | `linear` = straight Cartesian line holding orientation (default); `plan` = joint-space move to the IK solution. Returns achieved `ee_pose` and `target_error_mm`; `ok:false` + `error_code` if the move was refused or failed (arm stays where it stopped) |
| `move_joints` | `{"joints":[6 floats]}` | joint-space move (radians); returns `joints`, `ee_pose` |
| `gripper` | `{"action":"open"\|"close"\|<fraction 0..1>}` | returns `fraction` (0 closed … 1 open) and `width_m` (= fraction × 0.0955) |

In an **observation-only** session (see `status`), every motion command
(`move_ee`, `move_joints`, `gripper`) is refused with `error_code:
"READ_ONLY"` without moving anything.

## Cameras

- `wrist` — 640×360 RGB-D (Intel RealSense D405) mounted on the hand, moves
  with the arm. Depth is float32 metres, 0 = invalid, nominal range roughly
  0.07–1.0 m. Its pose comes from the arm's kinematics at capture time.
- `top` — 1920×1080 RGB only (Logitech BRIO), fixed above the table. It is
  already undistorted: the pinhole `intrinsics` apply directly to the saved
  PNG. `side` is accepted as an alias for `top`.

`frames/NNNN_calib.json` per capture: for each camera `intrinsics` (3×3 K),
`pose` (camera position + orientation **in the robot base frame** at capture
time), `image_size` [w,h], `distortion_model`, `distortion_coefficients`,
plus a `_meta` entry with the robot joints, `ee_pose` and gripper fraction at
that instant. Projection convention: for a base-frame point X,
`Xc = R^T (X - t)` (R = rotation matrix of `pose.rotation`, t = position),
pixel = `K @ (Xc / Xc[2])`.

## Budget

`status`/`help`/`state` are free. Everything else counts (the cap is shown by
`status`). The session also has a wall-clock limit set in your task prompt.

## Safety

- Real forces, real objects. Collisions are real: do not drive the arm or
  the gripper into the table or into anything you do not intend to touch.
- Every move is slow and a human watches; the human may stop the session at
  any moment, and the session may end without warning if anything looks unsafe.
- If a move fails part-way the response still reports the achieved `ee_pose`.

## Local tooling

- `python3` (stdlib) is enough for `robot_client.py`.
- numpy / PIL / scipy / cv2 are available in the interpreter
  `<CONNECTOR_PY>`
  (read-only use, for your own analysis scripts in `scratch/`).
- `ffmpeg` is on PATH.
- View any local image with your image-viewing tool.
- The run is video-recorded from a third camera for the human record; you
  have no access to that stream.

## Running robot commands from your tool harness

Every `robot_client.py` call blocks until the robot has finished the command
(motion and gripper commands take 2–60 s, `frames` about 1 s). On EVERY exec
call that contains a robot command — including the very first one — pass
`yield_time_ms: 90000`, so that the call returns the final JSON in one step.
Never use a shorter yield for a call with robot commands, and never poll a
running process for their result.
One exec call may run several robot commands in sequence and view the resulting
images in the same call; a separate call per command or per image is not needed.
A call that runs several robot commands still needs the 90000 yield.

