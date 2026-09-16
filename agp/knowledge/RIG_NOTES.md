# Rig notes — facts earlier operators learned on THIS robot

These are notes left by previous autonomous sessions on this exact arm, table
and cameras. Treat them as evidence, not orders: a note can be wrong or out of
date. If you find one is wrong, add a new row that says so with your own
evidence; do not delete rows. Keep this file short — if it grows past ~60 rows,
merge duplicates and drop notes marked `obsolete`.

Each row: id | date/session | one-line fact | when it applies | evidence (a
frame path or numbers you can check) | confidence (low/med/high) | status
(current/obsolete).

| id | session | fact | applies when | evidence | conf | status |
|----|---------|------|--------------|----------|------|--------|


<!-- from session 20260903_122441_pyramid1, merged 2026-09-03 -->
# Knowledge delta

| fact | applies when | evidence | confidence |
|---|---|---|---|
| **[OBSOLETE — marked by the operator 2026-09-03: a close at the block's own width is CONTACT, not a secure hold; see the pyramid2 rows below. The bridge now squeezes past contact.]** A top-down close on one of these blocks normally reports about 0.050-0.054 m when it is securely held; a close near 0 m is empty. | Checking whether a block grasp succeeded. | Gray and blue true grasps reported about 0.0506-0.0538 m and translated with the tool (`frames/0046_top.png`, `frames/0075_top.png`); an empty attempt reported 0.0007 m before `frames/0067_top.png`. | high |
| A straight-down pose at radius about 0.54 m is not practically usable near table height, but a roughly 60-degree tilted approach can reach and clamp a block there. | Retrieving a block near `(0.537, -0.003, -0.020)` m. | The tilted close reported about 0.0509 m and `frames/0054_wrist.png` showed the block held; `frames/0056_top.png` showed it staged closer. | high |
| `ACTION_TIMEOUT` and especially `STALE_FEEDBACK` do not guarantee that motion stopped at the first reported pose; the arm can continue settling substantially. Always refresh state and re-image before issuing a recovery command. | Any failed Cartesian command. | During the blue edge-roll, the first stale response was near `(0.292, 0.100, 0.130)`, while the following state was approximately `(0.238, 0.095, 0.078)`; `frames/0077_top.png` recorded the achieved location. | high |
| Changing a tilted tool orientation close to a stack can sweep the gripper or forearm through the blocks even when the intended endpoint is clear. | Transitioning between tilted and vertical grasps around a built structure. | The two-middle partial build in `frames/0048_top.png` was completely collapsed during the nearby orientation transition; see `frames/0049_top.png`. | high |

<!-- from session 20260903_142826_pyramid2, merged 2026-09-03 -->
# Knowledge delta

| fact | applies when | evidence | confidence |
|---|---|---|---|
| For these roughly 5 cm patterned blocks, a close near 0.050 m can be a genuinely centered hold; verify that the block remains large and fixed relative to the wrist camera after a short lift before committing to transport. | Top-down block pickup verification. | The centered close reported 0.0502 m; the block remained large between the fingers after lifting to `z≈0.043` in `frames/0019_wrist.png` and to `z≈0.102` in `frames/0020_wrist.png`. | high |
| A nonzero close around 0.054 m can still be only a marginal/corner pinch that immediately loses the block, so opening fraction alone is insufficient. | Pickups with even modest cross-axis centering error. | The first close reported 0.0537 m, but `frames/0008_wrist.png` showed the block shrinking with camera distance rather than staying fixed to the tool, demonstrating that it remained on the table. | high |
| In-air yaw rotation can cause a securely lifted patterned block to slip back to the table even when the gripper still reports a nonzero opening. Prefer retaining the pickup yaw through placement unless cube-face orientation is required. | Reorienting a held block about tool z. | The block was securely held in `frames/0019_wrist.png` and `frames/0020_wrist.png`; after the yaw change, `frames/0022_wrist.png` showed it back near the source while state still reported about 0.0529 m. | high |
| Long Cartesian translations sometimes fail immediately with `STALE_FEEDBACK`, while short `move_delta` segments succeed after refreshing state and imagery. | Controller is in a stale-feedback fault while an object is held. | Two long translations were refused around captures 21-23; a `move_delta` of `[0.04, 0.04, 0.008]` then completed and was verified in `frames/0024_wrist.png`. | medium |

<!-- from session 20260903_145637_pyramid3, merged 2026-09-03 -->
# Knowledge delta

| fact | applies when | evidence | confidence |
|---|---|---|---|
| A session can begin in a joint configuration that is not the documented observation posture even when the arm appears parked; if nearby Cartesian transitions repeatedly have no continuous IK path, `home` can establish the intended observation/manipulation branch. | Initial Cartesian moves fail immediately with endpoint/continuous-trajectory IK errors and no physical motion. | Before `home`, state was near `(-0.061, 0.175, 0.155)` and three nearby Cartesian targets were rejected; `home` completed and moved to about `(0.349, 0.016, 0.352)`, after which the first downward transit succeeded (`frames/0004_top.png` to `frames/0006_top.png`). | high |
| For a vertical grasp, switching to the perpendicular cube-face pair by adding 90 degrees of tool yaw can escape a local low-height descent stall while preserving a valid square-cube grasp. | A top-down descent with one cube-aligned wrist roll repeatedly times out above the cube, and the gripper is still open. | With tool yaw about -36 degrees, descent near `(0.302, -0.13)` stalled around `z=0.011` (`frames/0050_wrist.png`, `frames/0051_wrist.png`). After retracting and using the 90-degree-equivalent yaw about +54 degrees, the staged descent reached `z=-0.020` and produced a `0.0513 m` grasp (`frames/0053_wrist.png` through `frames/0058_wrist.png`). | high |
| Re-closing during segmented transport can take up small jaw relaxation without disturbing a centered top-down grasp; in this run it reduced the held width from about `0.0518 m` to `0.0500 m` before the final carry. | Carrying these patterned cubes through several short Cartesian segments with unchanged wrist yaw. | The final grey cube stayed fixed in the wrist view across `frames/0074_wrist.png` to `frames/0080_wrist.png`; the re-close after the first segment reported `0.0500 m`, and placement succeeded in `frames/0084_top.png`. | medium |

<!-- from session 20260903_155304_pyramid4, merged 2026-09-03 -->
# Knowledge delta

| fact | applies when | evidence | confidence |
|---|---|---|---|
| A successful session-server `status` response with `bridge_state: unknown` does not by itself prove that the YAM robot bridge is reachable; require a successful live `state` and camera capture before planning motion. | Session startup on this rig. | In this run the initial `status` was `ok:true`, while the immediately following `state` and `frames` calls returned `DISCONNECTED` because `127.0.0.1:9021` refused connections; session-local `server.log` records the repeated failures around 15:53-15:55. | high |

No repeatable manipulation procedure was demonstrated because the robot bridge never became available.

<!-- from session 20260903_155934_pyramid5, merged 2026-09-03 -->
# Knowledge delta

| fact | applies when | evidence | confidence |
|---|---|---|---|
| A nominal face-width close around `0.049-0.051 m` is not sufficient for a cube wrapped with loose/torn tape: the pads can load the tape and the cube can slip after several short transports. Aligning the closing axis with the perpendicular, visibly untaped face pair changed the same cube from repeated slips to a stable carry. | Transporting the taped grey patterned cube used in this run. | The q90 grasp tightened to `0.0497 m` yet slipped at `frames/0051_top.png`; after a q45 regrasp aligned full faces in `frames/0054_wrist.png`, the cube stayed fixed from `frames/0057_wrist.png` through placement in `frames/0069_wrist.png`. | high |
| A 90-degree top-down roll was usable down to about `z=0.03 m` at the build radius, but repeatedly stalled around `z=0.012-0.016 m` when asked to reach table-level cube center height there. | Choosing a closing-axis roll near build center `(0.356, -0.092)` m. | Linear and planned descents stopped near `z=0.016` and `z=0.012` in `frames/0013_wrist.png` and `frames/0014_wrist.png`; the same roll later completed middle-tier placements at `z=0.031` (`frames/0022_wrist.png`). | high |
| Limited jaw opening can release a cube while reducing the finger sweep toward adjacent same-tier cubes. In this run 80% cleared a `0.067 m` diagonal pinch, and 65% cleared a `0.0494 m` face grip, after which the tool lifted vertically. | Releasing into a closely packed stack when a full opening could contact a neighbor or support. | The 80% release produced the intact two-cube tier in `frames/0034_wrist.png`; the 65% top release produced the stable top cube in `frames/0069_wrist.png`. | medium |

<!-- from session 20260903_171812_pyramid6, merged 2026-09-03 -->
# Knowledge delta

| fact | applies when | evidence | confidence |
|---|---|---|---|
| A `STALE_FEEDBACK` rejection during an attempted in-air yaw can coincide with substantial jaw relaxation and loss of a previously verified cube even when the tool pose does not rotate. Immediately refresh state and image the hold rather than trusting the pre-fault close width. | A motion request faults while carrying one of these compressible patterned cubes. | The verified grip tightened to about 0.0495 m before the yaw request (`frames/0010_wrist.png`); the request was rejected with unchanged orientation, but the following state reported about 0.0552 m and `frames/0011_wrist.png` / `frames/0011_top.png` showed the cube back on the table. | high |
| Small base-z yaw corrections of 10-15 degrees can preserve a centered, face-aligned grip when done at `z≈0.10 m` immediately after re-closing and followed by visual verification. | A securely lifted cube needs a small orientation correction and the surrounding airspace is clear. | The 15-degree correction preserved the second cube from `frames/0036_wrist.png` through `frames/0038_wrist.png`; the 10-degree correction preserved the final cube from `frames/0057_wrist.png` through `frames/0059_wrist.png`. Both were subsequently transported and placed. | medium |

<!-- from session 20260903_182646_pyramid6, merged 2026-09-03 -->
# Knowledge delta

| fact | applies when | evidence | confidence |
|---|---|---|---|
| A securely face-grasped patterned cube can tolerate a cumulative yaw correction of about 45 degrees when the correction is split into 15-degree base-z steps at `z≈0.10 m`, with a re-close to about `0.0495 m` and a wrist-image hold check before every next step. | A source cube is face-aligned to a grasp orientation but must be rotated substantially before placement, and the surrounding airspace is clear. | The second grey cube remained fixed from `frames/0033_wrist.png` through `frames/0037_wrist.png`; the top grey cube independently remained fixed from `frames/0053_wrist.png` through `frames/0057_wrist.png`. Both were then transported and placed successfully. | high |
| For these cubes, a partial release near 65% (about `0.0614 m`) cleared a centered `0.0495 m` face grip at both the middle and top tiers, after which a vertical lift left the stack intact. | Releasing a face-grasped cube into this closely packed 3-2-1 stack with the gripper closing axis perpendicular to the row. | First middle placement: `frames/0020_wrist.png` to `frames/0021_wrist.png`; second middle placement: `frames/0043_wrist.png` to `frames/0044_wrist.png`; top placement: `frames/0064_wrist.png` to `frames/0065_wrist.png`. | high |

<!-- from session 20260903_210521_pyramid7, merged 2026-09-03 -->
# Knowledge delta

| fact | applies when | evidence | confidence |
|---|---|---|---|
| At a build center near `(0.361, -0.084)`, arranging the 3-2-1 row along base x makes the canonical vertical grasp (`w=0, x=1, y=0, z=0`) close perpendicular to the row; a 65% release cleared these approximately 50 mm cubes on both middle and top tiers without disturbing adjacent supports. | Building this cube pyramid with the canonical top-down roll and approximately 50 mm center spacing. | First middle release and withdrawal: `frames/0036_wrist.png` to `frames/0037_wrist.png`; second middle: `frames/0046_wrist.png` to `frames/0047_wrist.png`; top: `frames/0056_wrist.png` to `frames/0059_wrist.png`; final structure: `frames/0061_top.png`. | high |
| The existing three-step 15-degree in-air yaw-correction procedure also preserved a centered grasp on the light-blue/black-patterned cube, tightening to about `0.0497-0.0498 m` before each step. | Correcting an approximately 45-degree source-cube yaw at `z≈0.10 m` with clear surrounding airspace. | Cube remained fixed relative to the wrist camera across `frames/0019_wrist.png` through `frames/0023_wrist.png`, then was transported and placed successfully by `frames/0028_top.png`. | high |

<!-- from session 20260904_124349_pyramid8, merged 2026-09-04 -->
# Knowledge delta

| fact | applies when | evidence | confidence |
|---|---|---|---|
| At source pose near `(0.232, -0.246, 0.10)`, the approximately 81-degree top-down roll had no IK solution, while its square-cube-equivalent orthogonal face grasp near -8.6 degrees was reachable, produced a centered 0.0500 m close, and carried the cube stably to the stack. | A square cube is face-aligned but the preferred top-down wrist roll is unreachable; the perpendicular face pair is acceptable for the planned release. | The 81-degree linear and planned attempts were rejected before motion around `frames/0045_top.png`-`frames/0048_top.png`; the -8.6-degree branch reached the cube in `frames/0088_wrist.png`, closed at 0.0500 m, remained fixed in `frames/0093_wrist.png` and `frames/0094_wrist.png`, and was placed successfully by `frames/0099_wrist.png` / `frames/0102_top.png`. | high |

<!-- from session 20260904_132159_pyramid9, merged 2026-09-04 -->
# Knowledge delta

| fact | applies when | evidence | confidence |
|---|---|---|---|
| At build center near `(0.34, -0.17)`, both physically equivalent perpendicular top-down rolls stopped near grasp-point `z=0.010-0.011 m`, while the 45-degree roll reached `z=-0.019 m`. | Choosing a wrist roll for table-height work near this build location. | +90-degree descent ended at `z=0.0101` in `frames/0005_wrist.png`; -90-degree descent ended at `z=0.0110` in `frames/0008_wrist.png`; the 45-degree descent completed at `z=-0.0189` in `frames/0012_wrist.png`. | high |
| A diagonal close around 62.6 mm was an off-center corner contact and did not lift this patterned black cube; an accurately projection-centered face close around 50.2 mm did lift it. | Interpreting nonzero close widths on these cubes. | The 62.6 mm attempt left the cube on the table and the later re-close went to 0.8 mm (`frames/0030_wrist.png` to `frames/0031_wrist.png`); after wrist RGB-D re-centering, a 50.2 mm close remained held through `frames/0037_wrist.png` and subsequent yaw steps. | high |
| An accurately centered diagonal grasp around 67.7-68.0 mm can lift and make a short placement, but it can slip during a longer carry even after re-closing. | Carrying these taped patterned black cubes by opposite corners. | The staged cube stayed held from `frames/0049_wrist.png` through the short placement ending at `frames/0055_top.png`; another 67.8 mm diagonal pickup was held in `frames/0076_wrist.png` but was back on the table after the longer carry in `frames/0077_top.png`. | high |
| When lowering a held cube beside an existing base cube, a nominal 50 mm center spacing can stop early from side contact; about 54-55 mm center spacing cleared in this run. | Placing neighboring base cubes with small tape protrusions and pose error. | Descents at the closer targets stopped around grasp-point `z=0.021-0.022 m` in `frames/0062_wrist.png` and `frames/0065_wrist.png`; after moving about 4 mm farther away, descent reached `z=-0.021 m` and released by `frames/0070_top.png`. | medium |
