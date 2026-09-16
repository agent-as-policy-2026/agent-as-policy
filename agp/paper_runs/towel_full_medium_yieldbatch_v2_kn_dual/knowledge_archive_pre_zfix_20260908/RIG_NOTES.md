# Task notes — facts earlier operators learned doing THIS task on THIS robot

Notes left by previous autonomous sessions (and by an offline consolidation of
their reports). Treat them as evidence, not orders: a note can be wrong or out of
date. If you find one is wrong, add a new row that says so with your own
evidence; do not delete rows.

Each row: fact | applies when | evidence | confidence (low/med/high).


<!-- from session 20260908_175849_paper_towel_full_medium_yieldbatch_v2_kn_dual_01, merged 2026-09-08 -->
| fact | applies when | evidence | confidence |
|---|---|---|---|
| Initial towel footprint was approximately 0.70 m wide and 0.44 m near-to-far, estimated using overhead rays at z=-0.040 m. | Same towel; positions must be remeasured. | `frames/0001_top.png` and its corner deprojections. | medium |
| Left top-down near-corner grasp worked at z=-0.035 m with quaternion (w,x,y,z)=(0,0.707107,0.707107,0); close width 5.6 mm, then about 4.9 mm while held. | Flat near-left corner, grasp about 1–2 cm inside the visible hem; world XY depends on scene. | `frames/0004_top.png` and `frames/0005_top.png`, successful 70 mm lift. | high |
| Right wrist depth reported cloth z=-0.0576 and neighboring table z=-0.0638, lower than nominal table -0.045; do not assume nominal geometry and wrist depth agree. | This right-arm calibration and the examined right-side location; needs independent recheck elsewhere. | Deprojected pixels (360,240) and (530,240) of `frames_right/0008_wrist.png`. | medium |
| Right empty-grasp width was approximately 1 mm, but an inset raised hem was visibly carried after a close width of only 1.3 mm. Width alone cannot distinguish these cases reliably. | Thin hem pinch; require a small observed lift. | Empty `frames_right/0002_wrist.png`; held hem `frames_right/0012_wrist.png` and `frames/0014_top.png`. | high |
| Right successfully grasped a raised hem at z=-0.041 with quaternion (0,1,0,0), after the left arm lifted the opposite corner. It was inset from the right corner and produced a loose triangular end during folding. | Raised near hem; world XY is scene dependent. | `frames_right/0010_wrist.png` through `0012_wrist.png`; `frames/0020_top.png`. | high |
| Right grasp of a doubled flap at z=-0.042 with quaternion (0,0.965926,0,0.258819) gave a secure 5.8 mm close, but pulled multiple layers and translated the towel. | Attempting to isolate a flap resting on the underlying towel. | `frames_right/0016_wrist.png`, `frames/0021_top.png`, `frames/0022_top.png`. | high |
| Right top-down target (0.38,-0.46,0.17) was rejected by IK; the same position with quaternion (0,0.965926,0,0.258819) was reached. | This local reach situation; do not generalize to arbitrary poses. | Right commands 55 and 57; `frames_right/0014_wrist.png`. | high |
| A left anchor close reported GRIPPER_STALL even though the following state showed a 5.4 mm opening; its pose subsequently shifted while the opposite end was pulled. Do not treat such an anchor as rigid or reliable. | Faulted anchor grasp on layered towel. | `frames/0025_wrist.png`, left states before and after the pull, `frames/0026_top.png`. | high |
| Coordinated inset-hem folding and later flap pulling did not produce a clean first fold in this run. | Planning recovery from failed corner grasps. | `frames/0020_top.png` and final `frames/0029_top.png`. | high |

No successful folding playbook was demonstrated.
