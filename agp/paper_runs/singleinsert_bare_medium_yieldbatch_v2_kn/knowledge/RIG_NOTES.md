# Task notes — facts earlier operators learned doing THIS task on THIS robot

Notes left by previous autonomous sessions (and by an offline consolidation of
their reports). Treat them as evidence, not orders: a note can be wrong or out of
date. If you find one is wrong, add a new row that says so with your own
evidence; do not delete rows.

Each row: fact | applies when | evidence | confidence (low/med/high).


<!-- from session consolidation_20260907_211612, merged 2026-09-07 -->
# singleinsert: evidence delta

Reports and source scripts are available; captured images, calibration/depth arrays, script stdout and motion JSON are absent. Logs confirm command types/outcomes, not their parameters. Success and measurements below are report-backed, not independently image-verified. Existing knowledge contains no task facts or tools.

| fact | applies when | evidence | confidence |
|---|---|---|---|
| The smooth blue ring goes over the upright white stepped post and rests around its lower base, with white protruding above it. | This task pair; identify it anew rather than using demonstration positions. | inputs/20260907_210248_paper_singleinsert_bare_medium_yieldbatch_v2_03 RESULT: 1 grasp/insertion attempt, released assembly reported stable. | high (reports agree) |
| Table surface z ≈ -0.044 to -0.045 m. | Same table/base calibration; independent of part XY, but recheck surface depth. | inputs/20260907_205012_paper_singleinsert_bare_medium_yieldbatch_v2_01 RESULT: -0.044 m; inputs/20260907_205656_paper_singleinsert_bare_medium_yieldbatch_v2_02 RESULT: -0.044 m; inputs/20260907_210248_paper_singleinsert_bare_medium_yieldbatch_v2_03 RESULT: -0.045 m. | med |
| Ring upper rim z ≈ -0.018 m; rim-to-table difference ≈ 0.026 m (derived, not a separate size measurement). | Ring resting on the table. | inputs/20260907_205012_paper_singleinsert_bare_medium_yieldbatch_v2_01 RESULT: rim -0.018 m minus table -0.044 m. | med |
| White top z ≈ 0.035–0.039 m; top-to-table difference ≈ 0.079–0.083 m (derived). | Upright post on the same table. | inputs/20260907_205012_paper_singleinsert_bare_medium_yieldbatch_v2_01 RESULT: top 0.039 m/table -0.044 m; inputs/20260907_205656_paper_singleinsert_bare_medium_yieldbatch_v2_02 RESULT: top 0.035 m/table -0.044 m. | med |
| Successful closed gripper width was 0.0518–0.0519 m: evidence of the external grasp span, not a calibrated ring diameter. | External grasp with downward-facing fingers. | inputs/20260907_205012_paper_singleinsert_bare_medium_yieldbatch_v2_01 RESULT: 0.0519 m; inputs/20260907_205656_paper_singleinsert_bare_medium_yieldbatch_v2_02 and inputs/20260907_210248_paper_singleinsert_bare_medium_yieldbatch_v2_03 RESULT: 0.0518 m. | high (reported widths) |
| Grasp-point z = -0.020, -0.022 and -0.019 m all worked; downward tool orientation worked, but yaw/quaternion is not recorded. | Ring initially flat on this table; these are grasp-point heights, not ring-bottom heights. | inputs/20260907_205012_paper_singleinsert_bare_medium_yieldbatch_v2_01 RESULT: -0.020 m; inputs/20260907_205656_paper_singleinsert_bare_medium_yieldbatch_v2_02 RESULT: -0.022 m; inputs/20260907_210248_paper_singleinsert_bare_medium_yieldbatch_v2_03 RESULT: -0.019 m. | high for reported heights; med for qualitative orientation |
| Ring/post XY varied: (0.173,-0.069)/(0.294,-0.160), (0.287,-0.1585)/(0.273,-0.0375), (0.247,0.036)/(0.317,-0.041) m. | Historical grasp/alignment coordinates only; measure both positions in each new scene. | inputs/20260907_205012_paper_singleinsert_bare_medium_yieldbatch_v2_01, inputs/20260907_205656_paper_singleinsert_bare_medium_yieldbatch_v2_02, inputs/20260907_210248_paper_singleinsert_bare_medium_yieldbatch_v2_03 RESULT, respectively: listed XY pairs. | high |
| Reported insertion used concentric alignment; no separately quantified lateral correction or insertion clearance is available. | Aligning the held ring above the localized post. | inputs/20260907_205012_paper_singleinsert_bare_medium_yieldbatch_v2_01 RESULT: alignment ≈ (0.294,-0.160) m, successful staged descent; no offset measurement given. | med |
| Successful descent/release grasp heights: 0.082→0.050→0.015→-0.012 m; 0.055→0.025→-0.008 m; 0.045→0.008→-0.012 m. | After lifting and translating over the post; recheck against actual table, grasp and post height. | inputs/20260907_205012_paper_singleinsert_bare_medium_yieldbatch_v2_01, inputs/20260907_205656_paper_singleinsert_bare_medium_yieldbatch_v2_02, inputs/20260907_210248_paper_singleinsert_bare_medium_yieldbatch_v2_03 RESULT, respectively: listed z sequences. | high |
| Release followed by raising/withdrawal left the assembly visually stable; open width reported 0.0948 m. | Completion check with fingers clear; does not establish retention force. | inputs/20260907_205012_paper_singleinsert_bare_medium_yieldbatch_v2_01 RESULT: open 0.0948 m, raise z=0.150 m, withdraw ≈ (0.220,0.035,0.200) m; inputs/20260907_205656_paper_singleinsert_bare_medium_yieldbatch_v2_02 RESULT: open 0.0948 m and 2 final top observations. | high (reported visual outcome) |
| No failed grasp, insertion or refused motion is documented; there is no supported recovery routine. | These 3 successful runs only; no force/retention test established. | inputs/20260907_205012_paper_singleinsert_bare_medium_yieldbatch_v2_01 and inputs/20260907_205656_paper_singleinsert_bare_medium_yieldbatch_v2_02 server.log: 22 counted commands each; inputs/20260907_210248_paper_singleinsert_bare_medium_yieldbatch_v2_03 server.log: 21; every command ok=True. | high for log outcomes |
| Do not confuse the target with the toothed ring/white component beside the far-side arm. | Reports place the task pair on the left of the overhead image. | inputs/20260907_205656_paper_singleinsert_bare_medium_yieldbatch_v2_02 RESULT: 2 target parts on left, unrelated toothed ring and white component on right. | high (reported distinction) |
| Geometry script trial plane heights and selection bands are assumptions, not measured dimensions; ringfit.py does not actually fit a circle. | Reusing analysis code; neither ring bore nor white diameter is established by supplied outputs. | inputs/20260907_210248_paper_singleinsert_bare_medium_yieldbatch_v2_03 scratch: geometry.py uses a 0.070 m white-top trial plane; measure.py selects z=0.029–0.040 m; ringfit.py only prints percentiles and 5 point patches. | high (source inspection) |

<!-- from session 20260907_212417_paper_singleinsert_bare_medium_yieldbatch_v2_kn_01, merged 2026-09-07 -->
| fact | applies when | evidence | confidence |
|---|---|---|---|
| Table sample heights were -0.0436 to -0.0460 m; ring rim samples roughly -0.019 to -0.014 m; white top median about 0.0352 m. Approximate rim/table difference 25–31 mm and post height 79–81 mm; these are noisy RGB-D estimates. | This pair upright on the table; remeasure each scene. | frames/0001–0003 wrist depth transformed with associated calibration; geometry.py and measure.py output. | medium |
| Quaternion (w,x,y,z)=(0,1,0,0) worked for an external ring grasp, width 0.0517 m, commanded grasp z=-0.019 m (achieved -0.02205). | Smooth ring flat on this table; jaw closure along base y. | frames/0004_wrist.png visibly holds lifted ring; gripper response. | high |
| Concentric alignment command (0.310,-0.098) and staged z=0.050,0.015,-0.012 succeeded without lateral correction. | These XY values are scene-dependent; white measured center approximately (0.311,-0.098). | frames/0005–0006 wrist; frames/0007–0008 top. | high |
| Open width 0.0948 m, vertical lift to 0.150 m, and withdrawal to (0.200,0.045,0.200) left the assembly stable. | Withdrawal XY requires a clear path in the current scene. | frames/0007_top.png and frames/0008_top.png. | high |
| Successful motion responses still had target errors up to 3.9 mm; actual grasp and release heights were lower than commanded by about 3 mm. | Using small insertion tolerances. | move_ee achieved poses during this run. | high |

<!-- from session 20260907_213007_paper_singleinsert_bare_medium_yieldbatch_v2_kn_02, merged 2026-09-07 -->
| fact | applies when | evidence | confidence |
|---|---|---|---|
| Table samples z=-0.0440,-0.0445,-0.0465 m; close ring rim z=-0.0166 to -0.0134 m; white top median about 0.0332 m. Approximate ring height 28–33 mm and white height 77–80 mm, with depth noise. | This table and task pair; remeasure current scene. | Wrist depth/calibration in frames/0001–0003, geometry.py and measure.py outputs. | medium |
| Close-view blue surface bounds span approximately 48.6 mm in x and 51.1 mm in y; this is a visible rim estimate, not a bore measurement. | Flat smooth ring; selected cyan rim points. | frames/0003 depth, filtered measure.py extrema. | medium |
| Downward quaternion (0,1,0,0), commanded grasp z=-0.019 (actual -0.02109), and measured closed width 0.0517 m held the ring securely through lifting and translation. | External grasp on the smooth ring. | frames/0004_wrist.png and successful close/lift responses. | high |
| Post center command (0.2535,0.0655), descent z=0.050,0.015,-0.012 worked without lateral correction. | XY depends on scene; measured post center approximately (0.2534,0.0649). | frames/0005–0006 wrist and frames/0007–0008 top. | high |
| Opening to 0.0948 m, vertical lift to 0.150 and withdrawal to (0.190,0.045,0.200) left the ring engaged and stable. | Withdrawal XY needs a clear current path. | frames/0007–0008; final achieved arm z=0.19979. | high |
| Default python3 cannot import numpy; the analysis interpreter documented in README successfully runs the supplied geometry and measurement tools. | Analysis of calibrated RGB-D files. | Failed default import and subsequent successful tool outputs. | high |

<!-- from session 20260907_213524_paper_singleinsert_bare_medium_yieldbatch_v2_kn_03, merged 2026-09-07 -->
| fact | applies when | evidence | confidence |
|---|---|---|---|
| Table samples z=-0.04528 and -0.04531 m; close ring rim approximately -0.0177 to -0.0135 m; white top median 0.03524 m. Implied ring height about 28–32 mm and white height about 80 mm, with depth noise. | Upright parts on this table; remeasure current scene. | frames/0001–0003 calibrated depth, supplied geometry.py and measure.py outputs. | medium |
| Downward quaternion (0,1,0,0), ring grasp command z=-0.019 (achieved -0.01839), closed width 0.0516 m held the ring after lifting. | External grasp of smooth blue ring, closure along base y. | frames/0004_wrist.png and close/lift responses. | high |
| Concentric command XY=(0.262,0.054), descent z=0.050,0.015,-0.012 succeeded without correction; final achieved z=-0.01414 with 4.0 mm target error. | XY is scene-dependent; post center measured near (0.262,0.054). | frames/0005–0006 wrist and frames/0007–0008 top. | high |
| Opening to width 0.0949 m, lifting to z=0.150 and withdrawing to (0.190,0.045,0.200) left assembly stable. | Withdrawal path must be checked in each scene. | frames/0007_top.png and frames/0008_top.png. | high |
