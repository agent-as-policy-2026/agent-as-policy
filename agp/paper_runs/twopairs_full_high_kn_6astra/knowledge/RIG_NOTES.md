# Task notes — facts earlier operators learned doing THIS task on THIS robot

Notes left by previous autonomous sessions (and by an offline consolidation of
their reports). Treat them as evidence, not orders: a note can be wrong or out of
date. If you find one is wrong, add a new row that says so with your own
evidence; do not delete rows.

Each row: fact | applies when | evidence | confidence (low/med/high).


<!-- from session 20260910_192229_paper_twopairs_full_high_kn_6astra_01, merged 2026-09-10 -->
# Observed task notes

fact | applies when | evidence | confidence
--- | --- | --- | ---
Both white top faces were near base z=0.034 m, approximately 79 mm above the nominal table z=-0.045 m. | Upright white parts on this table; remeasure in a fresh run. | Wrist depth deprojection in capture 0002 at narrow top (181,130) and wide top (478,108) both gave z=0.0341 m. | medium
Hexagonal collar top was approximately z=-0.027 m (18 mm above nominal table); circular rim approximately z=-0.024 m (21 mm above nominal table). | Loose collars resting flat; measurements are approximate stereo depth estimates. | Capture 0002 hex edge depths -0.0277/-0.0265/-0.0272 m; circular rim -0.0233/-0.0254 m. | medium
Hex collar could be externally grasped at grasp z=-0.029 m with straight-down quaternion (0,1,0,0); closing returned 47.5 mm opening, fraction 0.4979. | Hex oriented with opposing flats approximately along base y; x/y depend on scene. | Captures 0005 and 0006 show contact and successful 130 mm lift. | high
Circular collar could be externally grasped at grasp z=-0.028 m with straight-down yaw-90 quaternion (0,0.70710678,0.70710678,0); closing returned 51.9 mm opening, fraction 0.5432. | Jaws close along base x, useful when another part is beside the collar along base y; x/y depend on scene. | Captures 0015 and 0016 show contact and successful 130 mm lift without touching adjacent white cylinder. | high
Hex collar inserted over narrow post via grasp heights 0.052 then 0.020 m, followed by low seating and release. | Collar held near its top, as in the recorded grasp. | Capture 0009 shows post through hole; capture 0012 shows released assembly. | high
A hex seating target of z=-0.022 m returned SETTLE_MISS at z=-0.01708 m; opening there allowed a stable seated assembly. | Same grasp geometry, with post visibly through hole; do not interpret every SETTLE_MISS as successful seating. | Captures 0010, 0011, 0012 and final 0023; returned total target error 5.7 mm. | high
Circular collar inserted via grasp heights 0.054, 0.021, -0.014 m; achieved final z=-0.01547 m; release let it settle farther down. | Same circular grasp geometry; remeasure target center. | Captures 0019 through 0023 show engagement, settling on release, and stability after withdrawal. | high
A small grasp-to-hole offset can remain along the axis the fingers do not close along; a roughly -3 mm base-y target correction worked for this circular grasp. | Yaw-90 grip with collar initially about 3 mm ahead of grasp center along base y; offset depends on the actual grasp. | Wrist center deprojection in capture 0013 estimated collar y=0.0477 versus pick y=0.044; used insertion grasp y=-0.039 for post near y=-0.036; captures 0019-0023 confirm success. | medium

<!-- from session 20260910_194115_paper_twopairs_full_high_kn_6astra_02, merged 2026-09-10 -->
# Observations from this run

fact | applies when | evidence | confidence
--- | --- | --- | ---
Narrow white top z=0.0344 m and wide white top z=0.0356 m, about 79–81 mm above nominal table z=-0.045 m. | Upright posts; stereo measurements, not precision metrology. | Capture 0002 wrist depth at (173,84) and (349,229). | medium
Hex collar top z about -0.027 m, about 18 mm above nominal table. | Loose hex collar flat on table. | Capture 0003 depth at (340,184) gave -0.0273; (292,163) gave -0.0261 m. | medium
Circular collar rim measured z=-0.0144 to -0.0164 m, about 29–31 mm above nominal table; this differed from older notes and should be remeasured. | Loose circular collar upright; thin rims can produce uncertain stereo depth. | Capture 0013 rim samples (283,170), (367,170), (327,207); earlier capture 0002 samples gave -0.0173/-0.0180 m. | medium
Hex external grasp at z=-0.029 m with quaternion (0,1,0,0) succeeded at width 48.0 mm, fraction 0.503. | Opposing sides aligned approximately with base-y jaw closure; center depends on scene. | Captures 0005–0006 show grasp and retention after 130 mm lift. | high
Circular external grasp at z=-0.028 m with canonical quaternion (0,1,0,0) succeeded at width 51.9 mm, fraction 0.5438. | Adequate clearance along base y; a yaw-90 tool rotation was unnecessary in this scene. | Captures 0015–0016 show grasp and retention after 130 mm lift. | high
Hex seated stably after requested insertion heights 0.052, 0.020, -0.014 m, then opening at achieved z=-0.01582 m. | Same grasp height and upright narrow counterpart, with post visibly through hole before final descent. | Captures 0009–0012 and 0023; all moves returned completed without SETTLE_MISS. | high
Circular collar engaged through heights 0.054, 0.021, -0.014 m; opening at achieved z=-0.01282 m let it settle farther down stably. | Same grasp geometry and upright wide cylinder; verify engagement first. | Captures 0019–0023. | high
Measured post centers could be used without an additional empirical insertion offset in this run. | Canonical orientation and these centered grasps only; do not assume every grasp has zero offset. | Hex target (0.294,0.114), circular target (0.182,0.004) m; successful staged insertion and final images. | medium

<!-- from session 20260910_195248_paper_twopairs_full_high_kn_6astra_03, merged 2026-09-10 -->
# Observations from this run

fact | applies when | evidence | confidence
--- | --- | --- | ---
The wrist-camera housing can contact a tall white part even when the open fingers clear a collar. A white cylinder approximately 90 mm ahead along base x obstructed a canonical hex descent. | Canonical downward approach with a tall neighboring part on the camera side; check the housing as well as fingers. | frames/0003_top.png to frames/0005_top.png; SETTLE_MISS at achieved (0.2125,0.1541,-0.0225) for requested (0.198,0.150,-0.029); cylinder shifted but stayed upright. | high
A yaw-90 hex grasp at z=-0.029 m held through transport at width 47.8 mm, but this grip repeatedly caught at the post top; setting down in clear space and regrasping canonically resolved it. | This collar orientation and grasp; yaw-90 is not proven unsuitable in general. | frames/0008–0020 show held collar and insertion misses; frames/0024–0035 show regrasp and successful assembly. | high
Canonical external hex grasp at z=-0.029 m succeeded at width 52.6 mm, fraction 0.5512, despite being wider than earlier 48 mm grasps. | Collar with corners presented to jaws; adequate clearance. | frames/0028_wrist.png and frames/0029_wrist.png show contact and retention after 130 mm lift. | high
Hex top was approximately z=-0.0274 m, about 18 mm above nominal table; narrow post top approximately z=0.0331 m, about 78 mm above table. | Flat loose hex and upright narrow post; approximate stereo measurements. | Capture 0003 wrist (320,137); capture 0030 wrist (325,182). | medium
Hex insertion with canonical orientation succeeded through grasp z=0.052, 0.020, -0.014 m and release at achieved -0.01420 m. | Grasp near collar top; center must be remeasured after any contact. | frames/0031–0035; insertion x/y=(0.269,-0.130) for measured post near (0.271,-0.130), scene-dependent. | high
Circular collar rim samples were z=-0.0144 to -0.0182 m, approximately 27–31 mm above nominal table. | Loose circular collar; thin rim stereo samples. | Capture 0036 wrist (370,53), capture 0037 wrist (326,132) and (285,174). | medium
Canonical circular grasp succeeded at requested z=-0.028 m, achieved z=-0.03155 m, width 51.7 mm, fraction 0.5409. | External grasp with sufficient clearance; grasp center varies by scene. | frames/0038–0040 show approach, closure and retention after 140 mm lift. | high
Circular insertion succeeded through z=0.054, 0.021, -0.014 m, with release at achieved -0.01396 m allowing additional settling. | Same grip geometry and upright wide counterpart. | frames/0042–0047 show engagement, release, settling and final stability. | high
The same stationary hex collar produced wrist plane-center estimates differing by about 7 mm in y after a 90-degree wrist turn; treat cross-orientation millimetre estimates cautiously. | Comparing wrist measurements across wrist orientations; cause not established, includes pixel-selection/calibration uncertainty. | Capture 0025 (330,142), z=-0.027 gave (0.1819,-0.0210); capture 0026 (328,138) gave (0.1842,-0.0281). | medium

<!-- from session 20260910_201525_paper_twopairs_full_high_kn_6astra_04, merged 2026-09-10 -->
# Observations from this run

fact | applies when | evidence | confidence
--- | --- | --- | ---
Narrow white post top measured z=0.0332 m and wide white cylinder top z=0.0355 m, approximately 78 and 81 mm above nominal table z=-0.045 m. | Upright white parts; stereo estimates, not precision metrology. | Wrist deprojection at capture 0007 (326,196) and capture 0013 (175,201). | medium
Hex collar top samples were z=-0.0274 and -0.0286 m, approximately 16–18 mm above nominal table. | Loose hex collar flat on table. | Capture 0002 wrist (221,122) and (205,147). | medium
Circular rim samples were z=-0.0156, -0.0171, -0.0182 m, approximately 27–29 mm above nominal table. | Loose circular collar upright; sample thin rims cautiously. | Capture 0013 wrist (283,140), (318,110), (353,144). | medium
Canonical external hex grasp at z=-0.029 m succeeded at width 53.4 mm, fraction 0.5589. | This collar presented corners toward the jaws; quaternion (0,1,0,0), x/y scene-dependent. | Capture 0005 contact and capture 0006 retention after 130 mm lift. | high
Canonical circular grasp at z=-0.028 m succeeded at width 51.8 mm, fraction 0.5424. | Upright circular collar; quaternion (0,1,0,0), x/y scene-dependent. | Captures 0016–0017 show contact and retention after 140 mm lift. | high
A requested opening fraction of 0.8 returned 77.0 mm and allowed canonical descent around the circular collar beside a white cylinder approximately 78 mm away along base y and 13 mm along x. | This relative layout; check both jaw and camera-housing clearance for new layouts. | Capture 0013 measurements and captures 0014–0017; white cylinder stayed upright and stationary. | high
Hex insertion succeeded at requested grasp z=0.052, 0.020, -0.014 m; release at achieved z=-0.01562 m left a stable assembly. | Canonical grasp at z=-0.029 m, with correct post alignment. | Captures 0008–0012 and final 0024. | high
Circular insertion succeeded at requested grasp z=0.054, 0.021, -0.014 m; release at achieved z=-0.01352 m allowed additional settling and stable engagement. | Canonical grasp at z=-0.028 m, with correct cylinder alignment. | Captures 0019–0024. | high
A white top-face center can have no valid depth even though another view provided a valid top measurement. | Untextured white surfaces under wrist illumination. | Capture 0018 wrist (322,195) failed depth; capture 0013 wrist (175,201) succeeded. | high

<!-- from session 20260910_202519_paper_twopairs_full_high_kn_6astra_05, merged 2026-09-10 -->
# Observations demonstrated in this run

fact | applies when | evidence | confidence
--- | --- | --- | ---
Hex collar top measured z=-0.0256 to -0.0280 m, about 17–19 mm above nominal table z=-0.045. | Loose hex lying flat; close wrist stereo estimates. | Capture 3 wrist pixels (330,135), (300,163), (357,169). | medium
Narrow post top measured z=0.0319 m, about 77 mm above nominal table; wide white top close samples measured z=0.0359/0.0363 m, about 81 mm above table. | Upright white parts; approximate stereo measurements. | Capture 7 wrist (322,197); capture 19 wrist (337,203) and (323,203). | medium
Circular top-rim samples measured z=-0.0138 to -0.0153 m, about 30–31 mm above nominal table. | Upright circular collar, close yaw-90 wrist view. | Capture 14 wrist (325,147), (279,188), (367,191). | medium
Canonical external hex grasp at requested z=-0.029 m worked at width 47.1 mm, fraction 0.4935. | Straight-down quaternion (0,1,0,0), opposing flats near base-y closure direction; center depends on scene. | frames/0005_wrist.png contact and frames/0006_wrist.png retention after 130 mm lift. | high
Hex insertion at grasp z=0.052, 0.020, -0.014 m worked; opening at achieved -0.01351 m left a stable assembly. | Same canonical grasp geometry and independently measured narrow-post center. | frames/0009_wrist.png engagement, frames/0012_wrist.png release stability, frames/0025_top.png final. | high
Yaw-90 circular grasp with requested opening fraction 0.8 (77.1 mm) cleared a nearby upright white cylinder; close at requested grasp z=-0.028 m returned 51.9 mm, fraction 0.5437. | Quaternion (0,0.70710678,0.70710678,0); nearby cylinder about +60 mm base x and -50 mm base y from collar, approximate scene-dependent separation. | frames/0015–0018 show approach, grip, lift, and unchanged upright neighbor. | high
Yaw-90 circular insertion worked at grasp z=0.064, 0.021, -0.014 m, followed by opening at achieved z=-0.01562 m and additional settling. | Same external grip; insertion center freshly measured in the same wrist orientation. | frames/0021–0025 show engagement, release, withdrawal, and stability. | high
A transfer SETTLE_MISS can occur while a held collar is visibly clear above its counterpart; inspect rather than assuming contact. | This run's target z=0.110 m stopped at z=0.10719, reported error 3.3 mm. | frames/0019_wrist.png and frames/0019_top.png show clearance; next corrected target completed normally. | high
Sparse depth at distant untextured white top faces gave less consistent heights than the closer view. | Initial observation samples had only 1 or 6 valid pixels; close views had 20–25. | Capture 2 gave white tops near z=0.026/0.0254; captures 7 and 19 gave about 0.032/0.036. | medium
