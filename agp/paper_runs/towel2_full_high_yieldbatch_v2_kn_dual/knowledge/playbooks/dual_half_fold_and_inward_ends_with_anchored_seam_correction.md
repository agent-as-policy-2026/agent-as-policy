# playbook: dual half-fold and inward ends with anchored seam correction

<!-- from session 20260909_154653_paper_towel2_full_high_yieldbatch_v2_kn_dual_04, merged 2026-09-09 -->
Worked in this run with a narrow center seam, slight skew, and a small right near-corner curl. All x/y coordinates below are placement-dependent examples. Heights must be checked against local depth and achieved robot poses. Keep both arms' paths and bodies separated; read the other arm's state before motion and inspect frames after each consequential action.

1. Study the demonstration and measure the flat towel. This run was about 0.72 by 0.45 m. Use downward q=(0,1,0,0) for initial pinches.
2. Pinch left (0.120,-0.033,-0.0335), right (0.121,-0.708,-0.033). Close, inspect, lift 65 mm, inspect both holds. Working settled widths were 5.8 and 8.7 mm.
3. Carry both corners concurrently to x about 0.31, z=0.155 with q=(0,0.965926,0,0.258819); then x about 0.45, z=0.080 with q=(0,0.92388,0,0.382683). Keep their y coordinates separated by about 0.675 m.
4. Place at x=0.505/0.507, z=-0.022, maintaining y and 45-degree tilt. Open, inspect, retreat 100 mm backward and 120 mm upward. Re-measure the doubled short edges.
5. Pinch their midpoints downward: x=0.445, left y=-0.015/z=-0.033, right y=-0.708/z=-0.032. Close and lift 55 mm; inspect actual layered holds.
6. Stage concurrently at x=0.445, left y=-0.180, right y=-0.550, z=0.110, 45-degree forward tilt. Place left at (0.445,-0.350,-0.014). Release, retreat backward/upward/toward own side, and home. Place right at (0.445,-0.355,-0.014), release and withdraw. Reobserve the shifted packet.
7. If a substantial left inner hem is turned back, re-measure its raised part. This run's command (0.412,-0.266,-0.011) downward achieved z=-0.024 and held 8.5 mm of cloth; inspect carefully. Lift to (0.411,-0.265,0.035) with 45-degree tilt, then extend to (0.411,-0.325,-0.014), release, retreat and home. This flattened the left flap.
8. If the right near free corner remains short of center, use an opposite anchor. This run's left anchor was (0.515,-0.140,-0.030), 45-degree tilt, achieved z about -0.024. Close and visually verify a hold. Right used the raised corner at (0.368,-0.393,0.008), downward, achieved z=0.012. Close and inspect.
9. With left stationary, carry right via (0.360,-0.320,0.025) to (0.350,-0.307,-0.016), 45-degree tilt. Check left still holds and that neither gripper approaches the other's held section. Release right, then left; retreat in diverging directions and home both. Confirm final overhead image and open gripper states.
