# playbook: dual-corner first fold with forward tilt (partial task only)

<!-- from session 20260909_115607_paper_towel_full_high_yieldbatch_v2_kn_dual_04, merged 2026-09-09 -->
This block records only the first long-edge fold that worked. The full folding task was unsuccessful. All x/y coordinates below are scene-dependent example values from the measured layout, not reusable targets. Quaternion and clearance choices still require reach and whole-arm checks.

1. Observe both arm states, localize both near corners using overhead and wrist depth. Open to fraction 0.5, use downward wxyz=(0,1,0,0), and grasp near each corner. In this run achieved centers were near left (0.166,0.011,-0.033) and right (0.152,-0.694,-0.033). Confirm nonempty width and a roughly 40 mm lift visually.
2. Carry both corners upward to z=0.17 m, keeping their approximately 0.70 m y separation; example x values were left 0.130 and right 0.125. Observe and remeasure the far edge because it moved.
3. Carry forward through example positions left (0.320,0.008,0.170), right (0.305,-0.691,0.170), with wxyz=(0,0.965926,0,0.258819), a 30-degree forward tilt. Observe both holds.
4. Approach the newly measured far edge with 60-degree forward tilt wxyz=(0,0.866025,0,0.5). Example intermediate positions were left (0.464,0.010,0.025), right (0.450,-0.694,0.025). Observe.
5. Place at example left (0.471,0.010,-0.023), right (0.456,-0.694,-0.023), same tilt. Open, inspect release, and retreat about (-0.070,0,+0.130). The resulting first-fold strip was broadly flat. Re-localize before either end fold.
