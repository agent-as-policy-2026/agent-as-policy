# playbook: projection-calibrated top-down pickup

<!-- from session 20260903_142826_pyramid2, merged 2026-09-03 -->
1. Move to a top-down pose about 0.10 m above the candidate block and capture wrist RGB-D.
2. From that capture's calibration, project the horizontal top-face plane point directly beneath the end-effector into the wrist image. Compare it with the observed center of the block's top face.
3. Correct x/y until the observed top-face center and the projected point-under-tool coincide within a few pixels. Align the closing axis with a block edge.
4. Descend to grasp-center `z≈-0.020` m and close. A centered block in this run reported about 0.0502 m.
5. Lift only to `z≈0.04` m and re-image. Continue only if the block remains large and fixed between the fingers (`frames/0019_wrist.png`); a block that shrinks in the wrist view stayed on the table.
6. Lift to transport height without yawing the tool. Use short Cartesian segments if long moves trip stale-feedback checks.
