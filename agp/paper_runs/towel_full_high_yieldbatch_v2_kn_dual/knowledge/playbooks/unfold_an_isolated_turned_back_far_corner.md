# playbook: unfold an isolated turned-back far corner

<!-- from session 20260908_220219_paper_towel_full_high_yieldbatch_v2_kn_dual_02, merged 2026-09-08 -->
This is a demonstrated corner-recovery procedure, not a fully successful end-to-end folding recipe. It succeeded for the two far triangles; remeasure all position-dependent coordinates and check the other arm's state before each motion.

1. Withdraw the inactive arm. Observe both the overhead and the active wrist image. Identify the actual free tip where the two hem segments meet; do not select the middle of the hem or the crease endpoint.
2. Measure that tip using wrist depth. For the first successful right correction, a tip near (0.391,-0.442,-0.016) m was grasped slightly inward at commanded (0.405,-0.444,-0.017) m; these positions are scene-dependent. Use about 0.30 gripper fraction (29 mm) and a 15-degree forward tilt, quaternion wxyz=(0,0.991445,0,0.130526).
3. Close, inspect residual opening and images, then request a short 35 mm vertical lift. Observe whether the tip follows. Close again to take up slack and observe again. In this instance the opening reduced from about 6.7 mm to about 0.9 mm and the cloth remained held.
4. Carry the tip partway toward its unfolded destination, first by about (+45,+30,+25) mm in this right-corner example. These displacement signs and distances depend on which corner is turned back. Verify continued cloth motion.
5. Place the corner near the far end of the center seam. This example used commanded (0.496,-0.338,0.005) m with forward 45-degree tilt, wxyz=(0,0.92388,0,0.382683); achieved center was about (0.492,-0.336,-0.004) m. The destination is scene-dependent and must remain above the table.
6. Open, inspect release, withdraw the active arm, and confirm from overhead that the corner remains unfolded. The first successful release was `frames/0061_top.png`.
7. A second corner needed 45-degree forward tilt at grasp and 60 degrees on the carry because it was farther away. It also moved the bundle. Treat that as a reach-specific adaptation requiring fresh observations, not permission to assume an extended pose will settle precisely.
