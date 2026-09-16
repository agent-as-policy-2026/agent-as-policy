# playbook: canonical hex and yaw-90 circular assembly

<!-- from session 20260910_202519_paper_twopairs_full_high_kn_6astra_05, merged 2026-09-10 -->
1. Study the demonstration, identify only the left set, and capture both cameras. Check all approach clearances including the wrist-camera housing.
2. Observe at (0.250,0.020,0.180) with quaternion (0,1,0,0); observation x/y are scene-dependent. Remeasure collar and post centers with closer wrist views.
3. Hex: approach above its center at z=0.090; use canonical quaternion. Descend to z=-0.029, close, inspect width and image, lift 0.130, and inspect retention. This run's pick x/y=(0.248,-0.112) is scene-dependent; returned width was 47.1 mm.
4. Measure narrow-post top from above at grasp z=0.100. Use its center for insertion; this run used x/y=(0.169,-0.023), scene-dependent. Descend through z=0.052, 0.020, -0.014, inspecting after each. Confirm the post passes through before the last descent. Open, inspect, withdraw 0.150, and verify stability.
5. For the circular collar, rotate while elevated to quaternion (0,0.70710678,0.70710678,0). This placed the housing away from the nearby white cylinder in this layout. Observe at z=0.120, then z=0.070, and remeasure in that orientation.
6. Set opening fraction 0.8 (77.1 mm returned), inspect clearance, descend to z=-0.028 at the measured circular center, close and inspect. This run's pick x/y=(0.217,0.139) is scene-dependent. Returned width was 51.9 mm. Lift 0.140 and confirm retention and neighbor stability.
7. Transfer above the measured wide-cylinder center near grasp z=0.110. Inspect any convergence miss and use achieved pose plus fresh images; never repeat an identical unsuccessful target. Close-view white-face depth supported a corrected insertion center x/y=(0.277,0.086), scene-dependent.
8. With the same yaw-90 orientation, descend through grasp z=0.064, 0.021, -0.014 with visual checks. Verify engagement before the lowest descent. Open at the achieved low pose, inspect settling, withdraw 0.150, and confirm stability.
9. Return home to observation, capture both cameras, verify both left assemblies stable and arm withdrawn, and check the gripper is open. All x/y values must be remeasured for each scene; recheck heights and clearance before applying the grasp heights.
