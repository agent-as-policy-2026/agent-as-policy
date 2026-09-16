# playbook: recover a square cube after a transport slip

<!-- from session 20260903_145637_pyramid3, merged 2026-09-03 -->
1. Stop relying on the old grasp state; refresh state and capture both cameras.
2. Open the gripper and withdraw vertically or to a clear high waypoint without changing orientation near the build.
3. Re-localize the cube from the wrist RGB-D at its new upright landing pose. Use the top-face plane and depth result together; align the gripper with the cube's current edge angle.
4. Descend in two stages, correcting any several-millimetre tracking offset before closing. Require about `0.050-0.052 m` here and verify a short lift visually.
5. Keep wrist yaw fixed, use roughly 2-6 cm transport segments, capture after each, and optionally re-close between segments if the width relaxes.
