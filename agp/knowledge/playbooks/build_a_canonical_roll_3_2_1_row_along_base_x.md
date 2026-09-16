# playbook: build a canonical-roll 3-2-1 row along base x

<!-- from session 20260903_210521_pyramid7, merged 2026-09-03 -->
1. Choose an anchor cube around radius 0.37 m and make the pyramid row run along base x, leaving base y as the gripper closing direction.
2. Use approximately 50 mm center spacing for the bottom row and half-spacing offsets for higher tiers.
3. Keep the canonical top-down roll throughout pickup, carry, and placement whenever source cubes are face-aligned.
4. Place the two middle cubes at center height about `0.030 m` and the top cube at about `0.080 m` for a table near `z=-0.045 m`.
5. Release each stacked cube at about 65% jaw opening, verify it remains seated, then lift vertically before fully opening in clear air.
6. Before homing, lift above the completed top and translate to a clear waypoint so the subsequent orientation change cannot sweep the stack.

Evidence: this sequence produced the stable final structure in `frames/0061_top.png` and `frames/0061_wrist.png`.
