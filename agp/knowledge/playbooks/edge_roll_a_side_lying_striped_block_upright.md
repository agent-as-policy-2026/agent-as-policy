# playbook: edge-roll a side-lying striped block upright

<!-- from session 20260903_122441_pyramid1, merged 2026-09-03 -->

1. From an unobstructed overhead frame, deproject the center of the side-lying block on the top-face plane (`z` about 0.005 m).
2. Approach vertically with the gripper closing axis aligned to a block edge, descend to grasp-center `z` about -0.020 m, and close. Require a clearly nonzero width (about 0.051 m in this run).
3. Lift and translate to empty space before changing orientation.
4. Rotate about the tool's closing axis by roughly 40-60 degrees while well above the table. For a vertical grasp with yaw `theta`, a 60-degree tool-y roll has quaternion approximately `(-0.5*sin(theta/2), 0.866*cos(theta/2), 0.866*sin(theta/2), 0.5*cos(theta/2))` in scalar-first order.
5. Refresh robot state after any feedback error. Using the actually achieved pose and orientation, lower the grasp point to about `z=-0.005` m, open, and lift away without changing orientation near the block. Gravity can roll the block onto its base.
6. Return to the observation posture and confirm the striped face is upward before attempting a normal top-down placement.

This procedure succeeded for a gray block (`frames/0036_top.png` to `frames/0038_top.png`) and a blue block (`frames/0075_top.png` to `frames/0079_top.png`).
