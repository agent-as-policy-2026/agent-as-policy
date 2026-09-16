# playbook: transport a loose-taped cube without pad slip

<!-- from session 20260903_155934_pyramid5, merged 2026-09-03 -->
1. At a high top-down approach, inspect the cube yaw and visible tape direction in the wrist camera.
2. Choose the perpendicular face pair so both pads meet rigid cube faces rather than a loose tape strip. For the final cube pose in this run, a 45-degree top-down roll (`w=0, x=0.9239, y=0.3827, z=0`) made the cube edges axis-aligned in `frames/0054_wrist.png`.
3. Center using the projected under-tool pixel and wrist depth, descend to cube center height, close, and verify a short lift. Require the cube to remain large and fixed in the wrist view.
4. Carry in roughly 25-30 mm Cartesian steps without yawing. Re-close after every step and re-image; stop immediately if the cube shrinks or changes pose relative to the fingers.
5. At placement, use only enough jaw opening to clear the measured held width, lift vertically, verify the cube stayed, then fully open after the fingers are clear.
