# playbook: stage before a diagonal seam-side placement

<!-- from session 20260904_132159_pyramid9, merged 2026-09-04 -->
1. Use wrist RGB-D at a high top-down pose to estimate the cube's top-surface
   center in base coordinates; do not rely on the outer silhouette center in
   the fixed top camera when side faces are visible.
2. Face-grasp and verify a short lift. Use this stable grasp for long transport
   and any required yaw correction.
3. Put the cube upright on a clear staging spot close to its final destination
   and withdraw vertically.
4. Re-center over the staged cube, rotate to the diagonal roll, and close. For
   a centered approximately 50 mm cube, the diagonal close was about 68 mm in
   this run.
5. Verify a short lift, then make only the short final carry. Release around
   80% for a 68 mm diagonal hold, and withdraw vertically.

Demonstrated for one successful short placement from `frames/0046_top.png`
through `frames/0055_top.png`. The failed long diagonal carry ending at
`frames/0077_top.png` is the reason to keep the diagonal segment short.
