# playbook: flatten an exposed raised near hem

<!-- from session 20260909_132848_paper_towel_full_high_yieldbatch_v2_kn_dual_05, merged 2026-09-09 -->
This local correction worked; it is not a verified complete folding recipe. All absolute XYZ coordinates below are example values dependent on where the towel lies. Quaternions are scalar first.

1. Withdraw the other arm and inspect the raised near hem with wrist RGB-D. In this run the measured exposed point was approximately (0.226,-0.326,0.033) m.
2. Open the working gripper to 0.4 (about 38 mm). Approach with quaternion (0,1,0,0) at example grasp point (0.228,-0.328,0.024) m, with the jaws straddling the hem.
3. Close. This grip settled to only about 0.4 mm opening, so do not infer success from width alone.
4. Lift and draw slightly toward the near side, to example (0.20,-0.327,0.046) m. Inspect a new frame and confirm the hem follows.
5. Extend a further 20 mm toward the near side and lower to example (0.18,-0.328,-0.005) m, using quaternion (0,0.965926,0,-0.258819). The total nearward displacement from the grasp is about 48 mm. Confirm the achieved pose and cloth shape.
6. Release to opening fraction 0.35 and retreat upward into the working arm's own side. Inspect the overhead image. In this run the raised left near corner became flat; the operation did not fix the separate right-panel buckle.
