# playbook: relocate a doubled towel using separated short-end grips

<!-- from session 20260909_162537_paper_towel2_full_high_yieldbatch_v2_kn_dual_05, merged 2026-09-09 -->
This procedure successfully moved the half-folded towel toward the arms while preserving both holds; it does not establish accurate final flap alignment.

1. Observe both arm states and locate both doubled short edges in overhead and wrist images. Choose one reachable edge per arm. All following x/y values are placement-dependent examples.
2. Approach downward, q=(0,1,0,0), at left (0.500,-0.005,0.020), right (0.470,-0.688,0.020). Inspect wrists.
3. Descend to commanded z=-0.033 left and -0.032 right, after confirming the local table and layered cloth heights. Inspect before closing.
4. Close and inspect; this run settled near 5.0 mm left and 5.4 mm right opening. Width alone is insufficient evidence of a durable grip.
5. Move both concurrently toward the robots by 0.130 m in x and up to z=0.025: left (0.370,-0.005,0.025), right (0.340,-0.688,0.025), same downward orientation. The grippers remain over 0.65 m apart.
6. Inspect overhead and both wrists. In this run both moves returned SETTLE_MISS with approximately 8–11 mm error, but fresh images confirmed both holds and successful towel relocation. Do not assume future settle errors are harmless.
7. Continue only from the measured achieved poses and observed cloth state.
