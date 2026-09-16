# playbook: isolate and unfold a raised first-fold cuff

<!-- from session 20260908_204543_paper_towel_full_high_yieldbatch_v2_kn_dual_01, merged 2026-09-08 -->
This is a locally successful cuff correction, not a validated recipe for completing the entire task. The run did not achieve satisfactory final alignment.

1. Withdraw the other arm and inspect the raised cuff with wrist RGB-D. In the successful right-side example, the cuff surface was near z=-0.011 to -0.015 m and the lower cloth was near z=-0.032 m. Re-measure these values for the current scene.
2. Orient top-down with jaws closing along world x: quaternion (w,x,y,z)=(0,0.707107,0.707107,0). Reachability must be tested at the intended pose.
3. Approach the raised hem shallowly. Example location-dependent target: x=0.449, y=-0.590, z=-0.008 m; achieved z was about -0.0115 m. Do not descend to the lower layer.
4. Close and inspect. The successful example retained approximately 0.0078 m width. Lift to target z=0.055 m (achieved about 0.047 m) and visually confirm that the cuff is held.
5. Carry the hem approximately 0.075 m forward in world x while lowering toward z=0.012 m, with quaternion (0.183013,0.683013,0.683013,0.183013). The example target x=0.524, y=-0.590 depends on the cuff location. Inspect the achieved pose: this move reported a settle miss and achieved approximately x=0.521, z=-0.0004 m.
6. Open once the cuff is laid forward. Inspect the release, withdraw, and verify the cuff has unfolded. In this run this removed the remaining broad right cuff, but did not make the underlying half fold perfectly aligned.
