# playbook: small in-air yaw correction on a verified cube

<!-- from session 20260903_171812_pyramid6, merged 2026-09-03 -->
1. Lift the face-aligned cube to about `z=0.10 m` in clear air without changing orientation, and confirm that it remains large and fixed between the pads.
2. Re-close once to take up jaw relaxation; verify the close remains near the demonstrated centered width (about 0.0495 m here).
3. Use a single base-z `move_delta` yaw of only the required small angle (10-15 degrees was demonstrated), with no translation.
4. Capture the wrist and overhead cameras immediately. Continue only if the cube remains fixed relative to the wrist camera; otherwise open, withdraw, and re-localize the dropped cube.
