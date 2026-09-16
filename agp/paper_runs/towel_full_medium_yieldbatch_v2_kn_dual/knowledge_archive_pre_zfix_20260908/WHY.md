Rows archived 2026-09-08 20:30 (removed from knowledge/RIG_NOTES.md, not served to agents).
Source: session 20260908_175849 (dual towel run 2), recorded while the right arm used the
cross-calibrated base transform with t_z = -0.01406 m. That z was wrong by ~13 mm (see
free_agent/config/right_base_in_left_base.json "z_correction"); every right-arm z in these
rows is offset: z_now = z_row + 0.014. Row 3 ("wrist depth disagrees with nominal table")
described the bug itself and is obsolete. Left-arm rows were correct but were archived with the
set so the store restarts clean under the corrected transform.
