#!/usr/bin/env python3
"""Print the intrinsics views that the solver's EXACT detector finds in 3
consecutive attempts (findChessboardCornersSB EXHAUSTIVE is flaky on some real
1080p frames and the solver aborts on a single miss). Usage:
  python filter_stable_views.py <dir> <calib|val>   -> stable paths on stdout,
  flaky ones reported on stderr."""
import sys, glob, cv2
from agp_yam_bridge.camera_acceptance import _detect_checkerboard
d, prefix = sys.argv[1], sys.argv[2]
for f in sorted(glob.glob(f"{d}/{prefix}_*.png")):
    rgb = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2RGB)
    ok = 0
    for _ in range(3):
        try:
            _detect_checkerboard(rgb, columns=9, rows=7, label="filter"); ok += 1
        except Exception:
            break
    if ok == 3:
        print(f)
    else:
        print(f"flaky ({ok}/3), excluded: {f}", file=sys.stderr)
