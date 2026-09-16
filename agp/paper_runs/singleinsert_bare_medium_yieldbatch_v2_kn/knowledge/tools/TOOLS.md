# Tools left by previous sessions (index generated from the file headers)

| file | category | purpose | usage | assumptions |
|---|---|---|---|---|
| `geometry.py` | geometry | Convert wrist depth samples or camera rays at a supplied height to base-frame points. | `python3 knowledge/tools/geometry.py CALIB {points DEPTH --uv U V [--uv U V] | plane --uv U V --z Z}    (run from the session directory)` | numpy/scipy; calibration camera pose and 3x3 intrinsics; scalar-first quaternion keys; depth in metres; points use a positive-depth 5x5 median by default. |
| `measure.py` | geometry | Summarize base-frame point bounds for a selected ring or post region, with optional color and height filters. | `python3 knowledge/tools/measure.py CALIB DEPTH --roi X1 Y1 X2 Y2 [--image RGB --rgb-min R G B --rgb-max R G B] [--z-range LOW HIGH]    (run from the session directory)` | numpy/scipy/Pillow and adjacent geometry.py; wrist RGB/depth registered; half-open pixel ROI; strict RGB/height bounds; midpoint is a visible-surface estimate, not a circle fit. |
