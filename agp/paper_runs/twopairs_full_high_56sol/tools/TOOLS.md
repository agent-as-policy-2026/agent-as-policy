# Tools left by previous sessions (index generated from the file headers)

| file | category | purpose | usage | assumptions |
|---|---|---|---|---|
| `analyze_depth.py` | geometry | Summarize base-frame XYZ extents of wrist RGB-D points inside an image ROI and height band. | `python3 knowledge/tools/analyze_depth.py <depth.npy> <calib.json> --roi X0 Y0 X1 Y1 --z ZMIN ZMAX` | Depth is aligned to the wrist intrinsics; calibration uses x/y/z position, w/x/y/z quaternion, and OpenCV-compatible distortion coefficients. |
