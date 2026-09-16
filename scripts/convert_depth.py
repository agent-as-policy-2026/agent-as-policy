#!/usr/bin/env python3
"""Convert wrist depth from float32 .npy to uint16 millimetre PNG.

`.npy` is not a format the Hub understands: it would not appear in the viewer,
would not be converted to parquet and would be absent from croissant. Storing
depth as 16-bit PNG in millimetres keeps it a first-class image while cutting
the size to roughly 4.5% of the original. Quantisation step is 1 mm, so the
round-trip error is bounded by 1 mm.
"""
import argparse, os, sys
import numpy as np
from PIL import Image

from _paths import add_fa_arg, require_sessions

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--limit", type=int, default=0, help="only convert the first N (0 = all)")
    add_fa_arg(ap)
    a = ap.parse_args()
    sessions = [l.strip() for l in open(a.snapshot) if l.strip()]
    require_sessions(a.fa, sessions, "--snapshot")
    n = src = dst = 0
    errs = []
    for sid in sessions:
        d = os.path.join(a.fa, "sessions", sid, "frames")
        if not os.path.isdir(d):
            continue
        od = os.path.join(a.out, "depth", sid)
        made = False
        for fn in sorted(os.listdir(d)):
            if not fn.endswith("_wrist_depth.npy"):
                continue
            p = os.path.join(d, fn)
            try:
                arr = np.load(p)
            except Exception as e:
                errs.append((p, str(e)[:60])); continue
            if not made:
                os.makedirs(od, exist_ok=True); made = True
            mm = np.nan_to_num(arr.astype(np.float32) * 1000.0, nan=0.0, posinf=0.0, neginf=0.0)
            mm = np.clip(mm, 0, 65535).astype(np.uint16)
            op = os.path.join(od, fn.replace("_wrist_depth.npy", "_wrist_depth.png"))
            Image.fromarray(mm).save(op, format="PNG", optimize=True, compress_level=9)
            src += os.path.getsize(p); dst += os.path.getsize(op); n += 1
            if a.limit and n >= a.limit:
                break
        if a.limit and n >= a.limit:
            break
    print(f"  转换 {n} 个  {src/2**30:.2f} GiB -> {dst/2**30:.2f} GiB  ({dst/src*100:.1f}%)")
    if errs:
        print(f"  失败 {len(errs)} 个:")
        for p, e in errs[:5]:
            print(f"    {os.path.basename(p)}: {e}")

if __name__ == "__main__":
    main()
