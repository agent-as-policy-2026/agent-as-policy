#!/usr/bin/env python3
"""Copy the published videos into the videofolder layout.

Destination paths must match videos/metadata.csv `file_name` exactly, which is
`<session_id>/<camera>.mp4`. Nothing other than mp4 files and the single
metadata.csv at the root may end up under videos/ : a stray parquet there is
skipped with only a debug log, leaving a config that looks fine and holds
nothing.
"""
import argparse, csv, os, shutil

from _paths import add_fa_arg

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    add_fa_arg(ap)
    a = ap.parse_args()
    vd = os.path.join(a.out, "videos")
    manifest = os.path.join(vd, "metadata.csv")
    rows = list(csv.DictReader(open(manifest)))
    done = skipped = 0
    total = 0
    missing = []
    for r in rows:
        sid, cam = r["session_id"], r["camera"]
        src = os.path.join(a.fa, "sessions", sid, f"run_video_{cam}.mp4")
        dst = os.path.join(vd, r["file_name"])
        if not os.path.exists(src):
            missing.append(r["file_name"]); continue
        if os.path.exists(dst) and os.path.getsize(dst) == os.path.getsize(src):
            skipped += 1; total += os.path.getsize(dst); continue
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(src, dst)
        done += 1; total += os.path.getsize(dst)
    print(f"  复制 {done} 个 / 已存在跳过 {skipped} 个 / 源缺失 {len(missing)} 个")
    print(f"  videos/ 合计 {total/2**30:.2f} GiB")
    if missing:
        print(f"  缺失示例: {missing[:3]}")
    stray = []
    for root, _, files in os.walk(vd):
        for f in files:
            if f.endswith(".mp4"):
                continue
            if root == vd and f == "metadata.csv":
                continue
            stray.append(os.path.relpath(os.path.join(root, f), vd))
    print(f"  非 mp4 杂项: {len(stray)} 个" + (f"  {stray[:3]}" if stray else "（干净）"))

if __name__ == "__main__":
    main()
