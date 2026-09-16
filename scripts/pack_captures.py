#!/usr/bin/env python3
"""Pack per-capture frames into WebDataset tar shards.

The Hub caps a single directory at 10,000 entries and recommends staying under
100,000 files per repository. Roughly 27,000 frame images spread over ~190
session directories would sit right on that line and make every listing slow,
so they ship as tar shards instead: one sample per capture, members sharing a
key, which is exactly what the webdataset builder expects.

Depth comes from the converted PNG16, never the original .npy — the Hub does
not read .npy at all.
"""
import argparse, io, json, os, tarfile

from _paths import add_fa_arg, require_sessions

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--depth-dir", required=True, help="converted PNG16 depth root")
    ap.add_argument("--batch", default="b01")
    ap.add_argument("--out", required=True)
    ap.add_argument("--shard-bytes", type=int, default=1_500_000_000)
    add_fa_arg(ap)
    a = ap.parse_args()

    sessions = [l.strip() for l in open(a.snapshot) if l.strip()]
    require_sessions(a.fa, sessions, "--snapshot")
    d = os.path.join(a.out, "captures")
    os.makedirs(d, exist_ok=True)

    shard = 0
    tf = None
    written = 0
    samples = 0
    members = 0
    missing = {"top": 0, "wrist": 0, "depth": 0, "calib": 0}

    def open_shard(i):
        # the batch id must be in the filename: shard numbering restarts at 0000
        # every batch, so captures-0000.tar from b02 would collide with b01's
        # under the card's captures/captures-*.tar glob.
        p = os.path.join(d, f"captures-{a.batch}-{i:04d}.tar")
        return tarfile.open(p, "w"), p

    tf, path = open_shard(shard)
    for sid in sessions:
        fd = os.path.join(a.fa, "sessions", sid, "frames")
        if not os.path.isdir(fd):
            continue
        idxs = sorted({f.split("_", 1)[0] for f in os.listdir(fd) if "_" in f and f.split("_", 1)[0].isdigit()})
        for idx in idxs:
            key = f"{sid}_{idx}"
            parts = [
                ("top.png", os.path.join(fd, f"{idx}_top.png")),
                ("wrist.png", os.path.join(fd, f"{idx}_wrist.png")),
                ("depth.png", os.path.join(a.depth_dir, sid, f"{idx}_wrist_depth.png")),
                ("calib.json", os.path.join(fd, f"{idx}_calib.json")),
            ]
            any_present = False
            for ext, src in parts:
                if not os.path.exists(src):
                    missing[ext.split(".")[0]] += 1
                    continue
                any_present = True
                ti = tarfile.TarInfo(f"{key}.{ext}")
                ti.size = os.path.getsize(src)
                ti.mtime = 0
                with open(src, "rb") as fh:
                    tf.addfile(ti, fh)
                written += ti.size
                members += 1
            if any_present:
                samples += 1
            if written >= a.shard_bytes:
                tf.close()
                print(f"    {os.path.basename(path)}  {os.path.getsize(path)/2**30:.2f} GiB")
                shard += 1
                tf, path = open_shard(shard)
                written = 0
    tf.close()
    print(f"    {os.path.basename(path)}  {os.path.getsize(path)/2**30:.2f} GiB")
    tot = sum(os.path.getsize(os.path.join(d, f)) for f in os.listdir(d) if f.endswith(".tar"))
    print(f"  分片 {shard+1} 个 / 样本 {samples} 组 / 成员 {members} 个 / {tot/2**30:.2f} GiB")
    print(f"  缺件: {missing}")

if __name__ == "__main__":
    main()
