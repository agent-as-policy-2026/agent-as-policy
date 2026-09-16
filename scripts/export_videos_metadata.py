#!/usr/bin/env python3
"""Build videos/metadata.csv for the HF videofolder config.

Layout rules that must hold (changing them later means re-uploading ~20 GB):
  * videos/ contains ONLY mp4 files plus this single metadata.csv at its root.
    A parquet dropped in here would be silently ignored, not rejected.
  * metadata.csv must sit at videos/ root, never in a subdirectory of the media.
  * every *_file_name column holds the FULL relative path, and `../` is not allowed.
  * the episodes config must set drop_labels: true, otherwise the ~190 session
    directories are inferred as a ClassLabel with one class per directory.
`side` is excluded from the main repository: it has no per-frame alignment key.
"""
import argparse, csv, glob, os

from _paths import add_fa_arg, require_sessions

MAIN_CAMS = ("top", "wrist", "right_wrist")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--trials", required=True, help="trials_evaluated parquet, for joining trial metadata")
    ap.add_argument("--batch", default="b01")
    ap.add_argument("--out", required=True)
    add_fa_arg(ap)
    a = ap.parse_args()

    sessions = [l.strip() for l in open(a.snapshot) if l.strip()]
    require_sessions(a.fa, sessions, "--snapshot")

    import pyarrow.parquet as pq, glob as _g
    meta = {}
    for p in _g.glob(os.path.join(os.path.dirname(a.trials), "trials_*.parquet")):
        # the session's role is the split it sits in (trials carries no task_role column)
        role = os.path.basename(p).split("-")[0][len("trials_"):]
        for r in pq.read_table(p).to_pylist():
            meta[r["session_id"]] = dict(r, task_role=role)

    import pyarrow.parquet as _pq
    alignp = os.path.join(a.out, "alignment", f"frames_index-{a.batch}.parquet")
    aligned = set()
    if os.path.exists(alignp):
        t = _pq.read_table(alignp, columns=["session_id", "camera"])
        aligned = set(zip(t.column("session_id").to_pylist(), t.column("camera").to_pylist()))

    rows, skipped = [], 0
    for sid in sessions:
        for cam in MAIN_CAMS:
            src = os.path.join(a.fa, "sessions", sid, f"run_video_{cam}.mp4")
            if not os.path.exists(src):
                continue
            m = meta.get(sid, {})
            rows.append(dict(
                file_name=f"{sid}/{cam}.mp4",
                session_id=sid, camera=cam, release_batch=a.batch,
                bytes=os.path.getsize(src),
                has_alignment=(sid, cam) in aligned,
                task=m.get("task", ""), task_role=m.get("task_role", ""),
                model=m.get("model", ""), effort=m.get("effort", ""),
                arms=m.get("arms", ""), batch=m.get("batch", ""), trial=m.get("trial", ""),
                outcome=m.get("outcome", "unlabeled"),
            ))
        if os.path.exists(os.path.join(a.fa, "sessions", sid, "run_video_side.mp4")):
            skipped += 1

    d = os.path.join(a.out, "videos")
    os.makedirs(d, exist_ok=True)
    p = os.path.join(d, "metadata.csv")
    with open(p, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    tot = sum(r["bytes"] for r in rows)
    noal = sum(1 for r in rows if not r["has_alignment"])
    print(f"  videos/metadata.csv  {len(rows)} 行 / {tot/2**30:.2f} GiB")
    print(f"    按机位: " + ", ".join(f"{c}={sum(1 for r in rows if r['camera']==c)}" for c in MAIN_CAMS))
    print(f"    无逐帧对齐键的视频: {noal}（只能按 rec_start_s 秒级粗对齐）")
    print(f"    side 已排除: {skipped} 个（无对齐键，另发副仓）")
    print(f"    未标注 outcome 的行: {sum(1 for r in rows if r['outcome']=='unlabeled')}")

if __name__ == "__main__":
    main()
