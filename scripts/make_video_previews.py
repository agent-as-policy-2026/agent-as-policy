#!/usr/bin/env python3
"""One small preview video per session, for the Viewer's player.

480 px wide, 12 fps, H.264 CRF 32, no audio, faststart (about 1 MB per session, so that 100 rows stay under the Viewer's 300 MB page scan). Source priority:
top > wrist > right_wrist > side. Writes <out>/preview/<session_id>.mp4 and
<out>/preview/metadata.csv (session_id, source_file, bytes). Idempotent: an
existing preview is kept, so later batches only encode their new sessions.
"""
import argparse, csv, os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

PRIORITY = (("videos", "top"), ("videos", "wrist"), ("videos", "right_wrist"), ("videos_side", "side"))


def encode(src, dst):
    cmd = ["ffmpeg", "-v", "error", "-y", "-i", src, "-vf", "scale=480:-2,fps=12", "-c:v", "libx264",
           "-preset", "veryfast", "-crf", "32", "-an", "-movflags", "+faststart", "-f", "mp4", dst + ".part"]
    subprocess.run(cmd, check=True)
    os.replace(dst + ".part", dst)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True, help="root holding videos/ and videos_side/")
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--out", default=None, help="defaults to --dataset")
    ap.add_argument("--jobs", type=int, default=4)
    a = ap.parse_args()
    out = os.path.join(a.out or a.dataset, "preview")
    os.makedirs(out, exist_ok=True)
    sessions = [l.strip() for l in open(a.snapshot) if l.strip()]
    jobs, rows = [], []
    for sid in sessions:
        src = next((os.path.join(sub, sid, f"{cam}.mp4") for sub, cam in PRIORITY
                    if os.path.isfile(os.path.join(a.dataset, sub, sid, f"{cam}.mp4"))), None)
        if not src:
            continue
        dst = os.path.join(out, f"{sid}.mp4")
        rows.append((sid, src, dst))
        if not os.path.exists(dst):
            jobs.append((os.path.join(a.dataset, src), dst))
    print(f"{len(rows)} sessions with video, {len(jobs)} to encode", flush=True)
    with ThreadPoolExecutor(a.jobs) as ex:
        for i, _ in enumerate(ex.map(lambda j: encode(*j), jobs), 1):
            if i % 20 == 0:
                print(f"  {i}/{len(jobs)}", flush=True)
    with open(os.path.join(out, "metadata.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["session_id", "source_file", "bytes"])
        for sid, src, dst in rows:
            w.writerow([sid, src, os.path.getsize(dst)])
    total = sum(os.path.getsize(d) for _, _, d in rows)
    print(f"done: {len(rows)} previews, {total / 2**20:.0f} MiB")


if __name__ == "__main__":
    main()
