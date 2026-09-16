#!/usr/bin/env python3
"""Re-encode the side camera to 720p for release.

The side view has no per-frame alignment key, so it cannot be joined to the
joint trajectories the way the other three cameras can. Its only use is
watching what the two arms did, and at 720p the arms, grippers, parts and the
burned-in wall clock all stay legible — pixel fidelity buys nothing usable here
and costs 23 GiB.

Kept separate from copy_videos.py on purpose: that script copies bytes
untouched, this one produces a derived stream, and conflating the two would
make it impossible to tell which files in the release are originals.
"""
import argparse, concurrent.futures as cf, json, os, subprocess, sys, time

from _paths import add_fa_arg, require_sessions

ARGS = ["-vf", "scale=-2:720", "-c:v", "libx264", "-preset", "veryfast",
        "-crf", "30", "-an", "-movflags", "+faststart"]

def probe(path, entries):
    r = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0",
                        "-show_entries", entries, "-of", "json", path],
                       capture_output=True, text=True)
    try:
        return json.loads(r.stdout)["streams"][0]
    except Exception:
        return {}

def one(sid, src, dst):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    t0 = time.time()
    r = subprocess.run(["nice", "-n", "19", "ffmpeg", "-nostdin", "-v", "error",
                        "-y", "-i", src, *ARGS, dst], capture_output=True, text=True)
    if r.returncode != 0:
        return sid, None, r.stderr[:120]
    a = probe(src, "stream=nb_read_frames,duration")
    b = probe(dst, "stream=nb_read_frames,duration")
    return sid, dict(src_bytes=os.path.getsize(src), dst_bytes=os.path.getsize(dst),
                     secs=time.time() - t0,
                     src_dur=a.get("duration"), dst_dur=b.get("duration")), None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--workers", type=int, default=4)
    add_fa_arg(ap)
    a = ap.parse_args()
    sessions = [l.strip() for l in open(a.snapshot) if l.strip()]
    require_sessions(a.fa, sessions, "--snapshot")
    jobs = []
    for sid in sessions:
        src = os.path.join(a.fa, "sessions", sid, "run_video_side.mp4")
        if os.path.exists(src):
            jobs.append((sid, src, os.path.join(a.out, "videos_side", sid, "side.mp4")))
    print(f"  待转码 {len(jobs)} 个", flush=True)
    src_tot = dst_tot = 0
    fails, drift = [], []
    done = 0
    with cf.ThreadPoolExecutor(max_workers=a.workers) as ex:
        for sid, info, err in ex.map(lambda j: one(*j), jobs):
            done += 1
            if err:
                fails.append((sid, err)); continue
            src_tot += info["src_bytes"]; dst_tot += info["dst_bytes"]
            try:
                if abs(float(info["src_dur"]) - float(info["dst_dur"])) > 0.5:
                    drift.append((sid, info["src_dur"], info["dst_dur"]))
            except (TypeError, ValueError):
                pass
            if done % 40 == 0:
                print(f"    {done}/{len(jobs)}  累计 {src_tot/2**30:.1f} -> {dst_tot/2**30:.2f} GiB", flush=True)
    print(f"  完成 {done - len(fails)}/{len(jobs)}")
    print(f"  {src_tot/2**30:.2f} GiB -> {dst_tot/2**30:.2f} GiB  ({dst_tot/max(src_tot,1)*100:.1f}%)")
    if fails:
        print(f"  失败 {len(fails)}: {fails[:3]}")
    print(f"  时长漂移超过 0.5 秒的: {len(drift)}" + (f"  {drift[:3]}" if drift else "（无）"))

if __name__ == "__main__":
    main()
