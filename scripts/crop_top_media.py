#!/usr/bin/env python3
"""Apply the per-(task, arms) top-camera crop to the videos and the evidence photographs.

Sources are the untouched originals — the live session tree for the videos, the operator's own
photographs for the stills — so the crop costs exactly one re-encode generation, not two. Files of
sessions that are not cropped (the dual-arm trials, which use the whole table) are linked through
unchanged. The capture shards keep their full-frame top.png: they are the reference the recorded
pixel coordinates refer to.
"""
import argparse, csv, glob, os, shutil, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

from _paths import add_fa_arg


def probe(path):
    """Actual frame size of a video file. The calibration grid is 1920x1080 for every session, but a
    few sessions recorded a 960x540 timelapse, so the rectangle must be scaled to the real file."""
    out = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                          "stream=width,height", "-of", "csv=p=0", path], capture_output=True, text=True).stdout
    w, h = (out.strip().split(",") + ["0", "0"])[:2]
    return (int(w), int(h)) if w.isdigit() and h.isdigit() else (1920, 1080)


def scaled(rect, sw, sh, W=1920, H=1080):
    """The rectangle is derived in the calibrated 1920x1080 grid; a few sessions recorded smaller."""
    if (sw, sh) == (W, H):
        return rect
    fx, fy = sw / W, sh / H
    x, y, w, h = rect
    r = [int(round(v * f / 2)) * 2 for v, f in ((x, fx), (y, fy), (w, fx), (h, fy))]
    r[2] = min(r[2], sw - r[0]); r[3] = min(r[3], sh - r[1])
    return tuple(r)


def link(src, dst):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    if os.path.lexists(dst):
        return
    os.symlink(os.path.realpath(src), dst)


def encode(src, dst, rect, crf, preset):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    x, y, w, h = rect
    cmd = ["nice", "-n", "19", "ffmpeg", "-nostdin", "-v", "error", "-y", "-i", src,
           "-vf", f"crop={w}:{h}:{x}:{y}", "-c:v", "libx264", "-preset", preset, "-crf", str(crf),
           "-pix_fmt", "yuv420p", "-an", "-movflags", "+faststart", "-f", "mp4", dst + ".part"]
    subprocess.run(cmd, check=True)
    os.replace(dst + ".part", dst)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True, help="flat tree: videos/, evidence/")
    ap.add_argument("--crops", required=True, help="crops-<batch>.csv")
    ap.add_argument("--out", required=True, help="staging tree to write videos/ and evidence/ into")
    ap.add_argument("--batch", default="b01")
    ap.add_argument("--crf", type=int, default=23)
    ap.add_argument("--preset", default="medium")
    ap.add_argument("--jobs", type=int, default=6)
    add_fa_arg(ap)
    ap.add_argument("--only", choices=["videos", "evidence"], default=None)
    a = ap.parse_args()

    rows = list(csv.DictReader(open(a.crops, newline="")))
    release = {r["session_id"] for r in rows}          # the crop table is the release list
    rect_of, size_of = {}, {}
    for r in rows:
        sw, sh = int(r["source_width"]), int(r["source_height"])
        size_of[r["session_id"]] = (sw, sh)
        if r["crop_w"]:
            rect_of[r["session_id"]] = (int(r["crop_x"]), int(r["crop_y"]), int(r["crop_w"]), int(r["crop_h"]))

    # ---- videos -------------------------------------------------------------
    if a.only in (None, "videos"):
        vrows = list(csv.DictReader(open(os.path.join(a.dataset, "videos", "metadata.csv"), newline="")))
        jobs, passthrough, real_of = [], 0, {}
        vrows = [r for r in vrows if r["session_id"] in release]   # never carry unreleased sessions across
        for r in vrows:
            sid, cam = r["session_id"], r["camera"]
            dst = os.path.join(a.out, "videos", sid, f"{cam}.mp4")
            src_pub = os.path.join(a.dataset, "videos", sid, f"{cam}.mp4")
            if cam != "top" or sid not in rect_of:
                link(src_pub, dst); passthrough += 1; continue
            live = os.path.join(a.fa, "sessions", sid, "run_video_top.mp4")
            src = live if os.path.exists(live) else src_pub
            real_of[sid] = probe(src)
            jobs.append((src, dst, scaled(rect_of[sid], *real_of[sid])))
        print(f"videos: 需重编码 {len(jobs)} 个，原样链接 {passthrough} 个", flush=True)
        todo = [(s, d, rc) for s, d, rc in jobs if not os.path.exists(d)]
        with ThreadPoolExecutor(a.jobs) as ex:
            for i, _ in enumerate(ex.map(lambda j: encode(*j, a.crf, a.preset), todo), 1):
                if i % 10 == 0:
                    print(f"  {i}/{len(todo)}", flush=True)
        out_dir = os.path.join(a.out, "videos")
        with open(os.path.join(out_dir, "metadata.csv"), "w", newline="") as f:
            names = list(vrows[0].keys()) + ["crop_x", "crop_y", "crop_w", "crop_h", "source_width", "source_height"]
            w = csv.DictWriter(f, fieldnames=names, lineterminator="\r\n"); w.writeheader()
            for r in vrows:
                sid, cam = r["session_id"], r["camera"]
                p = os.path.join(out_dir, sid, f"{cam}.mp4")
                r = dict(r, bytes=os.path.getsize(os.path.realpath(p)))
                sw, sh = real_of.get(sid) or size_of.get(sid, (1920, 1080))
                if cam == "top" and sid in rect_of:
                    x, y, cw, ch = scaled(rect_of[sid], sw, sh)
                    r.update(crop_x=x, crop_y=y, crop_w=cw, crop_h=ch, source_width=sw, source_height=sh)
                else:
                    r.update(crop_x="", crop_y="", crop_w="", crop_h="", source_width="", source_height="")
                w.writerow(r)
        tot = sum(os.path.getsize(os.path.realpath(os.path.join(out_dir, r["session_id"], f"{r['camera']}.mp4"))) for r in vrows)
        print(f"videos 完成：{len(vrows)} 个文件，合计 {tot / 2**30:.2f} GiB", flush=True)

    # ---- evidence photographs ----------------------------------------------
    if a.only in (None, "evidence"):
        from PIL import Image
        erows = list(csv.DictReader(open(os.path.join(a.dataset, "evidence", "metadata.csv"), newline="")))
        # the stills are keyed by (batch, trial); map them to their session's rectangle
        sid_of = {}
        import pyarrow.parquet as pq
        for sp in ("evaluated", "reset", "auxiliary"):
            p = os.path.join(a.dataset, "metadata", f"trials_{sp}-{a.batch}.parquet")
            if os.path.exists(p):
                for r in pq.read_table(p, columns=["session_id", "batch", "trial"]).to_pylist():
                    if r["batch"]:
                        sid_of[(r["batch"], r["trial"])] = r["session_id"]
        erows = [r for r in erows if sid_of.get((r["batch"], r["trial"])) in release]
        n_crop = n_link = 0
        out_dir = os.path.join(a.out, "evidence")
        for r in erows:
            src = os.path.join(a.dataset, "evidence", r["file_name"])
            dst = os.path.join(out_dir, r["file_name"])
            sid = sid_of.get((r["batch"], r["trial"]))
            rect = rect_of.get(sid) if sid else None
            if rect is None:
                link(src, dst); n_link += 1; continue
            if not os.path.exists(dst):
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                im = Image.open(src)
                rc = scaled(rect, im.width, im.height)
                im.crop((rc[0], rc[1], rc[0] + rc[2], rc[1] + rc[3])).save(dst + ".part", "PNG", optimize=True)
                os.replace(dst + ".part", dst)
            n_crop += 1
        with open(os.path.join(out_dir, "metadata.csv"), "w", newline="") as f:
            names = list(erows[0].keys()) + ["crop_x", "crop_y", "crop_w", "crop_h", "source_width", "source_height"]
            w = csv.DictWriter(f, fieldnames=names, lineterminator="\r\n"); w.writeheader()
            for r in erows:
                p = os.path.join(out_dir, r["file_name"])
                sid = sid_of.get((r["batch"], r["trial"]))
                rect = rect_of.get(sid) if sid else None
                r = dict(r, bytes=os.path.getsize(os.path.realpath(p)))
                if rect:
                    from PIL import Image as I
                    sw, sh = I.open(os.path.join(a.dataset, "evidence", r["file_name"])).size
                    rc = scaled(rect, sw, sh)
                    r.update(crop_x=rc[0], crop_y=rc[1], crop_w=rc[2], crop_h=rc[3], source_width=sw, source_height=sh)
                else:
                    r.update(crop_x="", crop_y="", crop_w="", crop_h="", source_width="", source_height="")
                w.writerow(r)
        tot = sum(os.path.getsize(os.path.realpath(os.path.join(out_dir, r["file_name"]))) for r in erows)
        print(f"evidence 完成：裁剪 {n_crop}，原样 {n_link}，合计 {tot / 2**20:.0f} MiB", flush=True)


if __name__ == "__main__":
    main()
