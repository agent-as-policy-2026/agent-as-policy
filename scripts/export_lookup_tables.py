#!/usr/bin/env python3
"""Export the three lookup tables: prompts, interfaces, goal_sets.

Each row is a unique document keyed by sha256, with the sessions that used it.
Local absolute paths and camera serials are scrubbed from the stored text.
"""
import argparse, hashlib, json, os, re
from collections import defaultdict

from _paths import add_fa_arg, require_sessions

HOME_RE = re.compile(r"/home/[^/\s]+")
SERIAL_RE = re.compile(r"usb-[0-9A-Za-z_]+_[0-9A-Za-z]{6,}")

def scrub(t):
    t = HOME_RE.sub("<HOME>", t)
    return SERIAL_RE.sub("usb-<CAMERA_SERIAL>", t)

def collect(sessions, filename, fa):
    by_sha = {}
    users = defaultdict(list)
    for sid in sessions:
        p = os.path.join(fa, "sessions", sid, filename)
        if not os.path.exists(p):
            continue
        raw = open(p, errors="replace").read()
        clean = scrub(raw)
        sha = hashlib.sha256(clean.encode()).hexdigest()
        by_sha.setdefault(sha, clean)
        users[sha].append(sid)
    return by_sha, users

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--batch", default="b01")
    ap.add_argument("--out", required=True)
    add_fa_arg(ap)
    a = ap.parse_args()
    sessions = [l.strip() for l in open(a.snapshot) if l.strip()]
    require_sessions(a.fa, sessions, "--snapshot")
    import pyarrow as pa, pyarrow.parquet as pq
    os.makedirs(os.path.join(a.out, "metadata"), exist_ok=True)

    for fname, table, textcol in (("PROMPT.md", "prompts", "prompt_text"),
                                  ("README_interface.md", "interfaces", "interface_text")):
        by_sha, users = collect(sessions, fname, a.fa)
        rows = [dict(sha256=s, **{textcol: by_sha[s]},
                     n_chars=len(by_sha[s]), n_sessions=len(users[s]),
                     sessions_json=json.dumps(sorted(users[s])),
                     release_batch=a.batch)
                for s in sorted(by_sha)]
        t = pa.Table.from_pylist(rows)
        p = os.path.join(a.out, "metadata", f"{table}-{a.batch}.parquet")
        pq.write_table(t, p, compression="zstd", compression_level=9)
        cov = sum(len(v) for v in users.values())
        print(f"  {table:<11} {len(rows):>4} 个唯一文档，覆盖 {cov}/{len(sessions)} 会话 -> {os.path.basename(p)}  {os.path.getsize(p)//1024} KB")

    # goal_sets: one row per goal set directory
    gs_root = os.path.join(a.fa, "goal_sets")
    rows = []
    if os.path.isdir(gs_root):
        for gid in sorted(os.listdir(gs_root)):
            d = os.path.join(gs_root, gid)
            if not os.path.isdir(d):
                continue
            files, total = [], 0
            for root, _, fns in os.walk(d):
                for fn in fns:
                    fp = os.path.join(root, fn)
                    try:
                        n = os.path.getsize(fp)
                    except OSError:
                        continue
                    h = hashlib.sha256(open(fp, "rb").read()).hexdigest()
                    files.append({"path": os.path.relpath(fp, d), "bytes": n, "sha256": h})
                    total += n
            rows.append(dict(goal_set_id=gid, n_files=len(files), total_bytes=total,
                             files_json=json.dumps(files, sort_keys=True),
                             release_batch=a.batch))
    t = pa.Table.from_pylist(rows)
    p = os.path.join(a.out, "metadata", f"goal_sets-{a.batch}.parquet")
    pq.write_table(t, p, compression="zstd", compression_level=9)
    print(f"  goal_sets   {len(rows):>4} 个目标集 -> {os.path.basename(p)}  {os.path.getsize(p)//1024} KB")

if __name__ == "__main__":
    main()
