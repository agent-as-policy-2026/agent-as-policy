#!/usr/bin/env python3
"""Copy the per-trial evidence photos into an imagefolder config.

These are the photos every human verdict was made from: the scene before the
trial, after it, and the last frame the agent itself saw.
"""
import argparse, csv, glob, os, re, shutil

from _paths import add_fa_arg

KINDS = ("before", "after", "final_agent_frame")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--labels", required=True)
    ap.add_argument("--batch", default="b01")
    ap.add_argument("--out", required=True)
    ap.add_argument("--copy", action="store_true", help="actually copy the files")
    add_fa_arg(ap)
    a = ap.parse_args()

    lab = {}
    for r in csv.DictReader(open(a.labels)):
        lab[(r["batch"], r["trial"])] = r

    rows, total = [], 0
    for p in sorted(glob.glob(f"{a.fa}/paper_runs/*/trial_*.png")):
        bd = os.path.basename(os.path.dirname(p))
        m = re.match(r"trial_([0-9a-z]+)_(.+)\.png$", os.path.basename(p))
        if not m:
            continue
        trial, kind = m.group(1), m.group(2)
        if kind not in KINDS:
            continue
        if (bd, trial) not in lab:
            continue          # only this batch's trials; paper_runs holds every batch ever run
        l = lab[(bd, trial)]
        rel = f"{bd}/trial_{trial}_{kind}.png"
        n = os.path.getsize(p)
        total += n
        rows.append(dict(file_name=rel, batch=bd, trial=trial, kind=kind,
                         release_batch=a.batch, bytes=n,
                         model=l.get("model", ""), outcome=l.get("verdict", "unlabeled"),
                         label_source=l.get("source", "none")))
        if a.copy:
            d = os.path.join(a.out, "evidence", bd)
            os.makedirs(d, exist_ok=True)
            shutil.copy2(p, os.path.join(d, os.path.basename(p)))

    d = os.path.join(a.out, "evidence")
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "metadata.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    from collections import Counter
    print(f"  evidence  {len(rows)} 张 / {total/2**30:.2f} GiB  ({'已复制' if a.copy else '仅清单'})")
    print(f"    按类型: {dict(Counter(r['kind'] for r in rows))}")
    print(f"    按判定: {dict(Counter(r['outcome'] for r in rows))}")
    print(f"    涉及批次 {len(set(r['batch'] for r in rows))} 个")

if __name__ == "__main__":
    main()
