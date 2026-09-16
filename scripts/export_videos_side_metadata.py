#!/usr/bin/env python3
"""Build videos_side/metadata.csv for the episodes_side config.

Kept as its own config, not merged into `episodes`: the side view has no
per-frame alignment key, and mixing it in would dilute a config where 87% of
rows can be joined to joint trajectories down to 61%.

Missing values are written as empty fields, never as a literal NA. The loader
reads both back as None, so a sentinel cannot express "not applicable" — it
only produces two spellings of the same thing. "Not applicable" is carried by
the positive column alignment_granularity instead.
"""
import argparse, csv, os

from _paths import add_fa_arg

def read_int_file(p):
    try:
        v = open(p).read().strip()
        return str(int(float(v)))
    except Exception:
        return ""

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--batch", default="b01")
    add_fa_arg(ap)
    a = ap.parse_args()
    vd = os.path.join(a.out, "videos")
    sd = os.path.join(a.out, "videos_side")

    # trial metadata: copy from the same session's rows in videos/metadata.csv
    by_sid = {}
    for r in csv.DictReader(open(os.path.join(vd, "metadata.csv"))):
        by_sid.setdefault(r["session_id"], r)

    rows = []
    for sid in sorted(os.listdir(sd)):
        f = os.path.join(sd, sid, "side.mp4")
        if not os.path.isfile(f):
            continue
        m = by_sid.get(sid, {})
        sess = os.path.join(a.fa, "sessions", sid)
        rs = read_int_file(os.path.join(sess, "rec_start_s"))
        re_ = read_int_file(os.path.join(sess, "rec_stop_s"))
        rows.append(dict(
            file_name=f"{sid}/side.mp4",
            session_id=sid, camera="side", release_batch=a.batch,
            bytes=os.path.getsize(f),
            has_alignment="False",
            alignment_granularity="second" if rs else "none",
            rec_start_s=rs, rec_stop_s=re_,
            task=m.get("task", ""), task_role=m.get("task_role", ""),
            model=m.get("model", ""), effort=m.get("effort", ""),
            arms=m.get("arms", ""), batch=m.get("batch", ""), trial=m.get("trial", ""),
            outcome=m.get("outcome", "unlabeled"),
        ))
    assert all(r["has_alignment"] == "False" for r in rows)
    with open(os.path.join(sd, "metadata.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    tot = sum(r["bytes"] for r in rows)
    print(f"  videos_side/metadata.csv  {len(rows)} 行 / {tot/2**30:.2f} GiB / {len(rows[0])} 列")
    print(f"    有 rec_start_s 的 {sum(1 for r in rows if r['rec_start_s'])} 行 -> alignment_granularity=second")
    print(f"    无 rec_start_s 的 {sum(1 for r in rows if not r['rec_start_s'])} 行 -> alignment_granularity=none")
    print(f"    没有对应 trial 元数据（side-only 会话）的 {sum(1 for r in rows if not r['task'])} 行")
    # alignment_granularity legitimately takes the value "none"; only the other
    # columns must be free of sentinel spellings.
    sentinels = sum(1 for r in rows for k, v in r.items()
                    if k != "alignment_granularity" and str(v).strip().upper() in ("NA", "N/A", "NONE", "NULL"))
    print(f"    写了 NA 字面量的单元格: {sentinels}")
    assert sentinels == 0, "missing values must be empty fields, not sentinels"

if __name__ == "__main__":
    main()
