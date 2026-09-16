#!/usr/bin/env python3
"""Export the bridge action logs as parquet, cut at the snapshot boundary.

These two files are appended to live while the robot runs, so a release must
slice them by wall_time_ns rather than taking whatever is on disk today.
`action` stays a JSON string: its shape varies by command kind.
"""
import argparse, json, os

from _paths import add_workspace_arg

LOGS = (("left", "hardware-bridge/logs/actions.jsonl"),
        ("right", "hardware-bridge/logs/actions_right.jsonl"))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cutoff-ns", type=int, required=True,
                    help="keep rows with wall_time_ns <= this (snapshot boundary)")
    ap.add_argument("--batch", default="b01")
    ap.add_argument("--out", required=True)
    add_workspace_arg(ap)
    a = ap.parse_args()

    import pyarrow as pa, pyarrow.parquet as pq
    d = os.path.join(a.out, "actions")
    os.makedirs(d, exist_ok=True)

    for arm, rel in LOGS:
        p = os.path.join(a.workspace, rel)
        if not os.path.exists(p):
            print(f"  {arm}: 缺 {rel}")
            continue
        kept, dropped, bad = [], 0, 0
        for line in open(p, errors="replace"):
            line = line.strip()
            if not line:
                continue
            try:
                o = json.loads(line)
            except Exception:
                bad += 1
                continue
            w = o.get("wall_time_ns")
            try:
                w = int(w)
            except (TypeError, ValueError):
                bad += 1
                continue
            if w > a.cutoff_ns:
                dropped += 1
                continue
            act = o.get("action")
            kept.append((arm, o.get("event", ""), int(o.get("request_id") or -1),
                         w, int(o.get("monotonic_ns") or 0),
                         act if isinstance(act, str) else json.dumps(act, ensure_ascii=False)))
        if not kept:
            print(f"  {arm}: 截止时刻之前无数据")
            continue
        t = pa.table({
            "arm": pa.array([r[0] for r in kept], pa.string()),
            "event": pa.array([r[1] for r in kept], pa.string()),
            "request_id": pa.array([r[2] for r in kept], pa.int32()),
            "wall_time_ns": pa.array([r[3] for r in kept], pa.int64()),
            "monotonic_ns": pa.array([r[4] for r in kept], pa.int64()),
            "action_json": pa.array([r[5] for r in kept], pa.string()),
            "release_batch": pa.array([a.batch] * len(kept), pa.string()),
        })
        op = os.path.join(d, f"actions_{arm}-{a.batch}.parquet")
        pq.write_table(t, op, compression="zstd", compression_level=9,
                       column_encoding={"wall_time_ns": "DELTA_BINARY_PACKED",
                                        "monotonic_ns": "DELTA_BINARY_PACKED"},
                       use_dictionary=["arm", "event", "release_batch"],
                       row_group_size=500000, write_page_index=True)
        print(f"  {arm:<6} 保留 {len(kept):>8} 行 / 截断 {dropped:>7} 行 / 损坏 {bad}"
              f"  {os.path.getsize(p)/2**20:.0f} MiB -> {os.path.getsize(op)/2**20:.1f} MiB")

if __name__ == "__main__":
    main()
