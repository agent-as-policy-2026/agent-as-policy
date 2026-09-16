#!/usr/bin/env python3
"""Export the frozen snapshot to the Hugging Face dataset layout.

Only the `trials` table is implemented here; other configs follow the same
pattern. Paths under the repository are never modified: everything is written
to --out.

Types are written with pyarrow directly, so enum columns land as `string`.
docs/SCHEMA.md must describe them as "string + enumerated values", not as
ClassLabel (writing them as ClassLabel would require the `datasets` library).
"""
import argparse, csv, json, os, re, sys
from collections import defaultdict

from _paths import add_fa_arg, require_sessions

HOME_RE = re.compile(r"/home/[^/\s]+")
# one spelling of the serial pattern across the exporters and the verifier
SERIAL_RE = re.compile(r"usb-[0-9A-Za-z_]+_[0-9A-Za-z]{6,}")

# matched as substrings of the session name, longest-first where one name contains another
TASKS = ("twopairsreset", "twopairs", "assembly", "singleinsert", "onebigpile",
         "twopiles", "pyramid", "dice", "towel2", "towel", "throw", "stack1")

def scrub(v):
    """Remove local absolute paths and device serials from a session.env value."""
    if not v:
        return v
    v = HOME_RE.sub("<HOME>", v)
    v = SERIAL_RE.sub("usb-<CAMERA_SERIAL>", v)
    return v

def parse_session_name(sid):
    """20260910_162434_paper_twopairs_full_low_6astra_01 -> (date, time, role, rest)"""
    parts = sid.split("_")
    date, tm = parts[0], parts[1]
    role_token = parts[2] if len(parts) > 2 else ""
    rest = "_".join(parts[2:])
    if role_token == "scatter":
        role = "reset"
    elif role_token == "paper":
        role = "evaluated"
    else:
        role = "auxiliary"
    task = next((t for t in TASKS if t in rest), "other")
    if task == "twopairsreset":
        role = "reset"
    return date, tm, role, task

def read_env(path):
    out = {}
    if not os.path.exists(path):
        return out
    for line in open(path, errors="replace"):
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k] = v.strip().strip('"').strip("'")
    return out


def table_with_types(rows, int_cols=(), bool_cols=(), float_cols=()):
    """Build a table whose all-empty columns still carry a real type.

    pa.Table.from_pylist infers `null` for a column that is None in every row,
    and a `null`-typed column refuses to concatenate with the same column from
    another shard where it has values. Auxiliary sessions have no results.csv
    row, so their numeric columns are None throughout — without an explicit
    schema those splits could never be stacked with `evaluated`.
    """
    import pyarrow as pa
    if not rows:
        return pa.table({})
    names = list(rows[0].keys())
    fields = []
    for n in names:
        if n in int_cols:
            t = pa.int64()
        elif n in bool_cols:
            t = pa.bool_()
        elif n in float_cols:
            t = pa.float64()
        else:
            t = pa.string()
        fields.append(pa.field(n, t, nullable=True))
    cols = []
    for n, f in zip(names, fields):
        vals = [r.get(n) for r in rows]
        if pa.types.is_string(f.type):
            vals = ["" if v is None else str(v) for v in vals]
        cols.append(pa.array(vals, type=f.type))
    return pa.Table.from_arrays(cols, schema=pa.schema(fields))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--labels", required=True)
    ap.add_argument("--batch", default="b01")
    ap.add_argument("--out", required=True)
    add_fa_arg(ap)
    a = ap.parse_args()

    sessions = [l.strip() for l in open(a.snapshot) if l.strip()]
    require_sessions(a.fa, sessions, "--snapshot")

    # labels keyed by (batch_dir, trial)
    labels = {}
    for r in csv.DictReader(open(a.labels)):
        labels[(r["batch"], r["trial"])] = r

    # session_id -> results.csv row (+ its batch dir)
    res = {}
    for f in sorted(__import__("glob").glob(f"{a.fa}/paper_runs/*/results.csv")):
        bd = os.path.basename(os.path.dirname(f))
        for r in csv.DictReader(open(f)):
            sid = (r.get("session") or "").strip()
            if sid:
                res.setdefault(sid, (bd, r))

    rows = []
    gaps = defaultdict(int)
    for sid in sessions:
        d = os.path.join(a.fa, "sessions", sid)
        env = read_env(os.path.join(d, "session.env"))
        date, tm, role, task = parse_session_name(sid)
        bd, rr = res.get(sid, (None, {}))
        lab = labels.get((bd, (rr.get("trial") or "").strip())) if bd else None

        arms = env.get("ARMS")
        if not arms:
            arms = "right" if "_right" in sid else "left"
            gaps["arms_inferred_from_name"] += 1
        arms = {"left,right": "dual"}.get(arms, arms)

        model = (rr.get("model") or "").strip() or env.get("AGENT_MODEL", "")
        if model in ("codex-default", ""):
            gaps["model_unresolved"] += 1
            model = model or "unknown"

        start_s = env.get("AGENT_START_S") or ""
        if not start_s:
            gaps["no_AGENT_START_S"] += 1

        for k in ("INTERFACE", "AGENT_EFFORT", "KNOWLEDGE_MODE"):
            if not env.get(k):
                gaps[f"no_{k}"] += 1

        rows.append(dict(
            session_id=sid,
            release_batch=a.batch,
            batch=bd or "",
            trial=(rr.get("trial") or "").strip(),
            task=task,
            arms=arms,
            model=model,
            backend=env.get("AGENT_BACKEND", ""),
            effort=env.get("AGENT_EFFORT", ""),
            # secondary run settings, folded into one JSON column so the main
            # view stays readable; keys are fixed so every row parses the same
            run_config_json=json.dumps(dict(
                harness=env.get("HARNESS", ""),
                harness_version=env.get("HARNESS_VERSION", ""),
                interface=env.get("INTERFACE", ""),
                knowledge_mode=env.get("KNOWLEDGE_MODE", ""),
                knowledge_scope=env.get("KNOWLEDGE_SCOPE", ""),
                service_tier=env.get("AGENT_SERVICE_TIER", ""),
                tools_inherited=int(env.get("TOOLS_INHERITED") or 0)), sort_keys=True),
            start_epoch_s=int(start_s) if start_s.isdigit() else None,
            end_epoch_s=int(env.get("AGENT_END_S")) if (env.get("AGENT_END_S") or "").isdigit() else None,
            duration_s=int(rr["duration_s"]) if (rr.get("duration_s") or "").isdigit() else None,
            timed_out=(rr.get("timed_out") or "").strip().lower() == "yes",
            aborted=env.get("AGENT_ABORTED") == "1",
            cmds_counted=int(rr["cmds_counted"]) if (rr.get("cmds_counted") or "").isdigit() else None,
            gripper_cmds=int(rr["gripper_cmds"]) if (rr.get("gripper_cmds") or "").isdigit() else None,
            failed_cmds=int(rr["failed_cmds"]) if (rr.get("failed_cmds") or "").isdigit() else None,
            tokens_input=int(rr["tokens_input"]) if (rr.get("tokens_input") or "").isdigit() else None,
            tokens_output=int(rr["tokens_output"]) if (rr.get("tokens_output") or "").isdigit() else None,
            tokens_reasoning=int(rr["tokens_reasoning"]) if (rr.get("tokens_reasoning") or "").isdigit() else None,
            outcome=(lab or {}).get("verdict", "unlabeled"),
            label_source=(lab or {}).get("source", "none"),
            agent_self_report=((rr.get("agent_verdict") or "").strip().split() or ["no_report"])[0],
            prompt_file=os.path.basename(scrub(env.get("PROMPT", ""))),
            _role=role,  # split selector only; not exported
        ))

    import pyarrow as pa, pyarrow.parquet as pq
    os.makedirs(os.path.join(a.out, "metadata"), exist_ok=True)
    counts = {}
    for split in ("evaluated", "reset", "auxiliary"):
        sub = [{k: v for k, v in r.items() if k != "_role"} for r in rows if r["_role"] == split]
        counts[split] = len(sub)
        if not sub:
            continue
        t = table_with_types(sub,
            int_cols=("start_epoch_s","end_epoch_s","duration_s",
                      "cmds_counted","gripper_cmds","failed_cmds",
                      "tokens_input","tokens_output","tokens_reasoning"),
            bool_cols=("timed_out","aborted"))
        p = os.path.join(a.out, "metadata", f"trials_{split}-{a.batch}.parquet")
        pq.write_table(t, p, compression="zstd", compression_level=9)
        print(f"  {split:<10} {len(sub):>4} 行 -> {os.path.basename(p)}  {os.path.getsize(p)//1024} KB")
    print(f"  恒等式 {counts['evaluated']}+{counts['reset']}+{counts['auxiliary']}"
          f"={sum(counts.values())} == 快照 {len(sessions)}  "
          f"{'OK' if sum(counts.values())==len(sessions) else '不成立 !!'}")
    export_commands(sessions, a.batch, a.out, a.fa)
    export_captures(sessions, a.batch, a.out, a.fa)
    export_alignment(sessions, a.batch, a.out, a.fa)
    export_joints(sessions, a.batch, a.out, a.fa)
    print("  字段缺口（导出器已兜底，需写进 docs/SCHEMA.md）:")
    for k, v in sorted(gaps.items(), key=lambda x: -x[1]):
        print(f"    {k:<28} {v}")



# ---------------------------------------------------------------- commands / captures
def export_commands(sessions, batch, out, fa):
    """1 row = 1 bridge request paired with its response."""
    import pyarrow as pa, pyarrow.parquet as pq
    rows = []
    unpaired = 0
    for sid in sessions:
        d = os.path.join(fa, "sessions", sid, "bridge")
        if not os.path.isdir(d):
            continue
        files = os.listdir(d)
        reqs = {f[4:-5]: f for f in files if f.startswith("req_") and f.endswith(".json")}
        resps = {f[5:-5]: f for f in files if f.startswith("resp_") and f.endswith(".json")}
        for cid in sorted(reqs):
            try:
                q = json.load(open(os.path.join(d, reqs[cid])))
            except Exception:
                continue
            p = None
            if cid in resps:
                try:
                    p = json.load(open(os.path.join(d, resps[cid])))
                except Exception:
                    p = None
            else:
                unpaired += 1
            args = q.get("args") if isinstance(q.get("args"), dict) else {}
            rows.append(dict(
                session_id=sid, release_batch=batch,
                cmd_id=int(cid) if cid.isdigit() else None,
                cmd=q.get("cmd", ""),
                request_t=q.get("t"),
                args_json=json.dumps(args, ensure_ascii=False, sort_keys=True) if args else "",
                arg_cam=str(args.get("cam", "")), arg_mode=str(args.get("mode", "")),
                arg_action=str(args.get("action", "")),
                has_response=p is not None,
                ok=bool(p.get("ok")) if p else None,
                safety_state=str(p.get("safety_state", "")) if p else "",
                bridge_state=str(p.get("bridge_state", "")) if p else "",
                motion_enabled=bool(p["motion_enabled"]) if p and "motion_enabled" in p else None,
                commands_used=p.get("commands_used") if p else None,
                gripper_fraction=p.get("gripper_fraction") if p else None,
            ))
    t = table_with_types(rows,
        int_cols=("cmd_id","commands_used"), bool_cols=("has_response","ok","motion_enabled"),
        float_cols=("request_t","gripper_fraction"))
    p = os.path.join(out, "metadata", f"commands-{batch}.parquet")
    pq.write_table(t, p, compression="zstd", compression_level=9)
    print(f"  commands   {len(rows):>6} 行 -> {os.path.basename(p)}  {os.path.getsize(p)//1024} KB  (无响应 {unpaired})")

def export_captures(sessions, batch, out, fa):
    """1 row = 1 `frames` capture group (calib + top + wrist + depth)."""
    import pyarrow as pa, pyarrow.parquet as pq, re
    rows = []
    for sid in sessions:
        d = os.path.join(fa, "sessions", sid, "frames")
        if not os.path.isdir(d):
            continue
        groups = defaultdict(set)
        for f in os.listdir(d):
            m = re.match(r"(\d+)_(.+)$", f)
            if m:
                groups[m.group(1)].add(m.group(2))
        for idx in sorted(groups):
            have = groups[idx]
            meta = {}
            cp = os.path.join(d, f"{idx}_calib.json")
            if os.path.exists(cp):
                try:
                    meta = json.load(open(cp)).get("_meta", {}) or {}
                except Exception:
                    meta = {}
            rows.append(dict(
                session_id=sid, release_batch=batch, capture_index=int(idx),
                has_top=("top.png" in have), has_wrist=("wrist.png" in have),
                has_depth=("wrist_depth.npy" in have),
                meta_json=json.dumps(meta, ensure_ascii=False, sort_keys=True) if meta else "",
            ))
    t = table_with_types(rows,
        int_cols=("capture_index",), bool_cols=("has_top","has_wrist","has_depth"))
    p = os.path.join(out, "metadata", f"captures-{batch}.parquet")
    pq.write_table(t, p, compression="zstd", compression_level=9)
    miss = sum(1 for r in rows if not (r["has_top"] and r["has_wrist"] and r["has_depth"]))
    print(f"  captures   {len(rows):>6} 行 -> {os.path.basename(p)}  {os.path.getsize(p)//1024} KB  (四件不齐 {miss} 组)")


# ---------------------------------------------------------------- alignment / joints
def export_alignment(sessions, batch, out, fa):
    """Frame-index -> wall/monotonic clock, the only reliable video alignment key.

    Must live outside videos/ : a parquet dropped into a videofolder data_dir is
    silently ignored (debug log only), so the config would look healthy and hold
    no data at all.
    """
    import pyarrow as pa, pyarrow.parquet as pq, pyarrow.csv as pacsv, glob as _g, re
    sess, cam, fidx, wall, mono = [], [], [], [], []
    files = 0
    for sid in sessions:
        for p in sorted(_g.glob(os.path.join(fa, "sessions", sid, "run_video_*_frames.csv"))):
            m = re.search(r"run_video_(.+)_frames\.csv$", os.path.basename(p))
            if not m:
                continue
            c = m.group(1)
            try:
                t = pacsv.read_csv(p)
            except Exception:
                continue
            files += 1
            n = t.num_rows
            names = t.schema.names
            sess += [sid] * n
            cam += [c] * n
            fidx += t.column(names[0]).to_pylist()
            wall += t.column(names[1]).to_pylist() if len(names) > 1 else [None] * n
            mono += t.column(names[2]).to_pylist() if len(names) > 2 else [None] * n
    tab = pa.table({
        "session_id": pa.array(sess, pa.string()),
        "camera": pa.array(cam, pa.string()),
        "frame_index": pa.array(fidx, pa.int32()),
        "wall_time_ns": pa.array(wall, pa.int64()),
        "monotonic_ns": pa.array(mono, pa.int64()),
        "release_batch": pa.array([batch] * len(sess), pa.string()),
    })
    d = os.path.join(out, "alignment")
    os.makedirs(d, exist_ok=True)
    p = os.path.join(d, f"frames_index-{batch}.parquet")
    pq.write_table(tab, p, compression="zstd", compression_level=9,
                   column_encoding={"frame_index": "DELTA_BINARY_PACKED",
                                    "wall_time_ns": "DELTA_BINARY_PACKED",
                                    "monotonic_ns": "DELTA_BINARY_PACKED"},
                   use_dictionary=["session_id", "camera", "release_batch"],
                   row_group_size=200000, write_page_index=True)
    print(f"  alignment  {tab.num_rows:>6} 行 ({files} 个 CSV) -> {os.path.basename(p)}  {os.path.getsize(p)//2**20} MiB")

def export_joints(sessions, batch, out, fa):
    """50 Hz joint trajectories, one parquet shard per release batch."""
    import pyarrow as pa, pyarrow.parquet as pq, pyarrow.csv as pacsv, glob as _g
    tables = []
    files = 0
    for sid in sessions:
        for p in sorted(_g.glob(os.path.join(fa, "sessions", sid, "run_joints*.csv"))):
            try:
                t = pacsv.read_csv(p)
            except Exception:
                continue
            files += 1
            cols = []
            for f in t.schema:
                c = t.column(f.name)
                cols.append(c.cast(pa.float32()) if pa.types.is_floating(f.type) else c)
            t = pa.Table.from_arrays(cols, names=t.schema.names)
            t = t.append_column("session_id", pa.array([sid] * t.num_rows, pa.string()))
            t = t.append_column("release_batch", pa.array([batch] * t.num_rows, pa.string()))
            tables.append(t)
    if not tables:
        print("  joints     无数据")
        return
    import pyarrow as pa
    tab = pa.concat_tables(tables, promote_options="default")
    d = os.path.join(out, "joints")
    os.makedirs(d, exist_ok=True)
    p = os.path.join(d, f"joints-{batch}.parquet")
    pq.write_table(tab, p, compression="zstd", compression_level=9,
                   use_dictionary=["session_id", "release_batch"], row_group_size=500000)
    print(f"  joints     {tab.num_rows:>8} 行 ({files} 个 CSV) -> {os.path.basename(p)}  {os.path.getsize(p)//2**20} MiB")


if __name__ == "__main__":
    main()
