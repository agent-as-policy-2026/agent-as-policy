#!/usr/bin/env python3
"""Fold the per-session artefacts into the trials table: one row per session.

The row is kept light (about 1.5 MB) so the Viewer's 100-row pages stay under its
300 MB scan limit; the bulky per-session material stays as files and the row links
to it. Runs AFTER the flat exporters and reads their outputs from --dataset.

Writes:
  trials/trials_evaluated-<batch>-NNNN.parquet   the table (the declared split)
  <split>/trials_<split>-<batch>-NNNN.parquet    splits named in --aside, kept beside it
  reference/<id>/<file>                          the goal images the agent was shown
  documents/{prompts,interfaces}/<stem>-<sha8>.md  every distinct prompt / interface text

The row (23 columns), in order:
  session_id, batch, trial, task
  evidence_before, evidence_after       Image: 960 px JPEG thumbnails
  video_preview                         Video: a 480 px preview of the trial
  arms, model, backend, effort, outcome
  labeling{source, agent_self_report}
  timing{start_epoch_s, end_epoch_s, duration_s, timed_out, aborted}
  bridge{cmds_counted, gripper_cmds, failed_cmds}
  tokens{input, output, reasoning}
  reasoning_summary                     the model's own text for the session, in order
  traces[]                              the session replayed step by step: what the model
                                        said, the command it ran, what the bridge answered,
                                        and — when the step took a picture — which capture,
                                        its depth image and the arm pose at that moment
  prompt_text, interface_text, reference{id, files[path, bytes, url]}, videos[]
"""
import argparse, csv, glob, hashlib, io, json, os, re, shutil, sys

import pyarrow as pa
import pyarrow.compute as pc
import pyarrow.parquet as pq
from datasets import Features, Image, List, Value, Video

from _paths import add_fa_arg, require_sessions

IMAGE_T = pa.struct([("bytes", pa.binary()), ("path", pa.string())])
# per-event truncation; a session whose trace is still too large is degraded further below
LIM_CMD, LIM_OUT = 2000, 2000   # model text is never truncated: all of it is 0.8 MiB
SESSION_TRACE_CAP = 524_288            # guard: bytes of trace text per row before degrading
SUMMARY_CAP = 200_000


DEC = json.JSONDecoder()
BRIDGE_TOOL = re.compile(r"robot_client\.py")


def clip(s, n):
    """Head 70% + tail 30%, so a truncated bridge reply still shows its start and its id."""
    if not s:
        return "", False
    s = str(s)
    if len(s) <= n:
        return s, False
    h = int(n * 0.7)
    return s[:h] + f"\n… [{len(s) - n} characters omitted] …\n" + s[-(n - h):], True


def bridge_replies(text):
    """Every bridge answer is a top-level JSON object starting {"ok"; reject objects nested
    inside a larger JSON (those are other sessions' logs the agent printed with cat/sed)."""
    out, i = [], 0
    if not text:
        return out
    while True:
        j = text.find('{"ok"', i)
        if j < 0:
            return out
        k = j - 1
        while k >= 0 and text[k] in " \t":
            k -= 1
        nested = k >= 0 and text[k] in ":,"
        try:
            obj, end = DEC.raw_decode(text, j)
            if not nested and isinstance(obj, dict):
                out.append(obj)
            i = end
        except ValueError:
            i = j + 1


def thumb(path, width=960, quality=85):
    """JPEG thumbnail bytes of a PNG photo, for the Viewer; the PNG stays a file."""
    from PIL import Image as PILImage
    im = PILImage.open(path).convert("RGB")
    if im.width > width:
        im = im.resize((width, round(im.height * width / im.width)), PILImage.LANCZOS)
    buf = io.BytesIO(); im.save(buf, "JPEG", quality=quality, optimize=True)
    return buf.getvalue()


def read_bytes(p):
    with open(p, "rb") as f:
        return f.read()


def group_index(table, key="session_id"):
    """Sort by key; return (sorted table, {key: (start, length)})."""
    import numpy as np
    t = table.sort_by(key)
    ids = t[key].to_numpy(zero_copy_only=False)
    if len(ids) == 0:
        return t, {}
    change = np.flatnonzero(ids[1:] != ids[:-1]) + 1
    starts = np.concatenate([[0], change]); ends = np.concatenate([change, [len(ids)]])
    return t, {ids[s]: (int(s), int(e - s)) for s, e in zip(starts, ends)}


def fill_tokens_from_traces(t, traces_dir):
    """results.csv lacks token counts for some codex sessions; the trace's turn.completed
    usage carries the same numbers (verified equal on every session that has both)."""
    cols = {c: t[c].to_pylist() for c in ("tokens_input", "tokens_output", "tokens_reasoning")}
    filled, checked, mismatch = 0, 0, []
    for i, sid in enumerate(t["session_id"].to_pylist()):
        p = os.path.join(traces_dir, f"{sid}.jsonl")
        if not os.path.exists(p):
            continue
        u = {"input_tokens": 0, "output_tokens": 0, "reasoning_output_tokens": 0}; seen = False
        for line in open(p):
            d = json.loads(line)
            if d.get("type") == "turn.completed" and isinstance(d.get("usage"), dict):
                seen = True
                for k in u:
                    u[k] += int(d["usage"].get(k) or 0)
        if not seen:
            continue
        vals = (u["input_tokens"], u["output_tokens"], u["reasoning_output_tokens"])
        if cols["tokens_input"][i] is None:
            cols["tokens_input"][i], cols["tokens_output"][i], cols["tokens_reasoning"][i] = vals; filled += 1
        else:
            checked += 1
            if (cols["tokens_input"][i], cols["tokens_output"][i], cols["tokens_reasoning"][i]) != vals:
                mismatch.append(sid)
    for c, v in cols.items():
        t = t.set_column(t.schema.get_field_index(c), c, pa.array(v, type=pa.int64()))
    print(f"  tokens: filled {filled} rows from traces; {checked} cross-checked, {len(mismatch)} mismatch {mismatch[:3]}", flush=True)
    assert not mismatch, "trace usage does not match results.csv tokens"
    return t


def _step(kind, item_id="", text="", command="", output="", exit_code=""):
    return dict(kind=kind, item_id=item_id, text=text, command=command, output=output,
                exit_code=exit_code, calls=[])


def codex_steps(path, rows_by_id, arms):
    """One step per item.completed; item.started carries nothing new. A single shell command
    can issue several bridge calls, so each step keeps a list of them."""
    steps, claimed = [], set()
    for line in open(path, errors="replace"):
        line = line.strip()
        if not line:
            continue
        try:
            ev = json.loads(line)
        except json.JSONDecodeError:
            continue
        if ev.get("type") != "item.completed":
            continue
        it = ev.get("item") or {}
        k = it.get("type")
        if k == "reasoning":
            steps.append(_step("reasoning", it.get("id", ""), text=it.get("text") or ""))
        elif k == "agent_message":
            steps.append(_step("message", it.get("id", ""), text=it.get("text") or ""))
        elif k == "file_change":
            steps.append(_step("file_change", it.get("id", ""),
                               command=json.dumps(it.get("changes") or [], ensure_ascii=False)))
        elif k == "error":
            steps.append(_step("error", it.get("id", ""), text=it.get("message") or ""))
        elif k == "command_execution":
            st = _step("command", it.get("id", ""), command=it.get("command") or "",
                       output=it.get("aggregated_output") or "",
                       exit_code="" if it.get("exit_code") is None else str(it.get("exit_code")))
            replies = bridge_replies(st["output"])
            # a dual-arm session runs two bridges, each numbering its requests and captures from 1,
            # and only the left one is in the tables — so every reply has to be attributed to an arm
            segs = [q for q in re.split(r"\n|&&|;|\|\|", st["command"]) if BRIDGE_TOOL.search(q)] if arms == "dual" else []
            for k, obj in enumerate(replies):
                arm = "left"
                if arms == "dual":
                    arm = obj.get("arm") if obj.get("arm") in ("left", "right") else None
                    if arm is None and len(segs) == len(replies):
                        # exact positional evidence: the k-th bridge segment issued the k-th reply,
                        # and a segment without --arm drives the left bridge
                        m = re.search(r"--arm\s+(left|right)", segs[k])
                        arm = m.group(1) if m else "left"
                    if arm is None and isinstance(obj.get("capture"), int):
                        arm = "right" if "frames_right/" in json.dumps(obj.get("files") or {}) else "left"
                    if arm is None:
                        arm = "left"
                cid = obj.get("id") if isinstance(obj.get("id"), int) else None
                cap0 = obj.get("capture") if isinstance(obj.get("capture"), int) else None
                cand = rows_by_id.get(cid) if (arm == "left" and cid is not None) else None
                if arms == "dual" and cand is not None and ((cand.get("cmd") == "frames") != (cap0 is not None)):
                    arm, cand = "right", None   # the reply's shape contradicts that row: the other bridge sent it
                if arms == "dual" and arm == "left" and cid is not None and (cid not in rows_by_id or cid in claimed):
                    arm, cand = "right", None   # an id that is unknown or already taken belongs to the other bridge
                row = cand
                if row is not None:
                    claimed.add(cid)
                cap = obj.get("capture") if isinstance(obj.get("capture"), int) else None
                st["calls"].append(dict(arm=arm, cmd_id=cid if row is not None else None,
                                        cmd=(row or {}).get("cmd") or "",
                                        args_json=(row or {}).get("args_json") or "",
                                        ok=obj.get("ok") if isinstance(obj.get("ok"), bool) else None,
                                        capture_index=cap if arm == "left" else None))
            steps.append(st)
    return steps


def _iso(t):
    if not t:
        return None
    try:
        from datetime import datetime, timezone
        return datetime.fromisoformat(t.replace("Z", "+00:00")).replace(tzinfo=timezone.utc).timestamp()
    except Exception:
        return None


def claude_steps(path, rows_sorted, caps_sorted):
    """Claude Code harness: one content block per assistant line, the tool result in the next
    user line. The bridge answer is often not in the trace at all, so the bridge rows are swept
    onto the Bash steps by wall clock — both sides use the same clock."""
    steps, pend = [], {}
    for line in open(path, errors="replace"):
        line = line.strip()
        if not line:
            continue
        try:
            ev = json.loads(line)
        except json.JSONDecodeError:
            continue
        t = ev.get("type")
        if t == "assistant":
            for b in ((ev.get("message") or {}).get("content") or []):
                if not isinstance(b, dict):
                    continue
                bt = b.get("type")
                if bt == "tool_use":
                    inp = b.get("input") if isinstance(b.get("input"), dict) else {}
                    cmd = inp.get("command") or inp.get("file_path") or json.dumps(inp, ensure_ascii=False, sort_keys=True)
                    st = _step("command", b.get("id") or "", command=f"{b.get('name') or ''}: {cmd}")
                    st["_tool"] = b.get("name") or ""
                    st["_t0"] = _iso(ev.get("timestamp")); st["_t1"] = None
                    steps.append(st); pend[b.get("id")] = st
                elif bt in ("thinking", "redacted_thinking") and (b.get("thinking") or "").strip():
                    steps.append(_step("reasoning", text=b["thinking"]))          # signature never published
                elif bt == "text" and (b.get("text") or "").strip():
                    steps.append(_step("message", text=b["text"]))
        elif t == "user":
            for b in ((ev.get("message") or {}).get("content") or []):
                if isinstance(b, dict) and b.get("type") == "tool_result" and b.get("tool_use_id") in pend:
                    st = pend[b["tool_use_id"]]
                    st["_t1"] = _iso(ev.get("timestamp"))
                    c = b.get("content")
                    st["output"] = c if isinstance(c, str) else "[image content omitted; the file is named in the command]"
                    st["exit_code"] = "error" if b.get("is_error") else "0"
        elif t == "error":
            steps.append(_step("error", text=json.dumps({k: v for k, v in ev.items() if k not in ("session_id", "release_batch")}, ensure_ascii=False)))
    # sweep the bridge rows onto the Bash steps, in order, by wall clock
    bash = [st for st in steps if st["kind"] == "command" and st.get("_tool") == "Bash"]
    ri, ci = 0, 0
    for n, st in enumerate(bash):
        t1 = st.get("_t1") or (bash[n + 1].get("_t0") if n + 1 < len(bash) else None) or float("inf")
        while ri < len(rows_sorted) and (rows_sorted[ri]["request_t"] or 0) <= t1:
            r = rows_sorted[ri]; ri += 1
            cap = None
            if r["cmd"] == "frames" and ci < len(caps_sorted):
                cap = caps_sorted[ci]; ci += 1
            st["calls"].append(dict(arm="left", cmd_id=int(r["cmd_id"]) if str(r["cmd_id"]).isdigit() else None,
                                    cmd=r["cmd"] or "", args_json=r["args_json"] or "",
                                    ok=None if str(r["ok"]) in ("", "None") else str(r["ok"]).strip().lower() == "true",
                                    capture_index=cap))
    if ri < len(rows_sorted) and bash:                      # anything after the last result
        for r in rows_sorted[ri:]:
            cap = None
            if r["cmd"] == "frames" and ci < len(caps_sorted):
                cap = caps_sorted[ci]; ci += 1
            bash[-1]["calls"].append(dict(arm="left", cmd_id=int(r["cmd_id"]) if str(r["cmd_id"]).isdigit() else None,
                                          cmd=r["cmd"] or "", args_json=r["args_json"] or "",
                                          ok=None if str(r["ok"]) in ("", "None") else str(r["ok"]).strip().lower() == "true",
                                          capture_index=cap))
    return steps


def assemble(sid, steps, cmeta, url, lim_out=LIM_OUT, lim_cmd=LIM_CMD):
    """Turn raw steps into the traces[] elements and the reasoning_summary."""
    out, texts = [], []
    for n, st in enumerate(steps):
        o, cut = clip(st["output"], lim_out)
        c, _ = clip(st["command"], lim_cmd)
        calls = []
        for call in st["calls"]:
            ci = call["capture_index"]
            m = cmeta.get(ci) if ci is not None else None
            calls.append(dict(arm=call["arm"], cmd_id=call["cmd_id"], cmd=call["cmd"],
                              args_json=call["args_json"], ok=call["ok"],
                              capture_index=ci if m else None,
                              depth_url=url(f"depth/{sid}/{ci:04d}_wrist_depth.png") if m and m["has_depth"] else "",
                              pose_json=m["meta_json"] if m else ""))
        out.append(dict(step=n, item_id=st["item_id"], kind=st["kind"], text=st["text"],
                        command=c, output=o, output_truncated=cut, exit_code=st["exit_code"], calls=calls))
        if st["kind"] in ("reasoning", "message") and st["text"].strip():
            texts.append(st["text"].strip())
    size = sum(len(e["text"]) + len(e["command"]) + len(e["output"]) for e in out)
    if size > SESSION_TRACE_CAP and lim_out > 400:          # guard; does not trigger on b01
        return assemble(sid, steps, cmeta, url, lim_out // 2, lim_cmd)
    summary = "\n\n".join(texts)
    if len(summary) > SUMMARY_CAP:
        summary = summary[:SUMMARY_CAP] + f"\n\n… [{len(summary) - SUMMARY_CAP} more characters]"
    return out, summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True, help="flat export root (holds metadata/, traces/, depth/, ...)")
    ap.add_argument("--batch", default="b01")
    ap.add_argument("--repo", default="Agent-as-Policy/agent-as-policy")
    ap.add_argument("--out", required=True, help="output root; writes <out>/trials/ and the side folders")
    ap.add_argument("--rows-per-shard", type=int, default=8)
    ap.add_argument("--aside", default="reset",
                    help="comma-separated splits written to <out>/<split>/ instead of trials/: kept in the repository but not a Viewer split")
    ap.add_argument("--sessions", default=None, help="release list: only these session_ids are written")
    add_fa_arg(ap)
    a = ap.parse_args()
    D = a.dataset
    url = lambda rel: f"https://huggingface.co/datasets/{a.repo}/resolve/main/{rel}"

    # --- scalar rows, per split
    keep = set(l.strip() for l in open(a.sessions) if l.strip() and not l.startswith("#")) if a.sessions else None
    splits = {}
    for sp in ("evaluated", "reset", "auxiliary"):
        sp_path = os.path.join(D, "metadata", f"trials_{sp}-{a.batch}.parquet")
        if not os.path.exists(sp_path):
            continue          # a batch need not contain every split: b02 has no reset or auxiliary runs
        t = pq.read_table(sp_path)
        if keep is not None:
            t = t.filter(pc.is_in(t["session_id"], value_set=pa.array(sorted(keep))))
        if t.num_rows:
            splits[sp] = fill_tokens_from_traces(t.sort_by("session_id"), os.path.join(D, "traces"))
    all_sids = [s for sp in splits.values() for s in sp["session_id"].to_pylist()]
    require_sessions(a.fa, all_sids, "--sessions" if a.sessions else f"trials_*-{a.batch}.parquet")

    # --- documents: the distinct prompt / interface texts, inline in the row and as files
    def doc_map(name, col, stem_of):
        by_sid, written = {}, 0
        for r in pq.read_table(os.path.join(D, "metadata", f"{name}-{a.batch}.parquet")).to_pylist():
            sids = [s for s in json.loads(r["sessions_json"]) if s in all_sids]
            if not sids:
                continue
            for s in sids:
                by_sid[s] = r[col]
            stem = stem_of(sids[0])
            dst = os.path.join(a.out, "documents", name, f"{stem}-{r['sha256'][:8]}.md")
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            if not os.path.exists(dst):
                open(dst, "w").write(r[col]); written += 1
        print(f"  documents/{name}: {written} files", flush=True)
        return by_sid
    prompt_file = {r["session_id"]: r.get("prompt_file") or "" for sp in splits.values() for r in sp.select(["session_id", "prompt_file"]).to_pylist()}
    prompt_of = doc_map("prompts", "prompt_text", lambda s: os.path.splitext(prompt_file.get(s) or "prompt")[0])
    interface_of = doc_map("interfaces", "interface_text", lambda s: "interface")

    # --- reference sets: id from the session env, files from the inventory, copied next to the table
    inv = {r["goal_set_id"]: json.loads(r["files_json"]) for r in pq.read_table(os.path.join(D, "metadata", f"goal_sets-{a.batch}.parquet")).to_pylist()}
    ref_of = {}
    for sid in all_sids:
        envp = os.path.join(a.fa, "sessions", sid, "session.env")
        gid = ""
        if os.path.exists(envp):
            for line in open(envp):
                if line.startswith("GOAL="):
                    gid = os.path.basename(line.strip().split("=", 1)[1].rstrip("/"))
        ref_of[sid] = {"id": gid, "files": [{"path": f["path"], "bytes": f["bytes"], "url": url(f"reference/{gid}/{f['path']}")} for f in inv.get(gid, [])]}
        for f in inv.get(gid, []):
            src, dst = os.path.join(a.fa, "goal_sets", gid, f["path"]), os.path.join(a.out, "reference", gid, f["path"])
            if os.path.exists(src) and not os.path.exists(dst):
                os.makedirs(os.path.dirname(dst), exist_ok=True); shutil.copyfile(src, dst)

    # --- bridge requests and captures, grouped by session: they become fields of traces[]
    cmd_t, cmd_i = group_index(pq.read_table(os.path.join(D, "metadata", f"commands-{a.batch}.parquet")))
    cap_t, cap_i = group_index(pq.read_table(os.path.join(D, "metadata", f"captures-{a.batch}.parquet")))

    # --- evidence photos by (batch, trial, kind)
    ev = {}
    for r in csv.DictReader(open(os.path.join(D, "evidence", "metadata.csv"), newline="")):
        ev[(r["batch"], r["trial"], r["kind"])] = (os.path.join(D, "evidence", r["file_name"]), r["file_name"])

    # --- videos and previews
    vids = {}
    for sub in ("videos", "videos_side"):
        for r in csv.DictReader(open(os.path.join(D, sub, "metadata.csv"), newline="")):
            vids.setdefault(r["session_id"], []).append(dict(
                camera=r["camera"], url=url(f"{sub}/{r['file_name']}"), bytes=int(r["bytes"]),
                rec_start_s=float(r["rec_start_s"]) if r.get("rec_start_s") else None,
                rec_stop_s=float(r["rec_stop_s"]) if r.get("rec_stop_s") else None))
    previews = {}
    pm = os.path.join(D, "preview", "metadata.csv")
    if os.path.exists(pm):
        for r in csv.DictReader(open(pm, newline="")):
            previews[r["session_id"]] = (os.path.join(D, "preview", f"{r['session_id']}.mp4"), r["source_file"])

    feats = Features({
        "session_id": Value("string"), "batch": Value("string"), "trial": Value("string"), "task": Value("string"),
        "evidence_before": Image(), "evidence_after": Image(),
        "video_preview": Video(),
        "arms": Value("string"), "model": Value("string"), "backend": Value("string"), "effort": Value("string"),
        "outcome": Value("string"),
        "labeling": {"source": Value("string"), "agent_self_report": Value("string")},
        "timing": {"start_epoch_s": Value("int64"), "end_epoch_s": Value("int64"), "duration_s": Value("int64"),
                   "timed_out": Value("bool"), "aborted": Value("bool")},
        "bridge": {"cmds_counted": Value("int64"), "gripper_cmds": Value("int64"), "failed_cmds": Value("int64")},
        "tokens": {"input": Value("int64"), "output": Value("int64"), "reasoning": Value("int64")},
        "reasoning_summary": Value("string"),
        "traces": List({"step": Value("int64"), "item_id": Value("string"), "kind": Value("string"),
                        "text": Value("string"), "command": Value("string"), "output": Value("string"),
                        "output_truncated": Value("bool"), "exit_code": Value("string"),
                        "calls": List({"arm": Value("string"), "cmd_id": Value("int64"), "cmd": Value("string"),
                                       "args_json": Value("string"), "ok": Value("bool"),
                                       "capture_index": Value("int64"), "depth_url": Value("string"),
                                       "pose_json": Value("string")})}),
        "prompt_text": Value("string"), "interface_text": Value("string"),
        "reference": {"id": Value("string"), "files": List({"path": Value("string"), "bytes": Value("int64"), "url": Value("string")})},
        # the overhead imagery is cropped per trial; the rectangles ship as files
        # (crops/crops-<batch>.csv and the manifests), not as columns
        "videos": List({"camera": Value("string"), "url": Value("string"), "bytes": Value("int64"),
                        "rec_start_s": Value("float64"), "rec_stop_s": Value("float64")}),
    })
    schema = feats.arrow_schema.with_metadata({"huggingface": json.dumps({"info": {"features": feats.to_dict()}})})

    stats = {}
    for sp, st in splits.items():
        sids = st["session_id"].to_pylist()
        rows = st.to_pylist()
        cols = {c: st[c] for c in ("session_id", "batch", "trial", "task", "arms", "model", "backend", "effort", "outcome")}
        cols["labeling"] = pa.array([{"source": r["label_source"], "agent_self_report": r["agent_self_report"]} for r in rows], type=schema.field("labeling").type)
        cols["timing"] = pa.array([{k: r[k] for k in ("start_epoch_s", "end_epoch_s", "duration_s", "timed_out", "aborted")} for r in rows], type=schema.field("timing").type)
        cols["bridge"] = pa.array([{k: r[k] for k in ("cmds_counted", "gripper_cmds", "failed_cmds")} for r in rows], type=schema.field("bridge").type)
        cols["tokens"] = pa.array([{"input": r["tokens_input"], "output": r["tokens_output"], "reasoning": r["tokens_reasoning"]} for r in rows], type=schema.field("tokens").type)

        traces, summaries, n_steps, n_bridge, n_cap, unmatched = [], [], 0, 0, 0, []
        for r in rows:
            sid = r["session_id"]
            st_, l = cmd_i.get(sid, (0, 0))
            brows = cmd_t.slice(st_, l).to_pylist() if l else []
            st_, l = cap_i.get(sid, (0, 0))
            crows = cap_t.slice(st_, l).to_pylist() if l else []
            cmeta = {c["capture_index"]: c for c in crows}
            p = os.path.join(D, "traces", f"{sid}.jsonl")
            if not os.path.exists(p):
                traces.append([]); summaries.append(""); continue
            if (r.get("backend") or "") == "claude":
                steps = claude_steps(p, sorted(brows, key=lambda x: int(x["cmd_id"]) if str(x["cmd_id"]).isdigit() else 0),
                                     sorted(c["capture_index"] for c in crows))
            else:
                steps = codex_steps(p, {int(x["cmd_id"]): x for x in brows if str(x["cmd_id"]).isdigit()}, r.get("arms") or "")
            el, summary = assemble(sid, steps, cmeta, url)
            got = {c["cmd_id"] for e in el for c in e["calls"] if c["cmd_id"] is not None}
            if len(got) != len(brows):
                unmatched.append((sid, len(got), len(brows)))
            traces.append(el); summaries.append(summary)
            n_steps += len(el); n_bridge += len(got)
            n_cap += len({c["capture_index"] for e in el for c in e["calls"] if c["capture_index"] is not None})
        cols["traces"] = pa.array(traces, type=schema.field("traces").type)
        cols["reasoning_summary"] = pa.array(summaries, pa.string())

        for kind, col in (("before", "evidence_before"), ("after", "evidence_after")):
            vals = []
            for r in rows:
                hit = ev.get((r["batch"], r["trial"], kind)) if r["batch"] else None
                vals.append({"bytes": thumb(hit[0]), "path": hit[1].replace(".png", ".jpg")} if hit and os.path.exists(hit[0]) else None)
            cols[col] = pa.array(vals, type=IMAGE_T)
        cols["video_preview"] = pa.array([{"bytes": read_bytes(previews[s][0]), "path": f"preview/{s}.mp4"} if s in previews else None for s in sids], type=IMAGE_T)
        cols["prompt_text"] = pa.array([prompt_of.get(s, "") for s in sids], pa.string())
        cols["interface_text"] = pa.array([interface_of.get(s, "") for s in sids], pa.string())
        cols["reference"] = pa.array([ref_of[s] for s in sids], type=schema.field("reference").type)
        cols["videos"] = pa.array([vids.get(s, []) for s in sids], type=schema.field("videos").type)

        table = pa.Table.from_arrays([cols[n] for n in schema.names], schema=schema)
        n = table.num_rows
        sub = sp if sp in a.aside.split(",") else "trials"
        os.makedirs(os.path.join(a.out, sub), exist_ok=True)
        for i in range(0, n, a.rows_per_shard):
            p = os.path.join(a.out, sub, f"trials_{sp}-{a.batch}-{i // a.rows_per_shard:04d}.parquet")
            with pq.ParquetWriter(p, schema, compression="zstd") as w:
                for j in range(i, min(i + a.rows_per_shard, n)):
                    w.write_table(table.slice(j, 1))
        size = sum(os.path.getsize(p) for p in glob.glob(os.path.join(a.out, sub, f"trials_{sp}-{a.batch}-*.parquet")))
        stats[sp] = dict(rows=n, dir=sub, shards=-(-n // a.rows_per_shard), MiB=round(size / 2**20, 1),
                         trace_steps=n_steps, bridge_matched=n_bridge, bridge_rows=sum(cmd_i.get(s, (0, 0))[1] for s in sids),
                         captures_linked=n_cap, capture_rows=sum(cap_i.get(s, (0, 0))[1] for s in sids),
                         unmatched_total=sum(b - g for _, g, b in unmatched),
                         evidence=sum(1 for c in ("evidence_before", "evidence_after") for v in cols[c].to_pylist() if v),
                         previews=sum(1 for v in cols["video_preview"].to_pylist() if v),
                         videos=int(pc.sum(pc.list_value_length(cols["videos"])).as_py() or 0),
                         summary_chars=sum(len(x) for x in summaries),
                         unmatched_bridge_sessions=unmatched[:5])
        print(sp, stats[sp], flush=True)
    json.dump(stats, open(os.path.join(a.out, "trials", f"build_stats-{a.batch}.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
