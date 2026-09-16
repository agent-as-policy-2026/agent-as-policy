#!/usr/bin/env python3
"""Render an AgP session's event log as a readable transcript.

    python3 tools/agent_transcript.py <session_dir> [-o OUT.md]

Handles both backends:
  * codex  (`codex exec --json`): item.completed events with item.type in
    reasoning / agent_message / command_execution.
  * agy    (`agy --output-format stream-json`): init / step_update (grouped by
    step_index; text deltas concatenated; tool steps summarised) / result.
  * claude (`claude -p --output-format stream-json`): system/init, assistant
    (thinking / text / tool_use blocks), user (tool_result), result.
Reads agent_events.jsonl, falling back to the pre-2026-09-04 codex_events.jsonl.
Robot commands (robot_client.py calls) are shown one per line with the start of
their output; everything else that is not text is shown as a compact JSON stub.
"""
from __future__ import annotations

import argparse
import json
import os
import sys


def _find(obj, keys, depth=0):
    """First value under any of `keys` anywhere in a nested dict/list."""
    if depth > 6:
        return None
    if isinstance(obj, dict):
        for k in keys:
            if k in obj and obj[k] not in (None, ""):
                return obj[k]
        for v in obj.values():
            r = _find(v, keys, depth + 1)
            if r is not None:
                return r
    elif isinstance(obj, list):
        for v in obj:
            r = _find(v, keys, depth + 1)
            if r is not None:
                return r
    return None


def _robot_line(cmd: str, output: str) -> str:
    core = cmd.split("robot_client.py . ", 1)[-1].strip().rstrip("\"' ") if "robot_client.py" in cmd else cmd.strip()
    out = " ".join(str(output).split())[:160]
    return f"- `robot {core[:150]}` → {out}"


def render_codex(events):
    out, n_r, n_m, n_c = [], 0, 0, 0
    for e in events:
        if e.get("type") != "item.completed":
            continue
        it = e.get("item") or {}
        t = it.get("type")
        if t == "reasoning":
            n_r += 1
            out.append(f"\n### [thinking {n_r}]\n{str(it.get('text', '')).strip()}\n")
        elif t == "agent_message":
            n_m += 1
            out.append(f"\n**[agent message {n_m}]** {str(it.get('text', '')).strip()}\n")
        elif t == "command_execution":
            n_c += 1
            out.append(_robot_line(str(it.get("command", "")), it.get("aggregated_output", "")))
    return out, {"thinking": n_r, "messages": n_m, "commands": n_c}


def render_agy(events):
    out, n_r, n_m, n_c = [], 0, 0, 0
    steps: dict[int, dict] = {}
    order: list[int] = []
    for e in events:
        ev = e.get("event")
        if ev == "init":
            ini = e.get("init") or {}
            out.append(f"_model {ini.get('model')} · cwd {ini.get('cwd')} · {len(ini.get('tools') or [])} tools_\n")
        elif ev == "step_update":
            su = e.get("step_update") or {}
            idx = int(su.get("step_index", -1))
            st = steps.setdefault(idx, {"type": su.get("step_type"), "text": "", "raw": []})
            if idx not in order:
                order.append(idx)
            if su.get("text_delta"):
                st["text"] += str(su["text_delta"])
            st["raw"].append(su)
        elif ev == "result":
            res = e.get("result") or {}
            out.append(f"\n**[result {res.get('status')}]** {str(res.get('response', '')).strip()[:2000]}\n")
    body = []
    for idx in order:
        st = steps[idx]
        t = st["type"]
        if t == "user_input":
            continue
        if t in ("agent_response", "assistant_message"):
            n_m += 1
            body.append(f"\n**[agent message {n_m}]** {st['text'].strip()}\n")
        elif t in ("thinking", "reasoning", "thought"):
            n_r += 1
            body.append(f"\n### [thinking {n_r}]\n{st['text'].strip()}\n")
        else:
            last = st["raw"][-1]
            cmd = _find(last, ("CommandLine", "command_line", "command", "Command"))
            outp = _find(last, ("output", "Output", "stdout", "result"))
            if cmd:
                n_c += 1
                body.append(_robot_line(str(cmd), outp if outp is not None else ""))
            elif st["text"].strip():
                body.append(f"\n[{t}] {st['text'].strip()[:1500]}\n")
            else:
                body.append(f"- [{t}] {json.dumps(last, ensure_ascii=False)[:300]}")
    return out + body, {"thinking": n_r, "messages": n_m, "commands": n_c}


def render_claude(events):
    """claude -p --output-format stream-json: system/init, assistant (message.content blocks: text,
    thinking, tool_use), user (tool_result blocks), result."""
    out, n_r, n_m, n_c = [], 0, 0, 0
    pending: dict[str, str] = {}   # tool_use id -> "Tool: command/path"
    for e in events:
        t = e.get("type")
        if t == "system" and e.get("subtype") == "init":
            out.append(f"_model {e.get('model')} · cwd {e.get('cwd')} · {len(e.get('tools') or [])} tools · skills {e.get('skills')}_\n")
        elif t == "assistant":
            for b in ((e.get("message") or {}).get("content") or []):
                bt = b.get("type")
                if bt == "thinking" and str(b.get("thinking", "")).strip():
                    n_r += 1
                    out.append(f"\n### [thinking {n_r}]\n{str(b.get('thinking', '')).strip()}\n")
                elif bt == "text" and str(b.get("text", "")).strip():
                    n_m += 1
                    out.append(f"\n**[agent message {n_m}]** {str(b.get('text', '')).strip()}\n")
                elif bt == "tool_use":
                    inp = b.get("input") or {}
                    cmd = inp.get("command") or inp.get("file_path") or inp.get("description") or json.dumps(inp, ensure_ascii=False)[:200]
                    pending[str(b.get("id"))] = f"{b.get('name')}: {cmd}"
        elif t == "user":
            for b in ((e.get("message") or {}).get("content") or []):
                if not isinstance(b, dict) or b.get("type") != "tool_result":
                    continue
                cmd = pending.pop(str(b.get("tool_use_id")), "?")
                c = b.get("content")
                text = c if isinstance(c, str) else " ".join(str(x.get("text", "")) for x in (c or []) if isinstance(x, dict))
                if "robot_client.py" in cmd:
                    n_c += 1
                    out.append(_robot_line(cmd.split(": ", 1)[-1], text))
                else:
                    out.append(f"- [{cmd[:120]}] {' '.join(str(text).split())[:160]}")
        elif t == "result":
            out.append(f"\n**[result {e.get('subtype')}]** {str(e.get('result', '')).strip()[:2000]}\n"
                       f"_turns {e.get('num_turns')} · cost {e.get('total_cost_usd')} USD · {e.get('duration_ms')} ms_\n")
    return out, {"thinking": n_r, "messages": n_m, "commands": n_c}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("-o", "--out")
    a = ap.parse_args()
    src = None
    for name in ("agent_events.jsonl", "codex_events.jsonl"):
        p = os.path.join(a.session, name)
        if os.path.exists(p):
            src = p
            break
    if src is None:
        print("no agent_events.jsonl / codex_events.jsonl in", a.session, file=sys.stderr)
        return 2
    events = []
    for line in open(src, encoding="utf-8", errors="replace"):
        line = line.strip()
        if not line:
            continue
        try:
            events.append(json.loads(line))
        except Exception:
            continue
    if any(e.get("event") in ("init", "step_update", "result") for e in events):
        backend, render = "agy", render_agy
    elif any(e.get("type") in ("assistant", "result") and ("message" in e or "usage" in e) for e in events):
        backend, render = "claude", render_claude
    else:
        backend, render = "codex", render_codex
    lines, counts = render(events)
    head = (f"# {os.path.basename(os.path.abspath(a.session))} — {backend} transcript "
            f"({counts['thinking']} thinking, {counts['messages']} messages, {counts['commands']} robot commands)\n")
    text = head + "\n".join(lines) + "\n"
    dst = a.out or os.path.join(a.session, "agent_transcript.md")
    open(dst, "w", encoding="utf-8").write(text)
    print(dst, counts)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
