#!/usr/bin/env python3
"""Token usage of an AgP session, normalised across backends.

    python3 tools/agent_usage.py <session_dir or agent_events.jsonl>   -> one JSON line

Keys: backend, input_tokens, cached_input_tokens, output_tokens, reasoning_output_tokens, cost_usd.
  codex  : the last `turn.completed` event's usage (input_tokens, cached_input_tokens, output_tokens,
           reasoning_output_tokens); no cost.
  claude : the `result` event's usage: input_tokens + cache_creation_input_tokens (uncached input),
           cache_read_input_tokens (cached), output_tokens, output_tokens_details.thinking_tokens,
           total_cost_usd.
  agy    : the `result` event's result.usage: input_tokens, cache_read_tokens, output_tokens,
           thinking_tokens; no cost.
Absent (run killed before the final event) -> empty strings.
"""
import json, os, sys

def load(path):
    if os.path.isdir(path):
        path = os.path.join(path, "agent_events.jsonl")
    out = []
    try:
        for line in open(path, encoding="utf-8", errors="replace"):
            line = line.strip()
            if line:
                try: out.append(json.loads(line))
                except Exception: pass
    except FileNotFoundError:
        pass
    return out

def usage(events):
    r = {"backend": "", "input_tokens": "", "cached_input_tokens": "", "output_tokens": "", "reasoning_output_tokens": "", "cost_usd": ""}
    for e in events:
        if e.get("type") == "turn.completed":                      # codex exec --json
            u = e.get("usage") or {}
            r.update(backend="codex", input_tokens=u.get("input_tokens", ""), cached_input_tokens=u.get("cached_input_tokens", ""),
                     output_tokens=u.get("output_tokens", ""), reasoning_output_tokens=u.get("reasoning_output_tokens", ""))
        elif e.get("type") == "result" and "usage" in e:          # claude -p --output-format stream-json
            u = e.get("usage") or {}
            r.update(backend="claude",
                     input_tokens=int(u.get("input_tokens", 0) or 0) + int(u.get("cache_creation_input_tokens", 0) or 0),
                     cached_input_tokens=u.get("cache_read_input_tokens", ""), output_tokens=u.get("output_tokens", ""),
                     reasoning_output_tokens=(u.get("output_tokens_details") or {}).get("thinking_tokens", ""),
                     cost_usd=e.get("total_cost_usd", ""))
        elif e.get("event") == "result":                           # agy --output-format stream-json
            u = (e.get("result") or {}).get("usage") or {}
            r.update(backend="agy", input_tokens=u.get("input_tokens", ""), cached_input_tokens=u.get("cache_read_tokens", ""),
                     output_tokens=u.get("output_tokens", ""), reasoning_output_tokens=u.get("thinking_tokens", ""))
    return r

if __name__ == "__main__":
    print(json.dumps(usage(load(sys.argv[1] if len(sys.argv) > 1 else "."))))
