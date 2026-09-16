#!/usr/bin/env python3
"""Export agent event streams, one jsonl per session, with local paths scrubbed.

These are Codex thread events (item.started / item.completed / turn.* / error),
NOT the Hub's session-trace format, so they will render as plain jsonl rather
than in the native trace viewer.
"""
import argparse, json, os, re

from _paths import add_fa_arg, require_sessions

# The local account name also leaks in bare form, e.g. the owner column of `ls -l`
# output captured in a tool result. Derive it instead of hard-coding it.
_USER = os.path.basename(os.path.expanduser("~"))

# The workspace root is rewritten to <REPO> before any other rule, so the released
# transcripts carry no local directory layout. The root is whatever holds the session
# tree this run reads from (--fa / $AGP_WORKSPACE), so the pattern follows the data.


def build_subs(workspace):
    """The substitutions applied, in order, to every raw line before json.loads."""
    ws = os.path.abspath(workspace)
    rel = os.path.relpath(ws, os.path.expanduser("~")).replace(os.sep, "/")
    outside_home = rel == os.pardir or rel.startswith(os.pardir + "/")
    # the trailing lookahead keeps a sibling directory (..._backup_20260908) from
    # being rewritten to "<REPO>_backup_20260908".
    if outside_home:      # a checkout outside every home directory: match it literally
        subs = [(re.compile(re.escape(ws) + r"(?![A-Za-z0-9_-])"), "<REPO>")]
    else:                 # under a home directory: any account name, same relative path
        subs = [(re.compile(r"/home/[^/\s\"']+/" + re.escape(rel) + r"(?![A-Za-z0-9_-])"), "<REPO>")]
    return subs + [
        (re.compile(r"/home/[^/\s\"']+/mz_workspace"), "<COLLEAGUE_RIG_ROOT>"),
        (re.compile(r"/home/[^/\s\"']+"), "<HOME>"),
        (re.compile(r"usb-[0-9A-Za-z_]+_[0-9A-Za-z]{6,}"), "usb-<CAMERA_SERIAL>"),
        (re.compile(r"\b(sk-[A-Za-z0-9_-]{20,}|hf_[A-Za-z0-9]{20,}|ghp_[A-Za-z0-9]{20,}|ya29\.[A-Za-z0-9_-]{20,})"), "<REDACTED_TOKEN>"),
        (re.compile(r"\b" + re.escape(_USER) + r"\b"), "<USER>"),
    ]

# The Claude backend writes a harness init line as the first event of a session.
# It is not model output: it carries the working directory, the MCP server list,
# permissionMode, memory paths and a messaging socket path containing the numeric
# uid. Regex scrubbing does not reach it because none of those are paths or
# tokens, so the line is rebuilt from an explicit allow-list instead.
INIT_KEEP = {"type", "subtype", "model", "claude_code_version", "output_style",
             "session_id", "release_batch"}

def strip_init(obj):
    if obj.get("type") == "system" and obj.get("subtype") == "init":
        return {k: v for k, v in obj.items() if k in INIT_KEEP}
    return obj

def scrub(t, subs):
    for pat, rep in subs:
        t = pat.sub(rep, t)
    return t

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--batch", default="b01")
    ap.add_argument("--out", required=True)
    add_fa_arg(ap)
    a = ap.parse_args()
    # the workspace root is the directory holding the session tree; it is what <REPO> hides
    subs = build_subs(os.path.dirname(os.path.abspath(a.fa)))
    sessions = [l.strip() for l in open(a.snapshot) if l.strip()]
    require_sessions(a.fa, sessions, "--snapshot")
    d = os.path.join(a.out, "traces")
    os.makedirs(d, exist_ok=True)
    n = src = dst = 0
    lines = bad = 0
    for sid in sessions:
        p = os.path.join(a.fa, "sessions", sid, "agent_events.jsonl")
        if not os.path.exists(p):
            continue
        src += os.path.getsize(p)
        op = os.path.join(d, f"{sid}.jsonl")
        with open(op, "w") as out:
            for line in open(p, errors="replace"):
                line = line.strip()
                if not line:
                    continue
                lines += 1
                try:
                    obj = json.loads(scrub(line, subs))
                except Exception:
                    bad += 1
                    continue
                obj = strip_init(obj)
                obj["session_id"] = sid
                obj["release_batch"] = a.batch
                out.write(json.dumps(obj, ensure_ascii=False) + "\n")
        dst += os.path.getsize(op)
        n += 1
    print(f"  traces  {n} 个会话 / {lines} 行  {src/2**30:.2f} GiB -> {dst/2**30:.2f} GiB  (解析失败 {bad} 行)")

if __name__ == "__main__":
    main()
