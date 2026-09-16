#!/usr/bin/env bash
# One-shot AgP experiment on the REAL left arm:
#   recorders + server up  ->  agent launched  ->  wait for the agent to exit  ->  everything down  ->  summary
#
#   run_experiment.sh <name> --prompt FILE [--goal DIR] [--allow-motion] [--timeout-min N] [--no-video]
#                     [--backend agy|codex] [--model MODEL] [--effort low|medium|high|xhigh] [--fast]
#                     [-- <server args>]
#   (--effort / --fast apply to codex: model_reasoning_effort and service_tier="fast";
#    also FA_EFFORT / FA_FAST env. Without them codex uses ~/.codex/config.toml.)
#
# Agent backend (--backend, or FA_BACKEND env; default: codex):
#   codex  OpenAI codex exec, danger-full-access, codex's default model unless --model.
#   agy    Google Antigravity CLI in print mode, full tool permissions, model default
#          gemini-3.8-flash-high (FA_MODEL / --model override; `agy models` lists them).
#          CAUTION (2026-09-04 trial): agy re-uploads every viewed image on every turn;
#          one turn stalled for >10 min at ~30 KB/s. Image resolution is not to be
#          reduced for this (user decision); the backend is kept only as an option.
# Per-session files are backend-neutral: agent.pid, agent_events.jsonl (stream-json / codex
# --json), agent_stderr.log, agent_last_message.txt. session.env records AGENT_BACKEND,
# AGENT_MODEL, AGENT_START_S / AGENT_END_S (CODEX_START_S / CODEX_END_S are still written
# for older tooling). tools/agent_transcript.py renders either event format.
#
# Recording is bracketed around the agent run: all three views (side camera via
# ffmpeg, top + wrist via the bridge recorder) start a few seconds before the agent
# and stop a few seconds after it exits.
#
# Preconditions are the launcher's (bridge up in the right mode, no other bridge
# observer, e-stop in hand for --allow-motion; FA_YES=1 confirms the motion
# checklist when there is no TTY). The chosen CLI must be logged in.
set -euo pipefail

FA="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$HOME/.local/bin:$PATH"

[ $# -ge 1 ] || { sed -n 2,23p "$0" | sed 's/^# \{0,1\}//'; exit 2; }
NAME="$1"; shift
TIMEOUT_MIN=150
BACKEND="${FA_BACKEND:-codex}"
MODEL="${FA_MODEL:-}"
EFFORT="${FA_EFFORT:-}"          # codex: model_reasoning_effort (low|medium|high|xhigh); empty = CLI default
FAST="${FA_FAST:-}"              # codex: service_tier="fast" when non-empty
PASS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --timeout-min) TIMEOUT_MIN="$2"; shift 2 ;;
    --backend) BACKEND="$2"; shift 2 ;;
    --model) MODEL="$2"; shift 2 ;;
    --effort) EFFORT="$2"; shift 2 ;;
    --fast) FAST=1; shift ;;
    --) PASS+=("$@"); break ;;
    *) PASS+=("$1"); shift ;;
  esac
done
case "$BACKEND" in
  agy)    [ -n "$MODEL" ] || MODEL="gemini-3.8-flash-high" ;;
  codex)  [ -n "$MODEL" ] || MODEL="codex-default" ;;
  claude) [ -n "$MODEL" ] || MODEL="claude-opus-5" ;;     # 2026-09-10: Claude Code in print mode (see the launch below)
  *) echo "[experiment] unknown --backend '$BACKEND' (agy|codex|claude)" >&2; exit 2 ;;
esac
command -v "$BACKEND" >/dev/null || { echo "[experiment] $BACKEND CLI not on PATH" >&2; exit 2; }

# 1. session up (server + knowledge + prompt; NO recorders yet — they start with the agent)
bash "$FA/run_real_probe.sh" start "$NAME" --no-video "${PASS[@]}" \
  | grep -vE 'launch line|nohup (codex|agy)|model_reasoning|_events.jsonl|--print=|--output-format|^\s*$' || true
# by NAME, newest (2026-09-09: with a left and a right session starting concurrently sessions/LATEST is ambiguous)
S=$(ls -d "$FA"/sessions/*_"$NAME" 2>/dev/null | sort | tail -1 || true); [ -n "$S" ] || S=$(readlink -f "$FA/sessions/LATEST")
[ -f "$S/server.pid" ] && kill -0 "$(cat "$S/server.pid")" 2>/dev/null || { echo "[experiment] session did not come up" >&2; exit 3; }

# 2026-09-10: Ctrl-C / TERM while the agent runs (e.g. aborting a chain) must still stop the bridge
# recorder and the server — otherwise the recorder stays "active" for a session that no longer
# exists and every later rec-start is ignored ("bridge did not confirm recording").
on_abort() {
  trap - INT TERM
  echo "[experiment] interrupted -> stopping agent, recorders and server of $S" >&2
  [ -f "$S/agent.pid" ] && kill -TERM "$(cat "$S/agent.pid")" 2>/dev/null || true
  echo "AGENT_ABORTED=1" >> "$S/session.env"
  bash "$FA/run_real_probe.sh" rec-stop "$S" >/dev/null 2>&1 || true
  bash "$FA/run_real_probe.sh" stop "$S" >/dev/null 2>&1 || true
  exit 130
}
trap on_abort INT TERM

# 2. agent, then all three views start recording together
cd "$S"
T0=$(date +%s)
TIER=$([ -n "$FAST" ] && echo fast || echo default)
{ echo "AGENT_BACKEND=$BACKEND"; echo "AGENT_MODEL=$MODEL"; echo "AGENT_EFFORT=${EFFORT:-default}"; echo "AGENT_SERVICE_TIER=$TIER"; echo "AGENT_START_S=$T0"; echo "CODEX_START_S=$T0"; } >> "$S/session.env"
echo "[experiment] launching $BACKEND ($MODEL, effort ${EFFORT:-default}, tier $TIER) in $S (timeout ${TIMEOUT_MIN} min)"
case "$BACKEND" in
  agy)
    # print mode: the prompt is passed attached to the flag (agy rejects a bare -p + stdin);
    # stream-json gives one NDJSON event per step; the CLI's own wait timeout gets a margin
    # over ours so that our TERM/KILL below is what ends an overlong run.
    nohup agy --print="$(cat "$S/PROMPT.md")" --output-format stream-json --model "$MODEL" \
        --dangerously-skip-permissions --disable-slash-commands --print-timeout "$(( TIMEOUT_MIN + 5 ))m" \
        < /dev/null > "$S/agent_events.jsonl" 2> "$S/agent_stderr.log" &
    ;;
  claude)
    # Claude Code, print mode, stream-json (one event per line: system/init, assistant, user, result).
    # --disable-slash-commands removes every skill (incl. the user's own robot skill) from the agent's
    # system prompt; the session dir is a fresh project for Claude Code, so no auto-memory, and there is
    # no CLAUDE.md anywhere on the path (checked 2026-09-10). FA_CLAUDE_CONFIG_DIR may point at a
    # separate config dir (CLAUDE_CONFIG_DIR); default = the user's own login.
    CARGS=(--model "$MODEL"); [ -n "$EFFORT" ] && CARGS+=(--effort "$EFFORT")
    # `${VAR:+NAME=value} nohup …` cannot work: bash expands it in command-name position, so the
    # assignment was silently dropped. `env NAME=value` passes it on (same pid: env execs nohup execs claude).
    CENV=(); [ -n "${FA_CLAUDE_CONFIG_DIR:-}" ] && CENV=(env "CLAUDE_CONFIG_DIR=$FA_CLAUDE_CONFIG_DIR")
    "${CENV[@]}" nohup claude -p --output-format stream-json --verbose \
        --no-session-persistence --disable-slash-commands --dangerously-skip-permissions "${CARGS[@]}" < "$S/PROMPT.md" \
        > "$S/agent_events.jsonl" 2> "$S/agent_stderr.log" &
    ;;
  codex)
    MARGS=(); [ "$MODEL" != "codex-default" ] && MARGS=(-m "$MODEL")
    [ -n "$EFFORT" ] && MARGS+=(-c "model_reasoning_effort=\"$EFFORT\"")
    [ -n "$FAST" ] && MARGS+=(-c 'service_tier="fast"')
    nohup codex exec --skip-git-repo-check --cd "$S" -s danger-full-access --json "${MARGS[@]}" \
        -c 'model_reasoning_summary="detailed"' -o "$S/agent_last_message.txt" - < "$S/PROMPT.md" \
        > "$S/agent_events.jsonl" 2> "$S/agent_stderr.log" &
    ;;
esac
CPID=$!; echo $CPID > "$S/agent.pid"
bash "$FA/run_real_probe.sh" rec-start "$S" | grep -E 'recording|WARN|recorder' || true

# 3. wait (exit, or timeout -> TERM then KILL)
while kill -0 "$CPID" 2>/dev/null; do
  if [ $(( $(date +%s) - T0 )) -ge $(( TIMEOUT_MIN * 60 )) ]; then
    echo "[experiment] TIMEOUT after ${TIMEOUT_MIN} min -> terminating $BACKEND"; echo "AGENT_TIMEOUT=1" >> "$S/session.env"
    kill -TERM "$CPID" 2>/dev/null || true; sleep 10; kill -KILL "$CPID" 2>/dev/null || true
    break
  fi
  sleep 15
done
T1=$(date +%s); { echo "AGENT_END_S=$T1"; echo "CODEX_END_S=$T1"; } >> "$S/session.env"
if [ "$BACKEND" = agy ] || [ "$BACKEND" = claude ]; then
  # the final answer lives in the terminal "result" event of the stream (agy: result.response; claude: result)
  python3 - "$S/agent_events.jsonl" "$S/agent_last_message.txt" <<'PY' || true
import json, sys
src, dst = sys.argv[1], sys.argv[2]
last = ""
for line in open(src, encoding="utf-8", errors="replace"):
    try:
        e = json.loads(line)
    except Exception:
        continue
    if e.get("event") == "result":
        last = str((e.get("result") or {}).get("response", ""))
    elif e.get("type") == "result":
        last = str(e.get("result") or "")
open(dst, "w", encoding="utf-8").write(last)
PY
fi
sleep 3   # recorders capture the final scene

# 4. all three views stop together, then everything down (server -> lease released ->
#    arm limp in motion mode; knowledge merged)
bash "$FA/run_real_probe.sh" rec-stop "$S" | grep -E 'video|WARN' || true
bash "$FA/run_real_probe.sh" stop "$S" | grep -E 'video|merged|NOTE|WARN' || true

# 5. summary
used=$(grep -oE 'used=[0-9]+' "$S/server.log" | tail -1 || echo used=0)
mok=$(grep -cE '#[0-9]+ (move_ee|move_delta|move_joints|home|gripper|run_program) ok=True' "$S/server.log" || true)
mfail=$(grep -cE '#[0-9]+ (move_ee|move_delta|move_joints|home|gripper|run_program) ok=False' "$S/server.log" || true)
cat <<EOF
==================== EXPERIMENT SUMMARY ====================
session   : $S
agent     : $BACKEND / $MODEL
duration  : $(( T1 - T0 )) s   ($(grep -q AGENT_TIMEOUT "$S/session.env" && echo "TIMED OUT" || echo "agent exited on its own"))
commands  : $used ; motions ok=$mok failed=$mfail ; frames=$(ls "$S/frames" | grep -c '_calib.json' || true)
RESULT.md : $([ -f "$S/scratch/RESULT.md" ] && echo present || echo MISSING)
knowledge : $([ -f "$S/scratch/knowledge_delta.md" ] && echo "delta written ($(wc -l < "$S/scratch/knowledge_delta.md") lines)" || echo "no delta")
videos    : $(ls "$S"/run_video_*.mp4 2>/dev/null | xargs -r -n1 basename | tr '\n' ' ')
transcript: python3 $FA/tools/agent_transcript.py "$S"
============================================================
EOF
