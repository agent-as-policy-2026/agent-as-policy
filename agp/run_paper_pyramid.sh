#!/usr/bin/env bash
# PAPER PROTOCOL — pyramid task, ONE trial per invocation (run it 10 times):
#
#   bash run_paper_pyramid.sh <trial_no>          e.g.  bash run_paper_pyramid.sh 1
#
# Fixed conditions (identical for every trial of a batch):
#   * NO prior notes (RIG_NOTES / playbooks are never copied or merged)
#   * TOOLS are inherited within the batch: trial 1 starts with an empty tools store
#     (paper_runs/pyramid_<batch>/tools/); each agent may leave helper scripts it wrote,
#     used successfully and judged general (see the "Leaving tools" section the launcher
#     appends to PROMPT.md); they are linted and passed on to the next trial.
#   * backend codex, model gpt-6-astra, reasoning effort high, service tier fast
#   * prompt PROMPT_stack_blocks.md (location and orientation free, neatness criterion),
#     goal set goal_sets/20260903_pyramid_plain (phone photo + overhead capture)
#   * motion enabled, PAPER_TIMEOUT_MIN (default 90) minute limit, three-view recording
# Batch: PAPER_BATCH (default "tools1"). Session name: paper_pyramid_<batch>_NN.
# Artifacts in paper_runs/pyramid_<batch>/:
#   trial_NN_before.png / trial_NN_after.png   overhead photos taken by this script
#   trial_NN_final_agent_frame.png             the agent's last overhead frame
#   tools/                                     the batch's tools store (+ TOOLS.md index)
#   results.csv                                one row per trial (operator_verdict left blank
#                                              for you to fill: success / fail)
# Between trials: scatter the cubes, park the arm, keep the e-stop in hand. The script
# takes the "before" photo, then waits for Enter before anything moves.
set -euo pipefail

FA="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Task selection (env; defaults = the pyramid protocol). Another block task = another goal
# set with the same generic prompt: PAPER_TASK=twopiles PAPER_GOAL=goal_sets/20260904_twopiles
TASK="${PAPER_TASK:-pyramid}"
# PAPER_BARE=1 (2026-09-07 ablation): server --bare (no deproject/home/move_delta, no advice texts),
# terse README_interface_bare.md, NO inherited tools, default batch "bare1" and the *_bare prompt.
# Default 0 = exactly the protocol used by every batch so far.
BARE="${PAPER_BARE:-0}"
# PAPER_HARNESS (2026-09-07 efficiency experiments): none (default) | yield | yield,batch -> appends the
# matching "tool harness" notes to the session's README_interface.md (see run_real_probe.sh).
HARNESS="${PAPER_HARNESS:-none}"
# PAPER_KNOWLEDGE=task (2026-09-07): a TASK-scoped knowledge store paper_runs/<task>_<batch>/knowledge/
# (RIG_NOTES.md + playbooks/ + tools/), copied into every trial and appended after each trial; the
# prompt sections allow task-specific numbers. Seed it with consolidate_task_knowledge.sh.
# Default (unset) = the classic tools-only store (paper_runs/<task>_<batch>/tools), or none when bare.
KNOW="${PAPER_KNOWLEDGE:-}"
# PAPER_PROGRAMS=1 (2026-09-14): the session offers buffered joint programs (server --programs; the bridge must run
# config/left_arm_throw.yaml). Default 1 for the throwing task only, unset for everything else.
TASK_DEFAULT_PROGRAMS=""; [ "$TASK" = throw ] && TASK_DEFAULT_PROGRAMS=1
PROGRAMS="${PAPER_PROGRAMS:-$TASK_DEFAULT_PROGRAMS}"
# PAPER_ARMS (2026-09-08): left (default = every batch so far) | left,right -> the session also runs the
# right arm's server (run_real_probe.sh --arms; right bridge on BRIDGE_PORT_RIGHT, default 9022, preflighted
# here as well); results.csv gets the column 'arms'. run_trial.sh --dual sets it.
ARMS="${PAPER_ARMS:-left}"
BATCH="${PAPER_BATCH:-$([ "$BARE" = 1 ] && echo bare1 || echo tools1)}"
PROMPT="${PAPER_PROMPT:-$([ "$BARE" = 1 ] && echo PROMPT_stack_blocks_bare.md || echo PROMPT_stack_blocks.md)}"
GOAL="${PAPER_GOAL:-goal_sets/20260903_pyramid_plain}"
MODEL="${PAPER_MODEL:-gpt-6-astra}"
BACKEND="${PAPER_BACKEND:-codex}"           # codex | claude | agy (2026-09-10 model comparison; run_experiment.sh --backend)
EFFORT="${PAPER_EFFORT:-high}"
FAST="${PAPER_FAST:-1}"             # 1 (default, as in the pyramid batch) = service_tier fast; 0 = codex default tier
TIMEOUT_MIN="${PAPER_TIMEOUT_MIN:-90}"

[ $# -eq 1 ] && [[ "$1" =~ ^[0-9]+$ ]] || { sed -n 2,25p "$0" | sed 's/^# \{0,1\}//'; exit 2; }
N=$(printf '%02d' "$1"); NAME="${PAPER_PREFIX:-paper}_${TASK}_${BATCH}_${N}"   # PAPER_PREFIX=scatter (2026-09-10): scene-reset runs, never counted as trials
OUT="$FA/paper_runs/${TASK}_${BATCH}"; TOOLS="$OUT/tools"; NOTES=""; KSCOPE=rig
KMODE=tools; BAREARG=(); KRO=""; EXPSTORE=""; MXSTORE=""; MXRO=""
if [ "$BARE" = 1 ]; then
  KMODE=none; BAREARG=(-- --bare)
  case "$PROMPT" in *_bare.md) ;; *) echo "[paper] WARNING: PAPER_BARE=1 but the prompt '$PROMPT' is not a *_bare prompt" ;; esac
fi
case "$KNOW" in
  "") ;;
  task) KMODE=all; KSCOPE=task; NOTES="$OUT/knowledge"; TOOLS="$OUT/knowledge/tools" ;;
  # task-ro (2026-09-10): the same store, READ ONLY — no knowledge_delta is asked for and nothing is
  # merged back, so every trial of the batch sees exactly the store it was seeded with
  task-ro) KMODE=all; KSCOPE=task; NOTES="$OUT/knowledge"; TOOLS="$OUT/knowledge/tools"; KRO=1 ;;
  # ckpt (2026-09-10): immutable rolling experience checkpoints, strictly the multiple-experience method's scheme
  # (four-field lesson + mandatory snapshotted artifacts + latest-only handoff); no notes/tools store
  ckpt) KMODE=none; KSCOPE=task; EXPSTORE="$OUT/experience" ;;
  # ckpt-ro (2026-09-10): the same checkpoint store READ ONLY — every trial reads the checkpoints it
  # was seeded with, no save instructions are injected and nothing is committed back
  ckpt-ro) KMODE=none; KSCOPE=task; EXPSTORE="$OUT/experience"; KRO=1 ;;
  # mx (2026-09-11): the multiple-experience memory, identical in what the model writes and
  # receives, each cycle in its own session (agp/mx/: carried checkpoints + shared scratch + stage
  # records, server-side episode / experience commands, its memory instructions)
  mx) KMODE=none; KSCOPE=task; MXSTORE="$OUT/mx" ;;
  # mx-ro (2026-09-11): a finished mx store READ ONLY (seeded once from --knowledge-from); every trial gets
  # all its checkpoints + shared scratch, its own assembly is recorded as cycle 1, nothing is saved back
  mx-ro) KMODE=none; KSCOPE=task; MXSTORE="$OUT/mx"; MXRO=1 ;;
  *) echo "[paper] PAPER_KNOWLEDGE must be empty, 'task', 'task-ro', 'ckpt', 'ckpt-ro', 'mx' or 'mx-ro' (got '$KNOW')"; exit 2 ;;
esac
ARMSARG=(); ARMSNOTE=""; PORT_LEFT="${BRIDGE_PORT:-9021}"; PORT_RIGHT="${BRIDGE_PORT_RIGHT:-9022}"; SOLO=""
case "$ARMS" in
  left) ;;
  left,right|right,left) ARMS=left,right; ARMSARG=(--arms "$ARMS"); ARMSNOTE=" arms=$ARMS" ;;
  right) ARMSARG=(--arms right); ARMSNOTE=" arms=right"; SOLO=1 ;;   # standalone right-arm trial (2026-09-09)
  *) echo "[paper] PAPER_ARMS must be left, right or left,right (got '$ARMS')"; exit 2 ;;
esac
if [ "${PAPER_DRYRUN:-0}" = 1 ]; then   # print the resolved configuration and touch nothing
  echo "[paper] DRYRUN task=$TASK batch=$BATCH session=$NAME prompt=$PROMPT goal=$GOAL backend=$BACKEND model=$MODEL effort=$EFFORT fast=$FAST bare=$BARE knowledge=$KMODE/$KSCOPE notes=${NOTES:-none} tools=$TOOLS harness=$HARNESS timeout=${TIMEOUT_MIN}min out=$OUT$ARMSNOTE${PROGRAMS:+ programs=1}"
  exit 0
fi
mkdir -p "$OUT" "$TOOLS"
need_seed=0
[ -n "$NOTES" ] && [ ! -f "$NOTES/RIG_NOTES.md" ] && need_seed=1
[ -n "$EXPSTORE" ] && ! ls -d "$EXPSTORE"/cycle_[0-9][0-9] >/dev/null 2>&1 && need_seed=1
if [ "$need_seed" = 1 ] && [ -n "${PAPER_KNOWLEDGE_FROM:-}" ]; then
  # 2026-09-10 knowledge transfer: a fresh batch starts from ANOTHER batch's store (e.g. terra inherits
  # what astra learned); copied once, provenance in knowledge/SEEDED_FROM.txt; later trials append as usual
  bash "$FA/tools/seed_knowledge.sh" "$PAPER_KNOWLEDGE_FROM" "${TASK}_${BATCH}" || exit 2
fi
if [ -n "$EXPSTORE" ]; then
  # mirrors the multiple-experience method's verify_handoff: cycle N may not start before checkpoints 1..N-1 exist
  mkdir -p "$EXPSTORE"
  have=$(find "$EXPSTORE" -maxdepth 1 -type d -name 'cycle_[0-9][0-9]' | wc -l)
  if [ "$KRO" = 1 ]; then
    # read-only: no handoff sequence to keep, but an empty store would silently mean "no knowledge"
    if [ "$have" -eq 0 ]; then
      echo "[paper] PAPER_KNOWLEDGE=ckpt-ro but there are no checkpoints in $EXPSTORE: seed it first (--knowledge-from <batch>)"; exit 2
    fi
    echo "[paper] experience store $EXPSTORE: READ-ONLY, $have checkpoint(s) for every trial of this batch"
  else
    want=$(( 10#$N - 1 ))
    if [ "$have" -ne "$want" ]; then
      echo "[paper] experience handoff: cycle $N needs $want committed checkpoint(s), found $have in $EXPSTORE"; exit 2
    fi
    echo "[paper] experience store $EXPSTORE: $have checkpoint(s) available to cycle $N"
  fi
fi
if [ -n "$MXRO" ]; then
  # read-only reuse: seed the batch's store once from the finished source batch, then every trial reads it
  if [ -z "$(find "$MXSTORE/experience" -maxdepth 1 -type d -name 'cycle_[0-9][0-9]' 2>/dev/null)" ] && [ -n "${PAPER_KNOWLEDGE_FROM:-}" ]; then
    python3 "$FA/mx/carry.py" seed "$FA/paper_runs/$PAPER_KNOWLEDGE_FROM/mx" "$MXSTORE" || exit 2
  fi
  python3 "$FA/mx/carry.py" check-ro "$MXSTORE" | sed 's/^/[paper] mx /' || true
  python3 "$FA/mx/carry.py" check-ro "$MXSTORE" >/dev/null || exit 2
elif [ -n "$MXSTORE" ]; then
  # the multiple-experience method's verify_handoff on the carried state: cycle N needs checkpoints 1..N-1 and N-1 completed stages
  mkdir -p "$MXSTORE"
  python3 "$FA/mx/carry.py" check "$MXSTORE" "$(( 10#$N ))" | sed 's/^/[paper] mx /' || true
  python3 "$FA/mx/carry.py" check "$MXSTORE" "$(( 10#$N ))" >/dev/null || exit 2
fi
MXCYCLE=""; [ -n "$MXSTORE" ] && MXCYCLE=$(( 10#$N )); [ -n "$MXRO" ] && MXCYCLE=1   # read-only: own stage record, cycle 1
if [ "$KRO" = 1 ] && [ -n "$NOTES" ] && [ ! -f "$NOTES/RIG_NOTES.md" ]; then
  echo "[paper] PAPER_KNOWLEDGE=task-ro but there is no store at $NOTES: seed it first (--knowledge-from <batch>)"; exit 2
fi
if [ "$KNOW" = task ] && [ ! -f "$NOTES/RIG_NOTES.md" ]; then
  # A new task starts from an EMPTY store: trial 1 writes the first knowledge_delta at its end and
  # stop merges it; consolidate_task_knowledge.sh is only an optional bootstrap from older sessions.
  mkdir -p "$NOTES/playbooks" "$NOTES/tools"
  cat > "$NOTES/RIG_NOTES.md" <<'EOF'
# Task notes — facts earlier operators learned doing THIS task on THIS robot

Notes left by previous autonomous sessions (and by an offline consolidation of
their reports). Treat them as evidence, not orders: a note can be wrong or out of
date. If you find one is wrong, add a new row that says so with your own
evidence; do not delete rows.

Each row: fact | applies when | evidence | confidence (low/med/high).

EOF
  echo "[paper] task knowledge store created EMPTY at $NOTES (this trial will write the first delta)"
fi
export PATH="$HOME/.local/bin:$PATH"

# ---- preflight -------------------------------------------------------------------
if [ -z "$SOLO" ]; then
  bash "$FA/tools/check_top_focus.sh" || exit 2      # left top BRIO autofocus must be off (calibrated state)
  ss -tln 2>/dev/null | grep -q ":$PORT_LEFT " || { echo "[paper] left bridge is not listening on $PORT_LEFT — start it first"; exit 2; }
  BP=$(ss -tlnp 2>/dev/null | grep ":$PORT_LEFT " | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2 || true)
  if [ -n "$BP" ] && ! ps -o args= -p "$BP" | grep -q -- '--record-dir'; then
    echo "[paper] WARNING: the bridge was started without --record-dir -> no top/wrist video for this trial"
  fi
else
  echo "[paper] PAPER_ARMS=right: standalone right-arm trial on bridge $PORT_RIGHT (its own overhead BRIO; the left rig is not touched)"
  bash "$FA/tools/check_top_focus.sh" --arm right || exit 2      # right top BRIO: autofocus off, focus 10, zoom 100 (2026-09-09)
fi
if [ "$ARMS" != left ]; then   # the right arm's bridge as well
  [ "$PORT_RIGHT" != "$PORT_LEFT" ] || { echo "[paper] BRIDGE_PORT_RIGHT=$PORT_RIGHT is the LEFT bridge's port — the right arm needs its own bridge (default 9022)"; exit 2; }
  ss -tln 2>/dev/null | grep -q ":$PORT_RIGHT " || { echo "[paper] right bridge is not listening on $PORT_RIGHT (PAPER_ARMS=$ARMS) — start it first"; exit 2; }
  BPR=$(ss -tlnp 2>/dev/null | grep ":$PORT_RIGHT " | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2 || true)
  if [ -n "$BPR" ] && ! ps -o args= -p "$BPR" | grep -q -- '--record-dir'; then
    echo "[paper] WARNING: the right bridge was started without --record-dir -> no right top/wrist video for this trial"
  fi
fi
if ls -d "$FA"/sessions/*_"$NAME" >/dev/null 2>&1; then
  echo "[paper] trial $N already has a session: $(ls -d "$FA"/sessions/*_"$NAME")"; echo "        pick the next number, or delete that session first"; exit 2
fi
# our own sessions only. 2026-09-09: a session on the OTHER arm's bridge is not a conflict (left and
# standalone-right trials run concurrently): servers are matched by their --port, run_experiment launchers by --arms;
# a dual (--arms left,right) session conflicts with both.
MYPORT="$PORT_LEFT"; [ -n "$SOLO" ] && MYPORT="$PORT_RIGHT"
busy=$(pgrep -af 'server_real|run_experiment' | grep -v -E "^[0-9]+ (awk|grep|pgrep) " | awk -v p="$MYPORT" -v solo="${SOLO:-0}" '
  /server_real/    { if (match($0, /--port [0-9]+/)) { split(substr($0, RSTART, RLENGTH), a, " "); if (a[2] != p) next } print; next }
  /run_experiment/ { if ($0 ~ /--arms left,right/) { print; next }
                     if ($0 ~ /--arms right/) { if (solo == "1") print; next }
                     if (solo != "1") print }' || true)
if [ -n "$busy" ]; then
  echo "[paper] another agp session is still running on this arm's bridge:"; echo "$busy"; exit 2
fi
command -v "$BACKEND" >/dev/null || { echo "[paper] $BACKEND CLI not on PATH"; exit 2; }
ntools_before=$(find "$TOOLS" -maxdepth 1 -type f \( -name '*.py' -o -name '*.sh' \) | wc -l)

FA_CAPTURE_ARM="${SOLO:+right}" bash "$FA/capture_top.sh" "$OUT/trial_${N}_before.png" >/dev/null
echo "[paper] trial $N (batch $BATCH): before-photo saved -> $OUT/trial_${N}_before.png ; tools inherited: $ntools_before"
if [ -t 0 ]; then
  read -r -p "[paper] cubes scattered, arm parked, e-stop in hand?  Enter = start trial $N, Ctrl-C = abort: " _
fi

# ---- the trial -----------------------------------------------------------------------
T_START=$(date '+%Y-%m-%d %H:%M:%S')
FASTARG=(); [ "$FAST" = 1 ] && FASTARG=(--fast)
FA_KNOWLEDGE="$KMODE" FA_KNOWLEDGE_SCOPE="$KSCOPE" FA_KNOWLEDGE_READONLY="$KRO" FA_EXPERIENCE_STORE="$EXPSTORE" FA_EXPERIENCE_CYCLE="$N" FA_MX_STORE="$MXSTORE" FA_MX_CYCLE="$MXCYCLE" FA_MX_READONLY="$MXRO" FA_PROGRAMS="$PROGRAMS" FA_NOTES_DIR="$NOTES" FA_TOOLS_DIR="$TOOLS" FA_HARNESS="$HARNESS" FA_YES=1 bash "$FA/run_experiment.sh" "$NAME" --allow-motion --prompt "$PROMPT" --goal "$GOAL" \
  --timeout-min "$TIMEOUT_MIN" --backend "$BACKEND" --model "$MODEL" --effort "$EFFORT" "${FASTARG[@]}" "${ARMSARG[@]}" "${BAREARG[@]}"
S=$(ls -d "$FA"/sessions/*_"$NAME" 2>/dev/null | sort | tail -1 || true)   # by NAME (LATEST is ambiguous with two concurrent trials, 2026-09-09)
[ -n "$S" ] && [ -d "$S" ] || { echo "[paper] no session dir for $NAME"; exit 3; }
ntools_after=$(find "$TOOLS" -maxdepth 1 -type f \( -name '*.py' -o -name '*.sh' \) | wc -l)

# ---- artifacts + results row -----------------------------------------------------------
last_top=$(ls "$S"/frames/*_top.png 2>/dev/null | tail -1 || true)
[ -n "$last_top" ] && cp "$last_top" "$OUT/trial_${N}_final_agent_frame.png"
sleep 2
FA_CAPTURE_ARM="${SOLO:+right}" bash "$FA/capture_top.sh" "$OUT/trial_${N}_after.png" >/dev/null 2>&1 || echo "[paper] after-photo failed (bridge busy?)"
python3 - "$S" "$OUT/results.csv" "$N" "$MODEL" "$EFFORT" "$T_START" "$ntools_before" "$ntools_after" "$BARE" "$HARNESS" "$ARMS" <<'PY'
import csv, os, re, sys
S, csv_path, n, model, effort, t_start, t_before, t_after, bare, harness, arms = sys.argv[1:12]
env = dict(l.strip().split("=", 1) for l in open(os.path.join(S, "session.env")) if "=" in l)
log = open(os.path.join(S, "server.log"), encoding="utf-8", errors="replace").read()
last_used = re.findall(r"used=(\d+)", log)
def count(pat): return len(re.findall(pat, log))
verdict = "no RESULT.md"
rp = os.path.join(S, "scratch", "RESULT.md")
if env.get("MX_STORE") and env.get("MX_CYCLE") and int(env["MX_CYCLE"]) < 5 and env.get("MX_READONLY") != "1":
    rp = os.path.join(S, "scratch", f"cycle_{int(env['MX_CYCLE']):02d}_RESULT.md")   # mx cycles 1-4 write their round report here (the mx protocol)
# mx sessions start with the carried scratch, which already holds earlier rounds' reports: only a report
# written by THIS session's agent counts
if env.get("MX_STORE") and os.path.exists(rp) and os.path.getmtime(rp) < float(env.get("AGENT_START_S", 0)):
    rp = rp + ".not_written_by_this_session"
if os.path.exists(rp):
    txt = open(rp, encoding="utf-8", errors="replace").read()
    # 2026-09-10: the concluding judgement is often mid-paragraph ("I judge both assemblies
    # successful."), which the line-anchored rules below miss. A bare relaxation is NOT safe:
    # these reports mention "unsuccessful" / "no motion failures" incidentally, so only an
    # explicit judgement sentence is matched anywhere; the old line-anchored rules remain the
    # fallback and their results are unchanged.
    # 2026-09-10b: also the noun form ("My judgement is success"), which the verb-only
    # pattern missed on the re-run of cycle 3.
    judge = (r"(?i)\b(?:(?:i|we)\s+(?:judge|assess|conclude|consider|deem)"
             r"|(?:my|our)\s+(?:honest\s+)?(?:judge?ment|assessment|conclusion|verdict))\b[^.\n]{0,160}")
    if re.search(judge + r"\b(?:unsuccessful|not\s+success|fail(?:ed|ure)?)", txt): verdict = "fail (agent)"
    elif re.search(judge + r"\bsuccess", txt): verdict = "success (agent)"
    elif re.search(r"(?im)^\W*(result\W*)?(unsuccessful|incomplete|not (fully )?(built|complete)|fail(ed|ure))", txt): verdict = "fail (agent)"
    elif re.search(r"(?im)^\W*(result\W*)?(success|.*\b(completed|built)\b)", txt): verdict = "success (agent)"
    else: verdict = "unclear (agent)"
start, end = int(env.get("AGENT_START_S", 0)), int(env.get("AGENT_END_S", 0))
# token usage, normalised across backends by tools/agent_usage.py (codex turn.completed; claude / agy
# result event); absent if the run was killed by the timeout (codex: ~/.codex/sessions/.../rollout-*.jsonl)
import json, subprocess
try:
    usage = json.loads(subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.dirname(S)), "tools", "agent_usage.py"), S],
                                      capture_output=True, text=True, check=True).stdout)
except Exception:
    usage = {}
row = {
    "trial": n, "session": os.path.basename(S), "model": model, "backend": env.get("AGENT_BACKEND", ""), "effort": effort,
    "service_tier": env.get("AGENT_SERVICE_TIER", ""), "interface": ("bare" if bare == "1" else "full"), "arms": arms, "harness": harness,
    "harness_version": env.get("HARNESS_VERSION", ""), "notes_used": ("mx-ro" if env.get("MX_READONLY") == "1" else "mx" if env.get("MX_STORE") else "ckpt" if env.get("EXPERIENCE_STORE") else "task-ro" if env.get("KNOWLEDGE_READONLY") == "1" else "task" if env.get("KNOWLEDGE_SCOPE") == "task" else "none"),
    "tools_inherited": t_before, "tools_left": int(t_after) - int(t_before),
    "start": t_start, "duration_s": end - start if start and end else "",
    "timed_out": "yes" if env.get("AGENT_TIMEOUT") else "no",
    "cmds_counted": last_used[-1] if last_used else "0",
    "moves_ok": count(r"#\d+ (move_ee|move_delta|move_joints|home|run_program) ok=True"),
    "gripper_cmds": count(r"#\d+ gripper ok=True"),
    "failed_cmds": count(r"ok=False"),
    "frames": count(r"#\d+ frames ok=True"),
    "tokens_input": usage.get("input_tokens", ""), "tokens_cached_input": usage.get("cached_input_tokens", ""),
    "tokens_output": usage.get("output_tokens", ""), "tokens_reasoning": usage.get("reasoning_output_tokens", ""),
    "cost_usd": usage.get("cost_usd", ""),
    "agent_verdict": verdict, "operator_verdict": "", "remarks": "",
}
new = not os.path.exists(csv_path)
if new:
    fields = list(row)
else:  # keep appending to a batch CSV written before a column existed: its header stays authoritative
    with open(csv_path, newline="", encoding="utf-8") as f:
        fields = next(csv.reader(f))
with open(csv_path, "a", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
    if new: w.writeheader()
    w.writerow(row)
print("[paper] results row:", {k: row[k] for k in ("trial", "duration_s", "timed_out", "cmds_counted", "gripper_cmds", "failed_cmds", "tools_inherited", "tools_left", "agent_verdict")})
PY
echo "[paper] trial $N finished. Fill operator_verdict in $OUT/results.csv (success / fail)."
echo "[paper] tools store: $TOOLS ($ntools_after file(s); see TOOLS.md) ; artifacts: $OUT/trial_${N}_{before,after,final_agent_frame}.png ; session $S"
