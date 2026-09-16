#!/usr/bin/env bash
# Seed / refresh a TASK-scoped knowledge store from previous sessions of that task — offline,
# no robot: a codex session reads the earlier agents' reports, scripts and command logs and
# writes a knowledge delta (RIG_NOTES rows + one playbook + reusable tools), which is then
# merged into the store with the same mechanical merge every trial uses.
#
#   bash consolidate_task_knowledge.sh <task> <batch_out_dir> <session_dir> [<session_dir> ...]
#   e.g. bash consolidate_task_knowledge.sh singleinsert paper_runs/singleinsert_bare_medium_yieldbatch_v2_kn \
#            sessions/20260907_2050*_v2_01 sessions/20260907_2056*_v2_02 sessions/20260907_2102*_v2_03
#
# Store layout (created if missing): <batch_out_dir>/knowledge/{RIG_NOTES.md, playbooks/, tools/}.
# The consolidation workspace is kept under <batch_out_dir>/knowledge/consolidation_<ts>/ (inputs,
# the codex event log, the delta) for the record. Re-running appends: the prompt tells the agent
# not to repeat what the store already holds. Humans never write notes: the delta is agent-authored.
# Env: CONSOLIDATE_MODEL (gpt-6-astra), CONSOLIDATE_EFFORT (high), CONSOLIDATE_TIMEOUT_MIN (20).
set -euo pipefail
FA="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ $# -ge 3 ] || { sed -n 2,16p "$0" | sed 's/^# \{0,1\}//'; exit 2; }
TASK="$1"; OUT=$(readlink -f "$2" 2>/dev/null || echo "$FA/$2"); shift 2
MODEL="${CONSOLIDATE_MODEL:-gpt-6-astra}"; EFFORT="${CONSOLIDATE_EFFORT:-high}"; TMIN="${CONSOLIDATE_TIMEOUT_MIN:-20}"
export PATH="$HOME/.local/bin:$PATH"
command -v codex >/dev/null || { echo "[consolidate] codex CLI not on PATH"; exit 2; }

K="$OUT/knowledge"; mkdir -p "$K/playbooks" "$K/tools"
if [ ! -f "$K/RIG_NOTES.md" ]; then cat > "$K/RIG_NOTES.md" <<'EOF'
# Task notes — facts earlier operators learned doing THIS task on THIS robot

Notes left by previous autonomous sessions (and by an offline consolidation of
their reports). Treat them as evidence, not orders: a note can be wrong or out of
date. If you find one is wrong, add a new row that says so with your own
evidence; do not delete rows.

Each row: fact | applies when | evidence | confidence (low/med/high).

EOF
fi
TS=$(date +%Y%m%d_%H%M%S); W="$K/consolidation_$TS"; mkdir -p "$W/inputs" "$W/scratch/knowledge_delta_tools" "$W/knowledge"
cp "$K/RIG_NOTES.md" "$W/knowledge/"; [ -d "$K/playbooks" ] && cp -r "$K/playbooks" "$W/knowledge/"; [ -d "$K/tools" ] && cp -r "$K/tools" "$W/knowledge/"
n=0
for S in "$@"; do
  S=$(readlink -f "$S"); [ -d "$S" ] || { echo "[consolidate] no such session: $S"; exit 2; }
  d="$W/inputs/$(basename "$S")"; mkdir -p "$d/scratch"
  [ -f "$S/scratch/RESULT.md" ] && cp "$S/scratch/RESULT.md" "$d/RESULT.md"
  cp "$S"/scratch/*.py "$d/scratch/" 2>/dev/null || true
  for f in "$S"/scratch/*.md; do [ "$(basename "$f")" = RESULT.md ] || cp "$f" "$d/scratch/" 2>/dev/null || true; done
  [ -f "$S/server.log" ] && cp "$S/server.log" "$d/server.log"
  [ $n -eq 0 ] && { cp "$S/PROMPT.md" "$W/inputs/TASK_PROMPT.md"; cp "$S/README_interface.md" "$W/inputs/README_interface.md"; }
  n=$((n+1))
done
echo "[consolidate] $n session(s) copied into $W/inputs"
{ echo "KNOWLEDGE_MODE=all"; echo "KNOWLEDGE_SCOPE=task"; echo "TOOLS_DIR=$K/tools"; echo "NOTES_DIR=$K"; echo "PURPOSE=consolidation"; } > "$W/session.env"

cat > "$W/PROMPT.md" <<EOF
# Consolidate task knowledge from previous autonomous sessions (offline — NO robot)

You are preparing notes and helper scripts for future autonomous sessions of a
REAL 6-DOF arm doing ONE specific task: "$TASK". There is no robot attached to
this workspace: do not try to run robot commands. Work only inside this directory.

- \`inputs/<session>/\` — one folder per previous session on this task: the
  agent's final report \`RESULT.md\`, its scripts and notes from \`scratch/\`, and
  the server command log \`server.log\` (every robot command with its outcome).
- \`inputs/TASK_PROMPT.md\` and \`inputs/README_interface.md\` — the task and the
  robot interface those sessions worked with (the next session gets the same).
- \`knowledge/\` — what the task's knowledge store already holds (may be empty).

Write \`scratch/knowledge_delta.md\` for the NEXT session on this same task:

1. Facts, as table rows \`| fact | applies when | evidence | confidence |\`:
   object sizes and heights that were measured, grasp widths and tool
   orientations that worked, insertion offsets, table height, pitfalls and
   what failed. Task-specific numbers are wanted. Give evidence as the input
   session folder plus the numbers, not frame paths. Only what the inputs
   actually show.
2. The procedure that worked, as ONE playbook block \`## playbook: <name>\`:
   step by step with the commands and parameters used, marking which numbers
   depend on where the parts lie (positions) and which do not (sizes, heights,
   widths, orientations).
3. Helper scripts worth reusing: copy the best version of each into
   \`scratch/knowledge_delta_tools/<name>.py\`, deduplicated across sessions,
   parametrised so that scene positions are arguments rather than constants,
   with this header at the top (plain comment lines, all seven keys, one line each):
       # tool: <name>.py
       # category: process | geometry | task
       # purpose: <what it does>
       # usage: python3 knowledge/tools/<name>.py <args>    (run from the session directory)
       # inputs/outputs: <what it reads; what it prints or writes>
       # assumptions: <hard-coded values, formats, sizes, tolerances — or "none">
       # verified: used successfully in the session that wrote it
   No session names, dates, frame numbers or absolute paths inside a tool. Then
   add to \`scratch/knowledge_delta.md\` one section \`## tool: <name>.py\` per
   script with one sentence on what it is for; if you keep none, add the line
   \`## tools: none\`.

Do not invent anything the inputs do not support. Do not repeat what
\`knowledge/\` already contains. Keep the whole delta under about 120 lines.
EOF

echo "[consolidate] launching codex ($MODEL, effort $EFFORT, offline, timeout ${TMIN} min) in $W"
T0=$(date +%s)
timeout "$((TMIN*60))" codex exec --skip-git-repo-check --cd "$W" -s danger-full-access --json -m "$MODEL" \
    -c "model_reasoning_effort=\"$EFFORT\"" -c 'model_reasoning_summary="detailed"' \
    -o "$W/agent_last_message.txt" - < "$W/PROMPT.md" > "$W/agent_events.jsonl" 2> "$W/agent_stderr.log" || echo "[consolidate] codex exited with $? (timeout or error) — merging whatever was written"
echo "[consolidate] codex finished in $(( $(date +%s) - T0 )) s"
[ -s "$W/scratch/knowledge_delta.md" ] || { echo "[consolidate] no scratch/knowledge_delta.md written; see $W/agent_stderr.log"; exit 3; }
echo "[consolidate] delta: $(wc -l < "$W/scratch/knowledge_delta.md") lines, tools: $(ls "$W/scratch/knowledge_delta_tools" 2>/dev/null | tr '\n' ' ')"
bash "$FA/run_real_probe.sh" merge "$W"
echo "[consolidate] store: $K"; echo "  RIG_NOTES.md: $(wc -l < "$K/RIG_NOTES.md") lines; playbooks: $(ls "$K/playbooks" 2>/dev/null | tr '\n' ' '); tools: $(ls "$K/tools" 2>/dev/null | grep -v TOOLS.md | tr '\n' ' ')"
echo "[consolidate] next: bash run_trial.sh $TASK <n> ... --knowledge task"
