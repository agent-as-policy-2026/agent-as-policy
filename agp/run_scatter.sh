#!/usr/bin/env bash
# Automatic scene reset between paper trials: a codex session dismantles the built
# structure and lays the six cubes out at a randomly sampled, well-separated layout;
# then the overhead camera is checked before the next trial may start.
#
#   bash run_scatter.sh <trial_no>        # reset AFTER trial <trial_no> of batch PAPER_BATCH
#
# Conditions: no notes, no tools, same codex model/effort as the trials (SCATTER_FAST=1
# for the fast tier; default = codex default tier), SCATTER_TIMEOUT_MIN (default 45)
# minute limit, session name scatter_<batch>_NN (never counted as a trial).
# Recording (SCATTER_VIDEO=1, default): exactly like a trial — the session starts with
# --no-video and `rec-start` then brings up the side camera via ffmpeg AND the bridge
# recorder (top + wrist videos + joints.csv at 50 Hz + per-frame timestamps) together;
# `rec-stop` ends both. SCATTER_VIDEO=0 records nothing.
# Artifacts in paper_runs/<task>_<batch>/:
#   scatter_NN_goal/        target_layout.{json,md} (seeded) + scatter_example.png
#   scatter_NN_after.png    overhead photo after the reset
#   scatter_NN_check.png    the check's annotated image
# Exit 0 = scene accepted by tools/check_scatter.py (3 cyan + 3 grey cubes, apart, in
# region); 1 = rejected (fix by hand); 2 = could not run.
set -euo pipefail

FA="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$FA")"
# the bridge venv python: it has numpy/opencv (and imports the vendored connector)
PY="${AGP_PYTHON:-$REPO/hardware-bridge/.venv/bin/python}"
PORT="${BRIDGE_PORT:-9021}"
TASK="${PAPER_TASK:-pyramid}"
BATCH="${PAPER_BATCH:-$([ "${PAPER_BARE:-0}" = 1 ] && echo bare1 || echo tools1)}"   # PAPER_BARE=1 -> same batch name as the trials; the reset itself always uses the full interface
MODEL="${PAPER_MODEL:-gpt-6-astra}"
EFFORT="${PAPER_EFFORT:-high}"
FAST="${SCATTER_FAST:-}"            # empty (default) = codex's default service tier; set to 1 for service_tier="fast"
VIDEO="${SCATTER_VIDEO:-1}"         # 1 (default) = side video + bridge top/wrist videos + joints.csv; 0 = no recording
TIMEOUT_MIN="${SCATTER_TIMEOUT_MIN:-45}"

[ $# -eq 1 ] && [[ "$1" =~ ^[0-9]+$ ]] || { sed -n 2,21p "$0" | sed 's/^# \{0,1\}//'; exit 2; }
N=$(printf '%02d' "$1"); NAME="scatter_${BATCH}_${N}"
OUT="$FA/paper_runs/${TASK}_${BATCH}"; G="$OUT/scatter_${N}_goal"; mkdir -p "$OUT"
export PATH="$HOME/.local/bin:$PATH"

# ---- preflight ------------------------------------------------------------------------
ss -tln 2>/dev/null | grep -q ":$PORT " || { echo "[scatter] left bridge is not listening on $PORT"; exit 2; }
if pgrep -af 'server_real|run_experiment' | grep -v -E "pgrep" >/dev/null; then
  echo "[scatter] another agp session is still running:"; pgrep -af 'server_real|run_experiment' | grep -v -E "pgrep"; exit 2
fi
command -v codex >/dev/null || { echo "[scatter] codex CLI not on PATH"; exit 2; }

# ---- seeded target layout ----------------------------------------------------------------
# seed = hash(task + batch) + trial number: reproducible per batch, different per trial and task
BSEED=$(printf '%s/%s' "$TASK" "$BATCH" | cksum | cut -d' ' -f1); SEED=$(( (BSEED % 100000) * 100 + 10#$N ))
rm -rf "$G"; mkdir -p "$G"
python3 "$FA/tools/scatter_layout.py" --out "$G" --seed "$SEED" >/dev/null
cp "$FA/goal_sets/scatter_example/scatter_example.png" "$G/"
echo "[scatter] after trial $N: target layout seed $SEED -> $G/target_layout.md"

# ---- session: no notes, no tools; recorders start with the agent (rec-start), never here ----
FA_KNOWLEDGE=none FA_YES=1 bash "$FA/run_real_probe.sh" start "$NAME" --allow-motion --no-video \
    --prompt "$FA/PROMPT_scatter.md" --goal "$G" | grep -E 'READY|BOOT_ERROR|abort|ABORT|die' || true
S=$(readlink -f "$FA/sessions/LATEST")
case "$S" in *"_$NAME") ;; *) echo "[scatter] session did not come up"; exit 2 ;; esac
[ -f "$S/server.pid" ] && kill -0 "$(cat "$S/server.pid")" 2>/dev/null || { echo "[scatter] server not running"; exit 2; }

cd "$S"
T0=$(date +%s)
TIER=$([ -n "$FAST" ] && echo fast || echo default)
TIERARGS=(); [ -n "$FAST" ] && TIERARGS=(-c 'service_tier="fast"')
{ echo "AGENT_BACKEND=codex"; echo "AGENT_MODEL=$MODEL"; echo "AGENT_EFFORT=$EFFORT"; echo "AGENT_SERVICE_TIER=$TIER"; echo "AGENT_START_S=$T0"; echo "PURPOSE=scene-reset"; } >> "$S/session.env"
echo "[scatter] launching codex ($MODEL, effort $EFFORT, tier $TIER) in $S (timeout ${TIMEOUT_MIN} min, recording $([ "$VIDEO" = 1 ] && echo 'side + bridge videos + joints' || echo off))"
nohup codex exec --skip-git-repo-check --cd "$S" -s danger-full-access --json -m "$MODEL" \
    -c "model_reasoning_effort=\"$EFFORT\"" "${TIERARGS[@]}" -c 'model_reasoning_summary="detailed"' \
    -o "$S/agent_last_message.txt" - < "$S/PROMPT.md" > "$S/agent_events.jsonl" 2> "$S/agent_stderr.log" &
CPID=$!; echo $CPID > "$S/agent.pid"
if [ "$VIDEO" = 1 ]; then bash "$FA/run_real_probe.sh" rec-start "$S" | grep -E 'recording|WARN|ABORT' || true; fi
while kill -0 "$CPID" 2>/dev/null; do
  if [ $(( $(date +%s) - T0 )) -ge $(( TIMEOUT_MIN * 60 )) ]; then
    echo "[scatter] TIMEOUT after ${TIMEOUT_MIN} min -> terminating codex"; echo "AGENT_TIMEOUT=1" >> "$S/session.env"
    kill -TERM "$CPID" 2>/dev/null || true; sleep 10; kill -KILL "$CPID" 2>/dev/null || true
    break
  fi
  sleep 10
done
T1=$(date +%s); echo "AGENT_END_S=$T1" >> "$S/session.env"
sleep 3
if [ "$VIDEO" = 1 ]; then bash "$FA/run_real_probe.sh" rec-stop "$S" | grep -E 'video|joints|WARN' || true; fi
bash "$FA/run_real_probe.sh" stop "$S" | grep -E 'NOTE|WARN' || true

# ---- check the scene ---------------------------------------------------------------------------
sleep 3
bash "$FA/capture_top.sh" "$OUT/scatter_${N}_after.png" >/dev/null
set +e
"$PY" "$FA/tools/check_scatter.py" --image "$OUT/scatter_${N}_after.png" --debug "$OUT/scatter_${N}_check.png"
rc=$?
set -e
echo "[scatter] reset took $(( T1 - T0 )) s; $([ -f "$S/scratch/SCATTER_DONE.md" ] && echo "SCATTER_DONE.md written" || echo "no SCATTER_DONE.md"); check exit $rc ($([ $rc = 0 ] && echo ACCEPTED || echo REJECTED)); see $OUT/scatter_${N}_check.png"
exit $rc
