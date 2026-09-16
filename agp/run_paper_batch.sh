#!/usr/bin/env bash
# Unattended paper batch: build trial -> robot resets the scene -> check -> next trial.
#
#   bash run_paper_batch.sh <from> <to>        e.g.  bash run_paper_batch.sh 7 10
#
# Per trial n: run_paper_pyramid.sh n (the usual trial, all recordings), then — unless n
# is the last — run_scatter.sh n (codex dismantles the structure and lays the cubes out
# at a seeded random layout, no video) followed by the overhead check. A rejected check
# (wrong cube count, cubes touching, cube outside the region) or a reset that could not
# run STOPS the batch and waits for a human: fix the scene by hand, press Enter to go on
# (Ctrl-C to abort). The human confirms the scene once at the very start; after that the
# only human duty is the emergency stop.
# Env: PAPER_BATCH, PAPER_TIMEOUT_MIN, SCATTER_TIMEOUT_MIN as in the two scripts.
set -euo pipefail

FA="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$FA")"
# the bridge venv python: it has numpy/opencv (and imports the vendored connector)
PY="${AGP_PYTHON:-$REPO/hardware-bridge/.venv/bin/python}"
[ $# -eq 2 ] && [[ "$1" =~ ^[0-9]+$ ]] && [[ "$2" =~ ^[0-9]+$ ]] && [ "$1" -le "$2" ] || { sed -n 2,13p "$0" | sed 's/^# \{0,1\}//'; exit 2; }
FROM=$1; TO=$2
TASK="${PAPER_TASK:-pyramid}"; BATCH="${PAPER_BATCH:-$([ "${PAPER_BARE:-0}" = 1 ] && echo bare1 || echo tools1)}"; OUT="$FA/paper_runs/${TASK}_${BATCH}"; mkdir -p "$OUT"   # PAPER_BARE=1 -> batch bare1 (see run_paper_pyramid.sh)
SCATTER_FIRST="${BATCH_SCATTER_FIRST:-0}"   # 1 = the scene currently holds the built goal: robot resets it before trial FROM
SCATTER_LAST="${BATCH_SCATTER_LAST:-0}"     # 1 = also reset after the last trial (leaves a scattered table)

wait_for_human() {   # $1 = message
  echo "[batch] $1"
  if [ -t 0 ]; then read -r -p "[batch] fix the scene by hand, then Enter to continue (Ctrl-C to abort): " _; else echo "[batch] non-interactive: stopping"; exit 1; fi
}

reset_scene() {   # $1 = trial number the reset follows (0 = before the first trial)
  local NN; NN=$(printf '%02d' "$1")
  echo "================ [batch] reset after trial $1 : $(date '+%H:%M:%S') ================"
  local rc=0
  bash "$FA/run_scatter.sh" "$1" </dev/null || rc=$?
  if [ "$rc" = 1 ]; then
    # rejected or timed out: one automatic second attempt from the half-scattered state
    # (same seeded targets); keep the first attempt's photos/check as *_try1
    for f in after check; do [ -f "$OUT/scatter_${NN}_$f.png" ] && mv "$OUT/scatter_${NN}_$f.png" "$OUT/scatter_${NN}_${f}_try1.png"; done
    echo "================ [batch] reset after trial $1 REJECTED -> automatic second attempt : $(date '+%H:%M:%S') ================"
    rc=0; bash "$FA/run_scatter.sh" "$1" </dev/null || rc=$?
  fi
  [ "$rc" = 0 ] && return 0
  until false; do
    wait_for_human "scene reset after trial $1 rejected or failed twice (rc $rc; see $OUT/scatter_${NN}_check.png)"
    # after a manual fix, re-check the scene without another robot reset
    bash "$FA/capture_top.sh" "$OUT/scatter_$(printf '%02d' "$1")_after.png" >/dev/null
    "$PY" "$FA/tools/check_scatter.py" --image "$OUT/scatter_$(printf '%02d' "$1")_after.png" --debug "$OUT/scatter_$(printf '%02d' "$1")_check.png" && break
  done
}

bash "$FA/capture_top.sh" "$OUT/batch_start_scene.png" >/dev/null
if [ "$SCATTER_FIRST" = 1 ]; then
  echo "[batch] start scene holds the built goal -> robot reset first"
else
  # the scene was scattered by a human; the check is advisory here (a human layout may
  # legitimately have cubes touching) — the human decides once, at the start
  if ! "$PY" "$FA/tools/check_scatter.py" --image "$OUT/batch_start_scene.png" --debug "$OUT/batch_start_check.png" >/dev/null; then
    echo "[batch] NOTE: the start scene does not pass the automatic check (see $OUT/batch_start_check.png); continuing is your call"
  fi
fi
if [ -t 0 ]; then read -r -p "[batch] task $TASK, trials $FROM..$TO of batch $BATCH: arm parked, e-stop in hand?  Enter = go, Ctrl-C = abort: " _; fi
[ "$SCATTER_FIRST" = 1 ] && reset_scene 0

for n in $(seq "$FROM" "$TO"); do
  echo "================ [batch] trial $n : $(date '+%H:%M:%S') ================"
  bash "$FA/run_paper_pyramid.sh" "$n" </dev/null || { wait_for_human "trial $n did not complete normally (exit $?)"; }
  if [ "$n" -lt "$TO" ] || [ "$SCATTER_LAST" = 1 ]; then
    reset_scene "$n"
  fi
done
echo "[batch] done: trials $FROM..$TO. Results: $OUT/results.csv (fill operator_verdict)."
