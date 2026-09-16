#!/usr/bin/env bash
# Unattended chain for the model comparison (2026-09-10): assembly trial -> agent scene reset -> next trial.
#
#   bash run_model_chain.sh <task> <from> <count> --model <slug> --effort <lvl> [--backend codex|claude|agy] [--right] [--no-final-reset]
#                           [--knowledge task [--knowledge-from <batch>]]
#                           [--reset-task twopairsreset] [--reset-model gpt-6-astra] [--reset-effort medium]
#   e.g.  bash run_model_chain.sh twopairs 1 5 --model gpt-5.6-sol --effort high --right
#         = right arm: trials 1..5 of batch twopairs_full_high_56sol_right, each followed by one
#           twopairsreset run (scatter_twopairsreset_full_medium_6astra_right_NN, next free NN,
#           never counted) that lays the parts out again for the next trial.
#
# Every trial and reset is a normal run_trial.sh run (all recordings, results.csv rows); the only
# human duties are the start confirmation and the emergency stop. A reset whose agent does not
# report success, or a run that exits abnormally, STOPS the chain and waits for a human (Enter to
# go on after fixing the scene by hand; non-interactive: stops). The final trial is followed by a
# reset too (leaves the set scattered for whatever comes next) unless --no-final-reset.
# One chain per arm: run a second one in another terminal with/without --right.
# Log: paper_runs/<task>_<batch>/chain_<timestamp>.log (everything the runs print).
set -euo pipefail
FA="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
usage() { sed -n 2,19p "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[ $# -ge 3 ] && [[ "$2" =~ ^[0-9]+$ ]] && [[ "$3" =~ ^[1-9][0-9]*$ ]] || usage
TASK="$1"; FROM="$2"; COUNT="$3"; shift 3
MODEL=""; EFFORT=""; RIGHT=""; FINAL_RESET=1; BACKEND="${PAPER_BACKEND:-codex}"; KNOW=""; KFROM=""
RESET_TASK="${CHAIN_RESET_TASK:-twopairsreset}"; RESET_MODEL="${CHAIN_RESET_MODEL:-gpt-6-astra}"; RESET_EFFORT="${CHAIN_RESET_EFFORT:-medium}"
while [ $# -gt 0 ]; do
  case "$1" in
    --model) MODEL="$2"; shift 2 ;;
    --backend) BACKEND="$2"; shift 2 ;;   # codex | claude | agy for the assembly trials (the reset stays codex)
    --knowledge) KNOW="$2"; shift 2 ;;    # task: store read+written (batch _kn); task-ro: store read only, never written back (batch _knro)
    --knowledge-from) KFROM="$2"; shift 2 ;;   # seed that store from another batch's (see run_trial.sh)
    --effort) EFFORT="$2"; shift 2 ;;
    --right) RIGHT=1; shift ;;
    --no-final-reset) FINAL_RESET=0; shift ;;
    --reset-task) RESET_TASK="$2"; shift 2 ;;
    --reset-model) RESET_MODEL="$2"; shift 2 ;;
    --reset-effort) RESET_EFFORT="$2"; shift 2 ;;
    *) echo "[chain] unknown option $1"; usage ;;
  esac
done
[ -n "$MODEL" ] && [ -n "$EFFORT" ] || { echo "[chain] --model and --effort are required (they name the batch)"; usage; }
case "$EFFORT" in mid|med) EFFORT=medium ;; esac
mtag() {   # identical to run_trial.sh: gpt-5.6-sol -> 56sol, claude-opus-5 -> copus5, gemini-3.1-pro-high -> g31prohigh
  case "$1" in gpt-*) echo "$1" | sed -e 's/^gpt-//' -e 's/[.-]//g' ;; claude-*) echo "c$(echo "$1" | sed -e 's/^claude-//' -e 's/[.-]//g')" ;;
    gemini-*) echo "g$(echo "$1" | sed -e 's/^gemini-//' -e 's/[.-]//g')" ;; *) echo "$1" | sed -e 's/[.-]//g' ;; esac; }
ARMFLAG=(); ARM=left; [ -n "$RIGHT" ] && { ARMFLAG=(--right); ARM=right; }
KTAG=""; [ "$KNOW" = task ] && KTAG="_kn"; [ "$KNOW" = task-ro ] && KTAG="_knro"; [ "$KNOW" = ckpt ] && KTAG="_ckpt"; [ "$KNOW" = ckpt-ro ] && KTAG="_ckptro"; [ "$KNOW" = mx ] && KTAG="_mx"; [ "$KNOW" = mx-ro ] && KTAG="_mxro"
ABATCH="${TASK}_full_${EFFORT}${KTAG}_$(mtag "$MODEL")${RIGHT:+_right}"           # as run_trial.sh names it
RBATCH="${RESET_TASK}_full_${RESET_EFFORT}_$(mtag "$RESET_MODEL")${RIGHT:+_right}"
TO=$(( FROM + COUNT - 1 ))
DRY="${PAPER_DRYRUN:-0}"
OUT="$FA/paper_runs/$ABATCH"; [ "$DRY" = 1 ] || mkdir -p "$OUT"
LOG="$OUT/chain_$(date +%Y%m%d_%H%M%S).log"; [ "$DRY" = 1 ] && LOG=/dev/null

next_reset_no() {   # next free NN of the reset batch (sessions are the ground truth; run_paper_pyramid refuses a taken NN)
  local last; last=$(find "$FA/sessions" -maxdepth 1 -type d -name "*_scatter_${RBATCH}_[0-9][0-9]" | sed 's/.*_//' | sort -n | tail -1)
  echo $(( 10#${last:-0} + 1 ))
}
verdict_of() {   # $1 = batch dir name, $2 = trial number -> agent_verdict of that row ('' if absent)
  python3 - "$FA/paper_runs/$1/results.csv" "$2" <<'PY'
import csv, sys
try:
    rows = list(csv.DictReader(open(sys.argv[1])))
except FileNotFoundError:
    sys.exit(0)
n = int(sys.argv[2])
for r in reversed(rows):
    if r.get("trial", "").lstrip("0") == str(n):
        print(r.get("agent_verdict", "")); break
PY
}
wait_for_human() {   # $1 = message
  echo "[chain] STOP: $1" | tee -a "$LOG"
  if [ -t 0 ]; then read -r -p "[chain] fix the scene by hand, then Enter to continue (Ctrl-C to abort): " _; else echo "[chain] non-interactive: stopping"; exit 1; fi
}
run_one() {   # $1 = label, rest = run_trial.sh args; returns run_trial's status
  echo "================ [chain] $1 : $(date '+%H:%M:%S') ================" | tee -a "$LOG"
  local rc=0
  FA_YES=1 bash "$FA/run_trial.sh" "${@:2}" </dev/null 2>&1 | tee -a "$LOG" || rc=${PIPESTATUS[0]}
  return "$rc"
}

echo "[chain] $TASK trials $FROM..$TO -> batch $ABATCH (backend $BACKEND, model $MODEL, effort $EFFORT, arm $ARM)"
echo "[chain] reset after each trial$([ "$FINAL_RESET" = 1 ] || echo ' except the last'): $RESET_TASK -> batch $RBATCH (model $RESET_MODEL, effort $RESET_EFFORT), sessions scatter_${RBATCH}_NN from $(next_reset_no)"
if [ -t 0 ] && [ "$DRY" != 1 ]; then
  read -r -p "[chain] this arm's set scattered, arm parked, e-stop in hand?  Enter = go, Ctrl-C = abort: " _
fi
for n in $(seq "$FROM" "$TO"); do
  KARGS=(); [ -n "$KNOW" ] && KARGS=(--knowledge "$KNOW"); [ -n "$KFROM" ] && KARGS+=(--knowledge-from "$KFROM")
  run_one "trial $n of $ABATCH" "$TASK" "$n" --backend "$BACKEND" --model "$MODEL" --effort "$EFFORT" "${KARGS[@]}" "${ARMFLAG[@]}" \
    || wait_for_human "trial $n did not complete normally"
  [ "$DRY" = 1 ] || echo "[chain] trial $n agent verdict: $(verdict_of "$ABATCH" "$n")" | tee -a "$LOG"
  if [ "$n" -eq "$TO" ] && [ "$FINAL_RESET" != 1 ]; then break; fi
  r=$(next_reset_no)
  run_one "reset $r of $RBATCH (after trial $n)" "$RESET_TASK" "$r" --nocount --model "$RESET_MODEL" --effort "$RESET_EFFORT" "${ARMFLAG[@]}" \
    || wait_for_human "reset $r did not complete normally"
  if [ "$DRY" != 1 ]; then
    v=$(verdict_of "$RBATCH" "$r"); echo "[chain] reset $r agent verdict: ${v:-none}" | tee -a "$LOG"
    case "$v" in success*) ;; *) wait_for_human "reset $r did not report success (${v:-no verdict}) — check that this arm's four parts lie apart on its half" ;; esac
  fi
done
echo "[chain] done: $TASK trials $FROM..$TO of $ABATCH. Results: $OUT/results.csv (operator_verdict still to fill); resets in paper_runs/$RBATCH/" | tee -a "$LOG"
