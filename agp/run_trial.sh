#!/usr/bin/env bash
# One supervised paper trial, no robot reset: YOU reset the scene by hand, then run
#
#   bash run_trial.sh <task> <trial_no> [--bare] [--effort high|medium|low|xhigh] [--harness yield|yield,batch] [--knowledge task] [--batch NAME] [--dual]
#     e.g.  bash run_trial.sh twopiles 3
#           bash run_trial.sh assembly 1 --bare --effort medium                  # -> batch assembly_bare_medium
#           bash run_trial.sh assembly 1 --bare --effort medium --harness yield  # -> batch assembly_bare_medium_yield
#           bash run_trial.sh towel 1 --bare --effort medium --dual              # -> batch towel_bare_medium_dual, both arms
#
# --backend codex|claude|agy (2026-09-10): which agent CLI runs the trial (claude = Claude Code print mode, models
#   claude-opus-5 / claude-fable-5-1; agy = Antigravity, models gemini-3.1-pro-high / gemini-3.8-flash-high).
# --model <slug> (2026-09-10): codex model for the model comparison (gpt-5.6-sol|terra|luna, gpt-6-astra); the
#   batch name gets the tag _56sol / _56terra / _56luna / _6astra. Per-arm prompts: PROMPT_<task>_left.md and
#   PROMPT_<task>_right.md (when present) are used for left / --right sessions (both arms doing the same task).
# --nocount (2026-09-10): a scene-reset run (e.g. task twopairsreset): everything is recorded like a trial, but the
#   session is named scatter_<task>_<batch>_NN instead of paper_..., the established marker for 'never counted'.
# --knowledge-from <batch> (2026-09-10): with --knowledge task, seed this batch's fresh store from paper_runs/<batch>/knowledge
#   (knowledge transfer between models; provenance in knowledge/SEEDED_FROM.txt; only when the store does not exist yet).
# --right (2026-09-09): PAPER_ARMS=right -> a standalone RIGHT-arm trial (right bridge 9022, its own overhead
#   camera, poses in right_base, single-arm prompt/README); batch dir gets the suffix _right. Runs at the same
#   time as a left trial (the two sessions share nothing but the codex CLI).
# --dual (2026-09-08): PAPER_ARMS=left,right -> the session also runs the right arm's server (right bridge on
# BRIDGE_PORT_RIGHT, default 9022, must be up), dual-arm README; batch name gets the suffix _dual; the prompt is
# PROMPT_<task>_dual.md / PROMPT_<task>_dual_bare.md when that file exists (else the single-arm rule below). Default = left only.
#
# <task> selects goal_sets/*_<task> (newest match; needs test_1.jpg, top_camera.png or a
# demo_*.mp4), the batch dir paper_runs/<task>_<batch>, and the prompt: PROMPT_<task>.md if
# that file exists (e.g. PROMPT_dice.md), otherwise the generic block prompt PROMPT_stack_blocks.md.
# Fixed conditions: no notes, tools inherited within the batch, codex gpt-6-astra, effort high
# unless --effort, NO fast tier, 90-min limit, three-view recording + joints.csv. The script
# takes the before-photo, then waits for Enter before the arm moves. Results -> results.csv.
# --bare (2026-09-07 ablation): the same trial on the bare interface — server --bare (no
# deproject/home/move_delta, no advice texts), README_interface_bare.md, NO inherited tools,
# prompt PROMPT_<task>_bare.md (else PROMPT_stack_blocks_bare.md).
# --harness (2026-09-07 efficiency experiments): appends "tool harness" notes to the session's
# README_interface.md — `yield` = robot commands block, use a long exec yield instead of polling;
# `batch` = one tool call may run several commands and view images. Default none.
# Batch name: --batch NAME, else $PAPER_BATCH, else with --effort "<full|bare>_<effort>"
# (e.g. assembly_full_low), else the classic "tools1" / "bare1"; a --harness value is appended
# without commas plus the wording version (e.g. bare_medium_yield_v2, bare_medium_yieldbatch_v2;
# the _yield / _yieldbatch batches without suffix are the v1 wording).
# Env overrides pass through (PAPER_TIMEOUT_MIN, PAPER_MODEL, PAPER_FAST, PAPER_PROMPT);
# PAPER_DRYRUN=1 prints the resolved configuration and starts nothing.
set -euo pipefail
FA="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
usage() { sed -n 2,25p "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[ $# -ge 2 ] && [[ "$2" =~ ^[0-9]+$ ]] || usage
TASK="$1"; N="$2"; shift 2
BARE=0; EFFORT=""; BATCH_OPT=""; HARNESS="${PAPER_HARNESS:-none}"; KNOW="${PAPER_KNOWLEDGE:-}"; ARMS="${PAPER_ARMS:-left}"; KFROM="${PAPER_KNOWLEDGE_FROM:-}"
PREFIX="${PAPER_PREFIX:-paper}"
MODEL_OPT="${PAPER_MODEL:-}"
BACKEND="${PAPER_BACKEND:-codex}"   # --backend codex|claude|agy (2026-09-10)   # --model <codex slug> (2026-09-10, model comparison): tags the batch name; empty = gpt-6-astra, untagged
while [ $# -gt 0 ]; do
  case "$1" in
    --bare) BARE=1; shift ;;
    --dual) ARMS=left,right; shift ;;      # both arms (see run_paper_pyramid.sh PAPER_ARMS / run_real_probe.sh --arms)
    --right) ARMS=right; shift ;;          # standalone RIGHT-arm trial (2026-09-09), runs concurrently with a left one
    --effort) EFFORT="$2"; shift 2 ;;
    --harness) HARNESS="$2"; shift 2 ;;
    --knowledge) KNOW="$2"; shift 2 ;;     # task = task-scoped store in the batch dir; task-ro = the same store READ ONLY (no delta asked for, nothing merged back)
    --knowledge-from) KFROM="$2"; shift 2 ;;   # seed this batch's (new) store from paper_runs/<batch>/knowledge (needs --knowledge task)
    --batch) BATCH_OPT="$2"; shift 2 ;;
    --model) MODEL_OPT="$2"; shift 2 ;;
    --backend) BACKEND="$2"; shift 2 ;;    # codex (default) | claude (Claude Code print mode) | agy (Antigravity)
    --nocount) PREFIX=scatter; shift ;;   # scene-reset run: session name scatter_<task>_<batch>_NN (never counted), data recorded as usual     # codex model slug, e.g. gpt-5.6-sol; batch tag _56sol (gpt- stripped, . and - removed)
    *) echo "[trial] unknown option $1"; usage ;;
  esac
done
case "$KNOW" in ""|task|task-ro|ckpt|ckpt-ro|mx|mx-ro) ;; *) echo "[trial] --knowledge must be 'task', 'task-ro', 'ckpt', 'ckpt-ro', 'mx' or 'mx-ro' (got '$KNOW')"; exit 2 ;; esac
if [ -n "$KFROM" ] && [ "$KNOW" = mx-ro ]; then
  # mx-ro (2026-09-11): the source batch's finished multiple-experience store (paper_runs/<batch>/mx)
  if ! ls -d "$FA/paper_runs/$KFROM"/mx/experience/cycle_[0-9][0-9] >/dev/null 2>&1; then
    echo "[trial] --knowledge-from: paper_runs/$KFROM has no mx/experience/cycle_NN"; exit 2
  fi
elif [ -n "$KFROM" ]; then
  case "$KNOW" in task|task-ro|ckpt|ckpt-ro) ;; *) echo "[trial] --knowledge-from needs --knowledge task|task-ro|ckpt|ckpt-ro|mx-ro"; exit 2 ;; esac
  # the source batch may hold either store kind (tools/seed_knowledge.sh copies whichever it finds)
  if [ ! -f "$FA/paper_runs/$KFROM/knowledge/RIG_NOTES.md" ] \
     && ! ls -d "$FA/paper_runs/$KFROM"/experience/cycle_[0-9][0-9] >/dev/null 2>&1; then
    echo "[trial] --knowledge-from: paper_runs/$KFROM has neither knowledge/RIG_NOTES.md nor experience/cycle_NN"; exit 2
  fi
fi
case "$BACKEND" in codex|claude|agy) ;; *) echo "[trial] --backend must be codex|claude|agy (got '$BACKEND')"; exit 2 ;; esac
for tok in ${HARNESS//,/ }; do case "$tok" in none|yield|batch) ;; *) echo "[trial] --harness tokens must be yield|batch (got '$tok')"; exit 2 ;; esac; done
HTAG=""; [ "$HARNESS" != none ] && HTAG="_${HARNESS//,/}_v2"   # v2 = wording of 2026-09-07 20:50 (see run_real_probe.sh)
[ "$KNOW" = task ] && HTAG="${HTAG}_kn"                          # task-knowledge batches get their own dir/store
[ "$KNOW" = task-ro ] && HTAG="${HTAG}_knro"                     # read-only knowledge (store used, never written back)
[ "$KNOW" = ckpt ] && HTAG="${HTAG}_ckpt"                        # immutable rolling experience checkpoints (multiple-experience method)
[ "$KNOW" = ckpt-ro ] && HTAG="${HTAG}_ckptro"                   # the same checkpoints, read only (never written back)
[ "$KNOW" = mx ] && HTAG="${HTAG}_mx"                            # the multiple-experience memory (mx/), 2026-09-11
[ "$KNOW" = mx-ro ] && HTAG="${HTAG}_mxro"                       # a finished mx store reused read only (never written back)
if [ -n "$MODEL_OPT" ]; then                                      # model batches likewise: gpt-5.6-sol -> _56sol, gpt-6-astra -> _6astra
  MC="$HOME/.codex/models_cache.json"
  if [ "$BACKEND" = codex ] && [ -f "$MC" ] && ! grep -q "\"slug\": *\"$MODEL_OPT\"" "$MC"; then echo "[trial] WARNING: model '$MODEL_OPT' is not in $MC (known: $(grep -oE '"slug": *"[^"]+"' "$MC" | cut -d'"' -f4 | tr '\n' ' '))"; fi
  # tag: gpt-5.6-sol -> _56sol, gpt-6-astra -> _6astra (unchanged); claude-opus-5 -> _copus5, claude-fable-5-1 -> _cfable51;
  # gemini-3.1-pro-high -> _g31prohigh (a vendor letter keeps the families apart)
  case "$MODEL_OPT" in
    gpt-*)    HTAG="${HTAG}_$(echo "$MODEL_OPT" | sed -e 's/^gpt-//' -e 's/[.-]//g')" ;;
    claude-*) HTAG="${HTAG}_c$(echo "$MODEL_OPT" | sed -e 's/^claude-//' -e 's/[.-]//g')" ;;
    gemini-*) HTAG="${HTAG}_g$(echo "$MODEL_OPT" | sed -e 's/^gemini-//' -e 's/[.-]//g')" ;;
    *)        HTAG="${HTAG}_$(echo "$MODEL_OPT" | sed -e 's/[.-]//g')" ;;
  esac
fi
ARMSNOTE=""
case "$ARMS" in
  left) ;;
  left,right|right,left) ARMS=left,right; HTAG="${HTAG}_dual"; ARMSNOTE="; arms $ARMS" ;;   # dual-arm batches get their own dir/store
  right) HTAG="${HTAG}_right"; ARMSNOTE="; arms right" ;;                                     # right-arm batches likewise (--batch overrides)
  *) echo "[trial] --dual / --right / PAPER_ARMS: arms must be left, right or left,right (got '$ARMS')"; exit 2 ;;
esac
case "$EFFORT" in
  "") ;;
  mid|med) EFFORT=medium ;;
  low|medium|high|xhigh) ;;
  *) echo "[trial] --effort must be low|medium|high|xhigh (got '$EFFORT')"; exit 2 ;;
esac
GOAL=$(ls -d "$FA"/goal_sets/*_"$TASK" 2>/dev/null | sort | tail -1 || true)
# INSTRUCTION.md (2026-09-14): a language-instructed task (throw) has no reference image or video
[ -n "$GOAL" ] && { [ -f "$GOAL/test_1.jpg" ] || [ -f "$GOAL/top_camera.png" ] || [ -f "$GOAL/INSTRUCTION.md" ] || ls "$GOAL"/demo_*.mp4 >/dev/null 2>&1; } \
  || { echo "[trial] no goal set goal_sets/*_$TASK with test_1.jpg, top_camera.png, INSTRUCTION.md or demo_*.mp4 — create it first (see README runbook B)"; exit 2; }
# throwing (2026-09-14): the archived throwing runs used a 60-minute limit; PAPER_TIMEOUT_MIN still overrides
[ "$TASK" = throw ] && export PAPER_TIMEOUT_MIN="${PAPER_TIMEOUT_MIN:-60}"
PROMPT="${PAPER_PROMPT:-}"
if [ -z "$PROMPT" ]; then
  DUAL=""; [ "$ARMS" = left,right ] && DUAL="_dual"   # two arms: PROMPT_<task>_dual[_bare].md when it exists, else the single-arm rule
  SIDE=""; case "$ARMS" in left) SIDE="_left" ;; right) SIDE="_right" ;; esac   # per-arm prompt (2026-09-10): both arms doing the same task side by side
  if [ "$BARE" = 1 ]; then
    if [ -n "$SIDE" ] && [ -f "$FA/PROMPT_${TASK}${SIDE}_bare.md" ]; then PROMPT="PROMPT_${TASK}${SIDE}_bare.md"
    elif [ -n "$DUAL" ] && [ -f "$FA/PROMPT_${TASK}${DUAL}_bare.md" ]; then PROMPT="PROMPT_${TASK}${DUAL}_bare.md"
    elif [ -f "$FA/PROMPT_${TASK}_bare.md" ]; then PROMPT="PROMPT_${TASK}_bare.md"; else PROMPT="PROMPT_stack_blocks_bare.md"; fi
  else
    if [ "${KNOW%-ro}" = mx ] && [ -n "$SIDE" ] && [ -f "$FA/PROMPT_${TASK}${SIDE}_mx.md" ]; then PROMPT="PROMPT_${TASK}${SIDE}_mx.md"   # mx / mx-ro: task text without the deliverable (the mx round report replaces it)
    elif [ -n "$SIDE" ] && [ -f "$FA/PROMPT_${TASK}${SIDE}.md" ]; then PROMPT="PROMPT_${TASK}${SIDE}.md"
    elif [ -n "$DUAL" ] && [ -f "$FA/PROMPT_${TASK}${DUAL}.md" ]; then PROMPT="PROMPT_${TASK}${DUAL}.md"
    elif [ -f "$FA/PROMPT_$TASK.md" ]; then PROMPT="PROMPT_$TASK.md"; else PROMPT="PROMPT_stack_blocks.md"; fi
  fi
fi
[ -f "$FA/$PROMPT" ] || { echo "[trial] prompt file not found: $FA/$PROMPT"; exit 2; }
IFACE=$([ "$BARE" = 1 ] && echo bare || echo full)
if [ -n "$BATCH_OPT" ]; then BATCH="$BATCH_OPT"
elif [ -n "${PAPER_BATCH:-}" ]; then BATCH="$PAPER_BATCH"
elif [ -n "$EFFORT" ]; then BATCH="${IFACE}_${EFFORT}${HTAG}"
else BATCH="$([ "$BARE" = 1 ] && echo bare1 || echo tools1)${HTAG}"; fi
echo "[trial] task $TASK -> goal set $(basename "$GOAL") ($(ls "$GOAL" | tr '\n' ' ')); prompt $PROMPT; interface $IFACE; effort ${EFFORT:-${PAPER_EFFORT:-high}}; harness $HARNESS; trial $N; batch $BATCH$ARMSNOTE"
cd "$FA"
PAPER_TASK="$TASK" PAPER_GOAL="goal_sets/$(basename "$GOAL")" PAPER_PROMPT="$PROMPT" PAPER_BATCH="$BATCH" PAPER_BARE="$BARE" PAPER_HARNESS="$HARNESS" PAPER_KNOWLEDGE="$KNOW" PAPER_ARMS="$ARMS" \
  PAPER_EFFORT="${EFFORT:-${PAPER_EFFORT:-high}}" PAPER_FAST="${PAPER_FAST:-0}" PAPER_MODEL="${MODEL_OPT:-gpt-6-astra}" PAPER_BACKEND="$BACKEND" PAPER_PREFIX="$PREFIX" PAPER_KNOWLEDGE_FROM="$KFROM" exec bash "$FA/run_paper_pyramid.sh" "$N"
