#!/usr/bin/env bash
# Seed a NEW task-knowledge store from an existing batch's store (knowledge transfer between models,
# 2026-09-10): copies paper_runs/<from>/knowledge/ (RIG_NOTES.md, playbooks/, tools/) to
# paper_runs/<to>/knowledge/ and records the provenance in SEEDED_FROM.txt. Refuses when the target
# store already exists (seeding happens once, before the target batch's first trial).
#
#   bash tools/seed_knowledge.sh <from-batch-dir-name> <to-batch-dir-name>
#   e.g. bash tools/seed_knowledge.sh twopairs_full_high_kn_6astra twopairs_full_high_kn_56terra
set -euo pipefail
FA="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[ $# -eq 2 ] || { sed -n 2,9p "$0" | sed 's/^# \{0,1\}//'; exit 2; }
# 2026-09-10: a batch carries EITHER an append-only notes/tools store (knowledge/) or an
# immutable experience-checkpoint store (experience/); seed whichever the source has.
if [ -d "$FA/paper_runs/$1/experience" ] && [ ! -f "$FA/paper_runs/$1/knowledge/RIG_NOTES.md" ]; then
  SRC="$FA/paper_runs/$1/experience"; DST="$FA/paper_runs/$2/experience"; KIND=experience
else
  SRC="$FA/paper_runs/$1/knowledge"; DST="$FA/paper_runs/$2/knowledge"; KIND=knowledge
fi
if [ "$KIND" = experience ]; then
  ls -d "$SRC"/cycle_[0-9][0-9] >/dev/null 2>&1 || { echo "[seed] no committed checkpoints at $SRC"; exit 2; }
  ls -d "$DST"/cycle_[0-9][0-9] >/dev/null 2>&1 && { echo "[seed] target store already has checkpoints: $DST"; exit 3; }
  mkdir -p "$DST"; cp -r "$SRC/." "$DST/"
  n=$(find "$DST" -maxdepth 1 -type d -name 'cycle_[0-9][0-9]' | wc -l)
  { echo "seeded_from: $1"; echo "date: $(date '+%Y-%m-%d %H:%M:%S')"; echo "checkpoints: $n"; } > "$DST/SEEDED_FROM.txt"
  echo "[seed] $DST <- $SRC ($n checkpoint(s))"; exit 0
fi
[ -f "$SRC/RIG_NOTES.md" ] || { echo "[seed] no knowledge store at $SRC"; exit 2; }
[ ! -f "$DST/RIG_NOTES.md" ] || { echo "[seed] target store already has RIG_NOTES.md: $DST (seeding is only for a fresh batch)"; exit 3; }
# an EMPTY $DST may already exist (run_paper_pyramid.sh creates <batch>/knowledge/tools before seeding)
mkdir -p "$DST"
cp -r "$SRC/." "$DST/"
rm -f "$DST"/.merge_lock 2>/dev/null || true
rows=$(grep -cE '^[^|#<-].*\|.*\|.*\|' "$DST/RIG_NOTES.md" || true); pbs=$(ls "$DST/playbooks" 2>/dev/null | wc -l); tls=$(ls "$DST/tools" 2>/dev/null | grep -vc TOOLS.md || true)
{ echo "seeded_from: $1"; echo "date: $(date '+%Y-%m-%d %H:%M:%S')"; echo "rig_notes_rows: $rows"; echo "playbooks: $pbs"; echo "tools: $tls"; } > "$DST/SEEDED_FROM.txt"
echo "[seed] $DST <- $SRC ($rows note rows, $pbs playbooks, $tls tools)"
