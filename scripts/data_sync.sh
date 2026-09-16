#!/usr/bin/env bash
# Sync the large data files of this repo (session videos, calibration .npz,
# hardware-bridge action log) between the working tree and Google Drive.
# Git tracks none of those bytes: they live in Drive, indexed by data_manifest.md5.
#
#   scripts/data_sync.sh push     upload new/changed files to Drive, refresh the manifest
#   scripts/data_sync.sh pull     download everything in the manifest, verify md5
#   scripts/data_sync.sh status   compare local files with Drive (no transfer)
#   scripts/data_sync.sh verify   check local files against data_manifest.md5
#
# One-time setup:   rclone config create gdrive drive scope=drive
# Remote (required): DATA_REMOTE=<rclone-remote>:<folder> scripts/data_sync.sh pull
#
# Needs rclone and coreutils (md5sum). scripts/data_sync.filter ships with the
# repo and defines what counts as large data; keep it in step with .gitignore.
# data_manifest.md5 does NOT ship: `push` writes it, and it indexes one particular
# rig's data, so a fresh checkout has none. `verify` then says so and `pull` skips
# the md5 check instead of failing.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SELF="$REPO_ROOT/scripts/$(basename "${BASH_SOURCE[0]}")"   # absolute: usage() reads it after cd
REMOTE="${DATA_REMOTE:-}"
FILTER="$REPO_ROOT/scripts/data_sync.filter"
MANIFEST="$REPO_ROOT/data_manifest.md5"
RCLONE_OPTS=(--filter-from "$FILTER" --skip-links --transfers 4 --checkers 8)

command -v rclone >/dev/null || {
  echo "rclone not found. Install it (https://rclone.org/install/) and run:" >&2
  echo "  rclone config create gdrive drive scope=drive" >&2
  exit 1
}
need_remote() {
  [ -n "$REMOTE" ] || {
    echo "DATA_REMOTE is not set: there is no default remote -- it names your own" >&2
    echo "rclone remote and the folder this repo's large files live in. Set it:" >&2
    echo "  rclone config create gdrive drive scope=drive   # one time" >&2
    echo "  DATA_REMOTE=gdrive:<folder> scripts/data_sync.sh ${1:-pull}" >&2
    exit 1
  }
}
[ -f "$FILTER" ] || {
  echo "missing $FILTER -- it ships with the repo; restore it before syncing." >&2
  exit 1
}
cd "$REPO_ROOT"

verify() {
  if [ ! -f "$MANIFEST" ]; then
    echo "no data_manifest.md5 in $REPO_ROOT: it is written by 'push' and is not part of" >&2
    echo "the checkout, so there is nothing to check against. Skipping the md5 check." >&2
    return 0
  fi
  md5sum --quiet -c "$MANIFEST" && echo "All $(wc -l < "$MANIFEST") files match data_manifest.md5."
}

case "${1:-}" in
  push)
    need_remote push
    rclone copy "${RCLONE_OPTS[@]}" --drive-chunk-size 64M --progress "$REPO_ROOT" "$REMOTE"
    rclone md5sum "${RCLONE_OPTS[@]}" "$REPO_ROOT" | sort -k2 > "$MANIFEST"
    echo "Uploaded to $REMOTE. Manifest lists $(wc -l < "$MANIFEST") files; keep it beside the repo."
    ;;
  pull)
    need_remote pull
    rclone copy "${RCLONE_OPTS[@]}" --progress "$REMOTE" "$REPO_ROOT"
    verify
    ;;
  status)
    need_remote status
    rclone check "${RCLONE_OPTS[@]}" "$REPO_ROOT" "$REMOTE"
    ;;
  verify)
    verify
    ;;
  -h|--help|help)
    sed -n '2,12p' "$SELF" | sed 's/^# \{0,1\}//'
    ;;
  *)
    sed -n '2,12p' "$SELF" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
