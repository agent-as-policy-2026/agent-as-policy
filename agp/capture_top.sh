#!/usr/bin/env bash
# Take one overhead-camera photo through the running bridge and save it.
#
#   capture_top.sh <output dir or file.png> [--wrist]
#
# Examples:
#   capture_top.sh agp/goal_sets/20260903_pyramid_plain/          -> top_camera.png in that dir
#   capture_top.sh /tmp/scene.png --wrist                                 -> scene.png + scene_wrist.png
#
# Needs: the left bridge up (read-only or motion mode) and NO other bridge observer
# (an agent session, a calibration capture). Observation-only: nothing moves.
set -euo pipefail
FA="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$FA")"
HOST="${BRIDGE_HOST:-127.0.0.1}"
# FA_CAPTURE_ARM=right (2026-09-09): photograph through the RIGHT bridge (its own overhead BRIO, 640x360)
ARM="${FA_CAPTURE_ARM:-left}"; ROBOT=yam_left
if [ "$ARM" = right ]; then PORT="${BRIDGE_PORT_RIGHT:-${BRIDGE_PORT:-9022}}"; ROBOT=yam_right; else PORT="${BRIDGE_PORT:-9021}"; fi   # right arm: its own port variable wins
[ $# -ge 1 ] || { sed -n 2,11p "$0" | sed 's/^# \{0,1\}//'; exit 2; }
if [ "$ARM" = right ]; then bash "$FA/tools/check_top_focus.sh" --arm right || exit 2; else bash "$FA/tools/check_top_focus.sh" || exit 2; fi   # refuse photos with autofocus on
OUT="$1"; shift

unset MUJOCO_GL GAP_SIM_ROBOT
export PYTHONDONTWRITEBYTECODE=1
PY="${AGP_PYTHON:-$REPO/hardware-bridge/.venv/bin/python}"   # the bridge venv; it imports the vendored connector

timeout 3 bash -c "exec 3<>/dev/tcp/$HOST/$PORT" 2>/dev/null || {
  echo "[capture_top] bridge not listening on $HOST:$PORT — start it first (any mode)" >&2; exit 2; }
others=$(pgrep -af "server_real|preview_camera" \
         | grep -v -E "^$$ |capture_top|shell-snapshots|pgrep|/bin/bash -c" \
         | awk -v p="$PORT" '{ if (match($0, /--port [0-9]+/)) { split(substr($0, RSTART, RLENGTH), a, " "); if (a[2] != p) next } print }' \
         | grep -E "python|uv run" || true)   # other-bridge clients (incl. a session on the other arm's port) are fine
[ -z "$others" ] || { echo "[capture_top] another bridge observer is running; stop it first:" >&2; echo "$others" >&2; exit 2; }

exec $PY "$FA/tools/capture_top.py" --out "$OUT" --host "$HOST" --port "$PORT" --robot "$ROBOT" "$@"
