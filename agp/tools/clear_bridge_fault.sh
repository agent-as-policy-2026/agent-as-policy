#!/usr/bin/env bash
# Clear a latched "command heartbeat timed out" fault on a bridge (after a Ctrl-C'd session).
#
#   bash tools/clear_bridge_fault.sh [--right] [--yes]
#
# Without --yes: report only. With --yes: one zero-length move to the arm's current joints,
# then the normal release (arm back in gravity-comp idle). Needs no other client on that bridge.
set -euo pipefail
FA="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(dirname "$FA")"
HOST="${BRIDGE_HOST:-127.0.0.1}"; PORT=9021; ROBOT=yam_left; YES=()
for x in "$@"; do
  case "$x" in
    --right) PORT=9022; ROBOT=yam_right ;;
    --yes) YES=(--yes) ;;
    *) sed -n 2,7p "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
  esac
done
unset MUJOCO_GL GAP_SIM_ROBOT
export PYTHONDONTWRITEBYTECODE=1
PY="${AGP_PYTHON:-$REPO/hardware-bridge/.venv/bin/python}"   # the bridge venv; it imports the vendored connector
timeout 3 bash -c "exec 3<>/dev/tcp/$HOST/$PORT" 2>/dev/null || { echo "bridge not listening on $HOST:$PORT" >&2; exit 2; }
others=$(pgrep -af "server_real|capture_top" \
         | grep -v -E "^$$ |clear_bridge_fault|shell-snapshots|pgrep|/bin/bash -c" \
         | awk -v p="$PORT" '{ if (match($0, /--port [0-9]+/)) { split(substr($0, RSTART, RLENGTH), a, " "); if (a[2] != p) next } print }' \
         | grep -E "python|uv run" || true)
[ -z "$others" ] || { echo "another client is running on this bridge; stop it first:" >&2; echo "$others" >&2; exit 2; }
exec $PY "$FA/tools/clear_bridge_fault.py" --host "$HOST" --port "$PORT" --robot "$ROBOT" "${YES[@]}"
