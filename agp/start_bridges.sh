#!/usr/bin/env bash
# Start / stop / show the LEFT (9021) and RIGHT (9022) hardware bridges with one command.
#
#   bash start_bridges.sh start [left|right|both]     (default both)  i2rt source, motion enabled,
#                                                     recorder armed (--record-dir per arm)
#   bash start_bridges.sh start left throw            LEFT bridge with config/left_arm_throw.yaml (2026-09-14:
#                                                     buffered joint programs, J4 180 deg/s / 360 deg/s^2 inside
#                                                     programs) — throwing sessions only; stop it and start the
#                                                     plain left bridge again for every other task
#   bash start_bridges.sh stop  [left|right|both]     clean SIGINT shutdown (gravity comp -> motors off)
#   bash start_bridges.sh status
#
# Each bridge is launched through Python Popen with SIGINT restored to default (a plain
# `setsid nohup … &` leaves SIGINT ignored and kill -INT then does nothing — 2026-09-04 lesson),
# in its own session, stdout+stderr -> agp/bridge_logs/bridge_<arm>_<ts>.log.
# Refuses to start an arm whose port is already taken, or while an agp session runs
# (stop). The left top BRIO focus state is checked first (tools/check_top_focus.sh).
set -euo pipefail
FA="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; HB="$(dirname "$FA")/hardware-bridge"
ACTION="${1:-status}"; ARMS="${2:-both}"; VARIANT="${3:-}"
case "$ARMS" in left) LIST="left" ;; right) LIST="right" ;; both) LIST="left right" ;; *) echo "arms must be left|right|both"; exit 2 ;; esac
case "$VARIANT" in "") ;; throw) [ "$ARMS" = left ] || { echo "the throw variant exists for the left arm only: start left throw"; exit 2; } ;; *) echo "variant must be empty or 'throw'"; exit 2 ;; esac
port_of() { [ "$1" = left ] && echo 9021 || echo 9022; }
cfg_of()  { if [ "$1" = left ]; then [ "$VARIANT" = throw ] && echo config/left_arm_throw.yaml || echo config/left_arm.yaml; else echo config/right_arm.yaml; fi; }
rec_of()  { [ "$1" = left ] && echo "$FA/bridge_recordings" || echo "$FA/bridge_recordings_right"; }
pid_of()  { ss -tlnp 2>/dev/null | grep ":$(port_of "$1") " | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2 || true; }

case "$ACTION" in
status)
  for a in left right; do p=$(pid_of $a); if [ -n "$p" ]; then echo "$a: SERVING on $(port_of $a) (pid $p, SigIgn $(grep SigIgn /proc/$p/status | awk '{print $2}'), $(ps -o etime= -p $p | tr -d ' ') up)"; else echo "$a: not running"; fi; done
  exit 0 ;;
stop)
  for a in $LIST; do p=$(pid_of $a); if [ -z "$p" ]; then echo "$a: not running"; continue; fi
    # only a session on THIS arm's bridge blocks its stop (every server is started with an explicit --port;
    # 2026-09-12: a left-arm trial on 9021 blocked stopping the idle right bridge). No grep -q: under pipefail an
    # early exit would SIGPIPE the upstream grep and turn a match into a false negative.
    if pgrep -af 'server_real.py' | grep -v -E "pgrep" | grep -E -- "--port $(port_of "$a")"'( |$)' >/dev/null; then
      echo "$a: an agp session is running on this arm's bridge ($(port_of "$a")) — stop it first"; exit 1; fi
    kill -INT "$p"; for _ in $(seq 1 60); do kill -0 "$p" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$p" 2>/dev/null && { echo "$a: still alive after 30 s, sending TERM"; kill -TERM "$p" || true; sleep 2; }
    echo "$a: stopped (pid $p)"; done; exit 0 ;;
start)
  # the BRIO resets autofocus=1 whenever it re-enumerates (replug / power cycle, seen twice on 2026-09-08):
  # restore the calibrated state (autofocus 0, focus 0, zoom 100) before the bridge opens the camera
  FA_FIX_TOP_FOCUS=1 bash "$FA/tools/check_top_focus.sh" || exit 2
  echo "top BRIO focus state ok (autofocus off, focus 0, zoom 100)"
  if [ "$ARMS" != left ]; then   # 2026-09-09: the right station's BRIO likewise (autofocus off, focus 10, zoom 100; 1080p since today)
    FA_FIX_TOP_FOCUS=1 bash "$FA/tools/check_top_focus.sh" --arm right || exit 2
    echo "right top BRIO focus state ok (autofocus off, focus 10, zoom 100)"
  fi
  for a in $LIST; do [ -z "$(pid_of $a)" ] || { echo "$a: already running on $(port_of $a) (pid $(pid_of $a))"; exit 1; }; done
  mkdir -p "$FA/bridge_logs" "$FA/bridge_recordings" "$FA/bridge_recordings_right"
  for a in $LIST; do
    python3 - "$a" "$(cfg_of $a)" "$(rec_of $a)" "$HB" "$FA" "$VARIANT" <<'PY'
import subprocess, signal, time, sys, re
arm, cfg, rec, HB, FA, variant = sys.argv[1:7]
ts = time.strftime("%Y%m%d_%H%M%S"); log = f"{FA}/bridge_logs/bridge_{arm}_{ts}.log"
cmd = ["uv","run","--locked","agp-yam-bridge","--config",cfg,"--source","i2rt","--acknowledge-i2rt-startup-motion",
       "--enable-motion","--acknowledge-first-motion-checklist","--record-dir",rec]
if variant == "throw":   # 2026-09-14: the agent reads the growing recording to time the release
    cmd.append("--record-live-readable")
p = subprocess.Popen(cmd, cwd=HB, stdin=subprocess.DEVNULL, stdout=open(log,"w"), stderr=subprocess.STDOUT,
                     start_new_session=True, preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
for _ in range(150):
    time.sleep(1); txt = open(log, errors="replace").read()
    if "status: SERVING" in txt or "BOOT_ERROR" in txt or "Traceback" in txt or p.poll() is not None: break
txt = open(log, errors="replace").read()
ok = "status: SERVING" in txt
lines = [l for l in txt.splitlines() if re.search(r"status:|endpoint:|mode:|recording:|joint programs:|refus|BOOT_ERROR|Error", l) and "motor id" not in l]
print(f"[{arm}] {'SERVING' if ok else 'FAILED'}  log {log}"); print("   " + "\n   ".join(lines[-5:]))
sys.exit(0 if ok else 1)
PY
  done
  sleep 1; bash "$0" status ;;
*) sed -n 2,17p "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
