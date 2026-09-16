#!/usr/bin/env bash
# AgP session launcher for the REAL YAM arms.
#
#   run_real_probe.sh start <name> [--allow-motion] [--prompt FILE] [--goal DIR] [--no-video] [--arms left,right|right] [-- <server args>]
#   run_real_probe.sh stop   [SESSION_DIR]          (default: sessions/LATEST)
#   run_real_probe.sh status [SESSION_DIR]
#   run_real_probe.sh rec-start|rec-stop [SESSION_DIR]   all three views (side ffmpeg + bridge top/wrist via SIGUSR1/2)
#   (one-shot experiment incl. codex launch: run_experiment.sh)
#
# --arms left,right (2026-09-08; default left = everything below exactly as before): the same
#         session also runs a second server_real.py --arm right against the RIGHT arm's bridge
#         (BRIDGE_PORT_RIGHT, default 9022): bridge_right/ frames_right/ record_right/
#         server_right.log server_right.pid server_right_boot.log; README_interface_dual*.md;
#         READY is awaited for BOTH servers (one failing stops both); rec-start/rec-stop also
#         drive the right bridge's recorder (FA_BRIDGE_REC_DIR_RIGHT -> run_video_right_*.mp4,
#         run_joints_right.csv); stop/status handle both servers (shared SERVER_STOP sentinel).
# --arms right (2026-09-09): a STANDALONE right-arm session, meant to run at the same time as an
#         independent left session: ONE server (server_real.py --arm right --standalone) against
#         the right bridge (BRIDGE_PORT_RIGHT), the single-arm layout and README (top = the right
#         bridge's own 640x360 BRIO, poses in right_base), the right bridge's recorder
#         (FA_BRIDGE_REC_DIR_RIGHT -> run_video_top/wrist.mp4, run_joints.csv), no side video
#         (that camera belongs to the left rig), no left-BRIO focus gate. session.env ARMS=right.
#         Observer guards ignore clients of the OTHER bridge (matched by their --port).
#
# start:  checks the bridge port, that no other bridge observer is running and
#         that the recorder camera is free; starts the continuous run recorder
#         (ffmpeg on the third BRIO D0CF5843), starts server_real.py, waits for
#         READY, copies the agent-facing files into the session and PRINTS the
#         codex launch line (never runs it: the human launches codex).
# stop:   SERVER_STOP sentinel -> server exits (bridge lease released; in motion
#         mode the arm goes limp ~0.5 s later) -> recorder gets SIGINT for a
#         clean mp4 trailer -> artifact list.
#
# Nothing here writes into hardware-bridge/, calib/ or the vendored connector
# under third_party/ (read-only: PYTHONDONTWRITEBYTECODE=1, no uv sync).
set -euo pipefail

FA="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$FA")"
BRIDGE_DIR="$REPO/hardware-bridge"
HOST="${BRIDGE_HOST:-127.0.0.1}"
PORT="${BRIDGE_PORT:-9021}"
PORT_RIGHT="${BRIDGE_PORT_RIGHT:-9022}"   # the right arm's bridge (sessions started with --arms left,right only)
CAM="${FA_RECORD_CAM:-/dev/v4l/by-id/usb-046d_Logitech_BRIO_D0CF5843-video-index0}"
PREVIEW_URL="${FA_PREVIEW_URL:-http://127.0.0.1:8768/}"   # calib/preview_camera.py MJPEG stream (if running)
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf

unset MUJOCO_GL GAP_SIM_ROBOT
export PYTHONDONTWRITEBYTECODE=1
PY="${AGP_PYTHON:-$REPO/hardware-bridge/.venv/bin/python}"   # the bridge venv; it imports the vendored connector

usage() { sed -n 2,24p "$0" | sed 's/^# \{0,1\}//'; exit 2; }

die() { echo "[agp] ABORT: $*" >&2; exit "${2:-2}"; }

port_open() {  # [$1 = port, default $PORT]
  timeout 3 bash -c "exec 3<>/dev/tcp/$HOST/${1:-$PORT}" 2>/dev/null
}

other_observers() {
  # Any other bridge observation client races the cameras -> SOURCE_ERROR.
  # Match real observer PROCESSES only: a python/uv interpreter running one of the
  # known scripts — never a shell whose command line merely mentions the names
  # (the launcher itself, run_experiment.sh, an operator's pgrep, Claude's shell wrapper).
  # Some names in the pattern are earlier probe/preview scripts that this repo does not ship;
  # they stay so that such a process, if still running on the rig, is still caught.
  # Clients of the OTHER arm's bridge (a different --port) do not touch this arm's cameras and are
  # ignored (2026-09-04: both arms run concurrently).
  pgrep -af "server_real|capture_top|preview_camera" \
    | grep -v -E "^$$ |run_real_probe|run_experiment|shell-snapshots|pgrep|/bin/bash -c" \
    | awk -v p="$PORT" '{ if (match($0, /--port [0-9]+/)) { split(substr($0, RSTART, RLENGTH), a, " "); if (a[2] != p) next } print }' \
    | grep -E "python|uv run" || true
}

start_recorder() {  # $1 = session dir
  local S="$1"
  if [ ! -e "$CAM" ]; then
    echo "[agp] recorder camera $CAM not present -> no video"; return 0
  fi
  # Source: the device directly, or — when calib/preview_camera.py already holds
  # it (one V4L2 streamer per node) — that preview's own MJPEG HTTP stream, so the
  # human's live view keeps running and the recorder reads from it.
  local -a common=(-nostdin -hide_banner -loglevel warning -y)
  local src="${FA_RECORD_SRC:-}"
  if [ -z "$src" ] && fuser "$CAM" >/dev/null 2>&1; then
    if curl -s -m 3 -o /dev/null -w '%{http_code}' "$PREVIEW_URL" 2>/dev/null | grep -q '^200$'; then
      src="$PREVIEW_URL"
      echo "[agp] $CAM is held by the preview server; recording its stream $PREVIEW_URL instead"
    else
      die "recorder camera $CAM is held by another process and no preview stream at $PREVIEW_URL: $(fuser -v "$CAM" 2>&1 | tail -1)"
    fi
  fi
  if [ -n "$src" ]; then
    common+=(-f mjpeg -use_wallclock_as_timestamps 1 -i "$src")
  else
    common+=(-f v4l2 -input_format mjpeg -video_size 1920x1080 -framerate 30 -use_wallclock_as_timestamps 1 -i "$CAM")
  fi
  local enc=(-c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p -g 60 -movflags +faststart -t 7200)
  local vf="drawtext=fontfile=$FONT:text='%{localtime}':x=12:y=12:fontsize=32:fontcolor=white:box=1:boxcolor=black@0.5"
  nohup ffmpeg "${common[@]}" -map 0:v -vf "$vf" "${enc[@]}" "$S/run_video_side.mp4" \
        -map 0:v -r 0.2 -update 1 "$S/snapshot.png" < /dev/null > "$S/recorder.log" 2>&1 &
  echo $! > "$S/recorder.pid"
  sleep 3
  if ! kill -0 "$(cat "$S/recorder.pid")" 2>/dev/null; then
    echo "[agp] recorder with timestamp overlay failed (see recorder.log); retrying without overlay"
    nohup ffmpeg "${common[@]}" -map 0:v "${enc[@]}" "$S/run_video_side.mp4" \
          -map 0:v -r 0.2 -update 1 "$S/snapshot.png" < /dev/null > "$S/recorder.log" 2>&1 &
    echo $! > "$S/recorder.pid"
    sleep 3
    kill -0 "$(cat "$S/recorder.pid")" 2>/dev/null || die "recorder failed twice: $(tail -3 "$S/recorder.log")"
  fi
  echo "[agp] recorder pid $(cat "$S/recorder.pid") -> $S/run_video_side.mp4 (snapshot.png every 5 s)"
}

stop_recorder() {  # $1 = session dir
  local S="$1" pid
  [ -f "$S/recorder.pid" ] || return 0
  pid=$(cat "$S/recorder.pid")
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 40); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$pid" 2>/dev/null && { echo "[agp] recorder did not finish, TERM"; kill -TERM "$pid" || true; sleep 2; }
  fi
  if [ -f "$S/run_video_side.mp4" ]; then
    local dur; dur=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$S/run_video_side.mp4" 2>/dev/null || echo "?")
    echo "[agp] video: $S/run_video_side.mp4 (side) duration ${dur}s size $(du -h "$S/run_video_side.mp4" | cut -f1)"
  fi
  # assemble top/wrist timelapses from the server's periodic JPEGs — only for a
  # view the bridge recorder did not already deliver as run_video_<cam>.mp4
  local cam
  for cam in top wrist; do
    local d="$S/record/$cam"
    [ -s "$S/run_video_$cam.mp4" ] && continue
    if [ -d "$d" ] && [ -n "$(ls "$d"/*.jpg 2>/dev/null)" ]; then
      local n; n=$(ls "$d"/*.jpg | wc -l)
      ffmpeg -nostdin -hide_banner -loglevel error -y -framerate 6 -pattern_type glob -i "$d/*.jpg" \
        -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p -movflags +faststart "$S/run_video_$cam.mp4" </dev/null 2>>"$S/recorder.log" \
        && echo "[agp] video: $S/run_video_$cam.mp4 ($cam, $n frames at 6 fps timelapse)" \
        || echo "[agp] WARN: could not assemble $cam video (see recorder.log)"
    fi
  done
  # right arm (sessions started with --arms left,right): the same fallback from record_right/<cam>/
  for d in "$S"/record_right/*/; do
    [ -d "$d" ] || continue
    cam=$(basename "$d")
    [ "$cam" = top ] && continue          # the right bridge's own top BRIO is not kept (2026-09-08)
    [ -s "$S/run_video_right_$cam.mp4" ] && continue
    if [ -n "$(ls "$d"*.jpg 2>/dev/null)" ]; then
      local nr; nr=$(ls "$d"*.jpg | wc -l)
      ffmpeg -nostdin -hide_banner -loglevel error -y -framerate 6 -pattern_type glob -i "$d*.jpg" \
        -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p -movflags +faststart "$S/run_video_right_$cam.mp4" </dev/null 2>>"$S/recorder.log" \
        && echo "[agp] video: $S/run_video_right_$cam.mp4 (right $cam, $nr frames at 6 fps timelapse)" \
        || echo "[agp] WARN: could not assemble right $cam video (see recorder.log)"
    fi
  done
}

stop_servers() {  # $1 = session dir. Failure paths of a dual start: SERVER_STOP for both servers, then wait
  local S="$1" f pid   # for every server pid file; TERM after 30 s, KILL 5 s later (never leave one running)
  touch "$S/SERVER_STOP"
  for f in server server_right; do
    [ -f "$S/$f.pid" ] || continue
    pid=$(cat "$S/$f.pid")
    for _ in $(seq 1 60); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    if kill -0 "$pid" 2>/dev/null; then
      echo "[agp] $f (pid $pid) still alive after 30 s, TERM" >&2; kill -TERM "$pid" 2>/dev/null || true
      for _ in $(seq 1 10); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
      kill -0 "$pid" 2>/dev/null && { echo "[agp] $f (pid $pid) ignored TERM, KILL" >&2; kill -KILL "$pid" 2>/dev/null || true; }
    fi
  done
}

session_has_right() {  # $1 = session dir: started with --arms left,right? (session.env ARMS=...; absent = left)
  grep -m1 '^ARMS=' "$1/session.env" 2>/dev/null | grep -q 'left,right'
}

SOLO=""   # set by cmd_start --arms right and by solo_env for a standalone right session
solo_env() {  # $1 = session dir: a standalone RIGHT session (session.env ARMS=right) talks to the right bridge
  if grep -m1 '^ARMS=' "$1/session.env" 2>/dev/null | grep -qx 'ARMS=right'; then
    SOLO=1; PORT="$PORT_RIGHT"; BRIDGE_REC_DIR="$BRIDGE_REC_DIR_RIGHT"
  fi
}

cmd_start() {
  [ $# -ge 1 ] || usage
  local NAME="$1"; shift
  local ALLOW="" PROMPT="$FA/PROMPT_readonly_check.md" GOAL="" VIDEO=1 ARMS=left RIGHT=""
  local -a EXTRA=()
  while [ $# -gt 0 ]; do
    case "$1" in
      --allow-motion) ALLOW=1; shift ;;
      --prompt) PROMPT="$2"; shift 2 ;;
      --goal) GOAL="$2"; shift 2 ;;
      --no-video) VIDEO=0; shift ;;
      --arms) ARMS="$2"; shift 2 ;;
      --) shift; EXTRA=("$@"); break ;;
      *) die "unknown option $1" ;;
    esac
  done
  case "$ARMS" in   # RIGHT=1 <=> the session also runs the right arm's server (default: left only, as always)
    left) ;;
    left,right|right,left) ARMS=left,right; RIGHT=1
      [ "$PORT" != "$PORT_RIGHT" ] || die "BRIDGE_PORT and BRIDGE_PORT_RIGHT are both $PORT: two arms need two bridges (left default 9021, right 9022)" ;;
    right) ARMS=right; SOLO=1; PORT="$PORT_RIGHT"; BRIDGE_REC_DIR="$BRIDGE_REC_DIR_RIGHT"; VIDEO=0   # standalone right session (2026-09-09)
      echo "[agp] --arms right: standalone RIGHT-arm session on $HOST:$PORT (its own overhead camera; no side video)" ;;
    *) die "--arms must be left, right or left,right (got '$ARMS')" ;;
  esac
  [ -f "$PROMPT" ] || die "prompt file not found: $PROMPT"
  # 2026-09-08: the left top BRIO must be in its calibrated focus state (autofocus off) — see tools/check_top_focus.sh
  if [ -z "$SOLO" ]; then
    bash "$FA/tools/check_top_focus.sh" || die "top camera focus check failed (see message above)"
  else
    bash "$FA/tools/check_top_focus.sh" --arm right || die "right top camera focus check failed (see message above)"
  fi
  local htok
  # ${FA_HARNESS:+...}: with FA_HARNESS unset (direct start, run_experiment.sh, run_scatter.sh) the 2026-09-07
  # form `${FA_HARNESS//,/ }` aborted under set -u ("FA_HARNESS: unbound variable"); since 2026-09-08 an unset
  # value means harness=none (the documented default, see HARNESS below). Set values behave exactly as before.
  for htok in ${FA_HARNESS:+${FA_HARNESS//,/ }}; do case "$htok" in none|yield|batch) ;; *) die "FA_HARNESS tokens must be yield|batch (got '$htok')" ;; esac; done

  # (1) bridge port
  if ! port_open; then
    cat >&2 <<EOF
[agp] ABORT 1: nothing listens on $HOST:$PORT. Start the bridge first:
  cd $BRIDGE_DIR
  uv run --locked agp-yam-preflight --config ${SOLO:+config/right_arm.yaml}${SOLO:-config/left_arm.yaml}      # expect: status: READY
  uv run --locked agp-yam-bridge --config ${SOLO:+config/right_arm.yaml}${SOLO:-config/left_arm.yaml} --source i2rt --acknowledge-i2rt-startup-motion${ALLOW:+ \\
      --enable-motion --acknowledge-first-motion-checklist    # motion session}
EOF
    exit 2
  fi
  if [ -n "$RIGHT" ] && ! port_open "$PORT_RIGHT"; then
    cat >&2 <<EOF
[agp] ABORT 1: nothing listens on $HOST:$PORT_RIGHT (right arm, --arms $ARMS). Start the right bridge first:
  cd $BRIDGE_DIR
  uv run --locked agp-yam-preflight --config config/right_arm.yaml      # expect: status: READY
  uv run --locked agp-yam-bridge --config config/right_arm.yaml --source i2rt --acknowledge-i2rt-startup-motion \\
      --record-dir $BRIDGE_REC_DIR_RIGHT${ALLOW:+ \\
      --enable-motion --acknowledge-first-motion-checklist    # motion session}
EOF
    exit 2
  fi
  # (2) one observer rule
  local others; others=$(other_observers)
  [ -z "$others" ] || die "another bridge observer is running (stop it first):"$'\n'"$others"
  # (3) motion checklist
  if [ -n "$ALLOW" ]; then
    cat <<'EOF'
==================== MOTION SESSION CHECKLIST ====================
 1. E-STOP within reach for the whole session.
 2. The bridge was started WITH --enable-motion --acknowledge-first-motion-checklist.
 3. Scene staged; nothing fragile within the arm's reach; hands clear.
 4. When the server exits the bridge lease drops ~0.5 s later: the arm goes
    LIMP and an object held in the gripper will DROP.
 5. The agent moves autonomously without asking. Watch it.
==================================================================
EOF
    if [ -n "$RIGHT" ]; then
      echo " TWO ARMS ($ARMS): item 2 applies to BOTH bridges ($HOST:$PORT and $HOST:$PORT_RIGHT); the"
      echo " arms may move at the same time and nothing prevents them from touching each other."
    fi
    if [ -t 0 ] && [ -z "${FA_YES:-}" ]; then
      read -r -p "Enter to start the motion session, anything else to abort: " ans
      [ -z "$ans" ] || die "aborted by operator"
    elif [ -z "${FA_YES:-}" ]; then
      die "non-interactive: set FA_YES=1 to confirm the motion checklist"
    fi
  fi

  local TS; TS=$(date +%Y%m%d_%H%M%S)
  local S="$FA/sessions/${TS}_${NAME}"
  mkdir -p "$S/scratch" "$S/goal" "$S/frames" "$S/bridge"
  if [ -n "$RIGHT" ]; then mkdir -p "$S/bridge_right" "$S/frames_right"; fi
  ln -sfn "$S" "$FA/sessions/LATEST"
  # Interface variant. Default = the full interface every experiment so far used. A server
  # arg `-- --bare` selects the ablation interface (2026-09-07): terse README, and the copied
  # client docstring loses its examples of the commands that do not exist in that mode.
  local IFACE=full
  case " ${EXTRA[*]:-} " in *" --bare "*) IFACE=bare ;; esac
  # Two arms (--arms left,right): the dual-arm README of the same variant (README_interface_dual*.md).
  local README
  if [ "$IFACE" = bare ]; then
    sed -e '/robot_client.py \. deproject/d' -e '/robot_client.py \. move_delta/d' "$FA/robot_client.py" > "$S/robot_client.py"
    chmod +x "$S/robot_client.py"
    README=README_interface_bare.md
    if [ -n "$RIGHT" ]; then README=README_interface_dual_bare.md; fi
  else
    cp "$FA/robot_client.py" "$S/robot_client.py"
    README=README_interface_real.md
    if [ -n "$RIGHT" ]; then README=README_interface_dual.md; fi
  fi
  if [ -n "$RIGHT" ] && [ ! -f "$FA/$README" ]; then die "dual-arm interface README not found: $FA/$README"; fi
  cp "$FA/$README" "$S/README_interface.md"
  # The agent's cwd is the session dir, so the interpreter it is told about must be absolute.
  # (& and the s### delimiter are the only characters sed would misread in a path.)
  local pyesc="${PY//&/\\&}"; pyesc="${pyesc//#/\\#}"
  sed -i "s#<CONNECTOR_PY>#$pyesc#" "$S/README_interface.md"
  if [ -n "$SOLO" ]; then   # the right bridge's own overhead BRIO: state its served resolution (from the installed calibration JSON)
    local rw; rw=$(python3 -c "import json; c=json.load(open('$BRIDGE_DIR/acceptance/top/top_brio_calibration.json'))['camera']; print(f\"{c['width']}×{c['height']}\")" 2>/dev/null || echo "1920×1080")
    [ "$rw" = "1920×1080" ] || sed -i "s/1920×1080/$rw/" "$S/README_interface.md"
  fi
  # Harness notes (efficiency experiments, 2026-09-07). FA_HARNESS = comma-separated tokens:
  #   yield -> robot commands block; call them with a long exec yield instead of polling
  #   batch -> one tool call may run several commands and view images
  # Default (unset / none) = no section: the interface docs every experiment so far used.
  local HARNESS="${FA_HARNESS:-none}"
  # wording version 2 (2026-09-07 20:50): one paragraph; the 90 s yield is demanded on EVERY call
  # that contains a robot command, the first one included, also when several commands are batched
  # (v1 runs: singleinsert_bare_medium_yield / _yieldbatch trial 01).
  local HARNESS_VERSION=2
  if [ "$HARNESS" != none ]; then
    local has_yield=0 has_batch=0 tok
    for tok in ${HARNESS//,/ }; do
      case "$tok" in yield) has_yield=1 ;; batch) has_batch=1 ;; *) die "FA_HARNESS tokens must be yield|batch (got '$tok')" ;; esac
    done
    printf '\n## Running robot commands from your tool harness\n\n' >> "$S/README_interface.md"
    if [ "$has_yield" = 1 ]; then cat >> "$S/README_interface.md" <<'EOF'
Every `robot_client.py` call blocks until the robot has finished the command
(motion and gripper commands take 2–60 s, `frames` about 1 s). On EVERY exec
call that contains a robot command — including the very first one — pass
`yield_time_ms: 90000`, so that the call returns the final JSON in one step.
Never use a shorter yield for a call with robot commands, and never poll a
running process for their result.
EOF
    fi
    if [ "$has_batch" = 1 ]; then cat >> "$S/README_interface.md" <<'EOF'
One exec call may run several robot commands in sequence and view the resulting
images in the same call; a separate call per command or per image is not needed.
A call that runs several robot commands still needs the 90000 yield.
EOF
    fi
    printf '\n' >> "$S/README_interface.md"
  fi
  # Buffered joint programs (2026-09-14, FA_PROGRAMS=1: throwing sessions on a bridge started with
  # config/left_arm_throw.yaml): the interface section for preview_program / run_program / program_report,
  # and the server is started with --programs (it refuses to boot on a bridge without them).
  if [ "${FA_PROGRAMS:-}" = 1 ]; then cat "$FA/README_interface_programs.md" >> "$S/README_interface.md"; fi
  cp "$PROMPT" "$S/PROMPT.md"
  if [ -n "$GOAL" ]; then
    [ -d "$GOAL" ] || die "goal dir not found: $GOAL"
    cp "$GOAL"/* "$S/goal/" 2>/dev/null || true
  fi

  # carry forward the agents' accumulated rig knowledge (read-only reference) and
  # ask the session to leave what it learned. This is process meta-information, not
  # a domain hint: notes and tools are written by prior agents, never by us.
  #   FA_KNOWLEDGE = all (default) | notes | tools | none    (FA_NO_KNOWLEDGE=1 == none)
  #   FA_TOOLS_DIR = the tools store (default $FA/knowledge/tools); may not exist yet
  local KMODE="${FA_KNOWLEDGE:-all}"; [ -n "${FA_NO_KNOWLEDGE:-}" ] && KMODE=none
  local TOOLS_DIR="${FA_TOOLS_DIR:-$FA/knowledge/tools}"
  #   FA_NOTES_DIR = the notes store (RIG_NOTES.md + playbooks/); default = the global $FA/knowledge
  #   FA_KNOWLEDGE_SCOPE = rig (default: notes/tools must generalise across tasks) | task (2026-09-07:
  #                        a store for ONE task — task-specific numbers and procedures are welcome)
  local NOTES_DIR="${FA_NOTES_DIR:-$FA/knowledge}"
  local KSCOPE="${FA_KNOWLEDGE_SCOPE:-rig}"
  #   FA_KNOWLEDGE_READONLY=1 (2026-09-10): the store is copied in and the READING instructions are
  #   injected, but the paragraphs asking for a knowledge_delta / left-behind tools are NOT, and
  #   `stop` does not merge anything back. Used when a model must work from ANOTHER model's
  #   knowledge without changing it (knowledge-transfer experiment).
  local KRO="${FA_KNOWLEDGE_READONLY:-0}"; [ "$KRO" = 1 ] || KRO=0
  case "$KSCOPE" in rig|task) ;; *) die "FA_KNOWLEDGE_SCOPE must be rig|task (got '$KSCOPE')" ;; esac
  local want_notes=0 want_tools=0 ntools=0
  case "$KMODE" in
    all) want_notes=1; want_tools=1 ;;
    notes) want_notes=1 ;;
    tools) want_tools=1 ;;
    none) ;;
    *) die "FA_KNOWLEDGE must be all|notes|tools|none (got '$KMODE')" ;;
  esac
  if [ "$want_notes" = 1 ] && [ -f "$NOTES_DIR/RIG_NOTES.md" ]; then
    mkdir -p "$S/knowledge"
    cp "$NOTES_DIR/RIG_NOTES.md" "$S/knowledge/"
    if [ -d "$NOTES_DIR/playbooks" ]; then cp -r "$NOTES_DIR/playbooks" "$S/knowledge/"; fi
    if [ "$KSCOPE" = task ]; then cat >> "$S/PROMPT.md" <<'EOF'

## Prior operators' notes (optional)

`knowledge/RIG_NOTES.md` and `knowledge/playbooks/` are notes left by previous
autonomous sessions on THIS robot doing THIS SAME task. They may help, and they
may be wrong or out of date — treat them as hypotheses, not conclusions. Any
criterion on which the task's success or the robot's safety depends (is the
object really held, is a pose really reachable, did a move really finish) must
be confirmed with your own observation at least once before you rely on it.
Where the parts lie changes between sessions; sizes, heights, grasp widths,
tool orientations and procedures usually do not. Prefer your own observations
when they conflict with a note. Do NOT read anything outside this session
directory.
EOF
      if [ "$KRO" != 1 ]; then cat >> "$S/PROMPT.md" <<'EOF'

Before you stop, append to `scratch/knowledge_delta.md` what a future session
on THIS task should know, as RIG_NOTES.md table rows (fact | applies when |
evidence | confidence): object sizes and heights you measured, grasp widths and
tool orientations that worked, insertion offsets, pitfalls. If your procedure
worked, also write it as one playbook block titled `## playbook: <name>`, step
by step with the numbers you used, marking which numbers depend on where the
parts lie. Only things YOUR run actually demonstrated. If you learned nothing
new, write "no new notes".
EOF
      fi
    else cat >> "$S/PROMPT.md" <<'EOF'

## Prior operators' notes (optional)

`knowledge/RIG_NOTES.md` and `knowledge/playbooks/` are notes left by previous
autonomous sessions on THIS robot. They may help, and they may be wrong or out
of date — treat them as hypotheses, not conclusions. Any criterion on which the
task's success or the robot's safety depends (is the object really held, is a
pose really reachable, did a move really finish) must be confirmed with your own
observation at least once before you rely on it. Prefer your own observations
when they conflict with a note. Do NOT read anything outside this session
directory.
EOF
      if [ "$KRO" != 1 ]; then cat >> "$S/PROMPT.md" <<'EOF'

Before you stop, if this run taught you something an evidence-backed note could
pass to the next operator (a rig quirk, a setting that worked, a recovery that
worked), append it to `scratch/knowledge_delta.md`: rig facts as RIG_NOTES.md
table rows (fact | applies when | evidence | confidence), and any repeatable
procedure as a short playbook block titled `## playbook: <name>`. Only things
YOUR run actually demonstrated. If you learned nothing new, write "no new notes".
EOF
      fi
    fi
  fi
  if [ "$want_tools" = 1 ]; then
    mkdir -p "$S/knowledge/tools"
    if [ -d "$TOOLS_DIR" ]; then
      cp "$TOOLS_DIR"/* "$S/knowledge/tools/" 2>/dev/null || true
      ntools=$(find "$S/knowledge/tools" -maxdepth 1 -type f \( -name '*.py' -o -name '*.sh' \) | wc -l)
    fi
    cat >> "$S/PROMPT.md" <<'EOF'

## Prior operators' tools (optional)

`knowledge/tools/` holds helper scripts left by previous autonomous sessions on
THIS robot (it may be empty; `knowledge/tools/TOOLS.md` lists them). Each script
starts with a header giving its purpose, usage, inputs/outputs, assumptions and
category. They are prior agents' artifacts, not verified by us: they may be wrong
or stale. Read a script before running it, run it from this session directory,
and do not edit the originals — copy one into `scratch/` if you need to change
it. Like everything else, they act on the robot only through `robot_client.py`.
EOF
    if [ "$KRO" != 1 ]; then
    if [ "$KSCOPE" = task ]; then cat >> "$S/PROMPT.md" <<'EOF'

## Leaving tools (do this once the task is finished, before you stop)

If you wrote helper scripts during this run that a future session on THIS task
would use again (measurement, geometry, a grasp-and-check sequence, ...), leave
them behind. A script qualifies if ALL of these hold:
1. you actually ran it in this session and it worked;
2. anything that depends on where the parts lie is a parameter or is clearly
   marked as an example value, not silently baked in;
3. it contains no session names, dates, frame numbers or absolute paths.
For each script you keep: copy it to `scratch/knowledge_delta_tools/<name>.py`
(or `.sh`) and put this header at the top — plain comment lines, all seven
keys, one line each:
EOF
    else cat >> "$S/PROMPT.md" <<'EOF'

## Leaving tools (do this once the task is finished, before you stop)

If you wrote helper scripts during this run, judge each one STRICTLY for
generality. Leave a script behind only if ALL of these hold:
1. you actually ran it in this session and it worked;
2. it contains nothing specific to this session's scene, goal, object positions,
   colours or images — no scene-specific numbers or file names baked in;
3. it would plausibly help a future session on this robot doing a DIFFERENT task.
Scripts that only made sense for this scene must NOT be left. For each script you
keep: copy it to `scratch/knowledge_delta_tools/<name>.py` (or `.sh`) and put
this header at the top — plain comment lines, all seven keys, one line each:
EOF
    fi
    cat >> "$S/PROMPT.md" <<'EOF'
    # tool: <name>.py
    # category: process | geometry | task
    # purpose: <what it does>
    # usage: python3 knowledge/tools/<name>.py <args>    (run from the session directory)
    # inputs/outputs: <what it reads; what it prints or writes>
    # assumptions: <hard-coded values, formats, sizes, tolerances — or "none">
    # verified: used successfully in the session that wrote it
Do not cite session names, dates or frame numbers anywhere in a tool: later
sessions cannot see them. Then add to `scratch/knowledge_delta.md` one section
`## tool: <name>.py` containing a single sentence on why the script is general.
If you leave no tools, add the line `## tools: none`.
EOF
    fi
  fi

  # ---- experience checkpoints (2026-09-10): the multiple-experience method's rolling-checkpoint scheme,
  # ported in tools/experience.py. Selected by FA_EXPERIENCE_STORE alone; FA_KNOWLEDGE is
  # 'none' in this mode, so the notes / tools paths above are not entered at all.
  if [ -n "${FA_EXPERIENCE_STORE:-}" ]; then
    cp "$FA/tools/experience.py" "$S/experience.py"
    python3 "$FA/tools/experience.py" install "$FA_EXPERIENCE_STORE" "$S" "${FA_EXPERIENCE_CYCLE:-1}" \
      | sed 's/^/[agp] experience: /'
    printf '\n' >> "$S/PROMPT.md"
    cat >> "$S/PROMPT.md" <<'EOFEXP'
## Experience checkpoints shared between cycles

A fresh agent process and context runs each cycle of this task. Agents share
persistent experience and validated artifacts through immutable checkpoints.
EOFEXP
    if [ "$KRO" = 1 ]; then cat >> "$S/PROMPT.md" <<'EOFEXP'
You are handed the experience earlier cycles accumulated. Reuse the verified
procedure with freshly observed object poses.
EOFEXP
    else cat >> "$S/PROMPT.md" <<'EOFEXP'
Cycle 1 starts with empty task experience and establishes a successful procedure.
On later cycles reuse the verified procedure with freshly observed object poses.
EOFEXP
    fi
    cat >> "$S/PROMPT.md" <<'EOFEXP'
Avoid repeating solved geometry analysis. Prefer compact parameterized scripts with
clear inputs for the current object poses and safe clearances. Keep the same motion
limits. Efficiency should come from fewer avoidable analysis steps, captures,
corrections and moves while preserving outcome verification.

Read the checkpoint you were handed before you plan anything:

    python3 experience.py show

`experience/cycle_NN/memory.json` is each immutable checkpoint committed so far (none on
cycle 1), the highest NN being the latest, and `experience/cycle_NN/artifacts/` holds the exact
files that checkpoint validated. `experience/assignment.json` gives your cycle number.
EOFEXP
    if [ "$KRO" != 1 ]; then cat >> "$S/PROMPT.md" <<'EOFEXP'

Once the task is finished and verified, save your own checkpoint before you stop:

    python3 experience.py save '{"cycle": N,
      "lesson": {"geometry": "...", "recipe": "...", "corrections": "...", "next_cycle": "..."},
      "artifacts": ["scratch/your_actual_successful_script.py", "scratch/your_measurements.json"]}'

All four lesson fields are required and the artifact list must name at least one real
file under `scratch/` that you actually used; replace the example paths with your own.
Keep the checkpoint concise and self-contained. Carry forward useful geometry,
procedure and corrections from the previous checkpoint, updating them with current
evidence, and include the dependencies needed to execute the saved scripts. The files
you name are snapshotted with their hashes, so the procedure is preserved exactly as
used even if you edit your working copies afterwards.

geometry may include uncertainty. recipe must distinguish commands from confirmed
physical results. corrections must include unsuccessful alternatives rather than
calling them successful reusable actions. next_cycle must say which earlier version
you used, which current observations validated reuse and what changed. The field
available_experience_version identifies the prior checkpoint. A checkpoint records
your evidence-based assessment. It does not automatically certify a physical result.
EOFEXP
    fi
    cat >> "$S/PROMPT.md" <<'EOFEXP'

Experience is scoped to this task and these repeats. Read the latest checkpoint and
its artifacts for prior task knowledge; use fresh observations to establish the
current object poses. Do NOT read anything outside this session directory.
EOFEXP
  fi

  # ---- multiple-experience memory (2026-09-11, FA_MX_STORE): the multiple-experience method with
  # every cycle in its own session (agp/mx/). Before the server starts, the session receives the
  # memory state the last committed cycle left (all checkpoints, the shared scratch, the stage records),
  # session.json switches the server's episode / experience commands on, and the prompt / interface get
  # its memory-related instructions. FA_KNOWLEDGE is 'none' in this mode (no notes / tools sections).
  if [ -n "${FA_MX_STORE:-}" ] && [ "${FA_MX_READONLY:-}" = 1 ]; then
    # read-only reuse (--knowledge mx-ro, 2026-09-11): another model gets every checkpoint and the shared
    # scratch a finished sequence left, a fresh stage record (its own assembly = cycle 1), commits refused,
    # nothing saved back on stop
    $PY "$FA/mx/carry.py" install-ro "$FA_MX_STORE" "$S" | sed 's/^/[agp] mx memory: /'
    $PY "$FA/mx/prompt.py" session-json "$S" 1 "$PROMPT" "$S/goal" --read-only | sed 's/^/[agp] mx session.json: /'
    $PY "$FA/mx/prompt.py" prompt-ro "$S" >> "$S/PROMPT.md"
    $PY "$FA/mx/prompt.py" readme-ro >> "$S/README_interface.md"
  elif [ -n "${FA_MX_STORE:-}" ]; then
    local MXC="${FA_MX_CYCLE:-1}"
    $PY "$FA/mx/carry.py" install "$FA_MX_STORE" "$S" "$MXC" | sed 's/^/[agp] mx memory: /'
    $PY "$FA/mx/prompt.py" session-json "$S" "$MXC" "$PROMPT" "$S/goal" | sed 's/^/[agp] mx session.json: /'
    $PY "$FA/mx/prompt.py" prompt "$MXC" >> "$S/PROMPT.md"
    $PY "$FA/mx/prompt.py" readme >> "$S/README_interface.md"
  fi

  [ "$VIDEO" = 1 ] && start_recorder "$S"

  # (4) server
  # exec inside the subshell with ALL fds redirected: the server must not inherit
  # the caller's stdout/stderr pipe (a launcher run through a pipeline would
  # otherwise never reach EOF until the server exits).
  ( cd "$S" && exec nohup $PY "$FA/server_real.py" --session "$S" ${SOLO:+--arm right --standalone} --host "$HOST" --port "$PORT" \
        --record-dir "$S/record" ${ALLOW:+--allow-motion} ${FA_PROGRAMS:+--programs} "${EXTRA[@]}" < /dev/null > "$S/server_boot.log" 2>&1 ) &
  echo $! > "$S/server.pid"
  echo "[agp] server pid $(cat "$S/server.pid"); waiting for READY ..."
  if [ -n "$RIGHT" ]; then
    # the right arm's server: same session dir, its own bridge port, record dir, boot log and pid;
    # the same EXTRA server args (e.g. --bare) and the same motion switch as the left server.
    ( cd "$S" && exec nohup $PY "$FA/server_real.py" --session "$S" --arm right --host "$HOST" --port "$PORT_RIGHT" \
          --record-dir "$S/record_right" ${ALLOW:+--allow-motion} "${EXTRA[@]}" < /dev/null > "$S/server_right_boot.log" 2>&1 ) &
    echo $! > "$S/server_right.pid"
    echo "[agp] right server pid $(cat "$S/server_right.pid"); waiting for READY ..."
  fi
  local ready="" i
  for i in $(seq 1 90); do
    if grep -q '^READY' "$S/server_boot.log" 2>/dev/null; then ready=1; break; fi
    if grep -qE '^BOOT_ERROR|Traceback' "$S/server_boot.log" 2>/dev/null; then break; fi
    sleep 2
  done
  if [ -z "$ready" ]; then
    echo "[agp] server did not become READY:" >&2; tail -20 "$S/server_boot.log" >&2
    if [ -n "$RIGHT" ]; then stop_servers "$S"; fi
    stop_recorder "$S"
    exit 3
  fi
  local rl; rl=$(grep -m1 '^READY' "$S/server_boot.log")
  echo "[agp] $rl"
  local rlr=""
  if [ -n "$RIGHT" ]; then
    local ready_r=""
    for i in $(seq 1 90); do
      if grep -q '^READY' "$S/server_right_boot.log" 2>/dev/null; then ready_r=1; break; fi
      if grep -qE '^BOOT_ERROR|Traceback' "$S/server_right_boot.log" 2>/dev/null; then break; fi
      sleep 2
    done
    if [ -z "$ready_r" ]; then
      echo "[agp] right server did not become READY (stopping the left one too):" >&2; tail -20 "$S/server_right_boot.log" >&2
      stop_servers "$S"
      stop_recorder "$S"
      exit 3
    fi
    rlr=$(grep -m1 '^READY' "$S/server_right_boot.log")
    echo "[agp] right: $rlr"
  fi
  cat > "$S/session.env" <<EOF
SESSION=$S
MODE=$([ -n "$ALLOW" ] && echo motion || echo observation-only)
BRIDGE=$HOST:$PORT
ARMS=$ARMS
PROMPT=$PROMPT
GOAL=$GOAL
RECORD_CAM=$([ "$VIDEO" = 1 ] && echo "$CAM" || echo none)
READY_LINE=$rl
KNOWLEDGE_MODE=$KMODE
KNOWLEDGE_SCOPE=$KSCOPE
KNOWLEDGE_READONLY=$KRO
EXPERIENCE_STORE=${FA_EXPERIENCE_STORE:-}
EXPERIENCE_CYCLE=${FA_EXPERIENCE_CYCLE:-}
MX_STORE=${FA_MX_STORE:-}
MX_CYCLE=${FA_MX_CYCLE:-}
MX_READONLY=${FA_MX_READONLY:-}
PROGRAMS=${FA_PROGRAMS:-}
NOTES_DIR=$NOTES_DIR
INTERFACE=$IFACE
HARNESS=$HARNESS
HARNESS_VERSION=$([ "$HARNESS" != none ] && echo "$HARNESS_VERSION" || echo 0)
TOOLS_DIR=$TOOLS_DIR
TOOLS_INHERITED=$ntools
EOF
  if [ -n "$RIGHT" ]; then
    { echo "BRIDGE_RIGHT=$HOST:$PORT_RIGHT"; echo "READY_LINE_RIGHT=$rlr"; } >> "$S/session.env"
  fi
  cat <<EOF

[agp] session ready: $S
  quick check :  cd "$S" && python3 robot_client.py . status
  live log    :  tail -f "$S/server.log"
  video still :  $S/snapshot.png

  agent launch: this script never launches the agent. Use run_experiment.sh
  (backend agy = Google Antigravity by default, --backend codex for OpenAI codex;
  it also records all three views), or run the backend CLI yourself from "$S"
  with PROMPT.md as the prompt, writing agent_events.jsonl / agent_stderr.log
  and the pid to agent.pid (see the header of run_experiment.sh for the exact
  per-backend command lines).

  stop        :  $0 stop "$S"
EOF
  if [ -n "$RIGHT" ]; then
    echo "  right arm   :  cd \"$S\" && python3 robot_client.py . --arm right status   (log: server_right.log)"
    echo
  fi
}

bridge_pid() {  # [$1 = port, default $PORT] pid of the process listening on the bridge port, or empty
  ss -tlnp 2>/dev/null | awk -v p=":${1:-$PORT}" '$4 ~ p"$" {print $NF}' | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2
}

wait_retime_done() {  # $1 = bridge pid (may be empty), $2 = that bridge's recording dir for this session
  # The bridge writes recording_stopped.txt BEFORE it re-times slow streams (ffmpeg -itsscale -c copy ->
  # <cam>.retimed.mp4, then replaces <cam>.mp4; the wrist D405 stream is re-timed on every session). Moving
  # <cam>.mp4 while that copy runs would keep the un-retimed (2x fast) file, so wait for it (bounded, 30 s).
  local i
  for i in $(seq 1 300); do
    [ -e "$2/top.retimed.mp4" ] || [ -e "$2/wrist.retimed.mp4" ] \
      || { [ -n "$1" ] && pgrep -P "$1" -x ffmpeg >/dev/null 2>&1; } || return 0
    sleep 0.1
  done
  echo "[agp] WARN: the bridge is still re-timing $2 after 30 s; collecting the files anyway"
}

# rec-start / rec-stop: the three views together. Side view = ffmpeg on the third
# BRIO (this script); top + wrist = the bridge's own recorder (bridge started with
# --record-dir $BRIDGE_REC_DIR; SIGUSR1 starts, SIGUSR2 stops; target.txt names the
# output dir so the files land straight in the session).
BRIDGE_REC_DIR="${FA_BRIDGE_REC_DIR:-$FA/bridge_recordings}"
BRIDGE_REC_DIR_RIGHT="${FA_BRIDGE_REC_DIR_RIGHT:-$FA/bridge_recordings_right}"   # the right bridge's --record-dir

cmd_rec_start() {
  local S; S=$(resolve_session "${1:-}")
  solo_env "$S"
  if [ -n "$SOLO" ]; then echo "[agp] standalone right session: no side video (that camera belongs to the left rig)"; else start_recorder "$S"; fi
  local pid; pid=$(bridge_pid || true)     # empty -> WARN branch below (grep's rc 1 must not abort the script)
  if [ -n "$pid" ] && [ -d "$BRIDGE_REC_DIR" ]; then
    mkdir -p "$S/bridge_rec"
    echo "$S/bridge_rec" > "$BRIDGE_REC_DIR/target.txt"
    kill -USR1 "$pid" && echo "[agp] bridge recording requested (pid $pid) -> $S/bridge_rec"
    for _ in $(seq 1 20); do [ -f "$S/bridge_rec/recording_started.txt" ] && break; sleep 0.25; done
    if [ ! -f "$S/bridge_rec/recording_started.txt" ] && pgrep -P "$pid" ffmpeg >/dev/null 2>&1; then
      # 2026-09-10: the bridge is still recording for an earlier session that was aborted/deleted
      # without rec-stop; the recorder ignores a start while active. Stop that stale recording, retry.
      echo "[agp] bridge recorder still active for an earlier session -> stopping it and retrying"
      kill -USR2 "$pid"; for _ in $(seq 1 70); do pgrep -P "$pid" ffmpeg >/dev/null 2>&1 || break; sleep 0.5; done
      echo "$S/bridge_rec" > "$BRIDGE_REC_DIR/target.txt"
      kill -USR1 "$pid"; for _ in $(seq 1 20); do [ -f "$S/bridge_rec/recording_started.txt" ] && break; sleep 0.25; done
    fi
    [ -f "$S/bridge_rec/recording_started.txt" ] && echo "[agp] bridge recording started" \
      || echo "[agp] WARN: bridge did not confirm recording (started without --record-dir?)"
  else
    echo "[agp] WARN: no bridge pid or $BRIDGE_REC_DIR missing -> top/wrist not recorded by the bridge"
  fi
  if session_has_right "$S"; then   # dual session: the right bridge's recorder too (-> bridge_rec_right/)
    local rpid; rpid=$(bridge_pid "$PORT_RIGHT" || true)
    if [ -n "$rpid" ] && [ -d "$BRIDGE_REC_DIR_RIGHT" ]; then
      mkdir -p "$S/bridge_rec_right"
      echo "$S/bridge_rec_right" > "$BRIDGE_REC_DIR_RIGHT/target.txt"
      kill -USR1 "$rpid" && echo "[agp] right bridge recording requested (pid $rpid) -> $S/bridge_rec_right"
      for _ in $(seq 1 20); do [ -f "$S/bridge_rec_right/recording_started.txt" ] && break; sleep 0.25; done
      [ -f "$S/bridge_rec_right/recording_started.txt" ] && echo "[agp] right bridge recording started" \
        || echo "[agp] WARN: right bridge did not confirm recording (started without --record-dir?)"
    else
      echo "[agp] WARN: no right bridge pid on $PORT_RIGHT or $BRIDGE_REC_DIR_RIGHT missing -> right top/wrist not recorded by the bridge"
    fi
  fi
  date +%s > "$S/rec_start_s"
}

cmd_rec_stop() {
  local S; S=$(resolve_session "${1:-}")
  solo_env "$S"
  local pid; pid=$(bridge_pid || true)     # empty -> skip the left collection (must not abort before the right bridge / stop_recorder)
  # dual session: signal the right bridge right away so both recordings end at the same instant;
  # its files are collected after the left ones below
  local rpid=""
  if session_has_right "$S"; then rpid=$(bridge_pid "$PORT_RIGHT" || true); fi
  if [ -n "$rpid" ] && [ -d "$S/bridge_rec_right" ]; then kill -USR2 "$rpid" 2>/dev/null || true; fi
  if [ -n "$pid" ] && [ -d "$S/bridge_rec" ]; then
    kill -USR2 "$pid" 2>/dev/null || true
    for _ in $(seq 1 120); do [ -f "$S/bridge_rec/recording_stopped.txt" ] && break; sleep 0.5; done
    wait_retime_done "$pid" "$S/bridge_rec"
    rm -f "$BRIDGE_REC_DIR/target.txt"
    local cam
    for cam in top wrist; do
      if [ -s "$S/bridge_rec/$cam.mp4" ]; then
        mv "$S/bridge_rec/$cam.mp4" "$S/run_video_$cam.mp4"
        echo "[agp] video: $S/run_video_$cam.mp4 ($cam, bridge recorder) $(ffprobe -v error -show_entries format=duration -of csv=p=0 "$S/run_video_$cam.mp4" 2>/dev/null)s"
      fi
      # per-frame capture timestamps of that video (bridge >= 2026-09-04)
      [ -s "$S/bridge_rec/${cam}_frames.csv" ] && mv "$S/bridge_rec/${cam}_frames.csv" "$S/run_video_${cam}_frames.csv"
    done
    # 50 Hz arm state over the recording window (bridge >= 2026-09-04): dataset side data
    if [ -s "$S/bridge_rec/joints.csv" ]; then
      mv "$S/bridge_rec/joints.csv" "$S/run_joints.csv"
      echo "[agp] joints: $S/run_joints.csv ($(( $(wc -l < "$S/run_joints.csv") - 1 )) rows @50 Hz)"
    fi
  fi
  if [ -n "$rpid" ] && [ -d "$S/bridge_rec_right" ]; then
    for _ in $(seq 1 120); do [ -f "$S/bridge_rec_right/recording_stopped.txt" ] && break; sleep 0.5; done
    wait_retime_done "$rpid" "$S/bridge_rec_right"
    rm -f "$BRIDGE_REC_DIR_RIGHT/target.txt"
    # Only the right WRIST video is kept. The right bridge's own top BRIO (B8C7F203, 640x360) is
    # recorded unavoidably (the bridge schema requires a top camera) but the agents never see it
    # and the operator has the left 1080p top video: drop it (user decision 2026-09-08).
    rm -f "$S/bridge_rec_right/top.mp4" "$S/bridge_rec_right/top_frames.csv"
    local rcam
    for rcam in wrist; do
      if [ -s "$S/bridge_rec_right/$rcam.mp4" ]; then
        mv "$S/bridge_rec_right/$rcam.mp4" "$S/run_video_right_$rcam.mp4"
        echo "[agp] video: $S/run_video_right_$rcam.mp4 (right $rcam, bridge recorder) $(ffprobe -v error -show_entries format=duration -of csv=p=0 "$S/run_video_right_$rcam.mp4" 2>/dev/null)s"
      fi
      [ -s "$S/bridge_rec_right/${rcam}_frames.csv" ] && mv "$S/bridge_rec_right/${rcam}_frames.csv" "$S/run_video_right_${rcam}_frames.csv"
    done
    if [ -s "$S/bridge_rec_right/joints.csv" ]; then
      mv "$S/bridge_rec_right/joints.csv" "$S/run_joints_right.csv"
      echo "[agp] joints: $S/run_joints_right.csv ($(( $(wc -l < "$S/run_joints_right.csv") - 1 )) rows @50 Hz, right arm)"
    fi
  fi
  stop_recorder "$S"     # side view + timelapse fallback for any view the bridge did not deliver
  date +%s > "$S/rec_stop_s"
}

resolve_session() {
  local S="${1:-$FA/sessions/LATEST}"
  S=$(readlink -f "$S")
  [ -d "$S" ] || die "no session dir: $S"
  echo "$S"
}

cmd_stop() {
  local S; S=$(resolve_session "${1:-}")
  solo_env "$S"
  echo "[agp] stopping session $S"
  # right arm (dual sessions): note whether its server is alive BEFORE the shared sentinel is touched
  local right_alive=""
  if [ -f "$S/server_right.pid" ] && kill -0 "$(cat "$S/server_right.pid")" 2>/dev/null; then right_alive=1; fi
  if [ -f "$S/server.pid" ] && kill -0 "$(cat "$S/server.pid")" 2>/dev/null; then
    touch "$S/SERVER_STOP"
    local pid; pid=$(cat "$S/server.pid")
    for _ in $(seq 1 60); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    if kill -0 "$pid" 2>/dev/null; then echo "[agp] server still alive after 30 s, TERM"; kill -TERM "$pid" || true; fi
    grep -q motion "$S/session.env" 2>/dev/null && grep -q '^MODE=motion' "$S/session.env" && \
      echo "[agp] NOTE: motion session closed -> the arm goes limp ~0.5 s after the lease drops"
  else
    echo "[agp] server not running"
  fi
  # right arm (sessions started with --arms left,right): SERVER_STOP is the shared sentinel, so when the
  # left server was running the right one is normally already gone by now; wait for it all the same
  if [ -f "$S/server_right.pid" ]; then
    if [ -n "$right_alive" ]; then
      touch "$S/SERVER_STOP"
      local rpid; rpid=$(cat "$S/server_right.pid")
      for _ in $(seq 1 60); do kill -0 "$rpid" 2>/dev/null || break; sleep 0.5; done
      if kill -0 "$rpid" 2>/dev/null; then echo "[agp] right server still alive after 30 s, TERM"; kill -TERM "$rpid" || true; fi
      echo "[agp] right server stopped (shared SERVER_STOP)"
      grep -q '^MODE=motion' "$S/session.env" 2>/dev/null && \
        echo "[agp] NOTE: motion session closed -> the RIGHT arm goes limp ~0.5 s after its lease drops"
    else
      echo "[agp] right server not running"
    fi
  fi
  stop_recorder "$S"
  merge_knowledge "$S"
  echo "[agp] artifacts:"; ls -la "$S" | sed 's/^/  /'
  echo "  videos: $(ls "$S"/run_video_*.mp4 2>/dev/null | xargs -r -n1 basename | tr '\n' ' ')"
  echo "  frames: $(ls "$S/frames" 2>/dev/null | wc -l) files; record: $(ls "$S/record"/*/*.jpg 2>/dev/null | wc -l) jpgs; commands: $(ls "$S/bridge" 2>/dev/null | grep -c '^resp_' || true)"
  if [ -d "$S/bridge_right" ]; then
    echo "  right : frames_right: $(ls "$S/frames_right" 2>/dev/null | wc -l) files; record_right: $(ls "$S/record_right"/*/*.jpg 2>/dev/null | wc -l) jpgs; commands: $(ls "$S/bridge_right" 2>/dev/null | grep -c '^resp_' || true)"
  fi
}

merge_knowledge() {  # $1 = session (or offline consolidation workspace) with scratch/knowledge_delta.md + session.env
  local S="$1"
  # merge the agent's knowledge delta back into the store (mechanical append; we
  # never edit its content — dedup / obsolete marking is the next agent's job).
  # What is merged depends on the mode the session was STARTED with (session.env):
  #   notes|all -> facts to RIG_NOTES.md, '## playbook:' sections to playbooks/
  #   tools|all -> '## tool: <file>' sections: the file from scratch/knowledge_delta_tools/
  #                is linted (header keys, category, no gap imports / absolute paths /
  #                session or frame references) and copied into the tools store, never
  #                overwriting (name_v2, _v3 ...); TOOLS.md index is regenerated.
  # multiple-experience sessions (FA_MX_STORE): the server already committed this cycle's checkpoint in the
  # session; carry it and the session's scratch / stage records into the batch store (only if committed)
  local mxstore; mxstore=$(grep -m1 '^MX_STORE=' "$S/session.env" 2>/dev/null | cut -d= -f2-)
  if [ -n "$mxstore" ] && grep -qx 'MX_READONLY=1' "$S/session.env" 2>/dev/null; then
    echo "[agp] mx memory: read-only session, store left unchanged ($mxstore)"; return 0
  fi
  if [ -n "$mxstore" ]; then
    local mxcycle; mxcycle=$(grep -m1 '^MX_CYCLE=' "$S/session.env" 2>/dev/null | cut -d= -f2-)
    $PY "$FA/mx/carry.py" save "$mxstore" "$S" "$mxcycle" | sed 's/^/[agp] mx memory: /'
    return 0
  fi
  # experience-checkpoint sessions (FA_EXPERIENCE_STORE): commit the staged checkpoint into the
  # immutable batch store instead of the append-only notes/tools merge below
  local estore; estore=$(grep -m1 '^EXPERIENCE_STORE=' "$S/session.env" 2>/dev/null | cut -d= -f2-)
  if [ -n "$estore" ]; then
    if grep -qx 'KNOWLEDGE_READONLY=1' "$S/session.env" 2>/dev/null; then
      echo "[agp] experience: read-only session, store left unchanged ($estore)"
    else
      python3 "$FA/tools/experience.py" commit "$estore" "$S" | sed 's/^/[agp] experience: /'
    fi
    return 0
  fi
  local delta="$S/scratch/knowledge_delta.md"
  local kmode tdir ndir
  kmode=$(grep -m1 '^KNOWLEDGE_MODE=' "$S/session.env" 2>/dev/null | cut -d= -f2-)
  tdir=$(grep -m1 '^TOOLS_DIR=' "$S/session.env" 2>/dev/null | cut -d= -f2-)
  ndir=$(grep -m1 '^NOTES_DIR=' "$S/session.env" 2>/dev/null | cut -d= -f2-)
  if [ -z "$kmode" ]; then [ -f "$S/knowledge/RIG_NOTES.md" ] && kmode=notes || kmode=none; fi
  # read-only knowledge session (FA_KNOWLEDGE_READONLY=1): the store was used, never written back
  if grep -qx 'KNOWLEDGE_READONLY=1' "$S/session.env" 2>/dev/null; then
    echo "[agp] knowledge: read-only session, store left unchanged ($ndir)"; return 0
  fi
  [ -n "$tdir" ] || tdir="$FA/knowledge/tools"
  [ -n "$ndir" ] || ndir="$FA/knowledge"
  if [ "$kmode" != none ] && [ -f "$delta" ] && [ -s "$delta" ]; then
    mkdir -p "$ndir"
    python3 - "$delta" "$ndir" "$(basename "$S")" "$kmode" "$tdir" <<'PY'
import re, sys, os, datetime, shutil
delta, kdir, sess, kmode, tdir = sys.argv[1:6]
want_notes = kmode in ("all", "notes"); want_tools = kmode in ("all", "tools")
text = open(delta, encoding="utf-8").read()
stamp = f"<!-- from session {sess}, merged {datetime.date.today()} -->"
pieces = re.split(r"(?m)^## (playbook|tool|tools): *(.*)$", text)
facts = pieces[0]
sections = [(pieces[i], pieces[i + 1].strip(), pieces[i + 2]) for i in range(1, len(pieces), 3)]
if want_notes:
    if facts.strip() and not re.match(r"(?is)\s*(#[^\n]*\n\s*)*no new notes", facts):
        with open(os.path.join(kdir, "RIG_NOTES.md"), "a", encoding="utf-8") as f:
            f.write("\n" + stamp + "\n" + facts.strip("\n") + "\n")
        print(f"[agp] knowledge: facts appended to RIG_NOTES.md ({facts.count(chr(10))} lines)")
    os.makedirs(os.path.join(kdir, "playbooks"), exist_ok=True)
    for kind, title, body in sections:
        if kind != "playbook":
            continue
        body = body.strip("\n")
        slug = re.sub(r"[^a-z0-9]+", "_", title.lower()).strip("_") or "playbook"
        path = os.path.join(kdir, "playbooks", slug + ".md")
        new = not os.path.exists(path)
        with open(path, "a", encoding="utf-8") as f:
            f.write((f"# playbook: {title}\n\n" if new else f"\n## addendum ({sess})\n\n") + stamp + "\n" + body + "\n")
        print(f"[agp] knowledge: playbook '{title}' -> playbooks/{slug}.md ({'new' if new else 'addendum'})")

KEYS = ["tool", "category", "purpose", "usage", "inputs/outputs", "assumptions", "verified"]
def header(src):
    head = "\n".join(src.splitlines()[:40])
    out = {}
    for k in KEYS:
        m = re.search(r"(?im)^\s*#\s*" + re.escape(k) + r"\s*:\s*(.*)$", head)
        if m:
            out[k] = m.group(1).strip()
    return out, head

def lint(path):
    if not os.path.isfile(path):
        return False, "file missing in scratch/knowledge_delta_tools/"
    if not path.endswith((".py", ".sh")):
        return False, "only .py / .sh are accepted"
    if os.path.getsize(path) > 32768:
        return False, "larger than 32 KB"
    src = open(path, encoding="utf-8", errors="replace").read()
    h, head = header(src)
    missing = [k for k in KEYS if k not in h]
    if missing:
        return False, "header missing: " + ", ".join(missing)
    if h["category"].split()[0].lower() not in ("process", "geometry", "task"):
        return False, "category must be process | geometry | task"
    # "imports gap": the vendored connector package is literally named `gap`; a tool may talk to
    # the robot only through robot_client.py, never by importing the connector itself.
    checks = [("imports gap", r"(?m)^\s*(from|import)\s+gap\b"), ("absolute /home path", r"/home/"),
              ("session reference", r"sessions/\d{8}_"),
              ("bridge internals", r"yam_real_env|hardware-bridge|agp_yam_bridge")]
    for label, pat in checks:
        if re.search(pat, src):
            return False, label
    if re.search(r"frames/\d{4}", head):
        return False, "frame number in the header"
    return True, h["category"].split()[0].lower()

if want_tools:
    src_dir = os.path.join(os.path.dirname(delta), "knowledge_delta_tools")
    os.makedirs(tdir, exist_ok=True)
    kept = 0
    for kind, title, body in sections:
        if kind != "tool":
            continue
        name = os.path.basename(title.strip("`* "))
        path = os.path.join(src_dir, name)
        ok, info = lint(path)
        if not ok:
            print(f"[agp] tools: REJECTED {name}: {info}")
            continue
        content = open(path, "rb").read()
        dst = os.path.join(tdir, name)
        if os.path.exists(dst):
            if open(dst, "rb").read() == content:
                print(f"[agp] tools: {name} identical to the stored copy, skipped")
                continue
            stem, ext = os.path.splitext(name); v = 2
            while os.path.exists(os.path.join(tdir, f"{stem}_v{v}{ext}")):
                v += 1
            dst = os.path.join(tdir, f"{stem}_v{v}{ext}")
        shutil.copyfile(path, dst); os.chmod(dst, 0o755); kept += 1
        print(f"[agp] tools: kept {os.path.basename(dst)} ({info}): {body.strip()[:120]}")
    # regenerate the index from the headers (never by hand)
    rows = []
    for fn in sorted(os.listdir(tdir)):
        if not fn.endswith((".py", ".sh")):
            continue
        h, _ = header(open(os.path.join(tdir, fn), encoding="utf-8", errors="replace").read())
        rows.append(f"| `{fn}` | {h.get('category','')} | {h.get('purpose','')} | `{h.get('usage','')}` | {h.get('assumptions','')} |")
    with open(os.path.join(tdir, "TOOLS.md"), "w", encoding="utf-8") as f:
        f.write("# Tools left by previous sessions (index generated from the file headers)\n\n"
                "| file | category | purpose | usage | assumptions |\n|---|---|---|---|---|\n" + "\n".join(rows) + "\n")
    print(f"[agp] tools: {kept} kept this session; store now has {len(rows)} tool(s) in {tdir}")
PY
  fi
}

cmd_status() {
  local S; S=$(resolve_session "${1:-}")
  echo "session: $S"; cat "$S/session.env" 2>/dev/null || true
  for p in server server_right recorder agent codex; do   # server_right.pid exists in dual sessions only
    if [ -f "$S/$p.pid" ]; then
      if kill -0 "$(cat "$S/$p.pid")" 2>/dev/null; then echo "$p: running (pid $(cat "$S/$p.pid"))"; else echo "$p: exited"; fi
    fi
  done
  echo "--- server.log (tail) ---"; tail -5 "$S/server.log" 2>/dev/null || true
  if [ -f "$S/server_right.log" ]; then echo "--- server_right.log (tail) ---"; tail -5 "$S/server_right.log" 2>/dev/null || true; fi
}

case "${1:-}" in
  start) shift; cmd_start "$@" ;;
  stop) shift; cmd_stop "$@" ;;
  status) shift; cmd_status "$@" ;;
  rec-start) shift; cmd_rec_start "$@" ;;
  rec-stop) shift; cmd_rec_stop "$@" ;;
  merge) shift; merge_knowledge "$(resolve_session "${1:-}")" ;;   # merge a workspace's scratch/knowledge_delta.md (offline consolidation)
  *) usage ;;
esac
