#!/usr/bin/env bash
# Record a HUMAN demonstration with the robot's own cameras and turn it into a goal set.
#
#   bash record_demo.sh <task>            e.g.  bash record_demo.sh towel
#
# Uses the bridge's built-in recorder (SIGUSR1/SIGUSR2, same path as a trial's rec-start)
# for the overhead BRIO (1080p, 15 fps). The recorder always captures the wrist camera too;
# the arm does not move during a demo, so that file is left in raw/ and NOT part of the
# goal set. Nothing moves: park the arm where you want it (the observe posture is fine)
# BEFORE running this; the script only takes photos and records.
#
# Flow:   Enter  -> demo_start.png (overhead, scene before the demo) + recording starts
#         you perform the demonstration under the overhead camera (hands leave the view
#         between steps if you want clean intermediate frames)
#         Enter  -> recording stops; top_camera.png (overhead, the finished state)
# Output: goal_sets/<date>_<task>/            (a later run picks the newest *_<task>)
#           demo_top.mp4        overhead video of the demonstration
#           demo_start.png      overhead still before the demonstration
#           top_camera.png      overhead still after it (= the goal state)
#           raw/                recorder side files (wrist.mp4, frame timestamps,
#                               joints.csv); not copied into sessions (top-level files only)
# Then:   bash run_trial.sh <task> 1        (prompt = PROMPT_<task>.md if it exists)
# Needs:  the left bridge up WITH --record-dir, no agp session running.
set -euo pipefail

FA="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REC_DIR="${FA_BRIDGE_REC_DIR:-$FA/bridge_recordings}"
PORT="${BRIDGE_PORT:-9021}"

[ $# -eq 1 ] && [[ "$1" =~ ^[a-z0-9]+$ ]] || { sed -n 2,24p "$0" | sed 's/^# \{0,1\}//'; exit 2; }
TASK="$1"

# ---- preflight ---------------------------------------------------------------------------
BP=$(ss -tlnp 2>/dev/null | awk -v p=":$PORT" '$4 ~ p"$" {print $NF}' | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2 || true)
[ -n "$BP" ] || { echo "[demo] left bridge is not listening on $PORT — start it first"; exit 2; }
ps -o args= -p "$BP" | grep -q -- '--record-dir' || { echo "[demo] bridge (pid $BP) runs without --record-dir: no recorder available"; exit 2; }
[ -d "$REC_DIR" ] || { echo "[demo] recorder dir $REC_DIR missing"; exit 2; }
if pgrep -af 'server_real|run_experiment|run_trial|run_paper' | grep -v -E "pgrep|record_demo" >/dev/null; then
  echo "[demo] an agp session/trial is running — it owns the recorder; wait for it to finish"; exit 2
fi
if [ -f "$REC_DIR/target.txt" ]; then   # no session is running (checked above) -> a marker left by an aborted run
  if pgrep -P "$BP" -f ffmpeg >/dev/null; then
    echo "[demo] bridge recorder still active from an aborted run (target $(cat "$REC_DIR/target.txt")) -> stopping it"
    kill -USR2 "$BP"; sleep 3
  fi
  rm -f "$REC_DIR/target.txt"; echo "[demo] removed stale $REC_DIR/target.txt"
fi
command -v ffprobe >/dev/null || { echo "[demo] ffprobe missing"; exit 2; }
bash "$FA/tools/check_top_focus.sh" || exit 2      # the demo is only usable with the calibrated focus state

# ---- goal set dir (auto-versioned so the newest *_<task> is this one) ----------------------
D=$(date +%Y%m%d); G="$FA/goal_sets/${D}_${TASK}"; k=2
while [ -e "$G" ]; do G="$FA/goal_sets/${D}_v${k}_${TASK}"; k=$((k+1)); done
RAW="$G/raw"; mkdir -p "$RAW"
echo "[demo] goal set -> $G"

recording=0
cleanup() {  # Ctrl-C while recording: stop the bridge recorder and free the marker
  if [ "$recording" = 1 ]; then kill -USR2 "$BP" 2>/dev/null || true; sleep 2; fi
  rm -f "$REC_DIR/target.txt"
}
trap cleanup INT TERM

# ---- start ------------------------------------------------------------------------------------
echo "[demo] arm parked out of the working area? scene laid out as the demo should START (hands away)?"
read -r -p "[demo] Enter = take the start photo and START recording, Ctrl-C = abort: " _
bash "$FA/capture_top.sh" "$G/demo_start.png" >/dev/null
echo "$RAW" > "$REC_DIR/target.txt"
kill -USR1 "$BP"; recording=1
for _ in $(seq 1 20); do [ -f "$RAW/recording_started.txt" ] && break; sleep 0.25; done
[ -f "$RAW/recording_started.txt" ] || { echo "[demo] bridge did not confirm the recording (see the bridge log)"; cleanup; exit 3; }
T0=$(date +%s)
echo "[demo] RECORDING (overhead) since $(date +%H:%M:%S). Perform the demonstration now."
read -r -p "[demo] Enter = STOP recording (finish the demo, hands out of view first): " _

# ---- stop ---------------------------------------------------------------------------------------
kill -USR2 "$BP"; recording=0
for _ in $(seq 1 120); do [ -f "$RAW/recording_stopped.txt" ] && break; sleep 0.5; done   # the recorder retimes the videos on stop
rm -f "$REC_DIR/target.txt"
[ -f "$RAW/recording_stopped.txt" ] || { echo "[demo] WARN: bridge did not confirm the stop; files may be incomplete"; }
T1=$(date +%s)
sleep 1
bash "$FA/capture_top.sh" "$G/top_camera.png" >/dev/null
if [ -s "$RAW/top.mp4" ]; then
  mv "$RAW/top.mp4" "$G/demo_top.mp4"
  echo "[demo] $G/demo_top.mp4  $(ffprobe -v error -show_entries format=duration -of csv=p=0 "$G/demo_top.mp4" 2>/dev/null)s  $(du -h "$G/demo_top.mp4" | cut -f1)"
else
  echo "[demo] FAILED: no overhead video from the recorder"; exit 3
fi
# the wrist video stays in raw/ (recorded unavoidably; the arm does not move during a demo)
echo "[demo] demo_start.png + top_camera.png saved; recording lasted $(( T1 - T0 )) s; raw side files in $RAW"
echo "[demo] next: lay the scene out as at the demo's start, park the arm, then   bash run_trial.sh $TASK 1"
