#!/usr/bin/env bash
# Solve the fixed top-camera (Logitech BRIO) calibration for ONE rig.
#
#   RIG=left (default) - OUR left station, BRIO 178B0DAE:
#     step 1: calibrate-top-intrinsics  on calib/out/top_intr/{calib,val}_NN.png
#             -> calib/out/top_brio_178B0DAE_intrinsics.json
#     step 2: calibrate-top-extrinsics  on calib/out/top_pairs/pairNN.npz
#             -> calib/out/top_brio_178B0DAE_calibration.json
#             (camera_to_world in the LEFT_BASE == world frame)
#   RIG=right - the RIGHT rig, BRIO B8C7F203:
#     calib/out/top_intr_right/  -> calib/out/top_brio_B8C7F203_intrinsics.json
#     calib/out/top_pairs_right/ -> calib/out/top_brio_B8C7F203_calibration.json
#             (camera_to_world in the RIGHT base frame; wrist distortion from
#             calib/out/right_d405_intrinsics.json = the right D405 353322271910,
#             dumped live by capture_top_pairs.py --rig right; solver config
#             config/right_calib.yaml = the right wrist-serial gate + right_gripper/
#             right_camera station bodies).  NOTHING is installed here: see
#             RIG=right install_top_calibration.sh (hand-off note).
#
# The rig table lives in calib/left_handeye_common.py (RIGS); this script
# evals its `--rig NAME --shell` output, so serials/paths/profile are never
# duplicated here.  DRY=1 (or --print-paths) prints the resolved inputs and
# outputs and exits 0 without running anything.
#
# PAIR SPLIT (explicit, deterministic): pairNN.npz are sorted by filename and
# every FOURTH pair (the 4th, 8th, 12th, ...; zero-based index % 4 == 3) is
# HELD OUT as a validation capture; the rest calibrate.  Because the operator
# moves the board progressively around the table, every-4th picks spatially
# spread hold-outs.  Minimums enforced: >= 3 calibration + >= 1 validation
# (i.e. >= 4 pairs; target 12-16).
#
# The wrist distortion (inverse_brown_conrady, 5 coefficients) is read from the
# rig's <side>_d405_intrinsics.json - the dump capture_top_pairs.py (and, for
# the left, capture_left_handeye.py) writes from THAT D405 - using the same
# no-scientific-notation formatter as solve_left.sh (argparse rejects tokens
# like -9.5e-05 as unknown options).
#
# Both subcommands run under the hardware-bridge uv project with
# --config <rig solver config>: calibrate-top-intrinsics ignores the config,
# but calibrate-top-extrinsics loads it for the wrist serial gate and the
# station MJCF (nominal top-camera pose).  NOTE: the solver's nominal is
# computed relative to the MJCF body `right_base`
# (camera_acceptance._station_world_from_top_camera).  For RIG=left that body
# is 0.61 m from left_base in y, so solved_from_nominal_translation_m sits
# near 0.61 m even for a perfect solve (it only tie-breaks the board
# orientation ambiguity with weight 1e-4 and does not gate PASS/FAIL).  For
# RIG=right right_base IS the world frame, so the number is a real sanity check.
#
# Usage: [RIG=left|right] [DRY=1] bash calib/solve_top.sh [--print-paths]
#        (from the repo root; every path is derived from this script's own
#        location, so any checkout location works)
# Exit code: 0 = both steps PASS (or paths printed), 1 = FAIL or setup error.
#
# Overridable (used by the offline synthetic end-to-end check):
#   TOP_INTR_DIR TOP_PAIRS_DIR TOP_OUT_DIR WRIST_INTRINSICS_JSON WRIST_SERIAL
#   TOP_DEVICE TOP_SERIAL CONFIG
set -euo pipefail

CALIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$CALIB_DIR")"
HB_DIR="$REPO_DIR/hardware-bridge"

RIG="${RIG:-left}"
PRINT_PATHS="${DRY:-0}"
for arg in "$@"; do
    case "$arg" in
        --print-paths) PRINT_PATHS=1 ;;
        --rig=*) RIG="${arg#--rig=}" ;;
        *)
            echo "usage: [RIG=left|right] [DRY=1] $0 [--print-paths] [--rig=left|right]" >&2
            exit 1
            ;;
    esac
done
case "$RIG" in left|right|cross) ;; *) echo "error: RIG must be left, right or cross, got '$RIG'" >&2; exit 1 ;; esac
# RIG=cross (2026-09-08): right arm + LEFT top BRIO -> left top camera in RIGHT_BASE (two-arm world
# frame); the left intrinsics are reused, so step 1 is skipped when its JSON already exists.
SKIP_INTRINSICS="${SKIP_INTRINSICS:-$([ "$RIG" = cross ] && echo 1 || echo 0)}"

cd "$HB_DIR"
# Single source of truth: RIG_* variables from the Python rig table.
eval "$(uv run --locked python "$CALIB_DIR/left_handeye_common.py" --rig "$RIG" --shell)"

TOP_INTR_DIR="${TOP_INTR_DIR:-$RIG_TOP_INTR_DIR}"
TOP_PAIRS_DIR="${TOP_PAIRS_DIR:-$RIG_TOP_PAIRS_DIR}"
TOP_OUT_DIR="${TOP_OUT_DIR:-$CALIB_DIR/out}"
WRIST_INTRINSICS_JSON="${WRIST_INTRINSICS_JSON:-$RIG_WRIST_INTRINSICS_JSON}"
WRIST_SERIAL="${WRIST_SERIAL:-$RIG_WRIST_SERIAL}"
TOP_SERIAL="${TOP_SERIAL:-$RIG_TOP_SERIAL}"
TOP_DEVICE="${TOP_DEVICE:-/dev/v4l/by-id/usb-046d_Logitech_BRIO_${TOP_SERIAL}-video-index0}"
CONFIG="${CONFIG:-$RIG_SOLVER_CONFIG}"
TOP_WIDTH="$RIG_TOP_WIDTH"
TOP_HEIGHT="$RIG_TOP_HEIGHT"
TOP_FPS="$RIG_TOP_FPS"

# Output names come from the rig table (left/right: unchanged top_brio_<serial>_*.json; cross:
# top_brio_178B0DAE_in_right_base_calibration.json so the LEFT calibration is never overwritten).
INTR_OUT="${INTR_OUT:-$RIG_TOP_INTRINSICS_JSON}"
CALIB_OUT="${CALIB_OUT:-$RIG_TOP_CALIBRATION_JSON}"

echo "rig:                    $RIG"
echo "top camera:             serial $TOP_SERIAL  device $TOP_DEVICE  ${TOP_WIDTH}x${TOP_HEIGHT}@${TOP_FPS}"
echo "intrinsics images:      $TOP_INTR_DIR"
echo "pairs:                  $TOP_PAIRS_DIR"
echo "wrist intrinsics json:  $WRIST_INTRINSICS_JSON  (D405 $WRIST_SERIAL, $RIG_CAN_CHANNEL)"
echo "solver config:          $HB_DIR/$CONFIG  (station bodies $RIG_FLANGE_BODY -> $RIG_CAMERA_BODY)"
echo "intrinsics report ->    $INTR_OUT"
echo "calibration report ->   $CALIB_OUT"
if [ "$PRINT_PATHS" = "1" ]; then
    if [ ! -f "$CONFIG" ]; then
        echo "error: solver config $HB_DIR/$CONFIG does not exist" >&2
        exit 1
    fi
    echo "(DRY/--print-paths: nothing run)"
    exit 0
fi

# ---------------------------------------------------------------- images ---
shopt -s nullglob
# stability pre-filter: the solver aborts on any single detection miss (EXHAUSTIVE
# detector is flaky on some real 1080p frames) -> keep only 3/3-stable views
FILTER="$CALIB_DIR/filter_stable_views.py"
mapfile -t calib_images < <(uv run --locked --project "$HB_DIR" python "$FILTER" "$TOP_INTR_DIR" calib)
mapfile -t val_images < <(uv run --locked --project "$HB_DIR" python "$FILTER" "$TOP_INTR_DIR" val)
pairs=("$TOP_PAIRS_DIR"/pair*.npz)
shopt -u nullglob

if [ "$SKIP_INTRINSICS" = 1 ]; then
    [ -f "$INTR_OUT" ] || { echo "error: SKIP_INTRINSICS=1 but $INTR_OUT does not exist (solve RIG=left first)" >&2; exit 1; }
    echo "intrinsics: REUSED $INTR_OUT (step 1 skipped)"
elif [ "${#calib_images[@]}" -lt 8 ]; then
    echo "error: need >= 8 calibration images in $TOP_INTR_DIR (target 20)," \
         "found ${#calib_images[@]} - run capture_top_intrinsics.py --rig $RIG" >&2
    exit 1
fi
if [ "${#val_images[@]}" -lt 1 ]; then
    echo "error: need >= 1 validation image in $TOP_INTR_DIR (target 5)," \
         "found ${#val_images[@]}" >&2
    exit 1
fi
if [ "${#calib_images[@]}" -lt 20 ] || [ "${#val_images[@]}" -lt 5 ]; then
    echo "warning: below target (20 calib / 5 val): have" \
         "${#calib_images[@]} calib, ${#val_images[@]} val" >&2
fi
echo "intrinsics images: ${#calib_images[@]} calibration, ${#val_images[@]} validation"

# ------------------------------------------------------------ pair split ---
if [ "${#pairs[@]}" -lt 4 ]; then
    echo "error: need >= 4 pair npz in $TOP_PAIRS_DIR (>= 3 calib + >= 1" \
         "held-out; target 12-16), found ${#pairs[@]} - run" \
         "capture_top_pairs.py --rig $RIG" >&2
    exit 1
fi
calib_pairs=()
val_pairs=()
index=0
for pair in "${pairs[@]}"; do
    if [ $((index % 4)) -eq 3 ]; then
        val_pairs+=("$pair")
    else
        calib_pairs+=("$pair")
    fi
    index=$((index + 1))
done
if [ "${#calib_pairs[@]}" -lt 3 ] || [ "${#val_pairs[@]}" -lt 1 ]; then
    echo "error: split gave ${#calib_pairs[@]} calib / ${#val_pairs[@]} val;" \
         "need >= 3 / >= 1" >&2
    exit 1
fi
echo "pairs: ${#pairs[@]} total -> ${#calib_pairs[@]} calibration:"
printf '    %s\n' "${calib_pairs[@]}"
echo "  ${#val_pairs[@]} validation (held out, every 4th by filename):"
printf '    %s\n' "${val_pairs[@]}"

# ----------------------------------------------- step 1: top intrinsics ---
if [ "$SKIP_INTRINSICS" != 1 ]; then
set -x
uv run --locked agp-yam-camera-acceptance \
    --config "$CONFIG" \
    calibrate-top-intrinsics \
    --images "${calib_images[@]}" \
    --validation-images "${val_images[@]}" \
    --device "$TOP_DEVICE" \
    --serial "$TOP_SERIAL" \
    --width "$TOP_WIDTH" --height "$TOP_HEIGHT" --fps "$TOP_FPS" \
    --columns 9 --rows 7 \
    --square-size-m 0.022 \
    --output "$INTR_OUT"
set +x
echo "top intrinsics report: $INTR_OUT"
fi

# -------------------------------------- wrist distortion (fixed-point) ---
if [ ! -f "$WRIST_INTRINSICS_JSON" ]; then
    echo "error: missing $WRIST_INTRINSICS_JSON (run capture_top_pairs.py --rig $RIG" \
         "once - it dumps the wrist D405's factory intrinsics at startup)" >&2
    exit 1
fi
# Same formatter as solve_left.sh: plain fixed-point decimals only.
read -r -a dist <<<"$(uv run --locked python - "$WRIST_INTRINSICS_JSON" "$WRIST_SERIAL" <<'PYEOF'
import json, sys
payload = json.load(open(sys.argv[1]))
expected = sys.argv[2]
assert payload["serial"] == expected, f"not the rig's wrist D405 {expected}: {payload['serial']}"
coeffs = payload["distortion_coefficients"]
assert len(coeffs) == 5, f"expected 5 distortion coefficients, got {len(coeffs)}"
def fixed(value):
    value = float(value)
    token = format(value, ".17f").rstrip("0").rstrip(".") or "0"
    assert "e" not in token and "E" not in token, token
    assert abs(float(token) - value) <= 1e-12 * max(1.0, abs(value)), token
    return token
print(payload["distortion_model"], " ".join(fixed(c) for c in coeffs))
PYEOF
)"
model="${dist[0]}"
coeffs=("${dist[@]:1}")
echo "wrist distortion model: $model"
echo "wrist distortion coefficients: ${coeffs[*]}"
if [ "$model" != "inverse_brown_conrady" ]; then
    echo "error: calibrate-top-extrinsics only supports wrist distortion" \
         "model 'inverse_brown_conrady' (or 'none'); got '$model'" >&2
    exit 1
fi
for c in "${coeffs[@]}"; do
    if ! [[ "$c" =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then
        echo "error: distortion coefficient '$c' is not plain fixed-point" >&2
        exit 1
    fi
done

# ----------------------------------------------- step 2: top extrinsics ---
set -x
uv run --locked agp-yam-camera-acceptance \
    --config "$CONFIG" \
    calibrate-top-extrinsics ${EXTR_GATES:---max-calibration-translation-rms-m 0.025 --max-validation-translation-error-m 0.035 --max-validation-rotation-error-deg 3.0} \
    --captures "${calib_pairs[@]}" \
    --validation-captures "${val_pairs[@]}" \
    --intrinsics "$INTR_OUT" \
    --wrist-distortion-model "$model" \
    --wrist-distortion-coefficients "${coeffs[@]}" \
    --output "$CALIB_OUT"
set +x

echo "top calibration report: $CALIB_OUT"
if [ "$RIG" = "cross" ]; then
    echo "next (NOT automatic): the two-arm base transform ->" \
         "CONNECTOR_VENV/python $REPO_DIR/agp/tools/solve_base_transform.py" \
         "--top-in-right-base $CALIB_OUT --check   (then without --check to write config/right_base_in_left_base.json)"
elif [ "$RIG" = "right" ]; then
    echo "next (NOT automatic): RIG=right bash $CALIB_DIR/install_top_calibration.sh" \
         "- validates the JSON through the bridge loader and writes the hand-off" \
         "note calib/out/RIGHT_RIG_HANDOFF.md; NOTHING is installed on this host."
else
    echo "next (NOT automatic): bash $CALIB_DIR/install_top_calibration.sh to" \
         "install it for the bridge (left_arm.yaml top_camera block)."
fi
