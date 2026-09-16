#!/usr/bin/env bash
# Solve the LEFT-arm eye-in-hand calibration from calib/out/left_pose*.npz.
#
# Reads the left D405's factory distortion (dumped by capture_left_handeye.py
# into left_d405_intrinsics.json - device-specific, never the right arm's
# values) and runs the bridge's offline calibrate-checkerboard solver under
# the hardware-bridge uv project.  Board: 22 mm squares, 9x7 inner corners
# (solver defaults --columns 9 --rows 7).
#
# Usage: bash calib/solve_left.sh   (from the repo root; the script derives
# every path from its own location, so any checkout location works)
# Exit code: 0 = PASS (3 px / 15 mm / 2 deg gates), 1 = FAIL or setup error.
set -euo pipefail

CALIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_DIR="$(dirname "$CALIB_DIR")/hardware-bridge"
INTRINSICS_JSON="$CALIB_DIR/out/left_d405_intrinsics.json"
REPORT="$CALIB_DIR/out/left_hand_eye_report.json"

if [ ! -f "$INTRINSICS_JSON" ]; then
    echo "error: missing $INTRINSICS_JSON (run capture_left_handeye.py first)" >&2
    exit 1
fi

shopt -s nullglob
captures=("$CALIB_DIR"/out/left_pose*.npz)
shopt -u nullglob
if [ "${#captures[@]}" -lt 3 ]; then
    echo "error: need at least 3 left_pose*.npz captures in $CALIB_DIR/out," \
         "found ${#captures[@]}" >&2
    exit 1
fi
echo "captures (${#captures[@]}):"
printf '  %s\n' "${captures[@]}"

cd "$HB_DIR"

# distortion model + the 5 device-specific coefficients from the json.
# Coefficients are emitted as plain fixed-point decimals (NEVER scientific
# notation: argparse rejects tokens like -9.5e-05 as unknown options).
read -r -a dist <<<"$(uv run --locked python - "$INTRINSICS_JSON" <<'PYEOF'
import json, sys
payload = json.load(open(sys.argv[1]))
assert payload["serial"] == "353322271204", f"not the left D405: {payload['serial']}"
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
echo "distortion model: $model"
echo "distortion coefficients: ${coeffs[*]}"

# self-check: every coefficient token must be a plain fixed-point float
# (parses as float, no 'e'/'E') or argparse below treats it as an option.
for c in "${coeffs[@]}"; do
    if ! [[ "$c" =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then
        echo "error: distortion coefficient '$c' is not plain fixed-point" >&2
        exit 1
    fi
done

set -x
uv run --locked agp-yam-camera-acceptance \
    --config config/left_calib.yaml \
    calibrate-checkerboard \
    --captures "${captures[@]}" \
    --square-size-m 0.022 \
    --distortion-model "$model" \
    --distortion-coefficients "${coeffs[@]}" \
    --output "$REPORT"
set +x

echo "report: $REPORT"
