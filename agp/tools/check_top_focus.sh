#!/usr/bin/env bash
# Refuse to proceed when the LEFT top BRIO (178B0DAE) is not in the state its 1080p calibration
# (2026-09-02) was taken in: autofocus OFF (focus_automatic_continuous=0), focus_absolute=0,
# zoom_absolute=100. The camera resets these controls on some power cycles (seen 2026-09-08);
# with autofocus on, the top-camera intrinsics/extrinsics no longer describe the images.
#
#   bash tools/check_top_focus.sh            exit 0 = ok, 3 = wrong controls, 2 = device/v4l2-ctl missing
#   FA_FIX_TOP_FOCUS=1 bash tools/check_top_focus.sh   set the controls back first, then check
#
# Reading/setting UVC controls does not need exclusive access: it is safe while a bridge streams.
#   bash tools/check_top_focus.sh --arm right   the RIGHT station's BRIO (B8C7F203): autofocus OFF, focus_absolute=10
#                                              (the value its autofocus had converged to at the table distance when it
#                                              was switched off on 2026-09-09; FA_RIGHT_TOP_FOCUS overrides), zoom 100
set -uo pipefail
ARM=left; [ "${1:-}" = "--arm" ] && ARM="${2:-left}"
if [ "$ARM" = right ]; then
  DEV="${FA_TOP_DEVICE_RIGHT:-/dev/v4l/by-id/usb-046d_Logitech_BRIO_B8C7F203-video-index0}"; WANT_FA="${FA_RIGHT_TOP_FOCUS:-10}"; LABEL="right top BRIO B8C7F203"
else
  DEV="${FA_TOP_DEVICE:-/dev/v4l/by-id/usb-046d_Logitech_BRIO_178B0DAE-video-index0}"; WANT_FA=0; LABEL="top BRIO 178B0DAE"
fi
command -v v4l2-ctl >/dev/null || { echo "[top-focus] v4l2-ctl not installed"; exit 2; }
[ -e "$DEV" ] || { echo "[top-focus] top camera device missing: $DEV"; exit 2; }
if [ "${FA_FIX_TOP_FOCUS:-0}" = 1 ]; then
  # sequential: focus_absolute is rejected (EIO) while autofocus is still on, and one failing control aborts a combined call
  v4l2-ctl -d "$DEV" --set-ctrl=focus_automatic_continuous=0 || true
  v4l2-ctl -d "$DEV" --set-ctrl=focus_absolute=$WANT_FA || true
  v4l2-ctl -d "$DEV" --set-ctrl=zoom_absolute=100 || true
fi
ctl=$(v4l2-ctl -d "$DEV" --get-ctrl=focus_automatic_continuous,focus_absolute,zoom_absolute 2>&1) || { echo "[top-focus] cannot read controls of $DEV: $ctl"; exit 2; }
af=$(echo "$ctl" | awk -F': ' '/focus_automatic_continuous/{print $2}'); fa=$(echo "$ctl" | awk -F': ' '/focus_absolute/{print $2}'); zm=$(echo "$ctl" | awk -F': ' '/zoom_absolute/{print $2}')
if [ "$af" = 0 ] && [ "$fa" = "$WANT_FA" ] && [ "$zm" = 100 ]; then
  exit 0
fi
echo "[top-focus] REFUSING: $LABEL controls are focus_automatic_continuous=$af focus_absolute=$fa zoom_absolute=$zm (calibrated with 0 / $WANT_FA / 100)."
echo "[top-focus] fix (in this order): v4l2-ctl -d $DEV --set-ctrl=focus_automatic_continuous=0 ; v4l2-ctl -d $DEV --set-ctrl=focus_absolute=$WANT_FA ; v4l2-ctl -d $DEV --set-ctrl=zoom_absolute=100   (or FA_FIX_TOP_FOCUS=1 bash tools/check_top_focus.sh${ARM:+ --arm $ARM})"
exit 3
