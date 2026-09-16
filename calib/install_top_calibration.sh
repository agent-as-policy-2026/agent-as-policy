#!/usr/bin/env bash
# Install (LEFT) or hand off (RIGHT) the SOLVED top-camera calibration.
#
#   RIG=left (default): install for OUR bridge.  DO NOT run until the top
#   calibration is solved, reviewed, and the user has decided to enable the
#   top camera; this script does NOT start the bridge.
#   RIG=right: the RIGHT rig - NOTHING is installed anywhere.  The solved
#   JSON is validated through the bridge's own loader (read-only) and a
#   hand-off note calib/out/RIGHT_RIG_HANDOFF.md is written listing the
#   produced files, their schema and the exact lines to change in the right-arm
#   bridge repo's first_acceptance.yaml.  That repo installs the files itself.
#
# What the bridge expects (read from config.py + camera.py):
#   * <arm>.yaml `top_camera.calibration_path` is resolved relative to the
#     config file and handed to agp_yam_bridge.camera.load_fixed_rgb_calibration
#     at bridge startup (OpenCvFixedRgbCamera.__init__).
#   * That loader accepts EXACTLY the JSON calibrate-top-extrinsics writes -
#     no conversion is needed.  Mandatory fields: schema_version == 1,
#     type == "fixed_rgb_camera_calibration", status == "PASS",
#     camera{name,serial,device,width,height,fps} (serial must appear in the
#     device basename; the connector client additionally demands name == "top_brio"),
#     distortion_model == "opencv_radtan", camera_matrix (3x3 pinhole),
#     distortion_coefficients (finite 5-vector), camera_to_world (rigid 4x4),
#     and metrics.validation_translation_max_m > 0.
#   * The bridge negotiates the BRIO stream profile from camera{width,height,
#     fps}, so the JSON's profile must equal the profile the capture tools
#     used.  That profile (and every serial/path) comes from the rig table in
#     calib/left_handeye_common.py - the same mapping the capture tools and
#     solve_top.sh use - never from a literal in this script.
#
# RIG=left does:
#   1. validate calib/out/top_brio_178B0DAE_calibration.json through the
#      bridge's OWN loader (the exact startup code path) + left identity checks;
#   2. copy it to hardware-bridge/acceptance/top_left/top_brio_calibration.json
#      (the path the left_arm.yaml block points at);
#   3. uncomment the top_camera block in left_arm.yaml (exact-match, idempotent);
#   4. prove left_arm.yaml loads end-to-end WITHOUT starting the bridge.
#
# DRY=1 (or --print-paths): print the resolved paths + expected profile, exit 0.
# Overridable for offline testing: LEFT_ARM_YAML, CALIB_JSON, TARGET_JSON,
# HANDOFF_MD.
set -euo pipefail

CALIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_DIR="$(dirname "$CALIB_DIR")/hardware-bridge"

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
case "$RIG" in left|right) ;; *) echo "error: RIG must be left or right, got '$RIG'" >&2; exit 1 ;; esac

cd "$HB_DIR"
# Single source of truth: RIG_* variables from the Python rig table.
eval "$(uv run --locked python "$CALIB_DIR/left_handeye_common.py" --rig "$RIG" --shell)"

CALIB_JSON="${CALIB_JSON:-$RIG_TOP_CALIBRATION_JSON}"
INTR_JSON="$RIG_TOP_INTRINSICS_JSON"
EXPECTED_SERIAL="$RIG_TOP_SERIAL"
EXPECTED_WIDTH="$RIG_TOP_WIDTH"
EXPECTED_HEIGHT="$RIG_TOP_HEIGHT"
EXPECTED_FPS="$RIG_TOP_FPS"
TARGET_JSON="${TARGET_JSON:-$RIG_INSTALL_TARGET_JSON}"
LEFT_ARM_YAML="${LEFT_ARM_YAML:-$RIG_RUNTIME_YAML}"
HANDOFF_MD="${HANDOFF_MD:-$RIG_HANDOFF_MD}"

echo "rig:                 $RIG"
echo "calibration json:    $CALIB_JSON"
echo "intrinsics json:     $INTR_JSON"
echo "expected serial:     $EXPECTED_SERIAL"
echo "expected profile:    ($EXPECTED_WIDTH, $EXPECTED_HEIGHT, $EXPECTED_FPS)"
if [ "$RIG" = "left" ]; then
    echo "install target:      $TARGET_JSON"
    echo "runtime yaml:        $LEFT_ARM_YAML"
else
    echo "install target:      (none - RIGHT rig is never installed here)"
    echo "hand-off note ->     $HANDOFF_MD"
fi
if [ "$PRINT_PATHS" = "1" ]; then
    echo "(DRY/--print-paths: nothing validated, nothing written)"
    exit 0
fi

if [ ! -f "$CALIB_JSON" ]; then
    echo "error: missing $CALIB_JSON (run RIG=$RIG solve_top.sh first)" >&2
    exit 1
fi

# 1. Validate through the bridge's own loader + rig identity checks (the
#    expected profile is the rig table's, NOT a literal).
uv run --locked python - "$CALIB_JSON" "$EXPECTED_SERIAL" "$EXPECTED_WIDTH" "$EXPECTED_HEIGHT" "$EXPECTED_FPS" "$RIG" <<'PYEOF'
import sys
from pathlib import Path
from agp_yam_bridge.camera import load_fixed_rgb_calibration

path = Path(sys.argv[1])
expected_serial = sys.argv[2]
expected_profile = (int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]))
rig = sys.argv[6]
calibration = load_fixed_rgb_calibration(path)
assert calibration.name == "top_brio", (
    f"camera name {calibration.name!r} != 'top_brio' (connector client contract)")
assert calibration.serial == expected_serial, (
    f"serial {calibration.serial!r} is not the {rig.upper()} rig's BRIO {expected_serial}")
profile = (calibration.width, calibration.height, calibration.fps)
assert profile == expected_profile, (
    f"profile {profile} != rig-table profile {expected_profile}")
print("calibration JSON validated through the bridge's own loader:")
print(f"  serial {calibration.serial}  device {calibration.device}  profile {profile}")
print(f"  camera_to_world translation ({rig} base frame): "
      f"{calibration.camera_to_world[:3, 3].round(4).tolist()}")
print(f"  held-out translation error bound: "
      f"{calibration.transform_translation_error_bound_m * 1000:.2f} mm")
PYEOF

if [ "$RIG" = "right" ]; then
    # ------------------------------------------------------- RIGHT: hand-off
    # Nothing is installed.  Write the hand-off note for the right-arm bridge repo.
    uv run --locked python - "$CALIB_JSON" "$INTR_JSON" "$RIG_WRIST_INTRINSICS_JSON" \
        "$RIG_TOP_PAIRS_DIR" "$HANDOFF_MD" "$RIG_TOP_SERIAL" "$RIG_TOP_DEVICE" \
        "$EXPECTED_WIDTH" "$EXPECTED_HEIGHT" "$EXPECTED_FPS" "$RIG_WRIST_SERIAL" \
        "$RIG_CAN_CHANNEL" "$RIG_FLANGE_BODY" "$RIG_CAMERA_BODY" <<'PYEOF'
import hashlib
import json
import sys
import time
from pathlib import Path

(calib_json, intr_json, wrist_json, pairs_dir, handoff_md, top_serial, top_device,
 width, height, fps, wrist_serial, can_channel, flange_body, camera_body) = sys.argv[1:15]
calib_json, intr_json, wrist_json = Path(calib_json), Path(intr_json), Path(wrist_json)
pairs_dir, handoff_md = Path(pairs_dir), Path(handoff_md)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else "(missing)"


calib = json.loads(calib_json.read_text(encoding="utf-8"))
metrics = calib.get("metrics", {})
camera = calib.get("camera", {})
t = calib["camera_to_world"]
translation = [round(t[i][3], 4) for i in range(3)]
pairs = sorted(pairs_dir.glob("pair*.npz")) if pairs_dir.is_dir() else []
files = [
    (calib_json, "顶部相机外参+内参合并报告：桥启动时直接加载的文件（唯一必须安装的文件）"),
    (intr_json, "顶部相机内参报告（信息用；内参已内嵌在上面的 calibration JSON 里）"),
    (wrist_json, f"右腕 D405 {wrist_serial} 出厂内参/畸变（solve 时用；信息用）"),
]
lines = []
lines.append("# 右臂机位顶部相机标定 —— 交接说明 / RIGHT RIG HAND-OFF")
lines.append("")
lines.append(f"生成时间: {time.strftime('%Y-%m-%d %H:%M:%S')}  by calib/install_top_calibration.sh (RIG=right)")
lines.append("")
lines.append("**本机不安装任何东西。** 以下文件由你自己拷进你的桥仓库（右臂桥仓库）的 hardware-bridge 并重启你的桥。")
lines.append("")
lines.append("## 1. 产出文件")
lines.append("")
lines.append("| 文件 | sha256 | 说明 |")
lines.append("|------|--------|------|")
for path, note in files:
    lines.append(f"| `{path}` | `{sha256(path)}` | {note} |")
lines.append(f"| `{pairs_dir}/` | ({len(pairs)} pair npz) | 采集原始 pair（可复算；桥不读） |")
lines.append("")
lines.append("## 2. 标定结果摘要")
lines.append("")
lines.append(f"* status: **{calib.get('status')}**  (schema_version {calib.get('schema_version')}, type `{calib.get('type')}`)")
lines.append(f"* 相机: name `{camera.get('name')}` serial `{camera.get('serial')}` device `{camera.get('device')}`"
             f" 档位 **{camera.get('width')}x{camera.get('height')}@{camera.get('fps')}** (V4L2 MJPG)")
lines.append(f"* camera_to_world 平移 (m, **世界系 = 你的右臂底座 = 桥的 world**): {translation}")
lines.append(f"* 标定 RMS: {metrics.get('calibration_translation_rms_m', float('nan')) * 1000:.2f} mm / "
             f"{metrics.get('calibration_rotation_rms_deg', float('nan')):.3f} deg; "
             f"留出验证最大误差: {metrics.get('validation_translation_max_m', float('nan')) * 1000:.2f} mm / "
             f"{metrics.get('validation_rotation_max_deg', float('nan')):.3f} deg "
             f"({calib.get('calibration_sample_count')} 标定 + {calib.get('validation_sample_count')} 验证 pair)")
lines.append(f"* 相对名义位姿 (station MJCF right_base -> top_camera): "
             f"{metrics.get('solved_from_nominal_translation_m', float('nan')) * 1000:.1f} mm / "
             f"{metrics.get('solved_from_nominal_rotation_deg', float('nan')):.2f} deg")
lines.append(f"* 手腕链: `{can_channel}`（i2rt 按通道自动施加你的 joint-4 零位偏置），station 体 `{flange_body}` -> `{camera_body}`"
             f"（我们 i2rt 副本里你的实测手眼），手腕 D405 `{wrist_serial}` 的畸变系数现读自该设备。")
lines.append("")
lines.append("## 3. JSON schema（与桥 `agp_yam_bridge.camera.load_fixed_rgb_calibration` 完全一致，无需转换）")
lines.append("")
lines.append("```")
lines.append("schema_version: 1")
lines.append("type: fixed_rgb_camera_calibration")
lines.append("status: PASS")
lines.append(f"camera: {{name: top_brio, model: Logitech BRIO, serial: {top_serial}, device: {top_device},")
lines.append(f"         width: {width}, height: {height}, fps: {fps}}}   # 桥按这三个数协商 BRIO 流档位")
lines.append("distortion_model: opencv_radtan")
lines.append("camera_matrix: 3x3 pinhole;  distortion_coefficients: 5 个有限值")
lines.append("camera_to_world: 4x4 刚体变换（世界系 = 右臂底座）")
lines.append("metrics.validation_translation_max_m > 0  (桥当作 transform_translation_error_bound_m)")
lines.append("reference_camera / checkerboard / nominal_camera_to_world / validation / thresholds: 信息用，桥不读")
lines.append("```")
lines.append("")
lines.append("已用桥自己的加载器 `load_fixed_rgb_calibration` 离线验证通过（name=top_brio、serial、"
             f"{width}x{height}@{fps}、rigid camera_to_world）。")
lines.append("")
lines.append("## 4. 你要改的两行（你的桥仓库 `hardware-bridge/config/first_acceptance.yaml` 的 `top_camera:` 块）")
lines.append("")
lines.append("先把文件拷到你的 acceptance 目录（旧的 640x360 文件请先备份，不要覆盖丢失）：")
lines.append("")
lines.append("```bash")
lines.append(f"cp {calib_json} \\")
lines.append("   <你的桥仓库>/hardware-bridge/acceptance/top/top_brio_calibration_1080p.json")
lines.append("```")
lines.append("")
lines.append("然后：")
lines.append("")
lines.append("```yaml")
lines.append("top_camera:")
lines.append("  calibration_path: ../acceptance/top/top_brio_calibration_1080p.json   # (1) 改这一行：指向新 JSON")
lines.append(f"  # (2) 分辨率不写在 yaml 里：桥从 JSON 的 camera.width/height/fps 协商 {width}x{height}@{fps}；")
lines.append("  #     确认 yaml/客户端里没有别处写死 640x360（BRIO 需在 USB3 口才能出 1080p30 MJPG）")
lines.append("  stale_after_s: 0.25")
lines.append("  max_joint_skew_s: 0.05")
lines.append("```")
lines.append("")
lines.append("（或者保留 `calibration_path` 不动、直接用新 JSON 覆盖 `acceptance/top/top_brio_calibration.json`——"
             "二选一；第 (2) 点始终成立。）")
lines.append("")
lines.append("## 5. 你那边的验证步骤")
lines.append("")
lines.append("1. 桥未启动时离线验证（在你自己的桥仓库里跑；bridge 包名按你仓库的实际名字，"
             "我们这边叫 `agp_yam_bridge`）：`cd <你的桥仓库>/hardware-bridge && uv run --locked python -c "
             "\"from pathlib import Path; from agp_yam_bridge.config import load_config; "
             "from agp_yam_bridge.camera import load_fixed_rgb_calibration; "
             "c=load_config(Path('config/first_acceptance.yaml')); "
             "print(load_fixed_rgb_calibration(c.top_camera.calibration_path).camera_to_world)\"`")
lines.append("2. 重启你的桥；`OpenCvFixedRgbCamera` 会按 JSON 档位协商，档位不符会直接拒绝启动。")
lines.append("3. 前提假设——若以下任一项以后变了，顶部外参必须重做：你的右臂底座/顶部相机位置；"
             "你的手腕手眼（station right_gripper->right_camera）；joint-4 零位偏置；BRIO 对焦/变焦"
             "（本次已锁定 autofocus=0, focus_absolute=0, zoom=100——请保持）。")
lines.append("")
handoff_md.parent.mkdir(parents=True, exist_ok=True)
handoff_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"hand-off note written: {handoff_md}")
PYEOF
    echo "done. RIGHT rig: NOTHING installed on this host; hand the files listed in $HANDOFF_MD to the right-arm bridge repo."
    exit 0
fi

# ------------------------------------------------------------ LEFT: install
# 2. Copy into place (the path the left_arm.yaml block points at).
mkdir -p "$(dirname "$TARGET_JSON")"
cp "$CALIB_JSON" "$TARGET_JSON"
echo "installed: $TARGET_JSON"

# 3. Uncomment the top_camera block in left_arm.yaml (exact match, idempotent).
uv run --locked python - "$LEFT_ARM_YAML" <<'PYEOF'
import sys
from pathlib import Path

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
commented = (
    "# top_camera:\n"
    "#   calibration_path: ../acceptance/top_left/top_brio_calibration.json\n"
    "#   stale_after_s: 0.25\n"
    "#   max_joint_skew_s: 0.05\n"
)
uncommented = (
    "top_camera:\n"
    "  calibration_path: ../acceptance/top_left/top_brio_calibration.json\n"
    "  stale_after_s: 0.25\n"
    "  max_joint_skew_s: 0.05\n"
)
if uncommented in text:
    print(f"{path}: top_camera block is already uncommented; nothing to do.")
elif commented in text:
    path.write_text(text.replace(commented, uncommented, 1), encoding="utf-8")
    print(f"{path}: top_camera block uncommented.")
else:
    sys.exit(
        f"error: {path} contains neither the expected commented-out nor the "
        "uncommented top_camera block - refusing to edit; uncomment manually."
    )
PYEOF

# 4. Prove the runtime config now loads end-to-end (path resolution +
#    calibration parse), WITHOUT starting the bridge.
uv run --locked python - "$LEFT_ARM_YAML" <<'PYEOF'
import sys
from pathlib import Path
from agp_yam_bridge.config import load_config
from agp_yam_bridge.camera import load_fixed_rgb_calibration

config = load_config(Path(sys.argv[1]))
calibration = load_fixed_rgb_calibration(config.top_camera.calibration_path)
print(f"{sys.argv[1]}: loads with top_camera.calibration_path -> "
      f"{config.top_camera.calibration_path} (serial {calibration.serial}, "
      f"{calibration.width}x{calibration.height}@{calibration.fps})")
PYEOF

echo "done. Review the left_arm.yaml diff; the bridge was NOT started."
