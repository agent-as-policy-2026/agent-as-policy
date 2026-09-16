"""Shared constants and kinematics helpers for hand-eye / top-camera calibration.

Everything here is offline math: no CAN, no camera.  ``capture_left_handeye.py``,
``capture_top_intrinsics.py``, ``capture_top_pairs.py`` and
``fit_joint_offsets.py`` import this module so the FK composition, the station
flange<-camera transform and the PER-RIG identity table (``RigSpec``) have
exactly one definition.

RIG TABLE (``--rig left|right`` in the capture tools, ``RIG=left|right`` for
the shell scripts): ``LEFT_RIG`` is OUR left station; ``RIGHT_RIG`` is the
right-arm rig, calibrated on this host with its bridge
stopped, whose outputs are handed to the right-arm bridge repo (never installed here).  The
left-named module constants and wrappers (``LEFT_*``, ``LeftArmFK``,
``station_left_flange_from_camera``) are kept byte-for-byte so every existing
left caller (capture_left_handeye.py, calib/analysis/) keeps
working unchanged.

Frame conventions (mirroring the bridge):
  * world == the arm's OWN base frame for BOTH rigs: the single-arm FK model's
    base sits at the origin.  (The station MJCF's root body ``left_base`` is
    at the origin and ``right_base`` at y = -0.61 m; that station offset is
    deliberately NOT applied - a right pair's ``camera_to_world`` is in the
    RIGHT base frame, exactly as the right-arm bridge reports it.)
  * ``camera_to_world`` (npz field) is ``T_world_camera`` =
    ``T_world_gripper(FK of measured joints) @ T_gripper_camera`` where the
    latter is the station MJCF body ``<side>_camera`` under ``<side>_gripper``.
  * MJCF / i2rt quaternions are wxyz; the camera frame is OpenCV convention
    (+Z optical axis).

Shell use (single source of truth for solve_top.sh / install_top_calibration.sh):
    uv run --locked python calib/left_handeye_common.py --rig right --shell
prints ``RIG_*=value`` lines for ``eval``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, fields
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]   # repo root (this file is calib/)
STATION_XML = (
    ROOT
    / "i2rt/i2rt/robot_models/station/yam_station_linear_4310_d405"
    / "yam_station_linear_4310_d405.xml"
)
CALIB_DIR = ROOT / "calib"
OUT_DIR = CALIB_DIR / "out"
HB_DIR = ROOT / "hardware-bridge"
INTRINSICS_JSON = OUT_DIR / "left_d405_intrinsics.json"

LEFT_CAMERA_SERIAL = "353322271204"
LEFT_CAN_CHANNEL = "can_follower_l"
LEFT_FLANGE_BODY = "left_gripper"
LEFT_CAMERA_BODY = "left_camera"
# A running LEFT bridge (agp-yam-bridge serve, config/left_arm.yaml) holds the
# left CAN channel and both left cameras; the standalone tools refuse to start
# while something listens here.
LEFT_BRIDGE_PORT = 9021

# The RIGHT rig (first_acceptance.yaml, read-only facts).
RIGHT_CAMERA_SERIAL = "353322271910"
RIGHT_CAN_CHANNEL = "can_follower_r"
RIGHT_FLANGE_BODY = "right_gripper"
RIGHT_CAMERA_BODY = "right_camera"
RIGHT_BRIDGE_PORT = 9020
RIGHT_INTRINSICS_JSON = OUT_DIR / "right_d405_intrinsics.json"

# Stream profile shared with the bridge's wrist pipeline (RGB8 + Z16 both
# support 640x360@30 on the D405; depth is aligned to color).
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 360
CAMERA_FPS = 30
MIN_DEPTH_M = 0.01
MAX_DEPTH_M = 1.0

# Fixed top-camera (Logitech BRIO) stream contract, identical for both rigs:
# the bridge negotiates the profile from the calibration JSON's
# camera{width,height,fps}, so the capture tools, the solver invocation and
# the install-time validation must all agree on THIS profile.
TOP_WIDTH = 1920   # 2026-09-01: raised from 640x360 (whole-table view too coarse)
TOP_HEIGHT = 1080
TOP_FPS = 30
TOP_FOURCC = "MJPG"

# Established CAD nominal T_leftgripper_leftcamera (from the station MJCF):
# translation and wxyz quaternion.  Used only as a cross-check in --dry-run;
# runtime code always re-reads the station XML.
CAD_FLANGE_CAMERA_XYZ = np.array([-0.070435, 0.0, -0.077006])
CAD_FLANGE_CAMERA_WXYZ = np.array([0.153051, 0.690345, 0.690345, 0.153048])


def _brio_device(serial: str) -> str:
    return f"/dev/v4l/by-id/usb-046d_Logitech_BRIO_{serial}-video-index0"


@dataclass(frozen=True)
class RigSpec:
    """Everything that differs between OUR left station and the
    right rig, in one place.  Paths are absolute; ``solver_config`` is
    relative to hardware-bridge (the uv project the solver runs from)."""

    side: str                    # "left" | "right"
    can_channel: str
    wrist_serial: str            # wrist D405 (pyrealsense2 serial)
    flange_body: str             # station MJCF flange body (<side>_gripper)
    camera_body: str             # station MJCF wrist-camera body (<side>_camera)
    wrist_intrinsics_json: Path  # factory D405 dump written by the pair tool
    top_serial: str              # fixed BRIO serial
    top_device: str              # stable /dev/v4l/by-id path (serial must appear)
    top_intr_dir: Path           # calib_NN.png / val_NN.png
    top_pairs_dir: Path          # pairNN.npz (+ frames/ sidecars)
    top_intrinsics_json: Path    # calibrate-top-intrinsics output
    top_calibration_json: Path   # calibrate-top-extrinsics output
    bridge_port: int             # refuse to start while something listens here
    solver_config: str           # hardware-bridge config for calibrate-top-extrinsics
    focus_lock: str              # "verify" (read-only check) | "lock" (set, then check)
    base_label: str              # operator text: "left base" / "right base"
    install_target_json: Path | None   # left only: where the bridge reads it
    runtime_yaml: Path | None          # left only: our left_arm.yaml
    handoff_md: Path | None            # right only: hand-off note for the right-arm bridge repo
    # 2026-09-08: further bridge ports whose listener would own one of THIS rig's devices
    # (cross rig: 9021 = our left bridge holds the left top BRIO, 9022 = our right bridge).
    extra_refuse_ports: tuple = ()

    @property
    def top_profile(self) -> tuple[int, int, int]:
        return (TOP_WIDTH, TOP_HEIGHT, TOP_FPS)

    def shell_lines(self) -> list[str]:
        """``RIG_<FIELD>=<value>`` lines (shell-quoted) for ``eval`` in bash."""
        import shlex

        lines = []
        for field in fields(self):
            value = getattr(self, field.name)
            lines.append(
                f"RIG_{field.name.upper()}="
                f"{shlex.quote('' if value is None else str(value))}"
            )
        lines.append(f"RIG_TOP_WIDTH={TOP_WIDTH}")
        lines.append(f"RIG_TOP_HEIGHT={TOP_HEIGHT}")
        lines.append(f"RIG_TOP_FPS={TOP_FPS}")
        lines.append(f"RIG_TOP_FOURCC={TOP_FOURCC}")
        return lines


LEFT_RIG = RigSpec(
    side="left",
    can_channel=LEFT_CAN_CHANNEL,
    wrist_serial=LEFT_CAMERA_SERIAL,
    flange_body=LEFT_FLANGE_BODY,
    camera_body=LEFT_CAMERA_BODY,
    wrist_intrinsics_json=INTRINSICS_JSON,
    top_serial="178B0DAE",
    top_device=_brio_device("178B0DAE"),
    top_intr_dir=OUT_DIR / "top_intr",
    top_pairs_dir=OUT_DIR / "top_pairs",
    top_intrinsics_json=OUT_DIR / "top_brio_178B0DAE_intrinsics.json",
    top_calibration_json=OUT_DIR / "top_brio_178B0DAE_calibration.json",
    bridge_port=LEFT_BRIDGE_PORT,
    solver_config="config/left_calib.yaml",
    # The left BRIO's autofocus was confirmed OFF when it was calibrated; the
    # left tools only VERIFY (read-only) so the installed calibration's focus
    # state is never silently changed.
    focus_lock="verify",
    base_label="left base",
    install_target_json=HB_DIR / "acceptance/top_left/top_brio_calibration.json",
    runtime_yaml=HB_DIR / "config/left_arm.yaml",
    handoff_md=None,
)

RIGHT_RIG = RigSpec(
    side="right",
    can_channel=RIGHT_CAN_CHANNEL,
    wrist_serial=RIGHT_CAMERA_SERIAL,
    flange_body=RIGHT_FLANGE_BODY,
    camera_body=RIGHT_CAMERA_BODY,
    wrist_intrinsics_json=RIGHT_INTRINSICS_JSON,
    top_serial="B8C7F203",
    top_device=_brio_device("B8C7F203"),
    top_intr_dir=OUT_DIR / "top_intr_right",
    top_pairs_dir=OUT_DIR / "top_pairs_right",
    top_intrinsics_json=OUT_DIR / "top_brio_B8C7F203_intrinsics.json",
    top_calibration_json=OUT_DIR / "top_brio_B8C7F203_calibration.json",
    bridge_port=RIGHT_BRIDGE_PORT,
    solver_config="config/right_calib.yaml",
    # Autofocus state of B8C7F203 is unknown: lock (autofocus off, focus 0,
    # zoom 100) at tool start, then verify; refuse if autofocus still reads 1.
    focus_lock="lock",
    base_label="right base",
    install_target_json=None,   # NEVER installed here: handed to the right-arm bridge repo
    runtime_yaml=None,
    handoff_md=OUT_DIR / "RIGHT_RIG_HANDOFF.md",
)

# 2026-09-08 "cross" rig for the TWO-ARM world frame: the RIGHT arm (can_follower_r, wrist
# D405 353322271910, station right_gripper -> right_camera) observed by OUR LEFT top BRIO
# 178B0DAE.  Solving pairs from this rig gives the LEFT top camera's camera_to_world in
# RIGHT_BASE; together with the installed left calibration (same camera in LEFT_BASE) that
# yields T_left_right = T_lb_top @ inv(T_rb_top) -> agp/tools/solve_base_transform.py
# --top-in-right-base.  The left intrinsics (top_intr/, top_brio_178B0DAE_intrinsics.json)
# are REUSED (same camera, same 1080p profile, autofocus verified off); nothing is installed
# into any bridge.  Refuses to start while 9020 (the earlier right-arm bridge), 9021 (our left
# bridge owns the top BRIO) or 9022 (our right bridge) listens.
CROSS_RIG = RigSpec(
    side="right",
    can_channel=RIGHT_CAN_CHANNEL,
    wrist_serial=RIGHT_CAMERA_SERIAL,
    flange_body=RIGHT_FLANGE_BODY,
    camera_body=RIGHT_CAMERA_BODY,
    wrist_intrinsics_json=RIGHT_INTRINSICS_JSON,
    top_serial="178B0DAE",
    top_device=_brio_device("178B0DAE"),
    top_intr_dir=OUT_DIR / "top_intr",
    top_pairs_dir=OUT_DIR / "top_pairs_cross",
    top_intrinsics_json=OUT_DIR / "top_brio_178B0DAE_intrinsics.json",
    top_calibration_json=OUT_DIR / "top_brio_178B0DAE_in_right_base_calibration.json",
    bridge_port=RIGHT_BRIDGE_PORT,
    solver_config="config/right_calib.yaml",
    focus_lock="verify",
    base_label="right base",
    install_target_json=None,
    runtime_yaml=None,
    handoff_md=None,
    extra_refuse_ports=(LEFT_BRIDGE_PORT, 9022),
)

RIGS: dict[str, RigSpec] = {"left": LEFT_RIG, "right": RIGHT_RIG, "cross": CROSS_RIG}
RIG_CHOICES = tuple(RIGS)


def get_rig(name: str) -> RigSpec:
    try:
        return RIGS[name]
    except KeyError:
        raise ValueError(f"unknown rig {name!r}; expected one of {RIG_CHOICES}") from None


def frame_transform(frame) -> np.ndarray:
    """4x4 world transform of one mujoco named body frame."""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(frame.xmat, dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(frame.xpos, dtype=np.float64)
    return transform


def station_flange_from_camera(
    flange_body: str,
    camera_body: str,
    station_xml: Path = STATION_XML,
) -> np.ndarray:
    """Rigid ``T_<flange>_<camera>`` from the station MJCF for any side.

    The camera bodies are fixed (joint-less) descendants of the gripper body,
    so the relative transform is pose independent; the default configuration
    is sufficient.  This mirrors the solver's ``_station_flange_from_camera``
    (which reads the same two body names from the bridge config).
    """
    model = mujoco.MjModel.from_xml_path(str(station_xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    station_from_flange = frame_transform(data.body(flange_body))
    station_from_camera = frame_transform(data.body(camera_body))
    return np.linalg.inv(station_from_flange) @ station_from_camera


def station_left_flange_from_camera(station_xml: Path = STATION_XML) -> np.ndarray:
    """Rigid ``T_leftgripper_leftcamera`` from the station MJCF (LEFT wrapper)."""
    return station_flange_from_camera(LEFT_FLANGE_BODY, LEFT_CAMERA_BODY, station_xml)


def rig_flange_from_camera(rig: RigSpec, station_xml: Path = STATION_XML) -> np.ndarray:
    """``T_<side>gripper_<side>camera`` for one rig (its station bodies)."""
    return station_flange_from_camera(rig.flange_body, rig.camera_body, station_xml)


class ArmFK:
    """Flange FK on i2rt's combined side-agnostic YAM + linear_4310 model.

    Mirrors the bridge's ``I2rtKinematicsBackend.frame_chain`` (base ->
    body ``gripper``) but without joint-limit validation, so slightly
    out-of-model-limit measured joints (or offset-shifted joints during the
    zero-offset fit) still produce FK.  Side independent: both arms are the
    same YAM model with the base at the origin; per-channel encoder offsets
    are applied by i2rt before the joints reach this class.
    """

    _ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
    _FINGER_JOINT_NAMES = ("joint7", "joint8")

    def __init__(self) -> None:
        from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml

        xml_path = combine_arm_and_gripper_xml(ArmType.YAM, GripperType.LINEAR_4310)
        try:
            self._model = mujoco.MjModel.from_xml_path(xml_path)
        finally:
            Path(xml_path).unlink(missing_ok=True)
        self._data = mujoco.MjData(self._model)
        self._arm_addresses = self._joint_addresses(self._ARM_JOINT_NAMES)
        self._finger_addresses = self._joint_addresses(self._FINGER_JOINT_NAMES)
        self._finger_mid = np.array(
            [
                float(np.mean(self._model.joint(name).range))
                for name in self._FINGER_JOINT_NAMES
            ]
        )

    def _joint_addresses(self, names: tuple[str, ...]) -> np.ndarray:
        addresses = []
        for name in names:
            joint_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise RuntimeError(f"combined model is missing joint {name!r}")
            addresses.append(int(self._model.jnt_qposadr[joint_id]))
        return np.asarray(addresses, dtype=np.int64)

    def base_from_gripper(self, joints_rad: np.ndarray) -> np.ndarray:
        """``T_base_gripper`` (flange body ``gripper``) for 6 arm joints."""
        joints = np.asarray(joints_rad, dtype=np.float64)
        if joints.shape != (6,) or not np.isfinite(joints).all():
            raise ValueError("joints must be a finite 6-vector in radians")
        q = self._model.qpos0.copy()
        q[self._arm_addresses] = joints
        q[self._finger_addresses] = self._finger_mid
        self._data.qpos[:] = q
        mujoco.mj_forward(self._model, self._data)
        return frame_transform(self._data.body("gripper"))


class LeftArmFK(ArmFK):
    """LEFT-named wrapper kept for existing callers; identical to ``ArmFK``."""


def load_intrinsics_json(
    path: Path = INTRINSICS_JSON, expected_serial: str = LEFT_CAMERA_SERIAL
) -> dict:
    """Load a factory-intrinsics dump written by the capture tools.

    ``expected_serial`` gates the file against the rig's wrist D405 (default:
    the left one, so existing left callers are unchanged).
    """
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    for key in ("serial", "distortion_model", "distortion_coefficients", "camera_matrix"):
        if key not in payload:
            raise ValueError(f"{path}: missing key {key!r}")
    coefficients = np.asarray(payload["distortion_coefficients"], dtype=np.float64)
    if coefficients.shape != (5,) or not np.isfinite(coefficients).all():
        raise ValueError(f"{path}: distortion_coefficients must be five finite values")
    if payload["serial"] != expected_serial:
        raise ValueError(
            f"{path}: serial {payload['serial']!r} is not the expected D405 "
            f"{expected_serial!r}"
        )
    return payload


def next_pose_path(out_dir: Path, label: str | None) -> Path:
    """Auto-numbered ``left_poseNN[_label].npz`` path (NN starts at 01)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    highest = 0
    for existing in out_dir.glob("left_pose*.npz"):
        digits = ""
        for char in existing.stem[len("left_pose"):]:
            if char.isdigit():
                digits += char
            else:
                break
        if digits:
            highest = max(highest, int(digits))
    suffix = ""
    if label:
        clean = "".join(c for c in label if c.isalnum() or c in "-_")
        if clean:
            suffix = f"_{clean}"
    return out_dir / f"left_pose{highest + 1:02d}{suffix}.npz"


def _main() -> int:
    """``--rig NAME --shell`` prints the rig table for bash ``eval``;
    ``--rig NAME`` alone prints it human-readably."""
    import argparse

    parser = argparse.ArgumentParser(description="print the per-rig calibration mapping")
    parser.add_argument("--rig", choices=RIG_CHOICES, default="left")
    parser.add_argument("--shell", action="store_true", help="RIG_*=value lines for eval")
    args = parser.parse_args()
    rig = get_rig(args.rig)
    if args.shell:
        print("\n".join(rig.shell_lines()))
    else:
        for field in fields(rig):
            print(f"{field.name:22s} {getattr(rig, field.name)}")
        print(f"{'top_profile':22s} {TOP_WIDTH}x{TOP_HEIGHT}@{TOP_FPS} {TOP_FOURCC}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
