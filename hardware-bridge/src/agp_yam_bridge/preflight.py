import argparse
import hashlib
import json
import re
import subprocess
import sys
from collections.abc import Callable
from dataclasses import dataclass
from importlib.metadata import version
from inspect import getsourcefile
from pathlib import Path

import numpy as np

from agp_yam_bridge.config import BridgeConfig, load_config


class PreflightError(RuntimeError):
    """The configured hardware baseline does not match observed facts."""


@dataclass(frozen=True)
class I2rtFacts:
    source: Path
    package_version: str
    git_revision: str
    tracked_diff_sha256: str
    get_yam_robot_source: Path
    arm: str
    gripper: str
    arm_model: Path
    gripper_model: Path
    arm_motor_offsets_deg: tuple[float, ...]


@dataclass(frozen=True)
class CanFacts:
    channel: str
    kind: str
    is_up: bool
    bitrate: int


@dataclass(frozen=True)
class CameraFacts:
    model: str
    serial: str
    physical_port: str


@dataclass(frozen=True)
class PreflightFacts:
    i2rt: I2rtFacts
    can: CanFacts
    camera: CameraFacts
    station_model: Path


def parse_can_facts(payload: str, channel: str) -> CanFacts:
    """Parse one ``ip -json -details link`` result without weakening missing fields."""
    try:
        [link] = json.loads(payload)
        linkinfo = link["linkinfo"]
        bitrate = linkinfo["info_data"]["bittiming"]["bitrate"]
        return CanFacts(
            channel=str(link["ifname"]),
            kind=str(linkinfo["info_kind"]),
            is_up="UP" in link["flags"],
            bitrate=int(bitrate),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise PreflightError(f"could not read complete CAN facts for {channel!r}: {exc}") from exc


def parse_realsense_facts(payload: str, expected_serial: str) -> CameraFacts:
    """Select exactly the configured RealSense and retain its stable USB port."""
    for section in payload.split("Device info:")[1:]:
        fields = {}
        for line in section.splitlines():
            if not line.strip():
                continue
            if line.startswith("Stream Profiles"):
                break
            if ":" in line:
                key, value = line.split(":", 1)
                fields[key.strip()] = value.strip()
        if fields.get("Serial Number") != expected_serial:
            continue
        physical_path = fields.get("Physical Port", "")
        port_match = re.search(r"/usb\d+/(\d+-\d+)(?:/|$)", physical_path)
        if not port_match:
            raise PreflightError(
                f"camera {expected_serial} has an unrecognized physical port: {physical_path!r}"
            )
        return CameraFacts(
            model=fields.get("Name", ""),
            serial=expected_serial,
            physical_port=port_match.group(1),
        )
    raise PreflightError(f"configured camera serial {expected_serial!r} was not detected")


def _git_output(source: Path, *args: str, text: bool = True) -> str | bytes:
    try:
        result = subprocess.run(
            ["git", *args],
            cwd=source,
            check=True,
            capture_output=True,
            text=text,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise PreflightError(f"could not inspect i2rt git state at {source}: {exc}") from exc
    return result.stdout


def hardware_source_diff_sha256(source: Path) -> str:
    """Fingerprint tracked runtime source and dependency metadata, excluding docs/examples."""
    diff = _git_output(
        source,
        "diff",
        "--binary",
        "HEAD",
        "--",
        "i2rt",
        "pyproject.toml",
        text=False,
    )
    assert isinstance(diff, bytes)
    return hashlib.sha256(diff).hexdigest()


def collect_i2rt_facts(config: BridgeConfig) -> I2rtFacts:
    """Report the imported i2rt package and its exact tracked working-tree state."""
    import i2rt
    from i2rt.robots.get_robot import get_yam_robot
    from i2rt.robots.utils import ArmType, GripperType, _load_arm_config

    source = Path(i2rt.__file__).resolve().parent.parent
    factory_source = getsourcefile(get_yam_robot)
    if factory_source is None:
        raise PreflightError("could not locate the imported get_yam_robot implementation")

    arm = ArmType.from_string_name(config.hardware.arm)
    gripper = GripperType.from_string_name(config.hardware.gripper)
    arm_hardware = _load_arm_config(arm)
    arm_offsets_rad = arm_hardware.motor_offsets_rad_by_channel.get(
        config.hardware.can_channel,
        np.zeros(len(arm_hardware.motor_list), dtype=float),
    )
    revision = str(_git_output(source, "rev-parse", "HEAD")).strip()
    return I2rtFacts(
        source=source,
        package_version=version("i2rt"),
        git_revision=revision,
        tracked_diff_sha256=hardware_source_diff_sha256(source),
        get_yam_robot_source=Path(factory_source).resolve(),
        arm=arm.value,
        gripper=gripper.value,
        arm_model=Path(arm.get_xml_path()).resolve(),
        gripper_model=Path(gripper.get_xml_path()).resolve(),
        arm_motor_offsets_deg=tuple(float(value) for value in np.degrees(arm_offsets_rad)),
    )


def _command_output(command: list[str]) -> str:
    try:
        result = subprocess.run(command, check=True, capture_output=True, text=True)
    except FileNotFoundError as exc:
        raise PreflightError(f"required startup probe is unavailable: {command[0]}") from exc
    except subprocess.CalledProcessError as exc:
        detail = exc.stderr.strip() or exc.stdout.strip() or f"exit {exc.returncode}"
        raise PreflightError(f"startup probe failed ({' '.join(command)}): {detail}") from exc
    return result.stdout


def collect_preflight_facts(config: BridgeConfig) -> PreflightFacts:
    """Collect source and device facts without constructing a robot or sending CAN frames."""
    station_model = config.hardware.station_model.resolve()
    if not station_model.is_file():
        raise PreflightError(f"configured station model does not exist: {station_model}")
    can_payload = _command_output(
        ["ip", "-json", "-details", "link", "show", "dev", config.hardware.can_channel]
    )
    camera_payload = _command_output(["rs-enumerate-devices"])
    return PreflightFacts(
        i2rt=collect_i2rt_facts(config),
        can=parse_can_facts(can_payload, config.hardware.can_channel),
        camera=parse_realsense_facts(camera_payload, config.hardware.camera_serial),
        station_model=station_model,
    )


def run_preflight(
    config: BridgeConfig,
    *,
    collector: Callable[[BridgeConfig], PreflightFacts] | None = None,
) -> str:
    """Collect and validate the startup baseline; return only a ready report."""
    if collector is None:
        collector = collect_preflight_facts
    return validate_preflight(config, collector(config))


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config/first_acceptance.yaml"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Fail-closed preflight for the AgP YAM hardware bridge"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args(argv)
    try:
        print(run_preflight(load_config(args.config)))
    except Exception as exc:
        print(f"status: BLOCKED\nreason: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


def validate_preflight(config: BridgeConfig, facts: PreflightFacts) -> str:
    """Reject a baseline mismatch, otherwise return the complete startup report."""
    from agp_yam_bridge.camera import load_fixed_rgb_calibration

    top_calibration = load_fixed_rgb_calibration(
        config.top_camera.calibration_path
    )
    expected = config.i2rt
    mismatches = []
    comparisons = (
        ("i2rt source", facts.i2rt.source, expected.source),
        (
            "get_yam_robot source",
            facts.i2rt.get_yam_robot_source,
            expected.source / "i2rt/robots/get_robot.py",
        ),
        ("i2rt package version", facts.i2rt.package_version, expected.package_version),
        ("i2rt git revision", facts.i2rt.git_revision, expected.git_revision),
        ("i2rt tracked diff", facts.i2rt.tracked_diff_sha256, expected.tracked_diff_sha256),
        ("arm variant", facts.i2rt.arm, config.hardware.arm),
        ("gripper variant", facts.i2rt.gripper, config.hardware.gripper),
        ("arm model", facts.i2rt.arm_model, config.hardware.arm_model),
        ("gripper model", facts.i2rt.gripper_model, config.hardware.gripper_model),
        ("CAN channel", facts.can.channel, config.hardware.can_channel),
        ("CAN bitrate", facts.can.bitrate, config.hardware.can_bitrate),
        ("camera model", facts.camera.model, config.hardware.camera_model),
        ("camera serial", facts.camera.serial, config.hardware.camera_serial),
        ("camera physical port", facts.camera.physical_port, config.hardware.camera_physical_port),
        ("station model", facts.station_model, config.hardware.station_model),
    )
    for label, actual, wanted in comparisons:
        if actual != wanted:
            mismatches.append(f"{label}: observed {actual!s}, expected {wanted!s}")
    if facts.can.kind != "can":
        mismatches.append(f"CAN kind: observed {facts.can.kind!r}, expected 'can'")
    if not facts.can.is_up:
        mismatches.append(f"CAN channel {facts.can.channel!r} is not UP")
    if mismatches:
        raise PreflightError("preflight failed closed:\n- " + "\n- ".join(mismatches))

    limits = config.acceptance.speed_limits
    workspace = config.acceptance.workspace
    camera = config.camera
    safety = config.safety
    return "\n".join(
        (
            "status: READY",
            f"i2rt_source: {facts.i2rt.source}",
            f"i2rt_get_yam_robot_source: {facts.i2rt.get_yam_robot_source}",
            f"i2rt_version: {facts.i2rt.package_version}",
            f"i2rt_revision: {facts.i2rt.git_revision}",
            f"i2rt_tracked_diff_sha256: {facts.i2rt.tracked_diff_sha256}",
            f"arm_variant: {facts.i2rt.arm}",
            f"gripper_variant: {facts.i2rt.gripper}",
            f"can_channel: {facts.can.channel}",
            f"arm_motor_offsets_deg: {list(facts.i2rt.arm_motor_offsets_deg)}",
            f"can_bitrate: {facts.can.bitrate}",
            f"camera_model: {facts.camera.model}",
            f"camera_serial: {facts.camera.serial}",
            f"camera_physical_port: {facts.camera.physical_port}",
            f"camera_stream: {camera.name} {camera.width}x{camera.height}@{camera.fps} RGB8+Z16",
            f"max_camera_joint_skew_ms: {camera.max_joint_skew_s * 1000}",
            "top_camera_stream: "
            f"{top_calibration.name} {top_calibration.width}x{top_calibration.height}"
            f"@{top_calibration.fps} RGB8",
            f"top_camera_serial: {top_calibration.serial}",
            f"top_camera_device: {top_calibration.device}",
            "top_camera_calibration_status: PASS",
            "max_top_camera_joint_skew_ms: "
            f"{config.top_camera.max_joint_skew_s * 1000}",
            f"arm_model: {facts.i2rt.arm_model}",
            f"gripper_model: {facts.i2rt.gripper_model}",
            f"station_model: {facts.station_model}",
            f"home_joints_deg: {list(config.acceptance.home_joints_deg)}",
            f"workspace_{workspace.frame}_{workspace.tcp}_m: x={list(workspace.x_m)}, y={list(workspace.y_m)}, z={list(workspace.z_m)}",
            f"control_frequency_hz: {limits.control_frequency_hz}",
            f"max_joint_velocity_deg_s: {limits.joint_velocity_deg_s}",
            f"max_joint_acceleration_deg_s2: {limits.joint_acceleration_deg_s2}",
            f"max_cartesian_translation_m_s: {limits.cartesian_translation_m_s}",
            f"max_cartesian_rotation_deg_s: {limits.cartesian_rotation_deg_s}",
            f"max_gripper_fraction_s: {limits.gripper_fraction_s}",
            f"command_heartbeat_timeout_s: {safety.command_heartbeat_timeout_s}",
            f"feedback_stale_after_s: {safety.feedback_stale_after_s}",
            "max_tracking_error_deg: "
            + (
                "disabled"
                if safety.max_tracking_error_deg is None
                else str(safety.max_tracking_error_deg)
            ),
            f"tracking_error_timeout_s: {safety.tracking_error_timeout_s}",
            f"joint_settle_tolerance_rad: {safety.joint_settle_tolerance_rad}",
            f"cartesian_position_tolerance_m: {safety.cartesian_position_tolerance_m}",
            "cartesian_orientation_tolerance_deg: "
            f"{safety.cartesian_orientation_tolerance_deg}",
            f"cartesian_settle_gain_s_inv: {safety.cartesian_settle_gain_s_inv}",
            f"cartesian_settle_max_bias_deg: {safety.cartesian_settle_max_bias_deg}",
            f"gripper_max_effort_nm: {safety.gripper_max_effort_nm}",
            f"gripper_contact_min_effort_nm: {safety.gripper_contact_min_effort_nm}",
            "gripper_contact_release_effort_nm: "
            f"{safety.gripper_contact_release_effort_nm}",
            "gripper_contact_confirmation_s: "
            f"{safety.gripper_contact_confirmation_s}",
            "gripper_contact_max_velocity_fraction_s: "
            f"{safety.gripper_contact_max_velocity_fraction_s}",
            "gripper_contact_min_open_fraction: "
            f"{safety.gripper_contact_min_open_fraction}",
            "gripper_stall: "
            f"velocity<{safety.gripper_stall_velocity_fraction_s} fraction/s, "
            f"error>{safety.gripper_stall_error_fraction}, "
            f"duration>{safety.gripper_stall_timeout_s}s",
            f"action_log_path: {safety.action_log_path}",
        )
    )
