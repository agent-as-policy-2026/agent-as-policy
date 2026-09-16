from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class I2rtConfig(StrictModel):
    source: Path
    package_version: str
    git_revision: str
    tracked_diff_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")


class HardwareConfig(StrictModel):
    arm: str
    gripper: str
    can_channel: str
    can_bitrate: int = Field(gt=0)
    camera_model: str
    camera_serial: str
    camera_physical_port: str
    arm_model: Path
    gripper_model: Path
    station_model: Path


class BridgeEndpointConfig(StrictModel):
    host: str = Field(min_length=1)
    port: int = Field(ge=1, le=65535)
    request_timeout_s: float = Field(gt=0)
    stale_after_s: float = Field(gt=0)


class CameraStreamConfig(StrictModel):
    name: str = Field(min_length=1)
    width: int = Field(gt=0)
    height: int = Field(gt=0)
    fps: int = Field(gt=0)
    frame_timeout_s: float = Field(gt=0)
    stale_after_s: float = Field(gt=0)
    max_joint_skew_s: float = Field(gt=0)
    min_depth_m: float = Field(gt=0)
    max_depth_m: float = Field(gt=0)
    transform_translation_error_bound_m: float = Field(gt=0)
    station_flange_body: str = Field(min_length=1)
    station_camera_body: str = Field(min_length=1)

    @model_validator(mode="after")
    def depth_range_is_ordered(self) -> "CameraStreamConfig":
        if self.min_depth_m >= self.max_depth_m:
            raise ValueError("min_depth_m must be less than max_depth_m")
        return self


class FixedRgbCameraConfig(StrictModel):
    calibration_path: Path
    stale_after_s: float = Field(gt=0)
    max_joint_skew_s: float = Field(gt=0)


class WorkspaceConfig(StrictModel):
    frame: str
    tcp: str
    x_m: tuple[float, float]
    y_m: tuple[float, float]
    z_m: tuple[float, float]

    @model_validator(mode="after")
    def bounds_are_ordered(self) -> "WorkspaceConfig":
        for axis, bounds in (("x_m", self.x_m), ("y_m", self.y_m), ("z_m", self.z_m)):
            if bounds[0] >= bounds[1]:
                raise ValueError(f"{axis} lower bound must be less than upper bound")
        return self


class SpeedLimits(StrictModel):
    control_frequency_hz: float = Field(gt=0)
    joint_velocity_deg_s: float = Field(gt=0)
    joint_acceleration_deg_s2: float = Field(gt=0)
    cartesian_translation_m_s: float = Field(gt=0)
    cartesian_rotation_deg_s: float = Field(gt=0)
    gripper_fraction_s: float = Field(gt=0)
    # 2026-09-14 buffered joint programs (port of the original throwing runtime): J4 ceilings
    # that apply ONLY to authored `joint_program` actions; the other five joints of a program and
    # every ordinary move keep joint_velocity_deg_s / joint_acceleration_deg_s2. Both unset (every
    # config before this date) = this bridge offers no programs at all (main.py attaches the plain
    # MotionController), so existing configs behave exactly as before.
    program_j4_velocity_deg_s: float | None = Field(default=None, gt=0)
    program_j4_acceleration_deg_s2: float | None = Field(default=None, gt=0)

    @model_validator(mode="after")
    def program_limits_are_paired(self) -> "SpeedLimits":
        if (self.program_j4_velocity_deg_s is None) != (self.program_j4_acceleration_deg_s2 is None):
            raise ValueError(
                "program_j4_velocity_deg_s and program_j4_acceleration_deg_s2 must be set together"
            )
        return self

    @property
    def programs_enabled(self) -> bool:
        return self.program_j4_velocity_deg_s is not None


class AcceptanceConfig(StrictModel):
    home_joints_deg: tuple[float, float, float, float, float, float]
    workspace: WorkspaceConfig
    speed_limits: SpeedLimits


class SafetyConfig(StrictModel):
    command_heartbeat_timeout_s: float = Field(gt=0)
    feedback_stale_after_s: float = Field(gt=0)
    max_tracking_error_deg: float | None = Field(default=None, gt=0)
    tracking_error_timeout_s: float = Field(gt=0)
    joint_settle_tolerance_rad: float = Field(gt=0)
    cartesian_position_tolerance_m: float = Field(gt=0)
    cartesian_orientation_tolerance_deg: float = Field(gt=0)
    cartesian_settle_gain_s_inv: float = Field(gt=0)
    cartesian_settle_max_bias_deg: float = Field(gt=0)
    gripper_max_effort_nm: float = Field(gt=0)
    gripper_contact_min_effort_nm: float = Field(gt=0)
    gripper_contact_release_effort_nm: float = Field(gt=0)
    gripper_contact_confirmation_s: float = Field(gt=0)
    gripper_contact_max_velocity_fraction_s: float = Field(gt=0)
    gripper_contact_min_open_fraction: float = Field(gt=0, lt=1)
    gripper_stall_velocity_fraction_s: float = Field(gt=0)
    gripper_stall_error_fraction: float = Field(gt=0, le=1)
    gripper_stall_timeout_s: float = Field(gt=0)
    # close this much further past the confirmed contact position (0 = hold at contact)
    gripper_contact_squeeze_fraction: float = Field(ge=0, lt=0.2, default=0.0)
    # Reaction when a Cartesian move has finished its trajectory but cannot settle within
    # the tolerances before its hard timeout. "idle" (legacy): ACTION_TIMEOUT fault and
    # gravity-comp idle of the whole chain, gripper included. "hold": keep the last PD
    # command active as a normal lease/heartbeat-monitored hold and return a completed
    # result whose detail starts with "SETTLE_MISS:". Added 2026-09-03 after the idle
    # reaction dropped held objects over few-mm settle misses.
    cartesian_settle_miss: Literal["idle", "hold"] = "idle"
    action_log_path: Path

    @model_validator(mode="after")
    def contact_open_fraction_exceeds_empty_stall_band(self) -> "SafetyConfig":
        if self.gripper_contact_release_effort_nm >= self.gripper_contact_min_effort_nm:
            raise ValueError(
                "gripper_contact_release_effort_nm must be below "
                "gripper_contact_min_effort_nm"
            )
        if self.gripper_contact_min_open_fraction <= self.gripper_stall_error_fraction:
            raise ValueError(
                "gripper_contact_min_open_fraction must exceed "
                "gripper_stall_error_fraction"
            )
        return self


class BridgeConfig(StrictModel):
    schema_version: Literal[1]
    bridge: BridgeEndpointConfig
    i2rt: I2rtConfig
    hardware: HardwareConfig
    camera: CameraStreamConfig
    top_camera: FixedRgbCameraConfig
    acceptance: AcceptanceConfig
    safety: SafetyConfig


def load_config(path: Path) -> BridgeConfig:
    """Load strict YAML and resolve every source/model path from its location."""
    config_path = path.resolve()
    with config_path.open(encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict):
        raise ValueError(f"bridge config must be a mapping: {config_path}")

    base = config_path.parent
    raw["i2rt"]["source"] = (base / raw["i2rt"]["source"]).resolve()
    for key in ("arm_model", "gripper_model", "station_model"):
        raw["hardware"][key] = (base / raw["hardware"][key]).resolve()
    raw["top_camera"]["calibration_path"] = (
        base / raw["top_camera"]["calibration_path"]
    ).resolve()
    raw["safety"]["action_log_path"] = (
        base / raw["safety"]["action_log_path"]
    ).resolve()
    return BridgeConfig.model_validate(raw)
