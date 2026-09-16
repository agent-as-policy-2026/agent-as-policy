"""Strict msgpack wire contract for the AgP/YAM bridge."""

from __future__ import annotations

import re
import socket
import struct
import time
from collections.abc import Mapping
from typing import Any

import msgpack
import msgpack_numpy
import numpy as np

SCHEMA_VERSION = 1
MAX_FRAME_BYTES = 16 * 1024 * 1024
# 2026-09-14 buffered joint programs: ids name the per-program report file on the bridge
PROGRAM_ID_PATTERN = r"[a-zA-Z0-9_-]{1,64}"
PROGRAM_METHODS = frozenset({"preview_program", "program_report"})

msgpack_numpy.patch()


class ProtocolError(ValueError):
    """A wire message violated the versioned bridge contract."""

    def __init__(self, code: str, message: str, *, retryable: bool = False) -> None:
        super().__init__(message)
        self.code = code
        self.retryable = retryable


def encode_message(message: Mapping[str, Any]) -> bytes:
    return msgpack.packb(dict(message), use_bin_type=True)


def decode_message(payload: bytes) -> dict[str, Any]:
    try:
        message = msgpack.unpackb(payload, raw=False)
    except Exception as exc:
        raise ProtocolError("INVALID_MSGPACK", f"could not decode msgpack: {exc}") from exc
    if not isinstance(message, dict):
        raise ProtocolError("INVALID_MESSAGE", "top-level message must be a map")
    return message


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise EOFError("peer closed the msgpack connection")
        chunks.extend(chunk)
    return bytes(chunks)


def send_framed(sock: socket.socket, message: Mapping[str, Any]) -> None:
    payload = encode_message(message)
    if len(payload) > MAX_FRAME_BYTES:
        raise ProtocolError("FRAME_TOO_LARGE", f"frame is {len(payload)} bytes")
    sock.sendall(struct.pack("!I", len(payload)) + payload)


def recv_framed(sock: socket.socket) -> dict[str, Any]:
    (size,) = struct.unpack("!I", _recv_exact(sock, 4))
    if size <= 0 or size > MAX_FRAME_BYTES:
        raise ProtocolError("INVALID_FRAME_SIZE", f"frame size {size} is not allowed")
    return decode_message(_recv_exact(sock, size))


def _require_nonnegative_int(message: Mapping[str, Any], field: str, code: str) -> int:
    value = message.get(field)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ProtocolError(code, f"{field} must be a non-negative integer")
    return value


def _validate_envelope(message: Mapping[str, Any], expected_type: str) -> None:
    schema_version = message.get("schema_version")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != SCHEMA_VERSION
    ):
        raise ProtocolError("UNSUPPORTED_SCHEMA", "schema_version must be 1")
    if message.get("message_type") != expected_type:
        raise ProtocolError("INVALID_MESSAGE_TYPE", f"message_type must be {expected_type!r}")
    _require_nonnegative_int(message, "request_id", "INVALID_REQUEST_ID")


def _require_exact_keys(
    message: Mapping[str, Any], expected: set[str], *, context: str
) -> None:
    if set(message) != expected:
        raise ProtocolError(
            "INVALID_MESSAGE",
            f"{context} keys must be exactly {sorted(expected)}",
        )


def _require_vector(message: Mapping[str, Any], field: str, size: int) -> np.ndarray:
    value = message.get(field)
    if not isinstance(value, np.ndarray):
        raise ProtocolError("INVALID_DTYPE", f"{field} must be an ndarray[float32]")
    if value.dtype != np.float32:
        raise ProtocolError("INVALID_DTYPE", f"{field} dtype must be float32")
    if value.shape != (size,):
        raise ProtocolError("INVALID_SHAPE", f"{field} shape must be ({size},)")
    if not np.isfinite(value).all():
        raise ProtocolError("NONFINITE", f"{field} contains a non-finite value")
    return value


def _require_positive_number(message: Mapping[str, Any], field: str) -> float:
    value = message.get(field)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError("INVALID_ACTION", f"{field} must be numeric")
    number = float(value)
    if not np.isfinite(number) or number <= 0:
        raise ProtocolError("INVALID_ACTION", f"{field} must be finite and positive")
    return number


def _require_matrix(
    message: Mapping[str, Any], field: str, columns: int
) -> np.ndarray:
    value = message.get(field)
    if not isinstance(value, np.ndarray) or value.dtype != np.float32:
        raise ProtocolError("INVALID_DTYPE", f"{field} must be an ndarray[float32]")
    if value.ndim != 2 or value.shape[0] == 0 or value.shape[1] != columns:
        raise ProtocolError(
            "INVALID_SHAPE", f"{field} shape must be (N, {columns}) with N > 0"
        )
    if not np.isfinite(value).all():
        raise ProtocolError("NONFINITE", f"{field} contains a non-finite value")
    return value


def _require_ndarray(
    message: Mapping[str, Any],
    field: str,
    *,
    dtype: np.dtype[Any],
    shape: tuple[int, ...],
) -> np.ndarray:
    value = message.get(field)
    if not isinstance(value, np.ndarray) or value.dtype != dtype:
        raise ProtocolError("INVALID_DTYPE", f"{field} must be ndarray[{dtype}]")
    if value.shape != shape:
        raise ProtocolError("INVALID_SHAPE", f"{field} shape must be {shape}")
    return value


def _validate_camera(
    camera: Any,
    *,
    context: str,
    rgbd: bool,
    now_monotonic_ns: int | None,
    max_camera_age_ns: int | None,
    max_joint_skew_ns: int | None,
) -> None:
    if not isinstance(camera, Mapping):
        raise ProtocolError("INVALID_CAMERA", f"{context} must be a map")
    common_keys = {
        "name",
        "serial",
        "frame_sequence",
        "observation_sequence",
        "frame_monotonic_ns",
        "frame_wall_time_ns",
        "joint_monotonic_ns",
        "joint_time_delta_ns",
        "transform_translation_error_bound_m",
        "color_order",
        "images",
        "intrinsics",
        "distortion_model",
        "distortion_coefficients",
        "camera_to_world",
    }
    rgbd_keys = {"device_timestamp_ms", "depth_unit", "invalid_depth_value"}
    _require_exact_keys(
        camera,
        common_keys | rgbd_keys if rgbd else common_keys,
        context=context,
    )
    if not isinstance(camera.get("name"), str) or not camera["name"]:
        raise ProtocolError("INVALID_CAMERA", f"{context}.name must be a non-empty string")
    if not isinstance(camera.get("serial"), str) or not camera["serial"]:
        raise ProtocolError("INVALID_CAMERA", f"{context}.serial must be a non-empty string")
    _require_nonnegative_int(camera, "frame_sequence", "INVALID_CAMERA")
    _require_nonnegative_int(camera, "observation_sequence", "INVALID_CAMERA")
    frame_time = _require_nonnegative_int(camera, "frame_monotonic_ns", "INVALID_TIMESTAMP")
    joint_time = _require_nonnegative_int(camera, "joint_monotonic_ns", "INVALID_TIMESTAMP")
    if frame_time == 0 or joint_time == 0:
        raise ProtocolError("INVALID_TIMESTAMP", "camera and joint timestamps must be positive")
    if _require_nonnegative_int(camera, "frame_wall_time_ns", "INVALID_TIMESTAMP") == 0:
        raise ProtocolError("INVALID_TIMESTAMP", "camera wall timestamp must be positive")
    skew_ns = _require_nonnegative_int(camera, "joint_time_delta_ns", "INVALID_TIMESTAMP")
    if skew_ns != abs(frame_time - joint_time):
        raise ProtocolError(
            "INVALID_TIMESTAMP",
            "joint_time_delta_ns must equal the camera/joint timestamp difference",
        )
    if camera.get("color_order") != "RGB":
        raise ProtocolError("INVALID_CAMERA", "camera color_order must be 'RGB'")
    transform_translation_error_bound_m = camera.get(
        "transform_translation_error_bound_m"
    )
    if (
        isinstance(transform_translation_error_bound_m, bool)
        or not isinstance(transform_translation_error_bound_m, (int, float))
        or not np.isfinite(float(transform_translation_error_bound_m))
        or float(transform_translation_error_bound_m) <= 0.0
    ):
        raise ProtocolError(
            "INVALID_CAMERA",
            f"{context}.transform_translation_error_bound_m must be finite and positive",
        )

    images = camera.get("images")
    expected_images = {"rgb", "depth"} if rgbd else {"rgb"}
    if not isinstance(images, Mapping) or set(images) != expected_images:
        description = "rgb and depth" if rgbd else "rgb"
        raise ProtocolError(
            "INVALID_CAMERA", f"{context} images must contain exactly {description}"
        )
    rgb = images.get("rgb")
    if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8:
        raise ProtocolError("INVALID_DTYPE", "camera RGB must be ndarray[uint8]")
    if rgb.ndim != 3 or rgb.shape[0] == 0 or rgb.shape[1] == 0 or rgb.shape[2] != 3:
        raise ProtocolError("INVALID_SHAPE", "camera RGB shape must be (H,W,3)")
    if rgbd:
        device_timestamp_ms = camera.get("device_timestamp_ms")
        if (
            isinstance(device_timestamp_ms, bool)
            or not isinstance(device_timestamp_ms, (int, float))
            or not np.isfinite(float(device_timestamp_ms))
            or float(device_timestamp_ms) < 0
        ):
            raise ProtocolError("INVALID_TIMESTAMP", "device_timestamp_ms is invalid")
        if camera.get("depth_unit") != "meter" or camera.get("invalid_depth_value") != 0.0:
            raise ProtocolError(
                "INVALID_CAMERA", "depth must use meters with invalid_depth_value 0.0"
            )
        depth = images.get("depth")
        if not isinstance(depth, np.ndarray) or depth.dtype != np.float32:
            raise ProtocolError("INVALID_DTYPE", "camera depth must be ndarray[float32]")
        if depth.shape != rgb.shape[:2]:
            raise ProtocolError("INVALID_SHAPE", "camera depth pixels must align with RGB")
        if not np.isfinite(depth).all():
            raise ProtocolError("NONFINITE", "camera depth contains a non-finite value")
        if (depth < 0).any() or not (depth > 0).any():
            raise ProtocolError("INVALID_CAMERA", "camera depth has no usable metric samples")

    intrinsics = _require_ndarray(
        camera,
        "intrinsics",
        dtype=np.dtype(np.float64),
        shape=(3, 3),
    )
    if not np.isfinite(intrinsics).all():
        raise ProtocolError("NONFINITE", "camera intrinsics contain a non-finite value")
    if (
        intrinsics[0, 0] <= 0
        or intrinsics[1, 1] <= 0
        or not np.allclose(intrinsics[2], [0.0, 0.0, 1.0], atol=1e-10)
    ):
        raise ProtocolError("INVALID_CAMERA", "camera intrinsics are not pinhole K")

    expected_distortion_model = "inverse_brown_conrady" if rgbd else "none"
    if camera.get("distortion_model") != expected_distortion_model:
        raise ProtocolError(
            "INVALID_CAMERA",
            f"{context}.distortion_model must be {expected_distortion_model!r}",
        )
    distortion = _require_ndarray(
        camera,
        "distortion_coefficients",
        dtype=np.dtype(np.float64),
        shape=(5,),
    )
    if not np.isfinite(distortion).all() or (
        not rgbd and not np.array_equal(distortion, np.zeros(5))
    ):
        raise ProtocolError(
            "INVALID_CAMERA",
            f"{context}.distortion_coefficients do not match the stream contract",
        )

    transform = _require_ndarray(
        camera,
        "camera_to_world",
        dtype=np.dtype(np.float64),
        shape=(4, 4),
    )
    rotation = transform[:3, :3]
    if (
        not np.isfinite(transform).all()
        or not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-10)
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5)
    ):
        raise ProtocolError("INVALID_TRANSFORM", "camera_to_world must be a rigid transform")

    if max_joint_skew_ns is not None and skew_ns > max_joint_skew_ns:
        raise ProtocolError(
            "CAMERA_JOINT_SKEW",
            f"camera/joint skew {skew_ns / 1e6:.3f}ms exceeds {max_joint_skew_ns / 1e6:.3f}ms",
            retryable=True,
        )
    if max_camera_age_ns is not None:
        now = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        age_ns = now - frame_time
        if age_ns < 0:
            raise ProtocolError("INVALID_TIMESTAMP", "camera timestamp is in the future")
        if age_ns > max_camera_age_ns:
            raise ProtocolError(
                "STALE_CAMERA",
                f"camera age {age_ns / 1e9:.3f}s exceeds {max_camera_age_ns / 1e9:.3f}s",
                retryable=True,
            )


def validate_observation_message(
    message: Mapping[str, Any],
    *,
    now_monotonic_ns: int | None = None,
    max_age_ns: int | None = None,
    max_camera_age_ns: int | None = None,
    max_joint_skew_ns: int | None = None,
) -> Mapping[str, Any]:
    """Validate one complete 6-arm-joint + gripper observation."""
    _validate_envelope(message, "observation")
    _require_exact_keys(
        message,
        {
            "schema_version",
            "message_type",
            "request_id",
            "sequence",
            "monotonic_ns",
            "wall_time_ns",
            "robot_joint_pos_0",
            "robot_joint_vel_0",
            "robot_joint_effort_0",
            "robot_cartesian_pos_0",
            "camera_0",
            "camera_1",
            "health",
        },
        context="observation",
    )
    _require_nonnegative_int(message, "sequence", "INVALID_SEQUENCE")
    monotonic_ns = _require_nonnegative_int(message, "monotonic_ns", "INVALID_TIMESTAMP")
    if monotonic_ns == 0:
        raise ProtocolError("INVALID_TIMESTAMP", "monotonic_ns must be positive")
    if _require_nonnegative_int(message, "wall_time_ns", "INVALID_TIMESTAMP") == 0:
        raise ProtocolError("INVALID_TIMESTAMP", "wall_time_ns must be positive")
    joint_position = _require_vector(message, "robot_joint_pos_0", 7)
    _require_vector(message, "robot_joint_vel_0", 7)
    _require_vector(message, "robot_joint_effort_0", 7)
    cartesian = _require_vector(message, "robot_cartesian_pos_0", 8)
    quaternion_norm = float(np.linalg.norm(cartesian[3:7]))
    if not np.isclose(quaternion_norm, 1.0, atol=1e-4):
        raise ProtocolError(
            "INVALID_QUATERNION",
            f"robot_cartesian_pos_0 wxyz quaternion must be unit length, got {quaternion_norm}",
        )
    gripper = float(cartesian[7])
    joint_gripper = float(joint_position[6])
    if not 0.0 <= joint_gripper <= 1.0:
        raise ProtocolError(
            "INVALID_GRIPPER",
            "robot_joint_pos_0 gripper fraction must be within [0, 1]",
        )
    if not 0.0 <= gripper <= 1.0:
        raise ProtocolError(
            "INVALID_GRIPPER",
            "robot_cartesian_pos_0 gripper fraction must be within [0, 1]",
        )
    if not np.isclose(gripper, joint_gripper, atol=1e-6, rtol=0.0):
        raise ProtocolError(
            "INCONSISTENT_OBSERVATION",
            "Cartesian and joint observations disagree on gripper fraction",
        )

    _validate_camera(
        message.get("camera_0"),
        context="camera_0",
        rgbd=True,
        now_monotonic_ns=now_monotonic_ns,
        max_camera_age_ns=max_camera_age_ns,
        max_joint_skew_ns=max_joint_skew_ns,
    )
    _validate_camera(
        message.get("camera_1"),
        context="camera_1",
        rgbd=False,
        now_monotonic_ns=now_monotonic_ns,
        max_camera_age_ns=max_camera_age_ns,
        max_joint_skew_ns=max_joint_skew_ns,
    )

    health = message.get("health")
    if not isinstance(health, Mapping):
        raise ProtocolError("INVALID_HEALTH", "health must be a map")
    if set(health) != {
        "state",
        "source_connected",
        "motion_enabled",
        "safety_state",
        "active_action_request_id",
        "detail",
    }:
        raise ProtocolError("INVALID_HEALTH", "health has unexpected or missing fields")
    if health.get("state") not in {"ok", "degraded", "error"}:
        raise ProtocolError("INVALID_HEALTH", "health.state is invalid")
    if not isinstance(health.get("source_connected"), bool):
        raise ProtocolError("INVALID_HEALTH", "health.source_connected must be bool")
    if health.get("state") == "ok" and health.get("source_connected") is not True:
        raise ProtocolError(
            "INVALID_HEALTH", "health.state 'ok' requires source_connected=true"
        )
    if not isinstance(health.get("motion_enabled"), bool):
        raise ProtocolError("INVALID_HEALTH", "health.motion_enabled must be bool")
    if health.get("safety_state") not in {"idle", "moving", "holding", "fault"}:
        raise ProtocolError("INVALID_HEALTH", "health.safety_state is invalid")
    active_request_id = health.get("active_action_request_id")
    if active_request_id is not None and (
        isinstance(active_request_id, bool)
        or not isinstance(active_request_id, int)
        or active_request_id < 0
    ):
        raise ProtocolError(
            "INVALID_HEALTH", "health.active_action_request_id must be null or non-negative"
        )
    if health.get("safety_state") in {"moving", "holding"} and active_request_id is None:
        raise ProtocolError(
            "INVALID_HEALTH", "active safety state requires an action request id"
        )
    if not isinstance(health.get("detail"), str):
        raise ProtocolError("INVALID_HEALTH", "health.detail must be a string")

    if max_age_ns is not None:
        now = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        age_ns = now - monotonic_ns
        if age_ns < 0:
            raise ProtocolError("INVALID_TIMESTAMP", "observation timestamp is in the future")
        if age_ns > max_age_ns:
            raise ProtocolError(
                "STALE_OBSERVATION",
                f"observation age {age_ns / 1e9:.3f}s exceeds {max_age_ns / 1e9:.3f}s",
                retryable=True,
            )
    return message


def validate_action_message(message: Mapping[str, Any]) -> Mapping[str, Any]:
    """Validate the P1 action envelope even though the server rejects execution."""
    _validate_envelope(message, "action")
    _require_nonnegative_int(message, "sequence", "INVALID_SEQUENCE")
    if _require_nonnegative_int(message, "monotonic_ns", "INVALID_TIMESTAMP") == 0:
        raise ProtocolError("INVALID_TIMESTAMP", "monotonic_ns must be positive")
    if _require_nonnegative_int(message, "wall_time_ns", "INVALID_TIMESTAMP") == 0:
        raise ProtocolError("INVALID_TIMESTAMP", "wall_time_ns must be positive")
    _require_exact_keys(
        message,
        {
            "schema_version",
            "message_type",
            "request_id",
            "sequence",
            "monotonic_ns",
            "wall_time_ns",
            "action",
        },
        context="action",
    )
    action = message.get("action")
    if not isinstance(action, Mapping):
        raise ProtocolError("INVALID_ACTION", "action must be a map")
    kind = action.get("kind")
    if kind == "absolute_joints":
        if set(action) != {
            "kind",
            "joint_target",
            "timeout_s",
            "position_tolerance_rad",
        }:
            raise ProtocolError(
                "INVALID_ACTION",
                "absolute_joints accepts only kind and six-joint joint_target",
            )
        _require_vector(action, "joint_target", 6)
        _require_positive_number(action, "timeout_s")
        _require_positive_number(action, "position_tolerance_rad")
        return message
    if kind == "gripper":
        if set(action) != {
            "kind",
            "open_fraction",
            "stop_on_contact",
            "timeout_s",
            "position_tolerance_fraction",
        }:
            raise ProtocolError(
                "INVALID_ACTION", "gripper accepts only kind and open_fraction"
            )
        gripper = action.get("open_fraction")
        if isinstance(gripper, bool) or not isinstance(gripper, (int, float)):
            raise ProtocolError("INVALID_GRIPPER", "open_fraction must be numeric")
        if not np.isfinite(float(gripper)) or not 0.0 <= float(gripper) <= 1.0:
            raise ProtocolError("INVALID_GRIPPER", "open_fraction must be within [0, 1]")
        if not isinstance(action.get("stop_on_contact"), bool):
            raise ProtocolError("INVALID_ACTION", "stop_on_contact must be bool")
        _require_positive_number(action, "timeout_s")
        tolerance = _require_positive_number(action, "position_tolerance_fraction")
        if tolerance > 1.0:
            raise ProtocolError(
                "INVALID_GRIPPER", "position_tolerance_fraction must not exceed 1"
            )
        return message
    if kind == "joint_trajectory":
        common = {
            "kind",
            "waypoints",
            "timeout_s",
            "position_tolerance_rad",
        }
        timing = {"waypoint_times_s", "control_hz"} & set(action)
        if len(timing) != 1 or set(action) != common | timing:
            raise ProtocolError(
                "INVALID_ACTION",
                "joint_trajectory requires exactly one of waypoint_times_s or control_hz",
            )
        waypoints = _require_matrix(action, "waypoints", 6)
        _require_positive_number(action, "timeout_s")
        _require_positive_number(action, "position_tolerance_rad")
        if "control_hz" in timing:
            _require_positive_number(action, "control_hz")
        else:
            times = _require_vector(action, "waypoint_times_s", len(waypoints))
            if times[0] < 0 or (len(times) > 1 and not np.all(np.diff(times) > 0)):
                raise ProtocolError(
                    "INVALID_ACTION",
                    "waypoint_times_s must be non-negative and strictly increasing",
                )
        return message
    if kind == "cancel_trajectory":
        if set(action) != {"kind", "trajectory_request_id"}:
            raise ProtocolError(
                "INVALID_ACTION",
                "cancel_trajectory accepts only kind and trajectory_request_id",
            )
        _require_nonnegative_int(action, "trajectory_request_id", "INVALID_ACTION")
        return message
    if kind == "heartbeat":
        if set(action) != {"kind", "action_request_id"}:
            raise ProtocolError(
                "INVALID_ACTION", "heartbeat accepts only kind and action_request_id"
            )
        _require_nonnegative_int(action, "action_request_id", "INVALID_ACTION")
        return message
    if kind == "cartesian_pose":
        if set(action) != {"kind", "frame", "tcp", "pose", "timeout_s"}:
            raise ProtocolError(
                "INVALID_ACTION",
                "cartesian_pose requires frame, tcp, pose, and timeout_s",
            )
        if action.get("frame") != "world":
            raise ProtocolError("INVALID_ACTION", "cartesian frame must be 'world'")
        if action.get("tcp") != "grasp_site":
            raise ProtocolError("INVALID_ACTION", "cartesian tcp must be 'grasp_site'")
        pose = _require_vector(action, "pose", 7)
        quaternion_norm = float(np.linalg.norm(pose[3:]))
        if not np.isclose(quaternion_norm, 1.0, atol=1e-4):
            raise ProtocolError(
                "INVALID_ACTION",
                f"cartesian quaternion wxyz must be unit length, got {quaternion_norm}",
            )
        _require_positive_number(action, "timeout_s")
        return message
    if kind == "joint_program":
        # 2026-09-14: an authored 50 Hz joint/gripper program (agp_yam_bridge.program). Only the
        # envelope is checked here; the program body has its own strict schema in the controller,
        # which also writes the per-program report the client retrieves with program_report.
        if set(action) != {"kind", "program_id", "program"}:
            raise ProtocolError(
                "INVALID_ACTION", "joint_program accepts only kind, program_id and program"
            )
        program_id = action.get("program_id")
        if not isinstance(program_id, str) or not re.fullmatch(PROGRAM_ID_PATTERN, program_id):
            raise ProtocolError(
                "INVALID_ACTION", "program_id must be 1-64 characters of [A-Za-z0-9_-]"
            )
        if not isinstance(action.get("program"), Mapping):
            raise ProtocolError(
                "INVALID_ACTION",
                "program must be a map with times_s, joint_deltas_rad and gripper_events",
            )
        return message
    raise ProtocolError(
        "INVALID_ACTION",
        "unsupported action kind",
    )


def validate_observation_request(message: Mapping[str, Any]) -> Mapping[str, Any]:
    _validate_envelope(message, "request")
    method = message.get("method")
    if method == "get_observation":
        _require_exact_keys(
            message,
            {"schema_version", "message_type", "request_id", "method"},
            context="get_observation request",
        )
        return message
    if method == "solve_ik":
        _require_exact_keys(
            message,
            {
                "schema_version",
                "message_type",
                "request_id",
                "method",
                "target_pose",
                "seed_joints",
            },
            context="solve_ik request",
        )
        pose = _require_vector(message, "target_pose", 7)
        quaternion_norm = float(np.linalg.norm(pose[3:]))
        if not np.isclose(quaternion_norm, 1.0, atol=1e-4):
            raise ProtocolError(
                "INVALID_QUATERNION",
                f"target_pose wxyz quaternion must be unit length, got {quaternion_norm}",
            )
        _require_vector(message, "seed_joints", 6)
        return message
    if method == "solve_position_ik":
        _require_exact_keys(
            message,
            {
                "schema_version",
                "message_type",
                "request_id",
                "method",
                "target_position",
                "seed_joints",
            },
            context="solve_position_ik request",
        )
        _require_vector(message, "target_position", 3)
        _require_vector(message, "seed_joints", 6)
        return message
    raise ProtocolError(
        "UNKNOWN_METHOD",
        "method must be 'get_observation', 'solve_ik', or 'solve_position_ik'",
    )


def validate_program_request(message: Mapping[str, Any]) -> Mapping[str, Any]:
    """2026-09-14 buffered joint programs: preview_program {program} / program_report {program_id}."""
    _validate_envelope(message, "request")
    method = message.get("method")
    common = {"schema_version", "message_type", "request_id", "method"}
    if method == "preview_program":
        _require_exact_keys(message, common | {"program"}, context="preview_program request")
        if not isinstance(message.get("program"), Mapping):
            raise ProtocolError("INVALID_ARGUMENT", "program must be a map")
        return message
    if method == "program_report":
        _require_exact_keys(message, common | {"program_id"}, context="program_report request")
        program_id = message.get("program_id")
        if not isinstance(program_id, str) or not re.fullmatch(PROGRAM_ID_PATTERN, program_id):
            raise ProtocolError(
                "INVALID_ARGUMENT", "program_id must be 1-64 characters of [A-Za-z0-9_-]"
            )
        return message
    raise ProtocolError(
        "UNKNOWN_METHOD", "method must be 'preview_program' or 'program_report'"
    )


def program_result_response(request_id: int, result: Any) -> dict[str, Any]:
    """Envelope of a preview_program / program_report answer (the payload is the controller's dict)."""
    return {
        "schema_version": SCHEMA_VERSION,
        "message_type": "program_result",
        "request_id": request_id,
        "monotonic_ns": time.monotonic_ns(),
        "result": result,
    }


def ik_result_response(
    request_id: int, joint_positions: np.ndarray | None
) -> dict[str, Any]:
    solution = (
        None
        if joint_positions is None
        else np.asarray(joint_positions, dtype=np.float32)
    )
    message = {
        "schema_version": SCHEMA_VERSION,
        "message_type": "ik_result",
        "request_id": request_id,
        "monotonic_ns": time.monotonic_ns(),
        "result": {
            "success": solution is not None,
            "joint_positions": solution,
        },
    }
    validate_ik_result_message(message)
    return message


def validate_ik_result_message(message: Mapping[str, Any]) -> Mapping[str, Any]:
    _validate_envelope(message, "ik_result")
    _require_exact_keys(
        message,
        {"schema_version", "message_type", "request_id", "monotonic_ns", "result"},
        context="ik_result",
    )
    if _require_nonnegative_int(message, "monotonic_ns", "INVALID_TIMESTAMP") == 0:
        raise ProtocolError("INVALID_TIMESTAMP", "monotonic_ns must be positive")
    result = message.get("result")
    if not isinstance(result, Mapping) or set(result) != {"success", "joint_positions"}:
        raise ProtocolError("INVALID_IK_RESULT", "result must contain success and joint_positions")
    success = result.get("success")
    if not isinstance(success, bool):
        raise ProtocolError("INVALID_IK_RESULT", "success must be bool")
    solution = result.get("joint_positions")
    if success:
        _require_vector(result, "joint_positions", 6)
    elif solution is not None:
        raise ProtocolError(
            "INVALID_IK_RESULT", "an unsuccessful IK result must not contain joints"
        )
    return message


def error_response(
    request_id: int,
    code: str,
    message: str,
    *,
    retryable: bool = False,
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "message_type": "error",
        "request_id": request_id,
        "monotonic_ns": time.monotonic_ns(),
        "error": {"code": code, "message": message, "retryable": retryable},
    }


def action_result_response(
    *,
    request_id: int,
    action_request_id: int,
    kind: str,
    status: str,
    started_monotonic_ns: int,
    finished_monotonic_ns: int,
    final_joint_pos_0: np.ndarray,
    max_tracking_error_rad: float,
    detail: str,
) -> dict[str, Any]:
    message = {
        "schema_version": SCHEMA_VERSION,
        "message_type": "action_result",
        "request_id": request_id,
        "monotonic_ns": time.monotonic_ns(),
        "wall_time_ns": time.time_ns(),
        "result": {
            "action_request_id": action_request_id,
            "kind": kind,
            "status": status,
            "started_monotonic_ns": started_monotonic_ns,
            "finished_monotonic_ns": finished_monotonic_ns,
            "final_joint_pos_0": final_joint_pos_0,
            "max_tracking_error_rad": max_tracking_error_rad,
            "detail": detail,
        },
    }
    validate_action_result_message(message)
    return message


def validate_action_result_message(
    message: Mapping[str, Any],
) -> Mapping[str, Any]:
    _validate_envelope(message, "action_result")
    _require_exact_keys(
        message,
        {
            "schema_version",
            "message_type",
            "request_id",
            "monotonic_ns",
            "wall_time_ns",
            "result",
        },
        context="action_result",
    )
    if _require_nonnegative_int(message, "monotonic_ns", "INVALID_TIMESTAMP") == 0:
        raise ProtocolError("INVALID_TIMESTAMP", "monotonic_ns must be positive")
    if _require_nonnegative_int(message, "wall_time_ns", "INVALID_TIMESTAMP") == 0:
        raise ProtocolError("INVALID_TIMESTAMP", "wall_time_ns must be positive")
    result = message.get("result")
    if not isinstance(result, Mapping):
        raise ProtocolError("INVALID_ACTION_RESULT", "result must be a map")
    _require_exact_keys(
        result,
        {
            "action_request_id",
            "kind",
            "status",
            "started_monotonic_ns",
            "finished_monotonic_ns",
            "final_joint_pos_0",
            "max_tracking_error_rad",
            "detail",
        },
        context="action result payload",
    )
    _require_nonnegative_int(result, "action_request_id", "INVALID_ACTION_RESULT")
    if result.get("kind") not in {
        "absolute_joints",
        "joint_trajectory",
        "gripper",
        "cartesian_pose",
        "cancel_trajectory",
        "heartbeat",
        "joint_program",
    }:
        raise ProtocolError("INVALID_ACTION_RESULT", "result.kind is invalid")
    if result.get("status") not in {"completed", "cancelled", "heartbeat"}:
        raise ProtocolError("INVALID_ACTION_RESULT", "result.status is invalid")
    started = _require_nonnegative_int(
        result, "started_monotonic_ns", "INVALID_ACTION_RESULT"
    )
    finished = _require_nonnegative_int(
        result, "finished_monotonic_ns", "INVALID_ACTION_RESULT"
    )
    if started == 0 or finished < started:
        raise ProtocolError("INVALID_ACTION_RESULT", "result timing is invalid")
    _require_vector(result, "final_joint_pos_0", 7)
    tracking_error = result.get("max_tracking_error_rad")
    if (
        isinstance(tracking_error, bool)
        or not isinstance(tracking_error, (int, float))
        or not np.isfinite(float(tracking_error))
        or float(tracking_error) < 0
    ):
        raise ProtocolError(
            "INVALID_ACTION_RESULT", "max_tracking_error_rad must be non-negative"
        )
    if not isinstance(result.get("detail"), str):
        raise ProtocolError("INVALID_ACTION_RESULT", "result.detail must be a string")
    return message
