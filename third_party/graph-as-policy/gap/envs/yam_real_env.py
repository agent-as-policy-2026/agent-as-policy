# Written for the Agent-as-Policy (AgP) project, 2026.  NOT upstream code:
# this file does not exist in graph-as-policy at
# f09e37e7755d0a908c1b6cf136edf13292b730c0.  It lives inside the vendored
# package because it must be importable as ``gap.envs.yam_real_env`` — the upstream
# connector resolves it by that name.  Licensed Apache-2.0 with the rest of
# this directory (AgP, Copyright (c) 2026 the AgP authors).
# Authorship: written by the AgP authors (not upstream code) as the P4/P5
# bridge client, adapted for this tree; published under Apache-2.0.
# Provenance: third_party/graph-as-policy/UPSTREAM.md.
"""Safety-owned single-arm YAM environment backed by the hardware bridge.

Bridge client for the safety-owned YAM bridge (P4/P5), adapted for this
tree:

- ``make_env`` populates the :class:`gap.envs.registry.EnvConfig` from
  :data:`gap.envs.robot_specs.YAM_REAL_LEFT_SPEC` (robot identity, gripper,
  workspace, planner routing) instead of leaving the Panda defaults. The
  ``robot_spec`` keyword selects :data:`~gap.envs.robot_specs.
  YAM_REAL_RIGHT_SPEC` for the right arm (default: left, unchanged).
- Default bridge port is 9021 (the LEFT arm bridge; the RIGHT arm's bridge
  owns 9022 — ``gap.connector.real("yam_right")`` defaults to it).

The safety machinery — strict observation/action validation, the action-lease
heartbeat thread, staleness/sequence checks — is kept verbatim. The bridge
owns frames (world == left_base), workspace limits, and speed caps; nothing
client-side re-checks them.

TCP frame: the bridge validates and moves the physical ``grasp_site``
(= the sim ``tcp_gap`` frame rotated Rz(pi) about the tool axis). This env
class speaks the BRIDGE (grasp_site) convention verbatim; the tcp_gap
conversion is owned by :class:`gap.connector.real.YamRealConnector` at its
boundary — nothing here converts orientations.
"""

from __future__ import annotations

import math
import socket
import struct
import threading
import time
from collections.abc import Mapping
from typing import TYPE_CHECKING, Any

import msgpack
import msgpack_numpy
import numpy as np

from gap.envs.base_env import BaseEnv

if TYPE_CHECKING:
    from gap.envs.robot_specs import RobotSpec

msgpack_numpy.patch()

_MAX_FRAME_BYTES = 16 * 1024 * 1024
_HOME_JOINTS = tuple(math.radians(v) for v in (0, 0, 0, 88, 90, 90))


class YamBridgeError(RuntimeError):
    def __init__(self, code: str, message: str, retryable: bool = False) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code
        self.retryable = retryable


class YamDisconnectedError(ConnectionError):
    pass


class YamStaleObservationError(YamBridgeError):
    pass


class YamOutOfOrderObservationError(YamBridgeError):
    pass


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise EOFError("bridge closed the connection")
        data.extend(chunk)
    return bytes(data)


def _send_framed(sock: socket.socket, message: Mapping[str, Any]) -> None:
    payload = msgpack.packb(dict(message), use_bin_type=True)
    if len(payload) > _MAX_FRAME_BYTES:
        raise YamBridgeError("FRAME_TOO_LARGE", f"frame is {len(payload)} bytes")
    sock.sendall(struct.pack("!I", len(payload)) + payload)


def _recv_framed(sock: socket.socket) -> dict[str, Any]:
    (size,) = struct.unpack("!I", _recv_exact(sock, 4))
    if size <= 0 or size > _MAX_FRAME_BYTES:
        raise YamBridgeError("INVALID_FRAME_SIZE", f"frame size {size} is not allowed")
    message = msgpack.unpackb(_recv_exact(sock, size), raw=False)
    if not isinstance(message, dict):
        raise YamBridgeError("INVALID_MESSAGE", "top-level response must be a map")
    return message


def _validate_vector(
    message: Mapping[str, Any], field: str, size: int = 7
) -> np.ndarray:
    value = message.get(field)
    if not isinstance(value, np.ndarray) or value.dtype != np.float32:
        raise YamBridgeError("INVALID_DTYPE", f"{field} must be ndarray[float32]")
    if value.shape != (size,):
        raise YamBridgeError("INVALID_SHAPE", f"{field} shape must be ({size},)")
    if not np.isfinite(value).all():
        raise YamBridgeError("NONFINITE", f"{field} contains non-finite values")
    return value


def _validate_camera_payload(
    camera: Any,
    *,
    context: str,
    rgbd: bool,
    expected_name: str,
    observation_sequence: int,
    observation_monotonic_ns: int,
    now_monotonic_ns: int,
    stale_after_ns: int,
    max_joint_skew_ns: int,
) -> tuple[str, dict[str, Any]]:
    if not isinstance(camera, Mapping):
        raise YamBridgeError("INVALID_CAMERA", f"{context} must be a map")
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
    expected_keys = common_keys | rgbd_keys if rgbd else common_keys
    if set(camera) != expected_keys:
        raise YamBridgeError(
            "INVALID_CAMERA", f"{context} has unexpected or missing fields"
        )
    name = camera.get("name")
    serial = camera.get("serial")
    if not isinstance(name, str) or not name or not isinstance(serial, str) or not serial:
        raise YamBridgeError("INVALID_CAMERA", "camera name and serial must be non-empty")
    if name != expected_name:
        raise YamBridgeError(
            "INVALID_CAMERA", f"{context} name must be {expected_name!r}"
        )
    frame_sequence = camera.get("frame_sequence")
    camera_observation_sequence = camera.get("observation_sequence")
    frame_time = camera.get("frame_monotonic_ns")
    frame_wall_time = camera.get("frame_wall_time_ns")
    joint_time = camera.get("joint_monotonic_ns")
    skew_ns = camera.get("joint_time_delta_ns")
    integers = (
        frame_sequence,
        camera_observation_sequence,
        frame_time,
        frame_wall_time,
        joint_time,
        skew_ns,
    )
    if any(isinstance(value, bool) or not isinstance(value, int) for value in integers):
        raise YamBridgeError("INVALID_TIMESTAMP", "camera counters and timestamps must be integers")
    if frame_sequence < 0 or camera_observation_sequence != observation_sequence:
        raise YamBridgeError(
            "INCONSISTENT_OBSERVATION",
            "camera bundle identity differs from observation",
        )
    if frame_time <= 0 or frame_wall_time <= 0 or joint_time <= 0 or skew_ns < 0:
        raise YamBridgeError("INVALID_TIMESTAMP", "camera timestamps must be positive")
    if joint_time != observation_monotonic_ns or skew_ns != abs(frame_time - joint_time):
        raise YamBridgeError(
            "INCONSISTENT_OBSERVATION",
            "camera/joint synchronization metadata is inconsistent",
        )
    if skew_ns > max_joint_skew_ns:
        raise YamBridgeError(
            "CAMERA_JOINT_SKEW",
            f"camera/joint skew {skew_ns / 1e6:.3f}ms exceeds {max_joint_skew_ns / 1e6:.3f}ms",
            True,
        )
    camera_age_ns = now_monotonic_ns - frame_time
    if camera_age_ns < 0:
        raise YamBridgeError("INVALID_TIMESTAMP", "camera timestamp is in the future")
    if camera_age_ns > stale_after_ns:
        raise YamStaleObservationError(
            "STALE_CAMERA",
            f"camera age {camera_age_ns / 1e9:.3f}s exceeds {stale_after_ns / 1e9:.3f}s",
            True,
        )
    if camera.get("color_order") != "RGB":
        raise YamBridgeError("INVALID_CAMERA", "camera color order must be RGB")
    transform_translation_error_bound_m = camera.get(
        "transform_translation_error_bound_m"
    )
    if (
        isinstance(transform_translation_error_bound_m, bool)
        or not isinstance(transform_translation_error_bound_m, (int, float))
        or not np.isfinite(float(transform_translation_error_bound_m))
        or float(transform_translation_error_bound_m) <= 0.0
    ):
        raise YamBridgeError(
            "INVALID_CAMERA",
            f"{context}.transform_translation_error_bound_m must be finite and positive",
        )
    images = camera.get("images")
    expected_images = {"rgb", "depth"} if rgbd else {"rgb"}
    if not isinstance(images, Mapping) or set(images) != expected_images:
        description = "rgb and depth" if rgbd else "rgb"
        raise YamBridgeError(
            "INVALID_CAMERA", f"{context} images must contain exactly {description}"
        )
    rgb = images.get("rgb")
    if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8:
        raise YamBridgeError("INVALID_DTYPE", "camera RGB must be ndarray[uint8]")
    if rgb.ndim != 3 or rgb.shape[0] == 0 or rgb.shape[1] == 0 or rgb.shape[2] != 3:
        raise YamBridgeError("INVALID_SHAPE", "camera RGB shape must be (H,W,3)")
    device_time: float | None = None
    if rgbd:
        if camera.get("depth_unit") != "meter" or camera.get("invalid_depth_value") != 0.0:
            raise YamBridgeError("INVALID_CAMERA", "depth must be meters with invalid value 0")
        raw_device_time = camera.get("device_timestamp_ms")
        if (
            isinstance(raw_device_time, bool)
            or not isinstance(raw_device_time, (int, float))
            or not np.isfinite(float(raw_device_time))
            or raw_device_time < 0
        ):
            raise YamBridgeError("INVALID_TIMESTAMP", "camera device timestamp is invalid")
        device_time = float(raw_device_time)
        depth = images.get("depth")
        if not isinstance(depth, np.ndarray) or depth.dtype != np.float32:
            raise YamBridgeError("INVALID_DTYPE", "camera depth must be ndarray[float32]")
        if depth.shape != rgb.shape[:2]:
            raise YamBridgeError("INVALID_SHAPE", "camera depth must be aligned to RGB")
        if not np.isfinite(depth).all():
            raise YamBridgeError("NONFINITE", "camera depth contains non-finite values")
        if (depth < 0).any() or not (depth > 0).any():
            raise YamBridgeError("INVALID_CAMERA", "camera depth contains no usable samples")

    intrinsics = camera.get("intrinsics")
    if (
        not isinstance(intrinsics, np.ndarray)
        or intrinsics.dtype != np.float64
        or intrinsics.shape != (3, 3)
        or not np.isfinite(intrinsics).all()
        or intrinsics[0, 0] <= 0
        or intrinsics[1, 1] <= 0
        or not np.allclose(intrinsics[2], [0, 0, 1], atol=1e-10)
    ):
        raise YamBridgeError("INVALID_CAMERA", "camera intrinsics must be float64 pinhole K")
    expected_distortion_model = "inverse_brown_conrady" if rgbd else "none"
    distortion_model = camera.get("distortion_model")
    distortion_coefficients = camera.get("distortion_coefficients")
    if distortion_model != expected_distortion_model:
        raise YamBridgeError(
            "INVALID_CAMERA",
            f"{context}.distortion_model must be {expected_distortion_model!r}",
        )
    if (
        not isinstance(distortion_coefficients, np.ndarray)
        or distortion_coefficients.dtype != np.float64
        or distortion_coefficients.shape != (5,)
        or not np.isfinite(distortion_coefficients).all()
        or (not rgbd and not np.array_equal(distortion_coefficients, np.zeros(5)))
    ):
        raise YamBridgeError(
            "INVALID_CAMERA",
            f"{context}.distortion_coefficients violate the stream contract",
        )
    transform = camera.get("camera_to_world")
    if not isinstance(transform, np.ndarray) or transform.dtype != np.float64 or transform.shape != (4, 4):
        raise YamBridgeError("INVALID_TRANSFORM", "camera_to_world must be float64[4,4]")
    rotation = transform[:3, :3]
    if (
        not np.isfinite(transform).all()
        or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-10)
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5)
    ):
        raise YamBridgeError("INVALID_TRANSFORM", "camera_to_world must be rigid")
    from scipy.spatial.transform import Rotation

    quaternion_xyzw = Rotation.from_matrix(rotation).as_quat()
    pose = np.concatenate(
        (transform[:3, 3], quaternion_xyzw[[3, 0, 1, 2]])
    ).astype(np.float64)
    validated = {
        "images": {"rgb": rgb.copy()},
        "intrinsics": intrinsics.copy(),
        "pose": pose,
        "pose_mat": transform.copy(),
        "serial": serial,
        "color_order": "RGB",
        "frame_sequence": frame_sequence,
        "observation_sequence": camera_observation_sequence,
        "frame_monotonic_ns": frame_time,
        "frame_wall_time_ns": frame_wall_time,
        "joint_monotonic_ns": joint_time,
        "joint_time_delta_ns": skew_ns,
        "transform_translation_error_bound_m": float(
            transform_translation_error_bound_m
        ),
        "distortion_model": distortion_model,
        "distortion_coefficients": distortion_coefficients.copy(),
        "age_s": camera_age_ns / 1e9,
    }
    if rgbd:
        validated["images"]["depth"] = depth.copy()
        validated["depth_unit"] = "meter"
        validated["device_timestamp_ms"] = device_time
    return name, validated


class _YamBridgeClient:
    def __init__(
        self,
        host: str,
        port: int,
        request_timeout_s: float,
        stale_after_s: float,
        camera_stale_after_s: float,
        max_camera_joint_skew_s: float,
        heartbeat_interval_s: float = 0.1,
    ):
        self.host = host
        self.port = port
        self.request_timeout_s = request_timeout_s
        self.stale_after_ns = int(stale_after_s * 1e9)
        self.camera_stale_after_ns = int(camera_stale_after_s * 1e9)
        self.max_camera_joint_skew_ns = int(max_camera_joint_skew_s * 1e9)
        self._socket: socket.socket | None = None
        self._closed = False
        self._request_id = 0
        self._request_id_lock = threading.Lock()
        self._action_sequence = 0
        self._last_sequence: int | None = None
        # One framed request/response stream is shared by ObservationStream
        # polling and synchronous robot.* reads. Keep each exchange atomic.
        self._request_lock = threading.RLock()
        self._lease_lock = threading.Lock()
        self._heartbeat_exchange_lock = threading.Lock()
        self._heartbeat_interval_s = heartbeat_interval_s
        self._heartbeat_socket: socket.socket | None = None
        self._heartbeat_thread: threading.Thread | None = None
        self._heartbeat_stop = threading.Event()
        self._lease_request_id: int | None = None
        self._heartbeat_error: BaseException | None = None

    def _next_request_id(self) -> int:
        with self._request_id_lock:
            request_id = self._request_id
            self._request_id += 1
            return request_id

    def _next_action_sequence(self) -> int:
        with self._request_id_lock:
            sequence = self._action_sequence
            self._action_sequence += 1
            return sequence

    def _connect(self) -> socket.socket:
        if self._closed:
            raise YamDisconnectedError("YAM bridge client is closed")
        if self._socket is None:
            try:
                self._socket = socket.create_connection(
                    (self.host, self.port), timeout=self.request_timeout_s
                )
                self._socket.settimeout(self.request_timeout_s)
            except OSError as exc:
                raise YamDisconnectedError(
                    f"could not connect to YAM bridge at {self.host}:{self.port}: {exc}"
                ) from exc
        return self._socket

    def _drop_primary_socket_locked(self) -> None:
        sock, self._socket = self._socket, None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        sock.close()

    def read(self) -> dict[str, Any]:
        with self._request_lock:
            try:
                return self._read_locked()
            except YamBridgeError:
                self._drop_primary_socket_locked()
                raise

    def solve_ik(
        self, target_pose: np.ndarray, *, seed_joints: np.ndarray
    ) -> np.ndarray | None:
        return self._solve_ik_request(
            method="solve_ik",
            target_field="target_pose",
            target=target_pose,
            seed_joints=seed_joints,
        )

    def solve_position_ik(
        self, target_position: np.ndarray, *, seed_joints: np.ndarray
    ) -> np.ndarray | None:
        return self._solve_ik_request(
            method="solve_position_ik",
            target_field="target_position",
            target=target_position,
            seed_joints=seed_joints,
        )

    def _solve_ik_request(
        self,
        *,
        method: str,
        target_field: str,
        target: np.ndarray,
        seed_joints: np.ndarray,
    ) -> np.ndarray | None:
        with self._request_lock:
            sock = self._connect()
            request_id = self._next_request_id()
            try:
                _send_framed(
                    sock,
                    {
                        "schema_version": 1,
                        "message_type": "request",
                        "request_id": request_id,
                        "method": method,
                        target_field: target,
                        "seed_joints": seed_joints,
                    },
                )
                response = _recv_framed(sock)
            except YamBridgeError:
                self._drop_primary_socket_locked()
                raise
            except (EOFError, OSError, msgpack.UnpackException) as exc:
                self._drop_primary_socket_locked()
                raise YamDisconnectedError(
                    f"YAM bridge disconnected during IK: {exc}"
                ) from exc
            try:
                return self._validate_ik_response(response, request_id=request_id)
            except YamBridgeError:
                self._drop_primary_socket_locked()
                raise

    def program_request(self, method: str, **fields: Any) -> dict[str, Any]:
        """2026-09-14 buffered joint programs: preview_program {program} / program_report {program_id}.

        Answered only by a bridge started with program J4 limits; any other bridge returns a
        READ_ONLY error, which surfaces here as YamBridgeError.
        """
        with self._request_lock:
            sock = self._connect()
            request_id = self._next_request_id()
            try:
                _send_framed(
                    sock,
                    {
                        "schema_version": 1,
                        "message_type": "request",
                        "request_id": request_id,
                        "method": method,
                        **fields,
                    },
                )
                response = _recv_framed(sock)
            except YamBridgeError:
                self._drop_primary_socket_locked()
                raise
            except (EOFError, OSError, msgpack.UnpackException) as exc:
                self._drop_primary_socket_locked()
                raise YamDisconnectedError(
                    f"YAM bridge disconnected during {method}: {exc}"
                ) from exc
            if response.get("message_type") == "error":
                self._raise_response_error(response)
            if (
                response.get("request_id") != request_id
                or response.get("message_type") != "program_result"
                or "result" not in response
            ):
                self._drop_primary_socket_locked()
                raise YamBridgeError(
                    "INVALID_ENVELOPE", f"expected a program_result for {method}"
                )
            return response["result"]

    @staticmethod
    def _validate_ik_response(
        response: Mapping[str, Any], *, request_id: int
    ) -> np.ndarray | None:
        schema_version = response.get("schema_version")
        if type(schema_version) is not int or schema_version != 1:
            raise YamBridgeError(
                "UNSUPPORTED_SCHEMA", "IK response schema_version must be 1"
            )
        response_request_id = response.get("request_id")
        if type(response_request_id) is not int or response_request_id < 0:
            raise YamBridgeError(
                "INVALID_REQUEST_ID",
                "IK response request_id must be a non-negative integer",
            )
        if response_request_id != request_id:
            raise YamBridgeError("REQUEST_MISMATCH", "IK result request_id mismatch")
        monotonic_ns = response.get("monotonic_ns")
        if type(monotonic_ns) is not int or monotonic_ns <= 0:
            raise YamBridgeError(
                "INVALID_TIMESTAMP",
                "IK response monotonic_ns must be a positive integer",
            )
        message_type = response.get("message_type")
        if message_type == "error":
            if set(response) != {
                "schema_version",
                "message_type",
                "request_id",
                "monotonic_ns",
                "error",
            }:
                raise YamBridgeError("INVALID_ERROR", "error envelope is invalid")
            error = response.get("error")
            if (
                not isinstance(error, Mapping)
                or set(error) != {"code", "message", "retryable"}
                or not isinstance(error.get("code"), str)
                or not error["code"]
                or not isinstance(error.get("message"), str)
                or not isinstance(error.get("retryable"), bool)
            ):
                raise YamBridgeError("INVALID_ERROR", "error payload is invalid")
            _YamBridgeClient._raise_response_error(response)
        if message_type != "ik_result":
            raise YamBridgeError("INVALID_ENVELOPE", "expected schema-1 ik_result")
        if set(response) != {
            "schema_version",
            "message_type",
            "request_id",
            "monotonic_ns",
            "result",
        }:
            raise YamBridgeError("INVALID_IK_RESULT", "IK result envelope is invalid")
        result = response.get("result")
        if not isinstance(result, Mapping) or set(result) != {"success", "joint_positions"}:
            raise YamBridgeError("INVALID_IK_RESULT", "IK result payload is invalid")
        if not isinstance(result.get("success"), bool):
            raise YamBridgeError("INVALID_IK_RESULT", "IK success must be bool")
        if not result["success"]:
            if result.get("joint_positions") is not None:
                raise YamBridgeError(
                    "INVALID_IK_RESULT",
                    "failed IK must not contain joint positions",
                )
            return None
        return _validate_vector(result, "joint_positions", 6).copy()

    def _read_locked(self) -> dict[str, Any]:
        sock = self._connect()
        request_id = self._next_request_id()
        try:
            _send_framed(
                sock,
                {
                    "schema_version": 1,
                    "message_type": "request",
                    "request_id": request_id,
                    "method": "get_observation",
                },
            )
            response = _recv_framed(sock)
        except YamBridgeError:
            raise
        except (EOFError, OSError, msgpack.UnpackException) as exc:
            raise YamDisconnectedError(f"YAM bridge disconnected: {exc}") from exc

        if response.get("request_id") != request_id:
            raise YamBridgeError(
                "REQUEST_MISMATCH",
                f"received request_id {response.get('request_id')!r}, expected {request_id}",
            )
        if response.get("message_type") == "error":
            error = response.get("error")
            if not isinstance(error, Mapping):
                raise YamBridgeError("INVALID_ERROR", "error payload must be a map")
            raise YamBridgeError(
                str(error.get("code", "UNKNOWN")),
                str(error.get("message", "")),
                bool(error.get("retryable", False)),
            )
        schema_version = response.get("schema_version")
        if (
            isinstance(schema_version, bool)
            or schema_version != 1
            or response.get("message_type") != "observation"
        ):
            raise YamBridgeError("INVALID_ENVELOPE", "expected schema-1 observation response")
        expected_keys = {
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
        }
        if set(response) != expected_keys:
            raise YamBridgeError(
                "INVALID_MESSAGE", "observation has unexpected or missing fields"
            )

        sequence = response.get("sequence")
        timestamp = response.get("monotonic_ns")
        wall_time = response.get("wall_time_ns")
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            raise YamBridgeError("INVALID_SEQUENCE", "sequence must be a non-negative integer")
        if (
            isinstance(timestamp, bool)
            or not isinstance(timestamp, int)
            or timestamp <= 0
            or isinstance(wall_time, bool)
            or not isinstance(wall_time, int)
            or wall_time <= 0
        ):
            raise YamBridgeError("INVALID_TIMESTAMP", "timestamps must be positive integers")
        now_monotonic_ns = time.monotonic_ns()
        age_ns = now_monotonic_ns - timestamp
        if age_ns < 0:
            raise YamBridgeError("INVALID_TIMESTAMP", "observation timestamp is in the future")
        if age_ns > self.stale_after_ns:
            raise YamStaleObservationError(
                "STALE_OBSERVATION",
                f"observation age {age_ns / 1e9:.3f}s exceeds {self.stale_after_ns / 1e9:.3f}s",
                True,
            )
        if self._last_sequence is not None and sequence <= self._last_sequence:
            raise YamOutOfOrderObservationError(
                "OUT_OF_ORDER",
                f"observation sequence {sequence} did not advance beyond {self._last_sequence}",
            )

        position = _validate_vector(response, "robot_joint_pos_0")
        velocity = _validate_vector(response, "robot_joint_vel_0")
        effort = _validate_vector(response, "robot_joint_effort_0")
        cartesian = _validate_vector(response, "robot_cartesian_pos_0", 8)
        quaternion_norm = float(np.linalg.norm(cartesian[3:7]))
        if not np.isclose(quaternion_norm, 1.0, atol=1e-4):
            raise YamBridgeError(
                "INVALID_QUATERNION",
                f"robot_cartesian_pos_0 wxyz quaternion must be unit length, got {quaternion_norm}",
            )
        gripper = float(cartesian[7])
        joint_gripper = float(position[6])
        if not 0.0 <= joint_gripper <= 1.0:
            raise YamBridgeError(
                "INVALID_GRIPPER",
                "robot_joint_pos_0 gripper fraction must be within [0, 1]",
            )
        if not 0.0 <= gripper <= 1.0:
            raise YamBridgeError(
                "INVALID_GRIPPER",
                "robot_cartesian_pos_0 gripper fraction must be within [0, 1]",
            )
        if not np.isclose(gripper, joint_gripper, atol=1e-6, rtol=0.0):
            raise YamBridgeError(
                "INCONSISTENT_OBSERVATION",
                "Cartesian and joint observations disagree on gripper fraction",
            )
        camera_name, camera = _validate_camera_payload(
            response.get("camera_0"),
            context="camera_0",
            rgbd=True,
            expected_name="wrist_d405",
            observation_sequence=sequence,
            observation_monotonic_ns=timestamp,
            now_monotonic_ns=now_monotonic_ns,
            stale_after_ns=self.camera_stale_after_ns,
            max_joint_skew_ns=self.max_camera_joint_skew_ns,
        )
        top_camera_name, top_camera = _validate_camera_payload(
            response.get("camera_1"),
            context="camera_1",
            rgbd=False,
            expected_name="top_brio",
            observation_sequence=sequence,
            observation_monotonic_ns=timestamp,
            now_monotonic_ns=now_monotonic_ns,
            stale_after_ns=self.camera_stale_after_ns,
            max_joint_skew_ns=self.max_camera_joint_skew_ns,
        )
        health = response.get("health")
        if not isinstance(health, Mapping):
            raise YamBridgeError("INVALID_HEALTH", "health must be a map")
        if (
            set(health)
            != {
                "state",
                "source_connected",
                "motion_enabled",
                "safety_state",
                "active_action_request_id",
                "detail",
            }
            or health.get("state") not in {"ok", "degraded", "error"}
            or not isinstance(health.get("source_connected"), bool)
            or (
                health.get("state") == "ok"
                and health.get("source_connected") is not True
            )
            or not isinstance(health.get("motion_enabled"), bool)
            or health.get("safety_state") not in {"idle", "moving", "holding", "fault"}
            or (
                health.get("active_action_request_id") is not None
                and (
                    isinstance(health.get("active_action_request_id"), bool)
                    or not isinstance(health.get("active_action_request_id"), int)
                    or health["active_action_request_id"] < 0
                )
            )
            or (
                health.get("safety_state") in {"moving", "holding"}
                and health.get("active_action_request_id") is None
            )
            or not isinstance(health.get("detail"), str)
        ):
            raise YamBridgeError("INVALID_HEALTH", "health fields violate the P4 contract")
        self._last_sequence = sequence
        observation = {
            "robot_joint_pos_0": position.copy(),
            "robot_joint_vel_0": velocity.copy(),
            "robot_joint_effort_0": effort.copy(),
            "robot_cartesian_pos_0": cartesian.copy(),
            "bridge_sequence": sequence,
            "bridge_monotonic_ns": timestamp,
            "bridge_wall_time_ns": wall_time,
            "bridge_age_s": age_ns / 1e9,
            "bridge_health": dict(health),
        }
        observation[camera_name] = camera
        observation[top_camera_name] = top_camera
        return observation

    @staticmethod
    def _raise_response_error(response: Mapping[str, Any]) -> None:
        error = response.get("error")
        if not isinstance(error, Mapping):
            raise YamBridgeError("INVALID_ERROR", "error payload must be a map")
        raise YamBridgeError(
            str(error.get("code", "UNKNOWN")),
            str(error.get("message", "")),
            bool(error.get("retryable", False)),
        )

    @staticmethod
    def _validate_action_result(
        response: Mapping[str, Any], *, request_id: int, action_request_id: int
    ) -> dict[str, Any]:
        if response.get("message_type") == "error":
            _YamBridgeClient._raise_response_error(response)
        if response.get("request_id") != request_id:
            raise YamBridgeError("REQUEST_MISMATCH", "action result request_id mismatch")
        if response.get("schema_version") != 1 or response.get("message_type") != "action_result":
            raise YamBridgeError("INVALID_ENVELOPE", "expected schema-1 action_result")
        expected_envelope = {
            "schema_version",
            "message_type",
            "request_id",
            "monotonic_ns",
            "wall_time_ns",
            "result",
        }
        if set(response) != expected_envelope:
            raise YamBridgeError("INVALID_ACTION_RESULT", "action result envelope is invalid")
        result = response.get("result")
        expected_result = {
            "action_request_id",
            "kind",
            "status",
            "started_monotonic_ns",
            "finished_monotonic_ns",
            "final_joint_pos_0",
            "max_tracking_error_rad",
            "detail",
        }
        if not isinstance(result, Mapping) or set(result) != expected_result:
            raise YamBridgeError("INVALID_ACTION_RESULT", "action result payload is invalid")
        if result.get("action_request_id") != action_request_id:
            raise YamBridgeError("ACTION_MISMATCH", "action result belongs to another lease")
        final = _validate_vector(result, "final_joint_pos_0")
        if result.get("status") not in {"completed", "cancelled", "heartbeat"}:
            raise YamBridgeError("INVALID_ACTION_RESULT", "action result status is invalid")
        tracking_error = result.get("max_tracking_error_rad")
        if (
            isinstance(tracking_error, bool)
            or not isinstance(tracking_error, (int, float))
            or not np.isfinite(float(tracking_error))
            or tracking_error < 0
        ):
            raise YamBridgeError("INVALID_ACTION_RESULT", "tracking error is invalid")
        return {**result, "final_joint_pos_0": final.copy()}

    def _action_message(self, request_id: int, action: Mapping[str, Any], *, sequence: int) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "message_type": "action",
            "request_id": request_id,
            "sequence": sequence,
            "monotonic_ns": time.monotonic_ns(),
            "wall_time_ns": time.time_ns(),
            "action": dict(action),
        }

    def _set_lease(self, request_id: int | None) -> None:
        with self._lease_lock:
            self._lease_request_id = request_id
            heartbeat_thread = self._heartbeat_thread
            if request_id is not None and (
                heartbeat_thread is None or not heartbeat_thread.is_alive()
            ):
                self._heartbeat_thread = threading.Thread(
                    target=self._heartbeat_loop,
                    name="yam_bridge_heartbeat",
                    daemon=True,
                )
                self._heartbeat_thread.start()

    def _heartbeat_loop(self) -> None:
        heartbeat_sequence = 0
        while not self._heartbeat_stop.wait(self._heartbeat_interval_s):
            with self._lease_lock:
                lease = self._lease_request_id
            if lease is None:
                continue
            try:
                with self._heartbeat_exchange_lock:
                    with self._lease_lock:
                        lease = self._lease_request_id
                    if lease is None:
                        continue
                    if self._heartbeat_socket is None:
                        self._heartbeat_socket = socket.create_connection(
                            (self.host, self.port), timeout=self.request_timeout_s
                        )
                        self._heartbeat_socket.settimeout(self.request_timeout_s)
                    request_id = self._next_request_id()
                    _send_framed(
                        self._heartbeat_socket,
                        self._action_message(
                            request_id,
                            {"kind": "heartbeat", "action_request_id": lease},
                            sequence=heartbeat_sequence,
                        ),
                    )
                    response = _recv_framed(self._heartbeat_socket)
                    heartbeat_sequence += 1
                    self._validate_action_result(
                        response,
                        request_id=request_id,
                        action_request_id=lease,
                    )
            except YamBridgeError as exc:
                if exc.code == "ACTION_MISMATCH":
                    # The action socket may still be validating a large
                    # trajectory before the bridge establishes its lease.
                    # Keep polling while this client still owns the pending
                    # request; the main response remains authoritative for a
                    # rejected or terminal action.
                    with self._lease_lock:
                        if self._lease_request_id == lease:
                            continue
                    continue
                with self._lease_lock:
                    if self._lease_request_id != lease:
                        continue
                    self._heartbeat_error = exc
                    self._heartbeat_thread = None
                    return
            except BaseException as exc:
                with self._lease_lock:
                    if self._lease_request_id != lease:
                        continue
                    self._heartbeat_error = exc
                    self._heartbeat_thread = None
                    return

    def send_action(self, action: Mapping[str, Any], *, timeout_s: float) -> dict[str, Any]:
        with self._request_lock:
            sock = self._connect()
            request_id = self._next_request_id()
            sequence = self._next_action_sequence()
            with self._lease_lock:
                previous_lease = self._lease_request_id
            try:
                with self._heartbeat_exchange_lock:
                    self._set_lease(None)
                    self._heartbeat_error = None
                    _send_framed(
                        sock,
                        self._action_message(request_id, action, sequence=sequence),
                    )
                    self._set_lease(request_id)
                sock.settimeout(timeout_s + self.request_timeout_s)
                response = _recv_framed(sock)
                sock.settimeout(self.request_timeout_s)
            except YamBridgeError:
                self._set_lease(None)
                self._heartbeat_error = None
                self._drop_primary_socket_locked()
                raise
            except (EOFError, OSError, msgpack.UnpackException) as exc:
                self._set_lease(None)
                self._heartbeat_error = None
                self._drop_primary_socket_locked()
                raise YamDisconnectedError(f"YAM bridge disconnected during action: {exc}") from exc
            try:
                result = self._validate_action_result(
                    response,
                    request_id=request_id,
                    action_request_id=request_id,
                )
            except YamBridgeError:
                # 2026-09-03: the bridge rejected or failed the action. When it failed
                # while PLANNING (IK_FAILED, speed-limit timeout, feedback drift) the
                # bridge still holds the PREVIOUS lease with the arm stiff where the last
                # action left it; keep heartbeating that lease so the hold is not dropped
                # into gravity-comp idle half a second later. If the bridge did idle
                # (mid-motion fault) its lease is gone and the heartbeats just get
                # ACTION_MISMATCH, which the heartbeat loop already tolerates.
                self._set_lease(previous_lease)
                self._heartbeat_error = None
                self._drop_primary_socket_locked()
                raise
            except BaseException:
                self._set_lease(None)
                self._heartbeat_error = None
                raise
            if self._heartbeat_error is not None:
                error = self._heartbeat_error
                self._heartbeat_error = None
                self._set_lease(None)
                raise YamDisconnectedError(f"YAM command heartbeat failed: {error}")
            if result["status"] == "cancelled":
                self._set_lease(None)
            return result

    def heartbeat_once(self) -> dict[str, Any]:
        with self._lease_lock:
            lease = self._lease_request_id
        if lease is None:
            raise YamBridgeError("NO_ACTIVE_ACTION", "there is no motion lease to heartbeat")
        with self._request_lock:
            sock = self._connect()
            request_id = self._next_request_id()
            sequence = self._next_action_sequence()
            _send_framed(
                sock,
                self._action_message(
                    request_id,
                    {"kind": "heartbeat", "action_request_id": lease},
                    sequence=sequence,
                ),
            )
            return self._validate_action_result(
                _recv_framed(sock), request_id=request_id, action_request_id=lease
            )

    def close(self) -> None:
        with self._request_lock:
            if self._closed:
                return
            self._closed = True
            with self._lease_lock:
                lease = self._lease_request_id
            if lease is not None and self._socket is not None:
                try:
                    request_id = self._next_request_id()
                    _send_framed(
                        self._socket,
                        self._action_message(
                            request_id,
                            {"kind": "cancel_trajectory", "trajectory_request_id": lease},
                            sequence=self._next_action_sequence(),
                        ),
                    )
                    self._validate_action_result(
                        _recv_framed(self._socket),
                        request_id=request_id,
                        action_request_id=lease,
                    )
                except Exception:
                    pass
            self._set_lease(None)
            self._heartbeat_stop.set()
            heartbeat_thread, self._heartbeat_thread = self._heartbeat_thread, None
            if heartbeat_thread is not None and heartbeat_thread is not threading.current_thread():
                heartbeat_thread.join(timeout=self.request_timeout_s)
            heartbeat_sock, self._heartbeat_socket = self._heartbeat_socket, None
            if heartbeat_sock is not None:
                try:
                    heartbeat_sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                heartbeat_sock.close()
            self._drop_primary_socket_locked()


class YamRealEnv(BaseEnv):
    """P4 environment: validated observations and safety-owned blocking motion."""

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 9021,
        request_timeout_s: float = 1.0,
        stale_after_s: float = 0.5,
        camera_stale_after_s: float = 0.25,
        max_camera_joint_skew_s: float = 0.05,
        heartbeat_interval_s: float = 0.1,
        camera_names: list[str] | None = None,
    ) -> None:
        super().__init__()
        self.camera_names = camera_names or ["wrist_d405", "top_brio"]
        self._client = _YamBridgeClient(
            host,
            port,
            request_timeout_s,
            stale_after_s,
            camera_stale_after_s,
            max_camera_joint_skew_s,
            heartbeat_interval_s,
        )
        self._closed = False
        self._sim_step_count = 0

    def get_observation(self) -> dict[str, Any]:
        if self._closed:
            raise YamDisconnectedError("YAM environment is closed")
        return self._client.read()

    def solve_ik(self, pose: Any, *, seed_joints: Any) -> np.ndarray | None:
        target = np.asarray(pose, dtype=np.float32)
        seed = np.asarray(seed_joints, dtype=np.float32)
        if target.shape != (7,) or not np.isfinite(target).all():
            raise ValueError("IK target must be a finite xyz+wxyz seven-vector")
        if not np.isclose(float(np.linalg.norm(target[3:])), 1.0, atol=1e-4):
            raise ValueError("IK target wxyz quaternion must be unit length")
        if seed.shape != (6,) or not np.isfinite(seed).all():
            raise ValueError("IK seed must be a finite six-joint vector")
        return self._client.solve_ik(target, seed_joints=seed)

    def solve_position_ik(
        self, position: Any, *, seed_joints: Any
    ) -> np.ndarray | None:
        target = np.asarray(position, dtype=np.float32)
        seed = np.asarray(seed_joints, dtype=np.float32)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError("position IK target must be a finite xyz three-vector")
        if seed.shape != (6,) or not np.isfinite(seed).all():
            raise ValueError("position IK seed must be a finite six-joint vector")
        return self._client.solve_position_ik(target, seed_joints=seed)

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        self._sim_step_count = 0
        return self.get_observation(), {}

    def step(
        self, action: Any
    ) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        self.move_to_joints_blocking(action)
        self._sim_step_count += 1
        return self.get_observation(), 0.0, False, False, {}

    def move_to_joints_blocking(
        self,
        joints: Any,
        *,
        timeout_s: float = 10.0,
        tolerance: float = 0.01,
        arm_id: int = 0,
    ) -> dict[str, Any]:
        if arm_id != 0:
            raise ValueError("YAM has exactly one arm with arm_id=0")
        target = np.asarray(joints, dtype=np.float32)
        if target.shape != (6,) or not np.isfinite(target).all():
            raise ValueError("YAM absolute joint target must be a finite six-vector")
        return self._client.send_action(
            {
                "kind": "absolute_joints",
                "joint_target": target,
                "timeout_s": float(timeout_s),
                "position_tolerance_rad": float(tolerance),
            },
            timeout_s=float(timeout_s),
        )

    def stream_joint_trajectory(
        self,
        waypoints: Any,
        *,
        waypoint_times_s: Any | None = None,
        control_hz: float | None = None,
        timeout_s: float = 30.0,
        position_tolerance_rad: float = 0.01,
        arm_id: int = 0,
    ) -> dict[str, Any]:
        if arm_id != 0:
            raise ValueError("YAM has exactly one arm with arm_id=0")
        path = np.asarray(waypoints, dtype=np.float32)
        if path.ndim != 2 or path.shape[0] < 2 or path.shape[1] != 6:
            raise ValueError("YAM trajectory waypoints must have shape (N, 6), N >= 2")
        if not np.isfinite(path).all():
            raise ValueError("YAM trajectory contains non-finite values")
        if (waypoint_times_s is None) == (control_hz is None):
            raise ValueError("provide exactly one of waypoint_times_s or control_hz")
        action: dict[str, Any] = {
            "kind": "joint_trajectory",
            "waypoints": path,
            "timeout_s": float(timeout_s),
            "position_tolerance_rad": float(position_tolerance_rad),
        }
        if waypoint_times_s is not None:
            times = np.asarray(waypoint_times_s, dtype=np.float32)
            if times.shape != (len(path),) or not np.isfinite(times).all():
                raise ValueError("waypoint_times_s must be a finite N-vector")
            action["waypoint_times_s"] = times
        else:
            action["control_hz"] = float(control_hz)
        return self._client.send_action(action, timeout_s=float(timeout_s))

    def _set_gripper(
        self,
        fraction: float,
        arm_id: int = 0,
        *,
        timeout_s: float = 5.0,
        tolerance: float = 0.01,
        stop_on_contact: bool = False,
    ) -> dict[str, Any]:
        if arm_id != 0:
            raise ValueError("YAM has exactly one arm with arm_id=0")
        fraction = float(fraction)
        if not np.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
            raise ValueError("gripper open fraction must be within [0, 1]")
        return self._client.send_action(
            {
                "kind": "gripper",
                "open_fraction": fraction,
                "stop_on_contact": bool(stop_on_contact),
                "timeout_s": float(timeout_s),
                "position_tolerance_fraction": float(tolerance),
            },
            timeout_s=float(timeout_s),
        )

    def move_to_cartesian_blocking(
        self,
        pose: Any,
        *,
        timeout_s: float = 10.0,
        arm_id: int = 0,
    ) -> dict[str, Any]:
        if arm_id != 0:
            raise ValueError("YAM has exactly one arm with arm_id=0")
        target = np.asarray(pose, dtype=np.float32)
        if target.shape != (7,) or not np.isfinite(target).all():
            raise ValueError("Cartesian target must be a finite xyz+wxyz seven-vector")
        quaternion_norm = float(np.linalg.norm(target[3:]))
        if not np.isclose(quaternion_norm, 1.0, atol=1e-4):
            raise ValueError("Cartesian wxyz quaternion must be unit length")
        return self._client.send_action(
            {
                "kind": "cartesian_pose",
                "frame": "world",
                "tcp": "grasp_site",
                "pose": target,
                "timeout_s": float(timeout_s),
            },
            timeout_s=float(timeout_s),
        )

    # ---- buffered joint programs (2026-09-14; bridge kind "joint_program") ----------------
    def run_joint_program(
        self, program: Mapping[str, Any], *, program_id: str, timeout_s: float
    ) -> dict[str, Any]:
        """Execute one authored 50 Hz joint/gripper program on the bridge (blocking).

        The bridge validates the program against its ordinary limits plus the configured J4
        program ceilings, executes it tick by tick and writes a per-program report that
        ``program_report(program_id)`` returns; the action result's ``detail`` is a JSON string
        with the measured status (completed | settle_miss | rejected).
        """
        return self._client.send_action(
            {
                "kind": "joint_program",
                "program_id": str(program_id),
                "program": dict(program),
            },
            timeout_s=float(timeout_s),
        )

    def preview_program(self, program: Mapping[str, Any]) -> dict[str, Any]:
        """Compile + validate a program against the current arm state; no motion."""
        return self._client.program_request("preview_program", program=dict(program))

    def program_report(self, program_id: str) -> dict[str, Any]:
        return self._client.program_request("program_report", program_id=str(program_id))

    def _step_once(self) -> None:
        self._client.heartbeat_once()
        self._sim_step_count += 1

    def compute_reward(self) -> float:
        return 0.0

    def task_completed(self) -> bool:
        return False

    def wait_ready(self, timeout_s: float, poll_s: float = 0.1) -> None:
        deadline = time.monotonic() + timeout_s
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                observation = self.get_observation()
                if observation["bridge_health"]["state"] == "ok":
                    return
            except YamDisconnectedError as exc:
                last_error = exc
            time.sleep(poll_s)
        detail = f": {last_error}" if last_error else ""
        raise YamDisconnectedError(
            f"YAM bridge was not ready within {timeout_s:.1f}s{detail}"
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._client.close()


def make_env(
    suite_name: str = "yam_real",
    task_id: int = 0,
    camera_names: list[str] | None = None,
    enable_render: bool = False,
    *,
    host: str = "127.0.0.1",
    port: int = 9021,
    request_timeout_s: float = 1.0,
    stale_after_s: float = 0.5,
    camera_stale_after_s: float = 0.25,
    max_camera_joint_skew_s: float = 0.05,
    heartbeat_interval_s: float = 0.1,
    robot_spec: str | RobotSpec = "yam_real_left",
    **extra: Any,
):
    """Registry factory for a real YAM arm behind its hardware bridge.

    Default ``port=9021`` is the LEFT bridge (the RIGHT arm's bridge owns
    9022). ``robot_spec`` names the :class:`~gap.envs.robot_specs.RobotSpec`
    the EnvConfig is populated from — ``"yam_real_left"`` (default, the
    historical behaviour) or ``"yam_real_right"``; a ``RobotSpec`` instance is
    accepted too. Only real-YAM bridge specs (6-DOF, ``ik_link ==
    "grasp_site"``) are valid: the bridge protocol is the same for both arms,
    so the spec selects identity / gripper / workspace numbers, nothing else.
    The chosen spec is what the connector, the runtime's ``ctx.robot``, and
    the curobo/geometry ``robot_file`` default all see. NOTE: ``robot_spec``
    does not change the port — pass ``port=`` (or use
    ``gap.connector.real("yam_right")``, which defaults it to 9022).
    """
    from gap.envs.registry import EnvConfig
    from gap.envs.robot_specs import RobotSpec, get_robot_spec

    spec = (
        robot_spec if isinstance(robot_spec, RobotSpec) else get_robot_spec(robot_spec)
    )
    if spec.arm_dof != 6 or spec.ik_link != "grasp_site":
        raise ValueError(
            f"yam_real make_env: robot_spec {spec.name!r} is not a real-YAM "
            "bridge spec (expected 'yam_real_left' or 'yam_real_right')"
        )
    env = YamRealEnv(
        host=host,
        port=port,
        request_timeout_s=request_timeout_s,
        stale_after_s=stale_after_s,
        camera_stale_after_s=camera_stale_after_s,
        max_camera_joint_skew_s=max_camera_joint_skew_s,
        heartbeat_interval_s=heartbeat_interval_s,
        camera_names=camera_names,
    )
    return env, EnvConfig(
        arm_dof=int(spec.arm_dof),
        num_arms=1,
        action_mode="absolute_joints",
        control_freq=50.0,
        home_joints=tuple(spec.home_joints),
        tcp_offset=tuple(spec.tcp_offset),
        tcp_rotation_z=spec.tcp_rotation_z,
        arm_bases=None,
        robot_urdf_path=spec.robot_urdf_path,
        default_cameras=tuple(env.camera_names),
        is_real=True,
        robot_model_source=(
            "i2rt.robots.utils.combine_arm_and_gripper_xml(yam,linear_4310)"
        ),
        tcp_frame="grasp_site",
        robot_name=spec.name,
        joint_names=tuple(spec.joint_names),
        ik_link=spec.ik_link,
        eef_to_ik_link_xyz=tuple(spec.eef_to_ik_link_xyz),
        eef_to_ik_link_wxyz=tuple(spec.eef_to_ik_link_wxyz),
        gripper=spec.gripper,
        workspace=spec.workspace,
        top_down_quat_wxyz=tuple(spec.top_down_quat_wxyz),
        wrist_camera=spec.wrist_camera,
        rgb_only_cameras=tuple(spec.rgb_only_cameras),
        primary_camera=spec.primary_camera,
        planner_robot_file=spec.planner_robot_file,
        planner_tool_frame=spec.planner_tool_frame,
        robot_link_prefixes=tuple(spec.robot_link_prefixes),
        joint_tolerance_rad=float(spec.joint_tolerance_rad),
        move_max_steps=int(spec.move_max_steps),
        post_move_settle_steps=int(spec.post_move_settle_steps),
        hand_to_fingertip_m=float(spec.gripper.hand_to_fingertip_m),
        robot_spec=spec,
    )
