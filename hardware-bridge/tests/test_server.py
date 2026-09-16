import socket
import threading
import time

import numpy as np

from agp_yam_bridge.config import load_config
from agp_yam_bridge.kinematics import I2rtKinematicsBackend
from agp_yam_bridge.motion import MotionController
from agp_yam_bridge.preflight import DEFAULT_CONFIG
from agp_yam_bridge.protocol import recv_framed, send_framed
from agp_yam_bridge.server import YamBridgeServer
from agp_yam_bridge.source import FakeYamSource


def _start_server(source=None):
    server = YamBridgeServer(source or FakeYamSource(), host="127.0.0.1", port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def _request(sock, message):
    send_framed(sock, message)
    return recv_framed(sock)


def test_server_returns_valid_observation_over_real_tcp() -> None:
    """A fake that bypasses framing would not catch a broken bridge transport."""
    server, thread = _start_server()
    try:
        with socket.create_connection(server.server_address, timeout=2) as sock:
            response = _request(
                sock,
                {"schema_version": 1, "message_type": "request", "request_id": 41, "method": "get_observation"},
            )
        assert response["message_type"] == "observation"
        assert response["request_id"] == 41
        assert response["robot_joint_pos_0"].shape == (7,)
        assert response["robot_joint_pos_0"].dtype == np.float32
        assert response["robot_cartesian_pos_0"].shape == (8,)
        assert response["robot_cartesian_pos_0"].dtype == np.float32
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_server_solves_ik_with_source_i2rt_backend_without_enabling_motion() -> None:
    """Planning must reuse the source's validated model and must not issue an action."""
    source = FakeYamSource()
    seed = source.joint_pos[:6].copy()
    target_pose = source.kinematics.forward(seed).astype(np.float32)
    server, thread = _start_server(source)
    try:
        with socket.create_connection(server.server_address, timeout=2) as sock:
            response = _request(
                sock,
                {
                    "schema_version": 1,
                    "message_type": "request",
                    "request_id": 410,
                    "method": "solve_ik",
                    "target_pose": target_pose,
                    "seed_joints": seed.astype(np.float32),
                },
            )
        assert response["message_type"] == "ik_result"
        assert response["request_id"] == 410
        assert response["result"]["success"] is True
        assert response["result"]["joint_positions"].dtype == np.float32
        assert response["result"]["joint_positions"].shape == (6,)
        achieved = source.kinematics.forward(response["result"]["joint_positions"])
        np.testing.assert_allclose(achieved[:3], target_pose[:3], atol=1e-4)
        assert server.motion_controller is None
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_server_solves_recorded_lift_position_without_enabling_motion() -> None:
    """Position-only planning must reach the task_02 lift without dispatching motion."""
    source = FakeYamSource()
    seed = np.array(
        [
            -0.12722209095954895,
            2.1215763092041016,
            1.3815137147903442,
            -0.8295745253562927,
            -0.007438773289322853,
            0.9565499424934387,
        ],
        dtype=np.float32,
    )
    target_position = np.array(
        [0.3768220841884613, -0.04692096635699272, 0.353],
        dtype=np.float32,
    )
    server, thread = _start_server(source)
    try:
        with socket.create_connection(server.server_address, timeout=2) as sock:
            response = _request(
                sock,
                {
                    "schema_version": 1,
                    "message_type": "request",
                    "request_id": 411,
                    "method": "solve_position_ik",
                    "target_position": target_position,
                    "seed_joints": seed,
                },
            )
        assert response["message_type"] == "ik_result"
        assert response["request_id"] == 411
        assert response["result"]["success"] is True
        achieved = source.kinematics.forward(response["result"]["joint_positions"])
        np.testing.assert_allclose(achieved[:3], target_position, atol=1e-4)
        assert server.motion_controller is None
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_server_rejects_out_of_limit_ik_seed_as_non_retryable_argument_error() -> None:
    source = FakeYamSource()
    seed = source.joint_pos[:6].copy()
    # joint2's model range starts at 0.0 rad. IK seeds are MEASURED joints, so they are
    # checked with kinematics._MEASUREMENT_BUFFER_RAD = 0.1 rad of slack (an arm on a hard
    # stop may legitimately report slightly past the model limit). -0.5 rad is out of limit
    # by any reading; the previous -0.01 now falls inside that buffer.
    seed[1] = -0.5
    target_pose = source.kinematics.forward(source.joint_pos[:6]).astype(np.float32)
    server, thread = _start_server(source)
    try:
        with socket.create_connection(server.server_address, timeout=2) as sock:
            for request_id, method, target_field, target in (
                (412, "solve_ik", "target_pose", target_pose),
                (413, "solve_position_ik", "target_position", target_pose[:3]),
            ):
                response = _request(
                    sock,
                    {
                        "schema_version": 1,
                        "message_type": "request",
                        "request_id": request_id,
                        "method": method,
                        target_field: target,
                        "seed_joints": seed,
                    },
                )
                assert response["message_type"] == "error"
                assert response["request_id"] == request_id
                assert response["error"]["code"] == "INVALID_ARGUMENT"
                assert response["error"]["retryable"] is False
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_server_rejects_actions_with_structured_read_only_error() -> None:
    """P1 must not expose a wire path that can reach a motor command."""
    server, thread = _start_server()
    try:
        with socket.create_connection(server.server_address, timeout=2) as sock:
            response = _request(
                sock,
                {
                    "schema_version": 1,
                    "message_type": "action",
                    "request_id": 42,
                    "sequence": 0,
                    "monotonic_ns": time.monotonic_ns(),
                    "wall_time_ns": time.time_ns(),
                        "action": {
                            "kind": "absolute_joints",
                            "joint_target": np.zeros(6, dtype=np.float32),
                            "timeout_s": 1.0,
                            "position_tolerance_rad": 0.01,
                        },
                },
            )
        assert response["message_type"] == "error"
        assert response["request_id"] == 42
        assert response["error"] == {
            "code": "READ_ONLY",
            "message": "bridge motion is disabled",
            "retryable": False,
        }
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_server_rejects_valid_timed_trajectory_as_read_only() -> None:
    """A future schema-1 action must be recognized even while execution is disabled."""
    server, thread = _start_server()
    try:
        with socket.create_connection(server.server_address, timeout=2) as sock:
            response = _request(
                sock,
                {
                    "schema_version": 1,
                    "message_type": "action",
                    "request_id": 44,
                    "sequence": 0,
                    "monotonic_ns": time.monotonic_ns(),
                    "wall_time_ns": time.time_ns(),
                    "action": {
                        "kind": "joint_trajectory",
                        "waypoints": np.zeros((2, 6), dtype=np.float32),
                        "waypoint_times_s": np.array([0.0, 0.1], dtype=np.float32),
                        "timeout_s": 1.0,
                        "position_tolerance_rad": 0.01,
                    },
                },
            )
        assert response["message_type"] == "error"
        assert response["error"]["code"] == "READ_ONLY"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_server_converts_bad_source_shape_to_error_response() -> None:
    """A source regression must not terminate the socket or publish malformed state."""
    source = FakeYamSource()
    source.joint_pos = np.zeros(6, dtype=np.float32)
    server, thread = _start_server(source)
    try:
        with socket.create_connection(server.server_address, timeout=2) as sock:
            response = _request(
                sock,
                {"schema_version": 1, "message_type": "request", "request_id": 43, "method": "get_observation"},
            )
        assert response["message_type"] == "error"
        assert response["error"]["code"] == "INVALID_SHAPE"
        assert response["error"]["retryable"] is False
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_motion_enabled_server_executes_action_and_returns_final_feedback(tmp_path) -> None:
    """Accepting an action without returning measured completion would break blocking moves."""
    config = load_config(DEFAULT_CONFIG)
    source = FakeYamSource(config)
    controller = MotionController(
        source,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "server-actions.jsonl",
        heartbeat_timeout_s=2.0,
    )
    server = YamBridgeServer(
        source,
        host="127.0.0.1",
        port=0,
        motion_controller=controller,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    target = source.joint_pos[:6].copy()
    target[0] += 0.005
    try:
        with socket.create_connection(server.server_address, timeout=2) as sock:
            response = _request(
                sock,
                {
                    "schema_version": 1,
                    "message_type": "action",
                    "request_id": 45,
                    "sequence": 0,
                    "monotonic_ns": time.monotonic_ns(),
                    "wall_time_ns": time.time_ns(),
                    "action": {
                        "kind": "absolute_joints",
                        "joint_target": target.astype(np.float32),
                        "timeout_s": 1.0,
                        "position_tolerance_rad": 0.002,
                    },
                },
            )
        assert response["message_type"] == "action_result"
        assert response["result"]["status"] == "completed"
        assert response["result"]["action_request_id"] == 45
        np.testing.assert_allclose(
            response["result"]["final_joint_pos_0"][:6], target, atol=0.002
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_motion_server_rejects_stale_action_before_dispatch(tmp_path) -> None:
    """A delayed but structurally valid command must never become fresh on receipt."""
    config = load_config(DEFAULT_CONFIG)
    source = FakeYamSource(config)
    initial = source.joint_pos.copy()
    controller = MotionController(
        source,
        I2rtKinematicsBackend.from_config(config),
        config,
        action_log_path=tmp_path / "stale-action.jsonl",
    )
    server = YamBridgeServer(
        source,
        host="127.0.0.1",
        port=0,
        motion_controller=controller,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    target = initial[:6].copy()
    target[0] += 0.005
    try:
        with socket.create_connection(server.server_address, timeout=2) as sock:
            response = _request(
                sock,
                {
                    "schema_version": 1,
                    "message_type": "action",
                    "request_id": 46,
                    "sequence": 0,
                    "monotonic_ns": time.monotonic_ns() - 1_000_000_000,
                    "wall_time_ns": time.time_ns(),
                    "action": {
                        "kind": "absolute_joints",
                        "joint_target": target.astype(np.float32),
                        "timeout_s": 1.0,
                        "position_tolerance_rad": 0.002,
                    },
                },
            )
        assert response["error"]["code"] == "STALE_ACTION"
        np.testing.assert_array_equal(source.joint_pos, initial)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
