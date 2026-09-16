"""Buffered joint programs over the real TCP contract (2026-09-14)."""

import json
import socket
import threading
import time
from pathlib import Path

import numpy as np

from agp_yam_bridge.config import load_config
from agp_yam_bridge.kinematics import I2rtKinematicsBackend
from agp_yam_bridge.motion import MotionController
from agp_yam_bridge.program_controller import ProgramController
from agp_yam_bridge.protocol import recv_framed, send_framed
from agp_yam_bridge.server import YamBridgeServer
from agp_yam_bridge.source import FakeYamSource

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def _request(sock, message):
    send_framed(sock, message)
    return recv_framed(sock)


def _action(request_id, sequence, action):
    return {
        "schema_version": 1,
        "message_type": "action",
        "request_id": request_id,
        "sequence": sequence,
        "monotonic_ns": time.monotonic_ns(),
        "wall_time_ns": time.time_ns(),
        "action": action,
    }


def _program():
    return {
        "times_s": [0, 0.1, 0.2, 0.3, 0.4],
        "joint_deltas_rad": [[0, 0, 0, -x, 0, 0] for x in [0, 0.0005, 0.0015, 0.0025, 0.003]],
        "gripper_events": [{"time_s": 0.2, "fraction": 1.0}],
    }


def _serve(tmp_path, config_name, programs):
    config = load_config(CONFIG_DIR / config_name)
    source = FakeYamSource(config)
    kinematics = I2rtKinematicsBackend.from_config(config)
    common = dict(action_log_path=tmp_path / "actions.jsonl", heartbeat_timeout_s=2.0)
    controller = (
        ProgramController(source, kinematics, config, log_dir=tmp_path / "programs", **common)
        if programs
        else MotionController(source, kinematics, config, **common)
    )
    server = YamBridgeServer(source, host="127.0.0.1", port=0, motion_controller=controller)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def _heartbeats(address, action_request_id, stop):
    """What the real connector does on its second socket while an action is in flight."""
    def loop():
        with socket.create_connection(address, timeout=2) as sock:
            sequence = 0
            while not stop.wait(0.1):
                try:
                    _request(sock, _action(1000 + sequence, sequence,
                                           {"kind": "heartbeat", "action_request_id": action_request_id}))
                except Exception:  # ACTION_MISMATCH before the lease exists, closed at the end
                    pass
                sequence += 1
    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    return thread


def test_program_bridge_previews_runs_and_reports_over_tcp(tmp_path) -> None:
    server, thread = _serve(tmp_path, "left_arm_throw.yaml", programs=True)
    stop = threading.Event()
    try:
        with socket.create_connection(server.server_address, timeout=15) as sock:
            preview = _request(sock, {
                "schema_version": 1, "message_type": "request", "request_id": 1,
                "method": "preview_program", "program": _program(),
            })
            assert preview["message_type"] == "program_result" and preview["request_id"] == 1
            assert preview["result"]["valid"] and len(preview["result"]["events"]) == 1

            # the program is 0.4 s of arm motion + a 2.4 s jaw ramp: longer than the 2 s heartbeat
            # window, so the run only survives while the second socket keeps heartbeating
            beats = _heartbeats(server.server_address, 2, stop)
            result = _request(sock, _action(2, 0, {
                "kind": "joint_program", "program_id": "tcp_run_1", "program": _program(),
            }))
            stop.set()
            beats.join(timeout=3)
            assert result["message_type"] == "action_result"
            assert result["result"]["kind"] == "joint_program"
            assert result["result"]["status"] == "completed"
            assert result["result"]["action_request_id"] == 2
            assert json.loads(result["result"]["detail"])["measured_status"] == "completed"
            assert len(result["result"]["final_joint_pos_0"]) == 7

            report = _request(sock, {
                "schema_version": 1, "message_type": "request", "request_id": 3,
                "method": "program_report", "program_id": "tcp_run_1",
            })
            assert report["message_type"] == "program_result"
            assert report["result"]["status"] == "completed" and report["result"]["trace"]

            missing = _request(sock, {
                "schema_version": 1, "message_type": "request", "request_id": 4,
                "method": "program_report", "program_id": "never_ran",
            })
            assert missing["message_type"] == "error"
            assert missing["error"]["code"] == "PROGRAM_NOT_FOUND"

            bad = _request(sock, {
                "schema_version": 1, "message_type": "request", "request_id": 5,
                "method": "preview_program", "program": {"times_s": [0, 1]},
            })
            assert bad["message_type"] == "error" and not bad["error"]["retryable"]

            observation = _request(sock, {
                "schema_version": 1, "message_type": "request", "request_id": 6,
                "method": "get_observation",
            })
            assert observation["message_type"] == "observation"
            assert observation["health"]["safety_state"] == "holding"
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_plain_bridge_refuses_programs_as_read_only(tmp_path) -> None:
    """A bridge started with left_arm.yaml (no program limits) keeps the pre-2026-09-14 surface."""
    server, thread = _serve(tmp_path, "left_arm.yaml", programs=False)
    try:
        with socket.create_connection(server.server_address, timeout=5) as sock:
            preview = _request(sock, {
                "schema_version": 1, "message_type": "request", "request_id": 1,
                "method": "preview_program", "program": _program(),
            })
            assert preview["message_type"] == "error" and preview["error"]["code"] == "READ_ONLY"
            result = _request(sock, _action(2, 0, {
                "kind": "joint_program", "program_id": "plain", "program": _program(),
            }))
            assert result["message_type"] == "error" and result["error"]["code"] == "READ_ONLY"
            # ordinary motion is untouched
            joints = np.asarray(server.source.joint_pos[:6], dtype=np.float32)
            ordinary = _request(sock, _action(3, 1, {
                "kind": "absolute_joints", "joint_target": joints,
                "timeout_s": 1.0, "position_tolerance_rad": 0.03,
            }))
            assert ordinary["message_type"] == "action_result"
            assert ordinary["result"]["status"] == "completed"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_program_envelope_is_validated_before_the_controller(tmp_path) -> None:
    server, thread = _serve(tmp_path, "left_arm_throw.yaml", programs=True)
    try:
        with socket.create_connection(server.server_address, timeout=5) as sock:
            for action in (
                {"kind": "joint_program", "program": _program()},                       # no id
                {"kind": "joint_program", "program_id": "bad id!", "program": _program()},
                {"kind": "joint_program", "program_id": "x", "program": "not a map"},
            ):
                response = _request(sock, _action(1, 0, action))
                assert response["message_type"] == "error"
                assert response["error"]["code"] == "INVALID_ACTION"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
