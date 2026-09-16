"""Threaded TCP server for observation and safety-owned YAM motion."""

from __future__ import annotations

import logging
import socketserver
import time
from typing import Any

from agp_yam_bridge.kinematics import (
    KinematicsValidationError,
    TrajectoryValidationError,
)
from agp_yam_bridge.motion import MotionController, MotionFault
from agp_yam_bridge.protocol import (
    PROGRAM_METHODS,
    ProtocolError,
    action_result_response,
    error_response,
    ik_result_response,
    program_result_response,
    recv_framed,
    send_framed,
    validate_action_message,
    validate_observation_message,
    validate_observation_request,
    validate_program_request,
)
from agp_yam_bridge.source import YamSource

logger = logging.getLogger(__name__)


def _request_id(message: Any) -> int:
    if isinstance(message, dict):
        value = message.get("request_id")
        if isinstance(value, int) and not isinstance(value, bool) and value >= 0:
            return value
    return 0


class _Handler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        server: YamBridgeServer = self.server  # type: ignore[assignment]
        last_action_sequence: int | None = None
        while True:
            try:
                message = recv_framed(self.request)
            except EOFError:
                return
            except ProtocolError as exc:
                send_framed(
                    self.request,
                    error_response(0, exc.code, str(exc), retryable=exc.retryable),
                )
                continue

            request_id = _request_id(message)
            try:
                if message.get("message_type") == "action":
                    validate_action_message(message)
                    sequence = int(message["sequence"])
                    if last_action_sequence is not None and sequence <= last_action_sequence:
                        raise ProtocolError(
                            "OUT_OF_ORDER",
                            f"action sequence {sequence} did not advance beyond "
                            f"{last_action_sequence}",
                        )
                    last_action_sequence = sequence
                    response = server.execute_action(message)
                elif isinstance(message, dict) and message.get("method") in PROGRAM_METHODS:
                    # 2026-09-14 buffered joint programs: preview / report through the request
                    # channel, answered by the ProgramController (READ_ONLY on any other bridge)
                    validate_program_request(message)
                    response = server.program_request(message)
                else:
                    validate_observation_request(message)
                    if server.source is None:
                        raise RuntimeError("YAM observation source is not attached")
                    if message["method"] == "solve_ik":
                        response = ik_result_response(
                            request_id,
                            server.source.kinematics.solve_ik(
                                message["target_pose"],
                                seed_joints=message["seed_joints"],
                            ),
                        )
                    elif message["method"] == "solve_position_ik":
                        response = ik_result_response(
                            request_id,
                            server.source.kinematics.solve_position_ik(
                                message["target_position"],
                                seed_joints=message["seed_joints"],
                            ),
                        )
                    else:
                        response = server.source.read(request_id=request_id)
                        response["health"] = server.health()
                        validate_observation_message(response)
            except ProtocolError as exc:
                response = error_response(
                    request_id, exc.code, str(exc), retryable=exc.retryable
                )
            except TrajectoryValidationError as exc:
                response = error_response(request_id, exc.code, str(exc))
            except KinematicsValidationError as exc:
                response = error_response(request_id, "INVALID_ARGUMENT", str(exc))
            except MotionFault as exc:
                response = error_response(request_id, exc.code, str(exc))
            except Exception as exc:
                logger.exception("YAM bridge request failed")
                response = error_response(
                    request_id,
                    "SOURCE_ERROR",
                    f"bridge request failed: {exc}",
                    retryable=True,
                )
            send_framed(self.request, response)


class YamBridgeServer(socketserver.ThreadingTCPServer):
    """A request/response bridge with an optional safety motion owner."""

    allow_reuse_address = True
    daemon_threads = True

    def __init__(
        self,
        source: YamSource | None,
        *,
        host: str,
        port: int,
        motion_controller: MotionController | None = None,
    ) -> None:
        self.source = source
        self.motion_controller = motion_controller
        super().__init__((host, port), _Handler)

    def attach_source(self, source: YamSource) -> None:
        if self.source is not None:
            raise RuntimeError("YAM bridge source is already attached")
        self.source = source

    def attach_motion_controller(self, controller: MotionController) -> None:
        if self.motion_controller is not None:
            raise RuntimeError("YAM bridge motion controller is already attached")
        if self.source is None:
            raise RuntimeError("attach the YAM source before enabling motion")
        self.motion_controller = controller

    def health(self) -> dict[str, Any]:
        if self.motion_controller is None:
            return {
                "state": "ok",
                "source_connected": self.source is not None,
                "motion_enabled": False,
                "safety_state": "idle",
                "active_action_request_id": None,
                "detail": "motion is disabled",
            }
        return self.motion_controller.health()

    def program_request(self, message: dict[str, Any]) -> dict[str, Any]:
        """preview_program / program_report (2026-09-14): only a ProgramController answers them."""
        from agp_yam_bridge.program_controller import ProgramController

        request_id = int(message["request_id"])
        controller = self.motion_controller
        if not isinstance(controller, ProgramController):
            return error_response(
                request_id,
                "READ_ONLY",
                "joint programs are not enabled on this bridge (no program J4 limits in its config)",
            )
        try:
            if message["method"] == "preview_program":
                result = controller.preview(message["program"])
            else:
                result = controller.report(message["program_id"])
        except (ValueError, TrajectoryValidationError) as exc:
            # an authored program that fails its schema or the rate/workspace checks: nothing moved
            return error_response(request_id, getattr(exc, "code", "INVALID_ARGUMENT"), str(exc))
        return program_result_response(request_id, result)

    def execute_action(self, message: dict[str, Any]) -> dict[str, Any]:
        controller = self.motion_controller
        if controller is None or self.source is None:
            return error_response(
                int(message["request_id"]),
                "READ_ONLY",
                "bridge motion is disabled",
            )
        request_id = int(message["request_id"])
        action = dict(message["action"])
        kind = str(action["kind"])
        if kind == "joint_program" and not hasattr(controller, "execute_program"):
            return error_response(
                request_id,
                "READ_ONLY",
                "joint programs are not enabled on this bridge (no program J4 limits in its config)",
            )
        controller.validate_action_timestamp(int(message["monotonic_ns"]))
        if kind == "heartbeat":
            action_request_id = int(action["action_request_id"])
            controller.heartbeat(action_request_id)
            state = self.source.read_motion_state()
            now = time.monotonic_ns()
            result = {
                "request_id": action_request_id,
                "kind": kind,
                "status": "heartbeat",
                "started_monotonic_ns": now,
                "finished_monotonic_ns": now,
                "final_joint_pos_0": state.position.astype("float32"),
                "max_tracking_error_rad": 0.0,
                "detail": "",
            }
        elif kind == "cancel_trajectory":
            action_request_id = int(action["trajectory_request_id"])
            cancelled = controller.cancel(action_request_id)
            state = self.source.read_motion_state()
            now = time.monotonic_ns()
            result = {
                "request_id": action_request_id,
                "kind": kind,
                "status": "cancelled",
                "started_monotonic_ns": now,
                "finished_monotonic_ns": now,
                "final_joint_pos_0": state.position.astype("float32"),
                "max_tracking_error_rad": 0.0,
                "detail": cancelled["detail"],
            }
        else:
            result = controller.execute(request_id=request_id, action=action)
            action_request_id = request_id
        return action_result_response(
            request_id=request_id,
            action_request_id=action_request_id,
            kind=result["kind"],
            status=result["status"],
            started_monotonic_ns=result["started_monotonic_ns"],
            finished_monotonic_ns=result["finished_monotonic_ns"],
            final_joint_pos_0=result["final_joint_pos_0"],
            max_tracking_error_rad=result["max_tracking_error_rad"],
            detail=result["detail"],
        )

    def server_close(self) -> None:
        try:
            try:
                if self.motion_controller is not None:
                    self.motion_controller.close()
            finally:
                if self.source is not None:
                    self.source.close()
        finally:
            super().server_close()
