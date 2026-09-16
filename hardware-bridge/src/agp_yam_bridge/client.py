"""Synchronous read-only client for the YAM bridge's observation socket."""

from __future__ import annotations

import socket
from typing import Any

from agp_yam_bridge.protocol import (
    ProtocolError,
    recv_framed,
    send_framed,
    validate_observation_message,
)


class BridgeDisconnected(ConnectionError):
    """The bridge socket closed or timed out."""


class BridgeResponseError(RuntimeError):
    """The bridge returned a structured protocol/source error."""

    def __init__(self, code: str, message: str, retryable: bool) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code
        self.retryable = retryable


class OutOfOrderObservationError(RuntimeError):
    """A response sequence did not advance strictly."""


class BridgeClient:
    def __init__(
        self,
        host: str,
        port: int,
        *,
        request_timeout_s: float = 1.0,
        stale_after_s: float = 0.5,
    ) -> None:
        self.host = host
        self.port = port
        self.request_timeout_s = request_timeout_s
        self.stale_after_ns = int(stale_after_s * 1e9)
        self._socket: socket.socket | None = None
        self._request_id = 0
        self._last_sequence: int | None = None

    def connect(self) -> None:
        if self._socket is not None:
            return
        try:
            self._socket = socket.create_connection(
                (self.host, self.port), timeout=self.request_timeout_s
            )
            self._socket.settimeout(self.request_timeout_s)
        except OSError as exc:
            raise BridgeDisconnected(
                f"could not connect to YAM bridge at {self.host}:{self.port}: {exc}"
            ) from exc

    def get_observation(self) -> dict[str, Any]:
        self.connect()
        assert self._socket is not None
        request_id = self._request_id
        self._request_id += 1
        try:
            send_framed(
                self._socket,
                {
                    "schema_version": 1,
                    "message_type": "request",
                    "request_id": request_id,
                    "method": "get_observation",
                },
            )
            response = recv_framed(self._socket)
        except (EOFError, OSError, ProtocolError) as exc:
            self.close()
            raise BridgeDisconnected(f"YAM bridge disconnected: {exc}") from exc

        if response.get("request_id") != request_id:
            raise BridgeResponseError(
                "REQUEST_MISMATCH",
                f"received request_id {response.get('request_id')!r}, expected {request_id}",
                False,
            )
        if response.get("message_type") == "error":
            error = response.get("error")
            if not isinstance(error, dict):
                raise BridgeResponseError("INVALID_ERROR", "error payload must be a map", False)
            raise BridgeResponseError(
                str(error.get("code", "UNKNOWN")),
                str(error.get("message", "")),
                bool(error.get("retryable", False)),
            )

        validate_observation_message(response, max_age_ns=self.stale_after_ns)
        sequence = int(response["sequence"])
        if self._last_sequence is not None and sequence <= self._last_sequence:
            raise OutOfOrderObservationError(
                f"observation sequence {sequence} did not advance beyond {self._last_sequence}"
            )
        self._last_sequence = sequence
        return response

    def close(self) -> None:
        sock, self._socket = self._socket, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()

    def __enter__(self) -> BridgeClient:
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
