import threading

import pytest

from agp_yam_bridge.client import (
    BridgeClient,
    BridgeResponseError,
    OutOfOrderObservationError,
)
from agp_yam_bridge.server import YamBridgeServer
from agp_yam_bridge.source import FakeYamSource


def _running_server(source=None):
    server = YamBridgeServer(source or FakeYamSource(), host="127.0.0.1", port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def test_client_reads_increasing_frames_from_fake_bridge() -> None:
    """Validating only a standalone payload would miss request/response divergence."""
    server, thread = _running_server()
    try:
        with BridgeClient(*server.server_address, stale_after_s=0.5) as client:
            first = client.get_observation()
            second = client.get_observation()
        assert (first["sequence"], second["sequence"]) == (0, 1)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_client_rejects_duplicate_or_out_of_order_sequence() -> None:
    """Accepting a replayed frame would disguise a stuck source as live."""
    source = FakeYamSource()
    server, thread = _running_server(source)
    try:
        with BridgeClient(*server.server_address, stale_after_s=0.5) as client:
            client.get_observation()
            source.sequence = 0
            with pytest.raises(OutOfOrderObservationError, match="sequence 0"):
                client.get_observation()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_client_surfaces_structured_source_error() -> None:
    """A malformed hardware frame must stay distinguishable from disconnection."""
    source = FakeYamSource()
    source.joint_pos = source.joint_pos[:6]
    server, thread = _running_server(source)
    try:
        with BridgeClient(*server.server_address, stale_after_s=0.5) as client:
            with pytest.raises(BridgeResponseError) as exc:
                client.get_observation()
        assert exc.value.code == "INVALID_SHAPE"
        assert exc.value.retryable is False
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
