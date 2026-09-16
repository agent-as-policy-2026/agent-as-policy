from types import SimpleNamespace

import pytest
from test_preflight import _matching_config_and_facts

import agp_yam_bridge.main as bridge_main
from agp_yam_bridge.config import load_config
from agp_yam_bridge.main import build_source
from agp_yam_bridge.preflight import DEFAULT_CONFIG
from agp_yam_bridge.source import FakeYamSource, StartupHoldError


def test_real_source_requires_explicit_startup_motion_acknowledgement() -> None:
    """A nominally read-only CLI must not hide i2rt's startup hold/calibration."""
    config = _matching_config_and_facts()[0]

    with pytest.raises(RuntimeError, match="startup may enable motors"):
        build_source("i2rt", config, acknowledge_i2rt_startup_motion=False)


def test_fake_source_never_constructs_i2rt_robot() -> None:
    """No-hardware contract work must have a structurally safe source."""
    config = load_config(DEFAULT_CONFIG)

    source = build_source("fake", config, acknowledge_i2rt_startup_motion=False)

    assert isinstance(source, FakeYamSource)


def test_real_motion_requires_first_motion_checklist_before_hardware_construction(
    monkeypatch,
) -> None:
    """The software flag alone must not bypass the physical P4 go/no-go checklist."""
    calls = []
    monkeypatch.setattr(bridge_main, "build_source", lambda *args, **kwargs: calls.append(True))

    with pytest.raises(RuntimeError, match="first-motion checklist"):
        bridge_main.main(
            [
                "--source",
                "i2rt",
                "--enable-motion",
                "--acknowledge-i2rt-startup-motion",
                "--port",
                "0",
            ]
        )

    assert calls == []


def test_main_binds_listener_before_constructing_source(monkeypatch) -> None:
    """A busy port must be discovered before real hardware is initialized."""
    calls = []

    monkeypatch.setattr(bridge_main, "load_config", lambda _: _matching_config_and_facts()[0])

    def fail_bind(*args, **kwargs):
        calls.append("bind")
        raise OSError("address already in use")

    monkeypatch.setattr(bridge_main, "YamBridgeServer", fail_bind)
    monkeypatch.setattr(
        bridge_main,
        "build_source",
        lambda *args, **kwargs: calls.append("source"),
    )

    with pytest.raises(OSError, match="address already in use"):
        bridge_main.main(["--source", "fake"])

    assert calls == ["bind"]


def test_main_retains_failed_startup_hold_until_explicit_interrupt(
    monkeypatch, capsys
) -> None:
    """Unwinding must not silently abandon a motor chain that is still holding."""
    config = _matching_config_and_facts()[0]

    class Chain:
        closed = False

        def close(self):
            self.closed = True

    chain = Chain()
    monkeypatch.setattr(bridge_main, "load_config", lambda _: config)
    monkeypatch.setattr(bridge_main, "run_preflight", lambda _: "status: READY")
    monkeypatch.setattr(
        bridge_main,
        "build_source",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            StartupHoldError(chain, hold_active=True)
        ),
    )
    monkeypatch.setattr(
        bridge_main.time, "sleep", lambda _: (_ for _ in ()).throw(KeyboardInterrupt())
    )

    status = bridge_main.main(
        ["--source", "i2rt", "--acknowledge-i2rt-startup-motion", "--port", "0"]
    )

    assert status == 2
    assert chain.closed is True
    assert "status: STARTUP_HOLD" in capsys.readouterr().out


def test_main_closes_attached_source_when_status_output_fails(monkeypatch) -> None:
    """Status output is inside the same ownership guard as the service loop."""
    config = _matching_config_and_facts()[0]
    source = FakeYamSource()
    monkeypatch.setattr(bridge_main, "load_config", lambda _: config)
    monkeypatch.setattr(bridge_main, "build_source", lambda *args, **kwargs: source)
    monkeypatch.setattr(
        "builtins.print",
        lambda *args, **kwargs: (_ for _ in ()).throw(OSError("closed stdout")),
    )

    with pytest.raises(OSError, match="closed stdout"):
        bridge_main.main(["--source", "fake", "--port", "0"])

    assert source.closed is True


def test_startup_hold_survives_status_output_failure_until_interrupt(monkeypatch) -> None:
    """A broken stdout must not skip the operator wait or leak the live chain."""
    config = _matching_config_and_facts()[0]
    chain = SimpleNamespace(closed=False)
    chain.close = lambda: setattr(chain, "closed", True)
    monkeypatch.setattr(bridge_main, "load_config", lambda _: config)
    monkeypatch.setattr(bridge_main, "run_preflight", lambda _: "status: READY")
    monkeypatch.setattr(
        bridge_main,
        "build_source",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            StartupHoldError(chain, hold_active=True)
        ),
    )
    print_calls = 0

    def fail_second_print(*args, **kwargs):
        nonlocal print_calls
        print_calls += 1
        if print_calls == 2:
            raise OSError("closed stdout")

    monkeypatch.setattr("builtins.print", fail_second_print)
    monkeypatch.setattr(
        bridge_main.time, "sleep", lambda _: (_ for _ in ()).throw(KeyboardInterrupt())
    )

    assert (
        bridge_main.main(
            ["--source", "i2rt", "--acknowledge-i2rt-startup-motion", "--port", "0"]
        )
        == 2
    )
    assert chain.closed is True
