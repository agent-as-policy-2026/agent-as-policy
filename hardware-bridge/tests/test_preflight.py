from dataclasses import replace
from pathlib import Path

import pytest

from agp_yam_bridge.config import BridgeConfig, load_config
from agp_yam_bridge.preflight import (
    DEFAULT_CONFIG,
    CameraFacts,
    CanFacts,
    I2rtFacts,
    PreflightError,
    PreflightFacts,
    validate_preflight,
)


def _matching_config_and_facts() -> tuple[BridgeConfig, PreflightFacts]:
    source = Path("/workspace/i2rt")
    arm_model = source / "i2rt/robot_models/arm/yam/v1/yam.xml"
    gripper_model = source / "i2rt/robot_models/gripper/linear_4310/linear_4310.xml"
    station_model = (
        source
        / "i2rt/robot_models/station/yam_station_linear_4310_d405/yam_station_linear_4310_d405.xml"
    )
    config = BridgeConfig.model_validate(
        {
            "schema_version": 1,
            "bridge": {
                "host": "127.0.0.1",
                "port": 9020,
                "request_timeout_s": 1.0,
                "stale_after_s": 0.5,
            },
            "i2rt": {
                "source": source,
                "package_version": "1.1.2",
                "git_revision": "abc123",
                "tracked_diff_sha256": "1" * 64,
            },
            "hardware": {
                "arm": "yam",
                "gripper": "linear_4310",
                "can_channel": "can_follower_r",
                "can_bitrate": 1_000_000,
                "camera_model": "Intel RealSense D405",
                "camera_serial": "353322271910",
                "camera_physical_port": "6-2",
                "arm_model": arm_model,
                "gripper_model": gripper_model,
                "station_model": station_model,
            },
            "camera": {
                "name": "wrist_d405",
                "width": 640,
                "height": 360,
                "fps": 30,
                "frame_timeout_s": 1.0,
                "stale_after_s": 0.25,
                "max_joint_skew_s": 0.05,
                "min_depth_m": 0.01,
                "max_depth_m": 1.0,
                "station_flange_body": "right_gripper",
                "station_camera_body": "right_camera",
                "transform_translation_error_bound_m": 0.0016787199367799639,
            },
            "top_camera": {
                "calibration_path": (
                    Path(__file__).resolve().parents[1] / "acceptance/top/top_brio_calibration.json"
                ),
                "stale_after_s": 0.25,
                "max_joint_skew_s": 0.05,
            },
            "acceptance": {
                "home_joints_deg": [0, 0, 0, 90, 90, 90],
                "workspace": {
                    "frame": "yam_base",
                    "tcp": "grasp_site",
                    "x_m": [-0.30, 0.30],
                    "y_m": [-0.55, -0.12],
                    "z_m": [0.03, 0.35],
                },
                "speed_limits": {
                    "control_frequency_hz": 50,
                    "joint_velocity_deg_s": 10,
                    "joint_acceleration_deg_s2": 20,
                    "cartesian_translation_m_s": 0.03,
                    "cartesian_rotation_deg_s": 10,
                    "gripper_fraction_s": 0.25,
                },
            },
            "safety": {
                "command_heartbeat_timeout_s": 0.5,
                "feedback_stale_after_s": 0.1,
                "max_tracking_error_deg": 5.0,
                "tracking_error_timeout_s": 0.5,
                "joint_settle_tolerance_rad": 0.03,
                "cartesian_position_tolerance_m": 0.0005,
                "cartesian_orientation_tolerance_deg": 0.5,
                "cartesian_settle_gain_s_inv": 4.0,
                "cartesian_settle_max_bias_deg": 2.0,
                "gripper_max_effort_nm": 0.9,
                "gripper_contact_min_effort_nm": 0.2,
                "gripper_contact_release_effort_nm": 0.15,
                "gripper_contact_confirmation_s": 0.1,
                "gripper_contact_max_velocity_fraction_s": 0.1,
                "gripper_contact_min_open_fraction": 0.05,
                "gripper_stall_velocity_fraction_s": 0.01,
                "gripper_stall_error_fraction": 0.02,
                "gripper_stall_timeout_s": 0.5,
                "action_log_path": Path("/tmp/agp-yam-test-actions.jsonl"),
            },
        }
    )
    facts = PreflightFacts(
        i2rt=I2rtFacts(
            source=source,
            package_version="1.1.2",
            git_revision="abc123",
            tracked_diff_sha256="1" * 64,
            get_yam_robot_source=source / "i2rt/robots/get_robot.py",
            arm="yam",
            gripper="linear_4310",
            arm_model=arm_model,
            gripper_model=gripper_model,
            arm_motor_offsets_deg=(0.0, 0.0, 0.0, 5.467439674492972, 0.0, 0.0),
        ),
        can=CanFacts(channel="can_follower_r", kind="can", is_up=True, bitrate=1_000_000),
        camera=CameraFacts(
            model="Intel RealSense D405",
            serial="353322271910",
            physical_port="6-2",
        ),
        station_model=station_model,
    )
    return config, facts


def test_matching_preflight_facts_produce_complete_startup_report() -> None:
    """Dropping any required startup fact from validation or reporting must fail."""
    config, facts = _matching_config_and_facts()

    report = validate_preflight(config, facts)

    for expected in (
        "status: READY",
        "i2rt_source: /workspace/i2rt",
        "i2rt_version: 1.1.2",
        "i2rt_revision: abc123",
        "arm_variant: yam",
        "gripper_variant: linear_4310",
        "can_channel: can_follower_r",
        "arm_motor_offsets_deg: [0.0, 0.0, 0.0, 5.467439674492972, 0.0, 0.0]",
        "camera_serial: 353322271910",
        "camera_stream: wrist_d405 640x360@30 RGB8+Z16",
        "max_camera_joint_skew_ms: 50.0",
        "top_camera_stream: top_brio 1920x1080@30 RGB8",
        "top_camera_serial: B8C7F203",
        "top_camera_calibration_status: PASS",
        "arm_model:",
        "gripper_model:",
        "station_model:",
        "home_joints_deg: [0.0, 0.0, 0.0, 90.0, 90.0, 90.0]",
        "workspace_yam_base_grasp_site_m:",
        "max_joint_velocity_deg_s: 10.0",
        "command_heartbeat_timeout_s: 0.5",
        "max_tracking_error_deg: 5.0",
        "joint_settle_tolerance_rad: 0.03",
        "cartesian_position_tolerance_m: 0.0005",
        "cartesian_orientation_tolerance_deg: 0.5",
        "cartesian_settle_gain_s_inv: 4.0",
        "cartesian_settle_max_bias_deg: 2.0",
        "gripper_max_effort_nm: 0.9",
        "gripper_contact_min_effort_nm: 0.2",
        "gripper_contact_release_effort_nm: 0.15",
        "gripper_contact_confirmation_s: 0.1",
        "gripper_contact_max_velocity_fraction_s: 0.1",
        "gripper_contact_min_open_fraction: 0.05",
        "action_log_path: /tmp/agp-yam-test-actions.jsonl",
    ):
        assert expected in report


@pytest.mark.parametrize(
    "mutate",
    [
        lambda facts: replace(facts, i2rt=replace(facts.i2rt, source=Path("/wrong/i2rt"))),
        lambda facts: replace(facts, i2rt=replace(facts.i2rt, package_version="9.9.9")),
        lambda facts: replace(facts, i2rt=replace(facts.i2rt, git_revision="wrong")),
        lambda facts: replace(facts, i2rt=replace(facts.i2rt, tracked_diff_sha256="2" * 64)),
        lambda facts: replace(
            facts,
            i2rt=replace(
                facts.i2rt, get_yam_robot_source=Path("/vendored/i2rt/robots/get_robot.py")
            ),
        ),
        lambda facts: replace(facts, i2rt=replace(facts.i2rt, arm="yam_pro")),
        lambda facts: replace(facts, i2rt=replace(facts.i2rt, gripper="crank_4310")),
        lambda facts: replace(facts, i2rt=replace(facts.i2rt, arm_model=Path("/wrong/yam.xml"))),
        lambda facts: replace(
            facts, i2rt=replace(facts.i2rt, gripper_model=Path("/wrong/gripper.xml"))
        ),
        lambda facts: replace(facts, can=replace(facts.can, channel="can_follower_l")),
        lambda facts: replace(facts, can=replace(facts.can, kind="ether")),
        lambda facts: replace(facts, can=replace(facts.can, is_up=False)),
        lambda facts: replace(facts, can=replace(facts.can, bitrate=500_000)),
        lambda facts: replace(facts, camera=replace(facts.camera, model="Intel RealSense D435")),
        lambda facts: replace(facts, camera=replace(facts.camera, serial="wrong")),
        lambda facts: replace(facts, camera=replace(facts.camera, physical_port="6-1")),
        lambda facts: replace(facts, station_model=Path("/wrong/station.xml")),
    ],
)
def test_any_startup_fact_mismatch_fails_closed(mutate) -> None:
    """Accepting one mismatched source, device, variant, or model would enable unsafe startup."""
    config, facts = _matching_config_and_facts()

    with pytest.raises(PreflightError, match="preflight failed closed"):
        validate_preflight(config, mutate(facts))


def test_load_config_resolves_all_source_paths_relative_to_the_yaml(tmp_path: Path) -> None:
    """Resolving paths from the process cwd instead of the config location must fail."""
    from agp_yam_bridge.config import load_config

    config_dir = tmp_path / "config"
    config_dir.mkdir()
    path = config_dir / "acceptance.yaml"
    path.write_text(
        """\
schema_version: 1
bridge:
  host: 127.0.0.1
  port: 9020
  request_timeout_s: 1.0
  stale_after_s: 0.5
i2rt:
  source: ../../i2rt
  package_version: 1.1.2
  git_revision: abc123
  tracked_diff_sha256: '1111111111111111111111111111111111111111111111111111111111111111'
hardware:
  arm: yam
  gripper: linear_4310
  can_channel: can_follower_r
  can_bitrate: 1000000
  camera_model: Intel RealSense D405
  camera_serial: '353322271910'
  camera_physical_port: 6-2
  arm_model: ../../i2rt/i2rt/robot_models/arm/yam/v1/yam.xml
  gripper_model: ../../i2rt/i2rt/robot_models/gripper/linear_4310/linear_4310.xml
  station_model: ../../i2rt/i2rt/robot_models/station/yam_station_linear_4310_d405/yam_station_linear_4310_d405.xml
camera:
  name: wrist_d405
  width: 640
  height: 360
  fps: 30
  frame_timeout_s: 1.0
  stale_after_s: 0.25
  max_joint_skew_s: 0.05
  min_depth_m: 0.01
  max_depth_m: 1.0
  station_flange_body: right_gripper
  station_camera_body: right_camera
  transform_translation_error_bound_m: 0.0016787199367799639
top_camera:
  calibration_path: ../acceptance/top_brio_calibration.json
  stale_after_s: 0.25
  max_joint_skew_s: 0.05
acceptance:
  home_joints_deg: [0, 0, 0, 90, 90, 90]
  workspace:
    frame: yam_base
    tcp: grasp_site
    x_m: [-0.30, 0.30]
    y_m: [-0.55, -0.12]
    z_m: [0.03, 0.35]
  speed_limits:
    control_frequency_hz: 50
    joint_velocity_deg_s: 10
    joint_acceleration_deg_s2: 20
    cartesian_translation_m_s: 0.03
    cartesian_rotation_deg_s: 10
    gripper_fraction_s: 0.25
safety:
  command_heartbeat_timeout_s: 0.5
  feedback_stale_after_s: 0.1
  max_tracking_error_deg: 5.0
  tracking_error_timeout_s: 0.5
  joint_settle_tolerance_rad: 0.03
  cartesian_position_tolerance_m: 0.0005
  cartesian_orientation_tolerance_deg: 0.5
  cartesian_settle_gain_s_inv: 4.0
  cartesian_settle_max_bias_deg: 2.0
  gripper_max_effort_nm: 0.9
  gripper_contact_min_effort_nm: 0.2
  gripper_contact_release_effort_nm: 0.15
  gripper_contact_confirmation_s: 0.1
  gripper_contact_max_velocity_fraction_s: 0.1
  gripper_contact_min_open_fraction: 0.05
  gripper_stall_velocity_fraction_s: 0.01
  gripper_stall_error_fraction: 0.02
  gripper_stall_timeout_s: 0.5
  action_log_path: ../logs/actions.jsonl
""",
        encoding="utf-8",
    )

    config = load_config(path)
    expected_source = (config_dir / "../../i2rt").resolve()

    assert config.i2rt.source == expected_source
    assert config.bridge.host == "127.0.0.1"
    assert config.bridge.port == 9020
    assert config.hardware.arm_model == expected_source / "i2rt/robot_models/arm/yam/v1/yam.xml"
    assert (
        config.hardware.gripper_model
        == expected_source / "i2rt/robot_models/gripper/linear_4310/linear_4310.xml"
    )
    assert config.hardware.station_model == (
        expected_source
        / "i2rt/robot_models/station/yam_station_linear_4310_d405/yam_station_linear_4310_d405.xml"
    )
    assert (
        config.top_camera.calibration_path
        == (config_dir / "../acceptance/top_brio_calibration.json").resolve()
    )
    assert config.safety.action_log_path == (config_dir / "../logs/actions.jsonl").resolve()


@pytest.mark.parametrize(
    "mutate",
    [
        lambda data: data.update(schema_version=2),
        lambda data: data["i2rt"].update(tracked_diff_sha256="not-a-sha256"),
        lambda data: data["acceptance"].update(home_joints_deg=[0, 0, 0, 90, 90]),
        lambda data: data["acceptance"].update(home_joints_deg=[float("nan"), 0, 0, 90, 90, 90]),
        lambda data: data["acceptance"]["workspace"].update(x_m=[0.30, -0.30]),
        lambda data: data["acceptance"]["speed_limits"].update(joint_velocity_deg_s=0),
        lambda data: data["camera"].update(max_joint_skew_s=0),
        lambda data: data["camera"].update(min_depth_m=1.0, max_depth_m=0.5),
        lambda data: data["camera"].update(name=""),
    ],
)
def test_invalid_or_ambiguous_acceptance_config_is_rejected(mutate) -> None:
    """Allowing an unknown schema or unusable safety limit must fail this test."""
    from pydantic import ValidationError

    config, _ = _matching_config_and_facts()
    data = config.model_dump()
    mutate(data)

    with pytest.raises(ValidationError):
        BridgeConfig.model_validate(data)


def test_can_probe_parser_preserves_type_up_state_and_bitrate() -> None:
    """Reading only the interface name would miss a down or wrong-bitrate CAN link."""
    from agp_yam_bridge.preflight import parse_can_facts

    payload = """[{"ifname":"can_follower_r","flags":["NOARP","UP","LOWER_UP"],"linkinfo":{"info_kind":"can","info_data":{"bittiming":{"bitrate":1000000}}}}]"""

    assert parse_can_facts(payload, "can_follower_r") == CanFacts(
        channel="can_follower_r",
        kind="can",
        is_up=True,
        bitrate=1_000_000,
    )


def test_realsense_probe_selects_the_configured_serial_and_physical_port() -> None:
    """Selecting the first D405 would silently bind the other arm's camera."""
    from agp_yam_bridge.preflight import parse_realsense_facts

    payload = """\
Device info:
    Name                          :  Intel RealSense D405
    Serial Number                 :  353322271204
    Physical Port                 :  /sys/devices/usb6/6-1/6-1:1.0/video4linux/video10

Device info:
    Name                          :  Intel RealSense D405
    Serial Number                 :  353322271910
    Physical Port                 :  /sys/devices/usb6/6-2/6-2:1.0/video4linux/video4
"""

    assert parse_realsense_facts(payload, "353322271910") == CameraFacts(
        model="Intel RealSense D405",
        serial="353322271910",
        physical_port="6-2",
    )

    with pytest.raises(PreflightError, match="configured camera serial"):
        parse_realsense_facts(payload, "missing")


def test_i2rt_probe_reports_the_actually_imported_local_factory_and_models() -> None:
    """Resolving the vendored controllers copy instead of the configured local i2rt must fail."""
    from agp_yam_bridge.preflight import collect_i2rt_facts

    # The editable i2rt dependency of this package (pyproject.toml [tool.uv.sources]
    # points at ../i2rt): the patched checkout described in third_party/i2rt/UPSTREAM.md.
    source = Path(__file__).resolve().parents[2] / "i2rt"
    if not source.is_dir():
        pytest.skip(f"no local i2rt checkout at {source} (see third_party/i2rt/UPSTREAM.md)")

    config, _ = _matching_config_and_facts()
    config = config.model_copy(update={"i2rt": config.i2rt.model_copy(update={"source": source})})

    facts = collect_i2rt_facts(config)

    assert facts.source == source
    assert facts.get_yam_robot_source == source / "i2rt/robots/get_robot.py"
    assert facts.arm == "yam"
    assert facts.gripper == "linear_4310"
    assert facts.arm_model == source / "i2rt/robot_models/arm/yam/v1/yam.xml"
    assert facts.gripper_model == source / "i2rt/robot_models/gripper/linear_4310/linear_4310.xml"
    assert facts.arm_motor_offsets_deg == pytest.approx(
        (0.0, 0.0, 0.0, 5.467439674492972, 0.0, 0.0)
    )


def test_run_preflight_only_returns_ready_after_collection_and_validation() -> None:
    """Returning success before collected facts are validated must fail this test."""
    from agp_yam_bridge.preflight import run_preflight

    config, facts = _matching_config_and_facts()

    assert run_preflight(config, collector=lambda _: facts).startswith("status: READY\n")

    wrong = replace(facts, camera=replace(facts.camera, serial="wrong"))
    with pytest.raises(PreflightError, match="camera serial"):
        run_preflight(config, collector=lambda _: wrong)


def test_cli_reports_blocked_and_returns_nonzero_on_any_preflight_error(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """Swallowing a startup error or returning zero would allow the bridge to continue."""
    from agp_yam_bridge import preflight

    monkeypatch.setattr(preflight, "load_config", lambda _: _matching_config_and_facts()[0])
    monkeypatch.setattr(
        preflight,
        "collect_preflight_facts",
        lambda _: (_ for _ in ()).throw(PreflightError("camera mismatch")),
    )

    assert preflight.main(["--config", "unused.yaml"]) == 1
    captured = capsys.readouterr()
    assert captured.out == ""
    assert "status: BLOCKED" in captured.err
    assert "camera mismatch" in captured.err


def test_i2rt_fingerprint_tracks_runtime_source_but_not_unrelated_docs(tmp_path: Path) -> None:
    """Hashing the whole repo would block startup after an unrelated README edit."""
    import subprocess

    from agp_yam_bridge.preflight import hardware_source_diff_sha256

    repo = tmp_path / "i2rt"
    package = repo / "i2rt"
    package.mkdir(parents=True)
    (package / "hardware.py").write_text("CHANNEL = 'can0'\n", encoding="utf-8")
    (repo / "pyproject.toml").write_text("[project]\nname = 'i2rt'\n", encoding="utf-8")
    (repo / "README.md").write_text("docs\n", encoding="utf-8")
    subprocess.run(["git", "init", "-q"], cwd=repo, check=True)
    subprocess.run(["git", "config", "user.email", "test@example.com"], cwd=repo, check=True)
    subprocess.run(["git", "config", "user.name", "Test"], cwd=repo, check=True)
    subprocess.run(["git", "add", "."], cwd=repo, check=True)
    subprocess.run(["git", "commit", "-qm", "baseline"], cwd=repo, check=True)
    baseline = hardware_source_diff_sha256(repo)

    (repo / "README.md").write_text("unrelated docs change\n", encoding="utf-8")
    assert hardware_source_diff_sha256(repo) == baseline

    (package / "hardware.py").write_text("CHANNEL = 'can1'\n", encoding="utf-8")
    assert hardware_source_diff_sha256(repo) != baseline


def test_shipped_joint_motion_envelope_uses_twenty_degree_peak_speed() -> None:
    config = load_config(DEFAULT_CONFIG)

    assert config.acceptance.speed_limits.joint_velocity_deg_s == 20.0
    assert config.acceptance.speed_limits.joint_acceleration_deg_s2 == 40.0


def test_first_acceptance_config_pins_every_runtime_safety_threshold() -> None:
    """Hidden controller defaults would make the first-motion envelope irreproducible."""
    config = load_config(DEFAULT_CONFIG)

    # Shipped workspace, aligned with left_arm.yaml / right_arm.yaml (deliberately
    # beyond the arm's reach; see the comment in first_acceptance.yaml). The old
    # constants (-0.60, 0.60) / (-0.75, -0.02) / (-0.06, 0.50) are the P4 first-motion
    # box that the config stopped carrying.
    assert config.acceptance.workspace.x_m == (-2.0, 2.0)
    assert config.acceptance.workspace.y_m == (-2.0, 2.0)
    assert config.acceptance.workspace.z_m == (-0.2, 1.0)
    assert config.safety.command_heartbeat_timeout_s == 0.5
    assert config.safety.feedback_stale_after_s == 0.1
    assert config.safety.max_tracking_error_deg is None
    assert config.safety.tracking_error_timeout_s == 0.5
    assert config.safety.joint_settle_tolerance_rad == 0.03
    assert config.safety.cartesian_position_tolerance_m == 0.0005
    assert config.safety.cartesian_orientation_tolerance_deg == 0.5
    assert config.safety.gripper_max_effort_nm == 0.9
    assert config.safety.gripper_contact_min_effort_nm == 0.2
    assert config.safety.gripper_contact_release_effort_nm == 0.15
    assert config.safety.gripper_contact_confirmation_s == 0.1
    assert config.safety.gripper_contact_max_velocity_fraction_s == 0.1
    assert config.safety.gripper_contact_min_open_fraction == 0.05
    assert config.safety.gripper_stall_velocity_fraction_s == 0.01
    assert config.safety.gripper_stall_error_fraction == 0.02
    assert config.safety.gripper_stall_timeout_s == 0.5
    assert config.safety.action_log_path.name == "actions.jsonl"
