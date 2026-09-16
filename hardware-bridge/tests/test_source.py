import threading
from types import SimpleNamespace

import numpy as np
import pytest
from i2rt.motor_drivers.dm_driver import DMChainEnableHoldingError
from i2rt.robots.get_robot import YamStartupHoldingError

from agp_yam_bridge.camera import (
    CameraFrameError,
    RgbdFrame,
    RgbFrame,
    load_fixed_rgb_calibration,
)
from agp_yam_bridge.config import BridgeConfig, load_config
from agp_yam_bridge.preflight import DEFAULT_CONFIG
from agp_yam_bridge.source import (
    I2rtYamMotionSource,
    I2rtYamSource,
    StartupHoldError,
)


class _FakeRobot:
    def __init__(self) -> None:
        self.closed = False
        self._joint_state = SimpleNamespace(
            timestamp=1.0,
            pos=np.append(np.radians([0, 0, 0, 90, 90, 90]), 0.75),
            vel=np.append(np.arange(6, dtype=np.float64) / 10, 0.05),
            eff=np.append(np.arange(6, dtype=np.float64) / 100, 0.01),
        )
        self._state_lock = threading.Lock()
        normal_motor_state = [
            SimpleNamespace(error_code="0x1", error_message="normal") for _ in range(7)
        ]
        self.motor_chain = SimpleNamespace(
            running=True,
            state=normal_motor_state,
            state_lock=threading.Lock(),
            _control_thread=SimpleNamespace(is_alive=lambda: True),
        )
        self.motor_chain.read_states = lambda: self.motor_chain.state
        self.commands = []
        self.idle_count = 0

    def num_dofs(self) -> int:
        return 7

    def advance_motor_feedback(self) -> None:
        with self.motor_chain.state_lock:
            self.motor_chain.state = [
                SimpleNamespace(error_code="0x1", error_message="normal")
                for _ in range(7)
            ]

    def _motor_state_to_joint_state(self, _motor_state):
        return self._joint_state

    def command_joint_pos(self, command) -> None:
        self.commands.append(np.asarray(command).copy())

    def enter_gravity_comp_idle(self) -> None:
        self.idle_count += 1

    def close(self) -> None:
        self.closed = True


class _FakeCamera:
    def __init__(self, *, monotonic_ns=None, sequence=0, sequence_step=1) -> None:
        self._monotonic_ns = monotonic_ns or __import__("time").monotonic_ns
        self.sequence = sequence
        self.sequence_step = sequence_step
        self.closed = False

    def read(self) -> RgbdFrame:
        timestamp = self._monotonic_ns()
        frame = RgbdFrame(
            sequence=self.sequence,
            monotonic_ns=timestamp,
            wall_time_ns=1_000_000_000,
            device_timestamp_ms=123.0 + self.sequence,
            serial="353322271910",
            rgb=np.ones((360, 640, 3), dtype=np.uint8),
            depth_m=np.full((360, 640), 0.2, dtype=np.float32),
            intrinsics=np.array(
                [[500.0, 0.0, 319.5], [0.0, 501.0, 179.5], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            ),
            distortion_model="inverse_brown_conrady",
            distortion_coefficients=np.array(
                [-0.05, 0.06, 0.0006, 0.0003, -0.02], dtype=np.float64
            ),
        )
        self.sequence += self.sequence_step
        return frame

    def close(self) -> None:
        self.closed = True


class _FakeTopCamera:
    def __init__(self, *, monotonic_ns=None, sequence=0, sequence_step=1) -> None:
        self._monotonic_ns = monotonic_ns or __import__("time").monotonic_ns
        self.sequence = sequence
        self.sequence_step = sequence_step
        self.closed = False
        self.calibration = load_fixed_rgb_calibration(
            _config().top_camera.calibration_path
        )

    def read(self) -> RgbFrame:
        timestamp = self._monotonic_ns()
        frame = RgbFrame(
            sequence=self.sequence,
            monotonic_ns=timestamp,
            wall_time_ns=1_000_000_000,
            serial=self.calibration.serial,
            rgb=np.full(
                (self.calibration.height, self.calibration.width, 3), 127, dtype=np.uint8
            ),
            intrinsics=self.calibration.camera_matrix.copy(),
            camera_to_world=self.calibration.camera_to_world.copy(),
            distortion_model="none",
            distortion_coefficients=np.zeros(5, dtype=np.float64),
        )
        self.sequence += self.sequence_step
        return frame

    def close(self) -> None:
        self.closed = True


def _config() -> BridgeConfig:
    return load_config(DEFAULT_CONFIG)


def test_i2rt_motion_source_owns_only_robot_feedback_and_fk(monkeypatch) -> None:
    """Workspace teaching must not couple arm ownership to either camera."""
    robot = _FakeRobot()
    monkeypatch.setattr(
        "agp_yam_bridge.source.RealSenseRgbdCamera",
        lambda _config: (_ for _ in ()).throw(AssertionError("camera opened")),
    )

    source = I2rtYamMotionSource(
        _config(), robot_factory=lambda **_: robot
    )
    try:
        state = source.read_motion_state()
    finally:
        source.close()

    assert state.position.shape == (7,)
    assert robot.closed is True


def test_i2rt_source_uses_factory_variants_and_builds_six_plus_one_contract() -> None:
    """Reading raw hardware YAML elsewhere or dropping the gripper must fail."""
    robot = _FakeRobot()
    calls = []

    def factory(**kwargs):
        calls.append(kwargs)
        return robot

    camera = _FakeCamera(sequence=17)
    top_camera = _FakeTopCamera(sequence=42)
    source = I2rtYamSource(
        _config(),
        robot_factory=factory,
        camera_source=camera,
        top_camera_source=top_camera,
    )
    try:
        frame = source.read(request_id=23)
    finally:
        source.close()

    assert len(calls) == 1
    assert calls[0]["channel"] == "can_follower_r"
    assert calls[0]["arm_type"].value == "yam"
    assert calls[0]["gripper_type"].value == "linear_4310"
    assert frame["request_id"] == 23
    assert frame["sequence"] == 0
    assert frame["camera_0"]["name"] == "wrist_d405"
    assert frame["camera_0"]["serial"] == "353322271910"
    assert frame["camera_0"]["frame_sequence"] == 17
    assert frame["camera_0"]["observation_sequence"] == 0
    assert frame["camera_0"]["color_order"] == "RGB"
    assert frame["camera_0"]["depth_unit"] == "meter"
    assert frame["camera_0"]["images"]["rgb"].dtype == np.uint8
    assert frame["camera_0"]["images"]["depth"].dtype == np.float32
    assert frame["camera_0"]["camera_to_world"].shape == (4, 4)
    assert frame["camera_0"]["joint_time_delta_ns"] <= 50_000_000
    assert frame["camera_0"].get(
        "transform_translation_error_bound_m"
    ) == pytest.approx(
        0.0016787199367799639
    )
    assert frame["camera_0"]["distortion_model"] == "inverse_brown_conrady"
    assert frame["camera_0"]["distortion_coefficients"].shape == (5,)
    assert frame["camera_1"]["name"] == "top_brio"
    assert frame["camera_1"]["serial"] == "B8C7F203"
    assert frame["camera_1"]["frame_sequence"] == 42
    assert frame["camera_1"]["observation_sequence"] == 0
    assert set(frame["camera_1"]["images"]) == {"rgb"}
    assert frame["camera_1"]["images"]["rgb"].dtype == np.uint8
    assert frame["camera_1"]["intrinsics"].dtype == np.float64
    assert frame["camera_1"]["camera_to_world"].shape == (4, 4)
    assert frame["camera_1"]["joint_time_delta_ns"] <= 50_000_000
    # metrics.validation_translation_max_m of the shipped 1920x1080 PASS artifact
    # acceptance/top/top_brio_calibration.json; 0.0042182384950071134 was the
    # superseded 640x360 calibration's bound.
    assert frame["camera_1"].get(
        "transform_translation_error_bound_m"
    ) == pytest.approx(
        0.023422225446907616
    )
    assert frame["camera_1"]["distortion_model"] == "none"
    np.testing.assert_array_equal(
        frame["camera_1"]["distortion_coefficients"], np.zeros(5)
    )
    np.testing.assert_array_equal(
        frame["robot_joint_pos_0"],
        np.array([0, 0, 0, np.pi / 2, np.pi / 2, np.pi / 2, 0.75], dtype=np.float32),
    )
    np.testing.assert_allclose(
        frame["robot_joint_vel_0"],
        np.array([0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.05], dtype=np.float32),
    )
    np.testing.assert_allclose(
        frame["robot_cartesian_pos_0"],
        np.array(
            [
                0.0009980164,
                -0.17029879,
                0.24750027,
                0.4999995,
                0.49999866,
                -0.49999866,
                0.50000316,
                0.75,
            ],
            dtype=np.float32,
        ),
        atol=1e-6,
    )
    assert frame["health"] == {
        "state": "ok",
        "source_connected": True,
        "motion_enabled": False,
        "safety_state": "idle",
        "active_action_request_id": None,
        "detail": "",
    }
    assert robot.closed
    assert camera.closed
    assert top_camera.closed


def test_i2rt_source_sequence_increments_once_per_snapshot() -> None:
    """Reusing a sequence number would make a new frame indistinguishable from replay."""
    source = I2rtYamSource(
        _config(),
        robot_factory=lambda **_: _FakeRobot(),
        camera_source=_FakeCamera(),
        top_camera_source=_FakeTopCamera(),
    )
    try:
        assert source.read(request_id=1)["sequence"] == 0
        source._robot.advance_motor_feedback()
        assert source.read(request_id=2)["sequence"] == 1
    finally:
        source.close()


def test_i2rt_source_snapshot_sequence_is_independent_of_camera_frame_gaps() -> None:
    """A slower observation client must not mistake skipped D405 frames for lost replies."""
    source = I2rtYamSource(
        _config(),
        robot_factory=lambda **_: _FakeRobot(),
        camera_source=_FakeCamera(sequence=272, sequence_step=4),
        top_camera_source=_FakeTopCamera(sequence=90, sequence_step=3),
    )
    try:
        first = source.read(request_id=1)
        second = source.read(request_id=2)
    finally:
        source.close()

    assert first["sequence"] == first["camera_0"]["observation_sequence"] == 0
    assert second["sequence"] == second["camera_0"]["observation_sequence"] == 1
    assert (first["camera_0"]["frame_sequence"], second["camera_0"]["frame_sequence"]) == (
        272,
        276,
    )
    assert (first["camera_1"]["frame_sequence"], second["camera_1"]["frame_sequence"]) == (
        90,
        93,
    )


def test_i2rt_source_rejects_wrong_robot_dof_before_serving() -> None:
    """Binding another YAM variant must fail before publishing misleading state."""
    wrong = _FakeRobot()
    wrong.num_dofs = lambda: 6

    try:
        I2rtYamSource(
            _config(), robot_factory=lambda **_: wrong, camera_source=_FakeCamera()
        )
    except RuntimeError as exc:
        assert "7 DOF" in str(exc)
    else:
        raise AssertionError("wrong-DOF robot was accepted")
    assert wrong.closed


def test_i2rt_source_closes_robot_if_post_factory_validation_fails() -> None:
    """Once factory ownership transfers, every later constructor error must close it."""
    robot = _FakeRobot()

    def fail_num_dofs():
        raise RuntimeError("could not inspect robot")

    robot.num_dofs = fail_num_dofs
    with pytest.raises(RuntimeError, match="could not inspect"):
        I2rtYamSource(
            _config(), robot_factory=lambda **_: robot, camera_source=_FakeCamera()
        )

    assert robot.closed is True


def test_i2rt_source_does_not_restamp_frozen_joint_state() -> None:
    """A stopped i2rt update loop must not turn cached state into new frames."""
    robot = _FakeRobot()
    joint_ticks = iter((100_000_000,))
    camera_ticks = iter((100_000_000, 600_000_000))
    top_camera_ticks = iter((100_000_000, 600_000_000))
    source = I2rtYamSource(
        _config(),
        robot_factory=lambda **_: robot,
        camera_source=_FakeCamera(monotonic_ns=lambda: next(camera_ticks)),
        top_camera_source=_FakeTopCamera(
            monotonic_ns=lambda: next(top_camera_ticks)
        ),
        monotonic_ns=lambda: next(joint_ticks),
        wall_time_ns=lambda: 1_000_000_000,
    )
    try:
        first = source.read(request_id=1)
        with pytest.raises(CameraFrameError, match="skew"):
            source.read(request_id=2)
    finally:
        source.close()

    assert first["sequence"] == 0
    assert first["monotonic_ns"] == 100_000_000


def test_i2rt_source_uses_motor_feedback_generation_not_poll_timestamp() -> None:
    """The i2rt poll layer restamps cached CAN state and cannot prove freshness."""
    robot = _FakeRobot()
    joint_ticks = iter((100_000_000,))
    camera_ticks = iter((100_000_000, 600_000_000))
    top_camera_ticks = iter((100_000_000, 600_000_000))
    source = I2rtYamSource(
        _config(),
        robot_factory=lambda **_: robot,
        camera_source=_FakeCamera(monotonic_ns=lambda: next(camera_ticks)),
        top_camera_source=_FakeTopCamera(
            monotonic_ns=lambda: next(top_camera_ticks)
        ),
        monotonic_ns=lambda: next(joint_ticks),
        wall_time_ns=lambda: 1_000_000_000,
    )
    try:
        first = source.read(request_id=1)
        robot._joint_state.timestamp += 1.0
        with pytest.raises(CameraFrameError, match="skew"):
            source.read(request_id=2)
    finally:
        source.close()

    assert robot._joint_state.timestamp > 1.0
    assert first["sequence"] == 0
    assert first["monotonic_ns"] == 100_000_000


def test_i2rt_source_retries_when_motor_generation_changes_during_copy() -> None:
    """The generation token and converted motor data must come from one snapshot."""
    robot = _FakeRobot()
    calls = 0

    def racing_read_states():
        nonlocal calls
        calls += 1
        snapshot = robot.motor_chain.state
        if calls == 1:
            robot.advance_motor_feedback()
        return snapshot

    robot.motor_chain.read_states = racing_read_states
    source = I2rtYamSource(
        _config(),
        robot_factory=lambda **_: robot,
        camera_source=_FakeCamera(),
        top_camera_source=_FakeTopCamera(),
    )
    try:
        frame = source.read(request_id=1)
    finally:
        source.close()

    assert calls == 2
    assert frame["sequence"] == 0


def test_i2rt_source_fails_when_motor_chain_stops() -> None:
    """Publishing cached arrays after the motor loop exits would hide disconnection."""
    robot = _FakeRobot()
    source = I2rtYamSource(
        _config(),
        robot_factory=lambda **_: robot,
        camera_source=_FakeCamera(),
        top_camera_source=_FakeTopCamera(),
    )
    try:
        robot.motor_chain.running = False
        with pytest.raises(RuntimeError, match="motor chain is not running"):
            source.read(request_id=1)
    finally:
        source.close()


def test_i2rt_source_fails_when_robot_update_thread_stops() -> None:
    """A live CAN wrapper cannot make a dead robot-state producer healthy."""
    robot = _FakeRobot()
    robot._server_thread = SimpleNamespace(is_alive=lambda: False)
    source = I2rtYamSource(
        _config(),
        robot_factory=lambda **_: robot,
        camera_source=_FakeCamera(),
        top_camera_source=_FakeTopCamera(),
    )
    try:
        with pytest.raises(RuntimeError, match="update thread is not running"):
            source.read(request_id=1)
    finally:
        source.close()


def test_i2rt_startup_failure_transfers_live_hold_to_bridge_error() -> None:
    """A startup hold must remain reachable instead of being lost during unwinding."""
    chain = SimpleNamespace(close=lambda: None)

    def fail_factory(**_):
        raise YamStartupHoldingError(chain, hold_active=True)

    with pytest.raises(StartupHoldError) as exc_info:
        I2rtYamSource(
            _config(), robot_factory=fail_factory, camera_source=_FakeCamera()
        )

    assert exc_info.value.motor_chain is chain
    assert exc_info.value.hold_active is True


def test_partial_motor_enable_hold_reaches_motion_source_owner(monkeypatch) -> None:
    """The exact low-level partial-enable failure must remain owned end to end."""
    motor_chain = SimpleNamespace(startup_hold_confirmed=True)
    monkeypatch.setattr(
        "i2rt.robots.get_robot.DMChainCanInterface",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            DMChainEnableHoldingError(motor_chain)
        ),
    )

    with pytest.raises(StartupHoldError) as exc_info:
        I2rtYamMotionSource(_config())

    assert exc_info.value.motor_chain is motor_chain
    assert exc_info.value.hold_active is True


def test_i2rt_source_exposes_one_full_joint_command_and_verified_idle() -> None:
    """P4 must use i2rt's normalized seven-joint command and gravity-idle API directly."""
    robot = _FakeRobot()
    source = I2rtYamSource(
        _config(),
        robot_factory=lambda **_: robot,
        camera_source=_FakeCamera(),
        top_camera_source=_FakeTopCamera(),
    )
    command = np.append(np.radians([1, 0, 0, 90, 90, 90]), 0.6)
    try:
        source.command_joint_positions(command)
        state = source.read_motion_state()
        source.enter_safe_idle()
    finally:
        source.close()

    np.testing.assert_array_equal(robot.commands, [command])
    assert state.position.shape == (7,)
    assert state.motor_errors == ()
    assert robot.idle_count >= 1


def test_source_close_releases_hardware_even_if_gravity_idle_transition_fails() -> None:
    """An idle API failure during Ctrl-C must not leak the camera or enabled motor chain."""
    robot = _FakeRobot()
    camera = _FakeCamera()
    robot.enter_gravity_comp_idle = lambda: (_ for _ in ()).throw(
        RuntimeError("idle failed")
    )
    source = I2rtYamSource(
        _config(),
        robot_factory=lambda **_: robot,
        camera_source=camera,
        top_camera_source=_FakeTopCamera(),
    )

    with pytest.raises(RuntimeError, match="idle failed"):
        source.close()

    assert camera.closed is True
    assert robot.closed is True
