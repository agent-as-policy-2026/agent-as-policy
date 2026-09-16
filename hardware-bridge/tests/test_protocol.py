import time

import numpy as np
import pytest

from agp_yam_bridge.protocol import (
    ProtocolError,
    action_result_response,
    decode_message,
    encode_message,
    validate_action_message,
    validate_action_result_message,
    validate_observation_message,
    validate_observation_request,
)


def _observation(**overrides):
    camera_time = time.monotonic_ns()
    joint_time = camera_time + 2_000_000
    message = {
        "schema_version": 1,
        "message_type": "observation",
        "request_id": 7,
        "sequence": 11,
        "monotonic_ns": joint_time,
        "wall_time_ns": time.time_ns(),
        "robot_joint_pos_0": np.array(
            [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 0.75], dtype=np.float32
        ),
        "robot_joint_vel_0": np.arange(7, dtype=np.float32) / 10,
        "robot_joint_effort_0": np.arange(7, dtype=np.float32) / 100,
        "robot_cartesian_pos_0": np.array(
            [0.1, -0.2, 0.3, 1.0, 0.0, 0.0, 0.0, 0.75], dtype=np.float32
        ),
        "camera_0": {
            "name": "wrist_d405",
            "serial": "353322271910",
            "frame_sequence": 27,
            "observation_sequence": 11,
            "frame_monotonic_ns": camera_time,
            "frame_wall_time_ns": time.time_ns(),
            "device_timestamp_ms": 1234.5,
            "joint_monotonic_ns": joint_time,
            "joint_time_delta_ns": 2_000_000,
            "transform_translation_error_bound_m": 0.0016787199367799639,
            "distortion_model": "inverse_brown_conrady",
            "distortion_coefficients": np.array(
                [-0.05, 0.06, 0.0006, 0.0003, -0.02], dtype=np.float64
            ),
            "color_order": "RGB",
            "depth_unit": "meter",
            "invalid_depth_value": 0.0,
            "images": {
                "rgb": np.ones((2, 4, 3), dtype=np.uint8),
                "depth": np.full((2, 4), 0.2, dtype=np.float32),
            },
            "intrinsics": np.array(
                [[100.0, 0.0, 1.5], [0.0, 101.0, 0.5], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            ),
            "camera_to_world": np.eye(4, dtype=np.float64),
        },
        "camera_1": {
            "name": "top_brio",
            "serial": "B8C7F203",
            "frame_sequence": 31,
            "observation_sequence": 11,
            "frame_monotonic_ns": camera_time,
            "frame_wall_time_ns": time.time_ns(),
            "joint_monotonic_ns": joint_time,
            "joint_time_delta_ns": 2_000_000,
            "transform_translation_error_bound_m": 0.0042182384950071134,
            "distortion_model": "none",
            "distortion_coefficients": np.zeros(5, dtype=np.float64),
            "color_order": "RGB",
            "images": {
                "rgb": np.full((2, 4, 3), 127, dtype=np.uint8),
            },
            "intrinsics": np.array(
                [[102.0, 0.0, 1.5], [0.0, 103.0, 0.5], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            ),
            "camera_to_world": np.eye(4, dtype=np.float64),
        },
        "health": {
            "state": "ok",
            "source_connected": True,
            "motion_enabled": True,
            "safety_state": "idle",
            "active_action_request_id": None,
            "detail": "",
        },
    }
    message.update(overrides)
    return message


def test_observation_msgpack_round_trip_preserves_float32_arrays() -> None:
    """Serializing through plain lists would lose the dtype contract."""
    decoded = decode_message(encode_message(_observation()))

    validated = validate_observation_message(decoded)

    assert validated["robot_joint_pos_0"].dtype == np.float32
    assert validated["robot_joint_pos_0"].shape == (7,)
    assert validated["robot_joint_vel_0"].dtype == np.float32
    assert validated["robot_joint_effort_0"].dtype == np.float32
    assert validated["robot_cartesian_pos_0"].dtype == np.float32
    assert validated["robot_cartesian_pos_0"].shape == (8,)
    assert validated["camera_0"]["images"]["rgb"].dtype == np.uint8
    assert validated["camera_0"]["images"]["depth"].dtype == np.float32
    assert validated["camera_0"]["intrinsics"].dtype == np.float64
    assert validated["camera_0"]["camera_to_world"].shape == (4, 4)
    assert validated["camera_0"]["frame_sequence"] == 27
    assert validated["camera_0"]["observation_sequence"] == 11
    assert validated["camera_0"]["transform_translation_error_bound_m"] == pytest.approx(
        0.0016787199367799639
    )
    assert validated["camera_0"]["distortion_model"] == "inverse_brown_conrady"
    assert validated["camera_0"]["distortion_coefficients"].shape == (5,)
    assert set(validated["camera_1"]["images"]) == {"rgb"}
    assert validated["camera_1"]["images"]["rgb"].dtype == np.uint8
    assert validated["camera_1"]["intrinsics"].dtype == np.float64
    assert validated["camera_1"]["camera_to_world"].shape == (4, 4)
    assert validated["camera_1"]["frame_sequence"] == 31
    assert validated["camera_1"]["observation_sequence"] == 11
    assert validated["camera_1"]["transform_translation_error_bound_m"] == pytest.approx(
        0.0042182384950071134
    )
    assert validated["camera_1"]["distortion_model"] == "none"


def test_solve_ik_request_is_an_explicit_i2rt_planning_contract() -> None:
    """A six-DOF caller must name solve_ik instead of selecting a backend by DOF."""
    request = {
        "schema_version": 1,
        "message_type": "request",
        "request_id": 8,
        "method": "solve_ik",
        "target_pose": np.array(
            [0.1, -0.2, 0.3, 1.0, 0.0, 0.0, 0.0], dtype=np.float32
        ),
        "seed_joints": np.zeros(6, dtype=np.float32),
    }

    assert validate_observation_request(request) is request


def test_solve_position_ik_request_carries_only_position_and_seed() -> None:
    request = {
        "schema_version": 1,
        "message_type": "request",
        "request_id": 9,
        "method": "solve_position_ik",
        "target_position": np.array([0.1, -0.2, 0.3], dtype=np.float32),
        "seed_joints": np.zeros(6, dtype=np.float32),
    }

    assert validate_observation_request(request) is request


@pytest.mark.parametrize(
    ("field", "value", "code"),
    [
        ("target_pose", np.zeros(6, dtype=np.float32), "INVALID_SHAPE"),
        ("target_pose", np.zeros(7, dtype=np.float64), "INVALID_DTYPE"),
        (
            "target_pose",
            np.array([0, 0, 0, 0, 0, 0, 0], dtype=np.float32),
            "INVALID_QUATERNION",
        ),
        ("seed_joints", np.zeros(7, dtype=np.float32), "INVALID_SHAPE"),
    ],
)
def test_solve_ik_request_fails_closed_on_invalid_pose_or_seed(field, value, code) -> None:
    request = {
        "schema_version": 1,
        "message_type": "request",
        "request_id": 8,
        "method": "solve_ik",
        "target_pose": np.array(
            [0.1, -0.2, 0.3, 1.0, 0.0, 0.0, 0.0], dtype=np.float32
        ),
        "seed_joints": np.zeros(6, dtype=np.float32),
    }
    request[field] = value

    with pytest.raises(ProtocolError) as exc:
        validate_observation_request(request)

    assert exc.value.code == code


@pytest.mark.parametrize(
    ("field", "value", "code"),
    [
        ("robot_joint_pos_0", np.zeros(6, dtype=np.float32), "INVALID_SHAPE"),
        ("robot_joint_vel_0", np.zeros(7, dtype=np.float64), "INVALID_DTYPE"),
        ("robot_joint_effort_0", np.full(7, np.nan, dtype=np.float32), "NONFINITE"),
        ("robot_cartesian_pos_0", np.zeros(7, dtype=np.float32), "INVALID_SHAPE"),
        ("sequence", -1, "INVALID_SEQUENCE"),
        ("monotonic_ns", 0, "INVALID_TIMESTAMP"),
    ],
)
def test_observation_rejects_contract_breaks(field, value, code) -> None:
    """Accepting one malformed field would let bad hardware state reach the client."""
    with pytest.raises(ProtocolError) as exc:
        validate_observation_message(_observation(**{field: value}))

    assert exc.value.code == code


@pytest.mark.parametrize(
    ("cartesian", "code"),
    [
        (
            np.array([0.1, -0.2, 0.3, 0.0, 0.0, 0.0, 0.0, 0.75], dtype=np.float32),
            "INVALID_QUATERNION",
        ),
        (
            np.array([0.1, -0.2, 0.3, 1.0, 0.0, 0.0, 0.0, 1.1], dtype=np.float32),
            "INVALID_GRIPPER",
        ),
        (
            np.array([0.1, -0.2, 0.3, 1.0, 0.0, 0.0, 0.0, 0.25], dtype=np.float32),
            "INCONSISTENT_OBSERVATION",
        ),
    ],
)
def test_observation_rejects_invalid_cartesian_semantics(cartesian, code) -> None:
    with pytest.raises(ProtocolError) as exc:
        validate_observation_message(_observation(robot_cartesian_pos_0=cartesian))
    assert exc.value.code == code


@pytest.mark.parametrize(
    ("joint_gripper", "cartesian_gripper", "code"),
    [
        (1.00001, 1.0, "INVALID_GRIPPER"),
        (0.750005, 0.75, "INCONSISTENT_OBSERVATION"),
    ],
)
def test_observation_strictly_validates_joint_gripper_semantics(
    joint_gripper, cartesian_gripper, code
) -> None:
    message = _observation()
    message["robot_joint_pos_0"][6] = joint_gripper
    message["robot_cartesian_pos_0"][7] = cartesian_gripper
    with pytest.raises(ProtocolError) as exc:
        validate_observation_message(message)
    assert exc.value.code == code


def test_observation_rejects_frame_older_than_threshold() -> None:
    """Treating an old frame as current would hide a disconnected bridge."""
    now = time.monotonic_ns()
    frame = _observation(monotonic_ns=now - 500_000_000)

    with pytest.raises(ProtocolError) as exc:
        validate_observation_message(frame, now_monotonic_ns=now, max_age_ns=100_000_000)

    assert exc.value.code == "STALE_OBSERVATION"


def test_schema_version_rejects_bool_and_health_cannot_contradict_ok() -> None:
    with pytest.raises(ProtocolError) as schema_exc:
        validate_observation_message(_observation(schema_version=True))
    assert schema_exc.value.code == "UNSUPPORTED_SCHEMA"

    message = _observation()
    message["health"] = {**message["health"], "source_connected": False}
    with pytest.raises(ProtocolError) as health_exc:
        validate_observation_message(message)
    assert health_exc.value.code == "INVALID_HEALTH"


def test_observation_rejects_unspecified_extra_fields() -> None:
    with pytest.raises(ProtocolError) as exc:
        validate_observation_message(_observation(unversioned_extra="unsafe"))
    assert exc.value.code == "INVALID_MESSAGE"


@pytest.mark.parametrize(
    ("mutate", "code"),
    [
        (lambda camera: camera.update(serial=""), "INVALID_CAMERA"),
        (lambda camera: camera.update(color_order="BGR"), "INVALID_CAMERA"),
        (
            lambda camera: camera["images"].update(
                depth=np.ones((3, 4), dtype=np.float32)
            ),
            "INVALID_SHAPE",
        ),
        (
            lambda camera: camera["images"].update(
                depth=np.full((2, 4), np.nan, dtype=np.float32)
            ),
            "NONFINITE",
        ),
        (
            lambda camera: camera.update(camera_to_world=np.zeros((4, 4), dtype=np.float64)),
            "INVALID_TRANSFORM",
        ),
    ],
)
def test_camera_contract_rejects_unusable_rgbd_or_calibration(mutate, code) -> None:
    message = _observation()
    mutate(message["camera_0"])
    with pytest.raises(ProtocolError) as exc:
        validate_observation_message(message)
    assert exc.value.code == code


@pytest.mark.parametrize(
    ("mutate", "code"),
    [
        (lambda camera: camera.update(serial=""), "INVALID_CAMERA"),
        (lambda camera: camera.update(depth_unit="meter"), "INVALID_MESSAGE"),
        (
            lambda camera: camera["images"].update(
                depth=np.ones((2, 4), dtype=np.float32)
            ),
            "INVALID_CAMERA",
        ),
        (
            lambda camera: camera.update(
                camera_to_world=np.zeros((4, 4), dtype=np.float64)
            ),
            "INVALID_TRANSFORM",
        ),
    ],
)
def test_top_camera_contract_is_strict_rgb_only_with_metric_calibration(
    mutate, code
) -> None:
    message = _observation()
    mutate(message["camera_1"])

    with pytest.raises(ProtocolError) as exc:
        validate_observation_message(message)

    assert exc.value.code == code


def test_camera_contract_rejects_joint_image_skew_and_stale_image() -> None:
    now = time.monotonic_ns()
    message = _observation(monotonic_ns=now)
    message["camera_0"]["frame_monotonic_ns"] = now - 200_000_000
    message["camera_0"]["joint_monotonic_ns"] = now - 140_000_000
    message["camera_0"]["joint_time_delta_ns"] = 60_000_000

    with pytest.raises(ProtocolError) as skew_exc:
        validate_observation_message(message, max_joint_skew_ns=50_000_000)
    assert skew_exc.value.code == "CAMERA_JOINT_SKEW"

    message["camera_0"]["joint_time_delta_ns"] = 10_000_000
    message["camera_0"]["joint_monotonic_ns"] = now - 190_000_000
    with pytest.raises(ProtocolError) as stale_exc:
        validate_observation_message(
            message,
            now_monotonic_ns=now,
            max_camera_age_ns=100_000_000,
        )
    assert stale_exc.value.code == "STALE_CAMERA"


def test_absolute_joint_action_has_six_float32_joints_and_no_gripper() -> None:
    """Putting the gripper in an arm target would violate the independent command contract."""
    action = {
        "schema_version": 1,
        "message_type": "action",
        "request_id": 9,
        "sequence": 3,
        "monotonic_ns": time.monotonic_ns(),
        "wall_time_ns": time.time_ns(),
        "action": {
            "kind": "absolute_joints",
            "joint_target": np.zeros(6, dtype=np.float32),
            "timeout_s": 2.0,
            "position_tolerance_rad": 0.01,
        },
    }

    validate_action_message(decode_message(encode_message(action)))

    action["action"]["joint_target"] = np.zeros(7, dtype=np.float32)
    with pytest.raises(ProtocolError) as exc:
        validate_action_message(action)
    assert exc.value.code == "INVALID_SHAPE"

    action["action"]["joint_target"] = np.zeros(6, dtype=np.float32)
    action["action"]["gripper_fraction"] = 0.5
    with pytest.raises(ProtocolError) as exc:
        validate_action_message(action)
    assert exc.value.code == "INVALID_ACTION"


def test_gripper_action_has_only_an_open_fraction() -> None:
    """A gripper command must stay independent and normalized to [0, 1]."""
    action = {
        "schema_version": 1,
        "message_type": "action",
        "request_id": 10,
        "sequence": 4,
        "monotonic_ns": time.monotonic_ns(),
        "wall_time_ns": time.time_ns(),
        "action": {
            "kind": "gripper",
            "open_fraction": 0.25,
            "stop_on_contact": True,
            "timeout_s": 2.0,
            "position_tolerance_fraction": 0.01,
        },
    }

    validate_action_message(action)

    action["action"]["open_fraction"] = 1.1
    with pytest.raises(ProtocolError) as exc:
        validate_action_message(action)
    assert exc.value.code == "INVALID_GRIPPER"

    action["action"]["open_fraction"] = 0.25
    action["action"]["stop_on_contact"] = "yes"
    with pytest.raises(ProtocolError) as exc:
        validate_action_message(action)
    assert exc.value.code == "INVALID_ACTION"


def _action(payload):
    return {
        "schema_version": 1,
        "message_type": "action",
        "request_id": 12,
        "sequence": 5,
        "monotonic_ns": time.monotonic_ns(),
        "wall_time_ns": time.time_ns(),
        "action": payload,
    }


def test_timed_joint_trajectory_action_contract() -> None:
    """Schema 1 must define timed trajectories before P2 enables execution."""
    validate_action_message(
        _action(
            {
                "kind": "joint_trajectory",
                "waypoints": np.zeros((3, 6), dtype=np.float32),
                "waypoint_times_s": np.array([0.0, 0.1, 0.2], dtype=np.float32),
                "timeout_s": 1.0,
                "position_tolerance_rad": 0.01,
            }
        )
    )
    validate_action_message(
        _action(
            {
                "kind": "joint_trajectory",
                "waypoints": np.zeros((3, 6), dtype=np.float32),
                "control_hz": 50.0,
                "timeout_s": 1.0,
                "position_tolerance_rad": 0.01,
            }
        )
    )

    with pytest.raises(ProtocolError, match="exactly one"):
        validate_action_message(
            _action(
                {
                    "kind": "joint_trajectory",
                    "waypoints": np.zeros((3, 6), dtype=np.float32),
                    "timeout_s": 1.0,
                    "position_tolerance_rad": 0.01,
                }
            )
        )


def test_trajectory_cancel_action_contract() -> None:
    validate_action_message(
        _action({"kind": "cancel_trajectory", "trajectory_request_id": 11})
    )


def test_heartbeat_names_the_motion_lease_it_keeps_alive() -> None:
    validate_action_message(
        _action({"kind": "heartbeat", "action_request_id": 11})
    )


def test_action_result_round_trip_carries_final_feedback_and_timing() -> None:
    result = action_result_response(
        request_id=13,
        action_request_id=11,
        kind="joint_trajectory",
        status="completed",
        started_monotonic_ns=100,
        finished_monotonic_ns=200,
        final_joint_pos_0=np.arange(7, dtype=np.float32),
        max_tracking_error_rad=0.01,
        detail="",
    )

    decoded = decode_message(encode_message(result))
    validate_action_result_message(decoded)

    assert decoded["request_id"] == 13
    assert decoded["result"]["action_request_id"] == 11
    assert decoded["result"]["final_joint_pos_0"].dtype == np.float32


def test_world_grasp_site_cartesian_action_contract() -> None:
    validate_action_message(
        _action(
            {
                "kind": "cartesian_pose",
                "frame": "world",
                "tcp": "grasp_site",
                "pose": np.array([0.1, -0.2, 0.3, 1, 0, 0, 0], dtype=np.float32),
                "timeout_s": 2.0,
            }
        )
    )

    bad = _action(
        {
            "kind": "cartesian_pose",
            "frame": "base",
            "tcp": "grasp_site",
            "pose": np.zeros(7, dtype=np.float32),
            "timeout_s": 2.0,
        }
    )
    with pytest.raises(ProtocolError, match="world"):
        validate_action_message(bad)

    nonunit = _action(
        {
            "kind": "cartesian_pose",
            "frame": "world",
            "tcp": "grasp_site",
            "pose": np.array([0.1, -0.2, 0.3, 2, 0, 0, 0], dtype=np.float32),
            "timeout_s": 2.0,
        }
    )
    with pytest.raises(ProtocolError, match="unit"):
        validate_action_message(nonunit)


def test_action_requires_wall_clock_timestamp() -> None:
    message = _action({"kind": "gripper", "open_fraction": 0.5})
    del message["wall_time_ns"]

    with pytest.raises(ProtocolError) as exc:
        validate_action_message(message)

    assert exc.value.code == "INVALID_TIMESTAMP"
