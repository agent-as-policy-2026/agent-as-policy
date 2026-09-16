from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import pytest
from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml

from agp_yam_bridge.config import load_config
from agp_yam_bridge.kinematics import (
    I2rtKinematicsBackend,
    KinematicsValidationError,
    TrajectoryValidationError,
    matrix_to_pose_wxyz,
    pose_wxyz_to_matrix,
)
from agp_yam_bridge.preflight import DEFAULT_CONFIG


@pytest.fixture(scope="module")
def backend() -> I2rtKinematicsBackend:
    return I2rtKinematicsBackend.from_config(load_config(DEFAULT_CONFIG))


@pytest.fixture(scope="module")
def direct_i2rt_model() -> mujoco.MjModel:
    xml_path = combine_arm_and_gripper_xml(ArmType.YAM, GripperType.LINEAR_4310)
    try:
        return mujoco.MjModel.from_xml_path(xml_path)
    finally:
        Path(xml_path).unlink(missing_ok=True)


def _direct_grasp_site_fk(model: mujoco.MjModel, joints: np.ndarray) -> np.ndarray:
    data = mujoco.MjData(model)
    for index, value in enumerate(joints, start=1):
        joint = model.joint(f"joint{index}")
        data.qpos[joint.qposadr[0]] = value
    for finger_name in ("joint7", "joint8"):
        finger = model.joint(finger_name)
        lo, hi = finger.range
        data.qpos[finger.qposadr[0]] = (lo + hi) / 2.0
    mujoco.mj_forward(model, data)
    site = data.site("grasp_site")
    transform = np.eye(4)
    transform[:3, :3] = site.xmat.reshape(3, 3)
    transform[:3, 3] = site.xpos
    return transform


def _home() -> np.ndarray:
    return np.radians([0.0, 0.0, 0.0, 90.0, 90.0, 90.0])


def test_shipped_operational_home_reserves_the_full_settle_bias() -> None:
    """A home target at a joint limit cannot compensate steady gravity error."""
    config = load_config(DEFAULT_CONFIG)
    backend = I2rtKinematicsBackend.from_config(config)
    home = np.radians(config.acceptance.home_joints_deg)
    _, upper = backend.joint_limits
    reserve = np.radians(config.safety.cartesian_settle_max_bias_deg)

    assert upper[3] - home[3] >= reserve - 1e-9


def test_pose_helpers_use_position_then_wxyz() -> None:
    """Swapping xyzw/wxyz must rotate a known pose around the wrong axis."""
    half_sqrt = np.sqrt(0.5)
    pose = np.array([0.1, -0.2, 0.3, half_sqrt, 0.0, 0.0, half_sqrt])
    transform = pose_wxyz_to_matrix(pose)

    np.testing.assert_allclose(
        transform,
        np.array(
            [
                [0.0, -1.0, 0.0, 0.1],
                [1.0, 0.0, 0.0, -0.2],
                [0.0, 0.0, 1.0, 0.3],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
        atol=1e-12,
    )
    np.testing.assert_allclose(matrix_to_pose_wxyz(transform), pose, atol=1e-12)


def test_fk_matches_current_i2rt_combined_model_for_twenty_legal_configs(
    backend: I2rtKinematicsBackend,
    direct_i2rt_model: mujoco.MjModel,
) -> None:
    """A copied link_6/TCP model would disagree with i2rt's real grasp_site."""
    rng = np.random.default_rng(20260819)
    lower, upper = backend.joint_limits
    margin = 0.05 * (upper - lower)

    for joints in rng.uniform(lower + margin, upper - margin, size=(20, 6)):
        expected = backend.world_from_base @ _direct_grasp_site_fk(direct_i2rt_model, joints)
        actual = backend.forward_matrix(joints)
        np.testing.assert_allclose(actual[:3, 3], expected[:3, 3], atol=1e-9)
        np.testing.assert_allclose(actual[:3, :3], expected[:3, :3], atol=1e-9)


def test_fk_exposes_world_base_gripper_grasp_site_chain(
    backend: I2rtKinematicsBackend,
) -> None:
    """Reversing either transform multiplication must break the reported TCP pose."""
    chain = backend.frame_chain(_home())
    expected = chain.world_from_base @ chain.base_from_gripper @ chain.gripper_from_grasp_site

    np.testing.assert_allclose(chain.world_from_grasp_site, expected, atol=1e-10)
    np.testing.assert_allclose(
        chain.world_from_grasp_site, backend.forward_matrix(_home()), atol=1e-9
    )
    np.testing.assert_allclose(
        chain.gripper_from_grasp_site[:3, 3], [0.0, 0.0, -0.1347], atol=1e-12
    )


def test_fk_applies_nontrivial_world_from_base_transform() -> None:
    config = load_config(DEFAULT_CONFIG)
    base_backend = I2rtKinematicsBackend.from_config(config)
    world_from_base = pose_wxyz_to_matrix([0.4, -0.2, 0.1, np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])
    transformed_backend = I2rtKinematicsBackend.from_config(config, world_from_base=world_from_base)

    np.testing.assert_allclose(
        transformed_backend.forward_matrix(_home()),
        world_from_base @ base_backend.forward_matrix(_home()),
        atol=1e-9,
    )


@pytest.mark.parametrize(
    "target_joints",
    [
        [0.0, 0.2, 0.3, 0.1, -0.1, 0.2],
        [0.3, 0.7, 1.0, -0.2, 0.2, -0.4],
        [-0.4, 1.2, 0.8, 0.3, -0.3, 0.6],
        [0.6, 1.8, 1.2, -0.5, 0.4, -0.8],
        [-0.8, 2.2, 1.8, 0.5, -0.5, 1.0],
    ],
)
def test_ik_fk_round_trip_respects_joint_limits(
    backend: I2rtKinematicsBackend,
    target_joints: list[float],
) -> None:
    """Returning a converged but out-of-limit or wrong-frame IK solution is unsafe."""
    target = backend.forward(target_joints)
    seed = np.asarray(target_joints) + 0.02
    solved = backend.solve_ik(target, seed_joints=seed)

    assert solved is not None
    lower, upper = backend.joint_limits
    assert np.all(solved >= lower)
    assert np.all(solved <= upper)
    np.testing.assert_allclose(
        backend.forward_matrix(solved), pose_wxyz_to_matrix(target), atol=1e-4
    )


def test_ik_rejects_unreachable_target(backend: I2rtKinematicsBackend) -> None:
    target = backend.forward(_home())
    target[:3] = [5.0, 5.0, 5.0]
    assert backend.solve_ik(target, seed_joints=_home()) is None


def test_position_ik_reaches_recorded_lift_target_without_orientation_constraint(
    backend: I2rtKinematicsBackend,
) -> None:
    """The task_02 lift point is reachable even though its recorded full pose is not."""
    target_pose = np.array(
        [
            0.3768220841884613,
            -0.04692096635699272,
            0.353,
            0.0011407437268644571,
            -0.5157453417778015,
            -0.8567337989807129,
            -0.0035611994098871946,
        ]
    )
    seed = np.array(
        [
            -0.12722209095954895,
            2.1215763092041016,
            1.3815137147903442,
            -0.8295745253562927,
            -0.007438773289322853,
            0.9565499424934387,
        ]
    )

    assert backend.solve_ik(target_pose, seed_joints=seed) is None
    solved = backend.solve_position_ik(target_pose[:3], seed_joints=seed)

    assert solved is not None
    lower, upper = backend.joint_limits
    assert np.all(solved >= lower)
    assert np.all(solved <= upper)
    np.testing.assert_allclose(backend.forward(solved)[:3], target_pose[:3], atol=1e-4)


@pytest.mark.parametrize("tolerance", [0.0, -1e-4, np.nan, np.inf])
def test_position_ik_rejects_non_positive_or_non_finite_tolerance(
    backend: I2rtKinematicsBackend,
    tolerance: float,
) -> None:
    with pytest.raises(KinematicsValidationError, match="position_tolerance_m"):
        backend.solve_position_ik(
            backend.forward(_home())[:3],
            seed_joints=_home(),
            position_tolerance_m=tolerance,
        )


def test_ik_rejects_seed_outside_model_joint_limits(
    backend: I2rtKinematicsBackend,
) -> None:
    target = backend.forward(_home())
    invalid_seed = _home()
    # -0.15 rad: beyond the 0.1 rad measurement buffer (a measured seed
    # resting slightly past a hard stop is now accepted by design).
    invalid_seed[1] = -0.15
    with pytest.raises(KinematicsValidationError, match="joint2"):
        backend.solve_ik(target, seed_joints=invalid_seed)


def test_neighboring_ik_targets_stay_on_neighboring_solution_branch(
    backend: I2rtKinematicsBackend,
) -> None:
    first_joints = np.array([0.1, 0.7, 0.9, -0.2, 0.1, 0.3])
    second_joints = first_joints + np.array([0.005, 0.004, -0.003, 0.004, -0.002, 0.003])
    first = backend.solve_ik(backend.forward(first_joints), seed_joints=first_joints)
    assert first is not None
    second = backend.solve_ik(backend.forward(second_joints), seed_joints=first)

    assert second is not None
    assert np.max(np.abs(second - first)) < 0.05


def test_valid_trajectory_passes_all_safety_checks(
    backend: I2rtKinematicsBackend,
) -> None:
    q0 = _home()
    waypoints = np.stack([q0, q0 + [0, 0, 0, 0, 0, 0.01], q0 + [0, 0, 0, 0, 0, 0.02]])
    checked = backend.validate_trajectory(waypoints, [0.0, 0.1, 0.2])
    np.testing.assert_array_equal(checked, waypoints)


def test_trajectory_rejects_joint_limit_violation(
    backend: I2rtKinematicsBackend,
) -> None:
    waypoints = np.stack([_home(), _home()])
    waypoints[1, 1] = -0.01
    with pytest.raises(TrajectoryValidationError) as exc:
        backend.validate_trajectory(waypoints, [0.0, 1.0])
    assert exc.value.code == "JOINT_LIMIT"


def test_trajectory_rejects_large_joint_jump(
    backend: I2rtKinematicsBackend,
) -> None:
    waypoints = np.stack([_home(), _home()])
    waypoints[1, 0] += 0.6
    with pytest.raises(TrajectoryValidationError) as exc:
        backend.validate_trajectory(waypoints, [0.0, 4.0])
    assert exc.value.code == "JOINT_JUMP"


def test_trajectory_accepts_joint_velocity_at_configured_limit(
    backend: I2rtKinematicsBackend,
) -> None:
    waypoints = np.stack([_home(), _home()])
    waypoints[1, 5] += np.radians(2.0)
    checked = backend.validate_trajectory(waypoints, [0.0, 0.1])
    np.testing.assert_array_equal(checked, waypoints)


def test_trajectory_rejects_excess_joint_velocity(
    backend: I2rtKinematicsBackend,
) -> None:
    waypoints = np.stack([_home(), _home()])
    waypoints[1, 5] += np.radians(2.1)
    with pytest.raises(TrajectoryValidationError) as exc:
        backend.validate_trajectory(waypoints, [0.0, 0.1])
    assert exc.value.code == "JOINT_VELOCITY"


def test_trajectory_rejects_excess_joint_acceleration(
    backend: I2rtKinematicsBackend,
) -> None:
    waypoints = np.stack([_home(), _home(), _home()])
    waypoints[2, 5] += 0.015
    with pytest.raises(TrajectoryValidationError) as exc:
        backend.validate_trajectory(waypoints, [0.0, 0.1, 0.2])
    assert exc.value.code == "JOINT_ACCELERATION"


def test_trajectory_rejects_grasp_site_outside_configured_workspace(
    backend: I2rtKinematicsBackend,
) -> None:
    """The Cartesian gate must read acceptance.workspace, not a hard-coded box."""
    # grasp_site at all-zero joints is (0.245, 0.000, 0.174) m in the base frame. The
    # shipped workspace is deliberately set beyond the arm's reach (see left_arm.yaml /
    # right_arm.yaml: "WIDE OPEN by explicit user decision"), so a rejectable target only
    # exists inside a finite box: this narrows the SAME config to the original P4
    # first-motion envelope, whose y range [-0.75, -0.02] excludes y = 0.
    config = load_config(DEFAULT_CONFIG)
    config_data = config.model_dump()
    workspace = config_data["acceptance"]["workspace"]
    workspace["x_m"] = [-0.60, 0.60]
    workspace["y_m"] = [-0.75, -0.02]
    workspace["z_m"] = [-0.06, 0.50]
    narrow_backend = I2rtKinematicsBackend.from_config(type(config).model_validate(config_data))

    waypoints = np.zeros((2, 6))
    with pytest.raises(TrajectoryValidationError) as exc:
        narrow_backend.validate_trajectory(waypoints, [0.0, 1.0])
    assert exc.value.code == "WORKSPACE"

    # The identical waypoints pass under the shipped workspace, which is what makes the
    # rejection above attributable to the configured box rather than to the waypoints.
    np.testing.assert_array_equal(backend.validate_trajectory(waypoints, [0.0, 1.0]), waypoints)


def test_trajectory_rejects_workspace_exit_between_legal_endpoints() -> None:
    """Joint interpolation may bow outside the Cartesian box between endpoints."""
    config = load_config(DEFAULT_CONFIG)
    config_data = config.model_dump()
    workspace = config_data["acceptance"]["workspace"]
    workspace["x_m"] = [-0.30, 0.30]
    workspace["y_m"] = [-0.55, -0.12]
    workspace["z_m"] = [0.03, 0.35]
    narrow_backend = I2rtKinematicsBackend.from_config(type(config).model_validate(config_data))
    waypoints = np.array(
        [
            [-1.215969, 2.048187, 1.042210, 1.132579, -0.870296, 0.715313],
            [-1.187339, 1.761822, 0.794987, 1.277082, -1.350056, 0.388162],
        ]
    )
    for waypoint in waypoints:
        position = narrow_backend.forward_matrix(waypoint)[:3, 3]
        assert -0.30 <= position[0] <= 0.30
        assert -0.55 <= position[1] <= -0.12
        assert 0.03 <= position[2] <= 0.35

    with pytest.raises(TrajectoryValidationError) as exc:
        narrow_backend.validate_trajectory(waypoints, [0.0, 10.0])
    assert exc.value.code == "WORKSPACE"


def test_trajectory_requires_strictly_increasing_times(
    backend: I2rtKinematicsBackend,
) -> None:
    waypoints = np.stack([_home(), _home()])
    with pytest.raises(TrajectoryValidationError) as exc:
        backend.validate_trajectory(waypoints, [0.0, 0.0])
    assert exc.value.code == "INVALID_TIMING"


def test_trajectory_rejects_negative_start_time(
    backend: I2rtKinematicsBackend,
) -> None:
    waypoints = np.stack([_home(), _home()])
    with pytest.raises(TrajectoryValidationError) as exc:
        backend.validate_trajectory(waypoints, [-0.1, 0.0])
    assert exc.value.code == "INVALID_TIMING"


def test_ik_trajectory_checks_seed_to_first_waypoint_jump(
    backend: I2rtKinematicsBackend,
) -> None:
    seed = _home()
    distant = seed.copy()
    distant[5] -= 0.6
    target = backend.forward(distant)

    with pytest.raises(TrajectoryValidationError) as exc:
        backend.solve_ik_trajectory(
            np.stack([target, target]),
            [0.0, 4.0],
            seed_joints=seed,
        )
    assert exc.value.code == "JOINT_JUMP"
