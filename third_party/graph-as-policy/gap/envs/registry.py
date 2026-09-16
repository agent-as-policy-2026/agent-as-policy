# Modified by the Agent-as-Policy (AgP) authors, 2026.
# Original: graph-as-policy @ f09e37e7755d0a908c1b6cf136edf13292b730c0
#   https://github.com/graph-robots/graph-as-policy  (Apache-2.0, see
#   third_party/graph-as-policy/LICENSE)
# Changes: ``EnvConfig`` gained the robot-identity fields mirrored from
#   ``gap.envs.robot_specs`` (Panda-equivalent defaults, so existing
#   envs are unchanged) and the ``yam_real`` env registration.
#   (+52/-2)
#   Per-file diffstat and the full A/B/C classification in
#   third_party/graph-as-policy/UPSTREAM.md.
"""Environment registry — env names → lazy factories + per-env config.

The registry is the seam between the connector layer and the simulation
envs: :func:`resolve` maps a user-facing env name (``"libero_object"``,
``"libero_object_all_variance"``, ``"libero_grocery_packing_object"``)
to a factory callable plus the canonical suite key to hand it.

Factories are registered as *lazy dotted paths* (``"gap.envs.libero_env:
make_env"``) so importing this module never pulls mujoco / robosuite /
libero — the heavy sim stack only loads when a factory is actually
resolved and called. Keep it that way: no top-level imports beyond the
stdlib (``gap.envs.robot_specs`` is stdlib-only and safe to import).

Factory contract::

    make_env(suite_name, task_id, camera_names, enable_render, **extra)
        -> (env, EnvConfig)

where ``env`` satisfies the :class:`gap.envs.base_env.BaseEnv` surface and
``EnvConfig`` carries the static robot/control metadata the connector
needs (action mode, DOF, home joints, TCP, cameras).
"""

from __future__ import annotations

import importlib
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from .robot_specs import PANDA_SPEC, GripperSpec, WorkspaceSpec

# Historical FrankaLiberoEnv literal for the gripper0_eef -> panda_hand
# translation (the geometrically exact value is -0.097; the -0.107 is kept
# for the Panda so ``robot_cartesian_pos_0`` stays byte-identical).
_PANDA_LEGACY_EEF_TO_IK_LINK_XYZ: tuple[float, float, float] = (0.0, 0.0, -0.107)


@dataclass(frozen=True)
class EnvConfig:
    """Static per-env robot/control metadata consumed by the connector.

    The original fields (``arm_dof`` .. ``is_real``) keep their historical
    defaults; the robot-identity fields below default to the Panda values
    so existing constructors keep working unchanged. Non-Panda envs are
    populated from :mod:`gap.envs.robot_specs` by the env factory.
    """

    arm_dof: int = 7
    num_arms: int = 1
    action_mode: str = "absolute_joints"
    control_freq: float = 20.0
    home_joints: tuple | None = None
    tcp_offset: tuple | None = None
    tcp_rotation_z: float | None = None
    arm_bases: tuple | None = None
    robot_urdf_path: str | None = None
    default_cameras: tuple = ("agentview", "robot0_eye_in_hand")
    is_real: bool = False
    # Documentation-only provenance fields (ported from the GaP-Yam fork's
    # EnvConfig): where the robot model authority lives and which physical
    # TCP frame the env's cartesian state/actions refer to (the real YAM
    # bridge validates and moves ``grasp_site``). Nothing branches on them.
    robot_model_source: str | None = None
    tcp_frame: str | None = None

    # --- robot identity (mirrors RobotSpec; Panda-equivalent defaults) ---
    robot_name: str = "panda"
    joint_names: tuple = PANDA_SPEC.joint_names
    ik_link: str = PANDA_SPEC.ik_link
    eef_to_ik_link_xyz: tuple = _PANDA_LEGACY_EEF_TO_IK_LINK_XYZ
    eef_to_ik_link_wxyz: tuple = PANDA_SPEC.eef_to_ik_link_wxyz
    gripper: GripperSpec = PANDA_SPEC.gripper
    workspace: WorkspaceSpec = PANDA_SPEC.workspace
    top_down_quat_wxyz: tuple = PANDA_SPEC.top_down_quat_wxyz
    wrist_camera: str = PANDA_SPEC.wrist_camera
    # Camera roles (see RobotSpec): ``rgb_only_cameras`` get no ``depth``
    # entry in the observation; ``primary_camera`` (None = exterior-first)
    # is the metric authority / identification view for perception.
    rgb_only_cameras: tuple = PANDA_SPEC.rgb_only_cameras
    primary_camera: str | None = PANDA_SPEC.primary_camera
    planner_robot_file: str = PANDA_SPEC.planner_robot_file
    planner_tool_frame: str = PANDA_SPEC.planner_tool_frame
    robot_link_prefixes: tuple = PANDA_SPEC.robot_link_prefixes
    joint_tolerance_rad: float = PANDA_SPEC.joint_tolerance_rad
    move_max_steps: int = PANDA_SPEC.move_max_steps
    post_move_settle_steps: int = PANDA_SPEC.post_move_settle_steps
    hand_to_fingertip_m: float = PANDA_SPEC.gripper.hand_to_fingertip_m
    # The exact RobotSpec instance the env was built with, including any
    # per-task derivation (e.g. a raised base shifting ``workspace.table_z``).
    # ``None`` = resolve by ``robot_name`` from the registry (legacy).
    robot_spec: Any = None


@dataclass(frozen=True)
class _Entry:
    """One registry row: where the factory lives and what key it gets."""

    factory_path: str  # "module.path:attr" — imported lazily by resolve()
    key: str  # canonical suite name passed to the factory
    prefix: bool  # True: entry also matches any env_name it prefixes


_REGISTRY: dict[str, _Entry] = {}


def register_env(
    name: str,
    factory_dotted_path: str,
    prefix: bool = False,
    *,
    key: str | None = None,
) -> None:
    """Register an env name.

    Args:
        name: User-facing env name (exact match) or prefix when
            ``prefix=True``.
        factory_dotted_path: Lazy ``"module.path:attr"`` reference to the
            factory; imported only when :func:`resolve` is called.
        prefix: When True the entry also matches any ``env_name`` that
            starts with ``name`` (the matched env_name itself becomes the
            suite key, e.g. ``"libero_object_with_mug"`` under the
            ``"libero"`` prefix entry).
        key: Canonical suite name handed to the factory; defaults to
            ``name``. Use for aliases (dev configs say
            ``libero_grocery_packing_object``, the vab task dir is
            ``libero_object_packing``).
    """
    _REGISTRY[name] = _Entry(factory_dotted_path, key or name, prefix)


def _load_factory(dotted_path: str) -> Callable:
    module_name, _, attr = dotted_path.partition(":")
    if not attr:
        raise ValueError(
            f"factory path {dotted_path!r} must be 'module.path:attr'"
        )
    module = importlib.import_module(module_name)
    return getattr(module, attr)


def resolve(env_name: str) -> tuple[Callable, str]:
    """Resolve an env name to ``(factory, suite_key)``.

    Exact registrations win; otherwise the longest matching ``prefix=True``
    entry is used with ``env_name`` itself as the suite key. Raises
    ``KeyError`` for unknown names.
    """
    entry = _REGISTRY.get(env_name)
    if entry is not None:
        return _load_factory(entry.factory_path), entry.key

    prefix_matches = [
        name
        for name, e in _REGISTRY.items()
        if e.prefix and env_name.startswith(name)
    ]
    if prefix_matches:
        best = max(prefix_matches, key=len)
        return _load_factory(_REGISTRY[best].factory_path), env_name

    raise KeyError(
        f"Unknown env {env_name!r}. Registered: {sorted(_REGISTRY)}"
    )


def registered_envs() -> dict[str, str]:
    """Mapping of registered names to their canonical suite keys."""
    return {name: entry.key for name, entry in _REGISTRY.items()}


# ---------------------------------------------------------------------------
# Pre-registrations
# ---------------------------------------------------------------------------

_LIBERO_FACTORY = "gap.envs.libero_env:make_env"

# Default: any libero* suite name routes to the libero factory; the loader
# then decides per suite between the vab task-dir format and the LIBERO-PRO
# benchmark registry (see gap.envs.loader.load_libero_task).
register_env("libero", _LIBERO_FACTORY, prefix=True)

# Variational-Automation-Benchmark (vab) suites — self-contained YAML task
# dirs under third_party/Variational-Automation-Benchmark/tasks/<name>/.
for _suite in (
    "libero_object_all_variance",
    "libero_object_target_pos_var20x20",
    "libero_object_target_permutation_variance",
    "libero_object_target_basket_swap_variance",
    "libero_object_packing",
    "permutation_packing",
):
    register_env(_suite, _LIBERO_FACTORY)

# Dev-config aliases for the packing suites.
register_env(
    "libero_grocery_packing_object", _LIBERO_FACTORY, key="libero_object_packing"
)
register_env(
    "libero_grocery_packing_permutation", _LIBERO_FACTORY, key="permutation_packing"
)

# Real hardware. ``franka_real`` is the robots_realtime msgpack bridge
# (the env binds the server; gap.connector.real() optionally spawns the
# rr-session client); ``ur_zed`` is the perception-only UR + ZED env
# (direct pyzed capture + read-only RTDE joint state).
register_env("franka_real", "gap.envs.franka_real_env:make_env")
register_env("ur_zed", "gap.envs.ur_zed_env:make_env")
# ``yam_real`` is the LEFT YAM arm behind the safety-owned hardware bridge
# (msgpack/TCP, default 127.0.0.1:9021; the right arm's bridge is 9022).
register_env("yam_real", "gap.envs.yam_real_env:make_env")

# Classic LIBERO-PRO benchmark-registry suites. The "libero" prefix entry
# already covers every libero_* name; these exact rows pin the canonical
# benchmark suites for discoverability (registered_envs()).
for _suite in (
    "libero_object",
    "libero_spatial",
    "libero_goal",
    "libero_90",
    "libero_10",
    "libero_object_swap",
    "libero_object_basket_swap",
):
    register_env(_suite, _LIBERO_FACTORY)
