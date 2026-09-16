# Written for the Agent-as-Policy (AgP) project, 2026.  NOT upstream code:
# this file does not exist in graph-as-policy at
# f09e37e7755d0a908c1b6cf136edf13292b730c0.  It lives inside the vendored
# package because it must be importable as ``gap.envs.robot_specs`` — the upstream
# connector resolves it by that name.  Licensed Apache-2.0 with the rest of
# this directory (AgP, Copyright (c) 2026 the AgP authors).
# Modified relative to the AgP working copy it was vendored from: the
# ``_YAM_ULTRA_URDF`` path arithmetic was removed (see the note above
# the robot table).
# Provenance: third_party/graph-as-policy/UPSTREAM.md.
"""Per-robot specification table for the simulation env / connector layer.

Every Franka-specific literal that used to be scattered across
``libero_env.py``, ``core.py``, ``ik.py`` and the skill scripts is keyed
here by robot name so the same code path serves the 7-DOF Panda (LIBERO's
default) and the 6-DOF i2rt YAM Ultra + linear_4310 gripper.

Selection: :func:`gap.env_config.sim_robot` reads ``GAP_SIM_ROBOT``
(default ``"panda"``); ``gap run --robot <name>`` sets that variable before
the env is built. :func:`get_robot_spec` resolves a name to a
:class:`RobotSpec`.

**Panda defaults are byte-identical to the historical literals** — the
``"panda"`` entry must never drift from what the env / connector used before
this table existed (see the regression tests in ``tests/envs``).

Conventions (pinned so every consumer agrees):

- ``tcp_offset`` is in gap's *negated* link->TCP convention used by the IK
  backends: ``link_pos = tcp_pos + R_link @ tcp_offset``.
- ``ik_link`` is the tool frame whose ``+z`` points out of the fingers and
  whose ``y`` is the finger-slide axis (Panda: ``panda_hand``; YAM Ultra:
  the fixed ``tcp_gap`` URDF link = link_6 * [xyz 0 0 -0.1347, rpy 0 pi 0]).
  With that contract gap's canonical top-down quaternion ``(w,x,y,z) =
  (0,1,0,0)`` (= Rx(180 deg)) means "tool z straight down, jaw line along
  world Y" for both robots.
- ``eef_to_ik_link_*`` is the SE3 applied to robosuite's ``gripper0_eef``
  body to obtain the ``ik_link`` frame (Panda: undo the Rz(-90) of
  ``right_gripper`` and back up 0.097 m along z to ``panda_hand``; YAM: the
  robosuite ``eef`` body IS placed at ``tcp_gap`` so it is the identity).
- Gripper: robosuite's ``format_action`` sign (``-1`` = open, ``+1`` =
  close) is assumed by the whole stack (``cmd = 1 - 2 * fraction``); a
  non-Panda robosuite GripperModel MUST implement the same convention and
  expose ``dof == GripperSpec.action_dof``. Open fraction is
  ``(q - qpos_closed) / (qpos_open - qpos_closed)`` of ``finger_joints[0]``.
- Naming contract the robosuite model must satisfy: bodies ``base`` and
  ``eef``, site ``grip_site``, observables ``robot0_joint_pos`` /
  ``robot0_gripper_qpos`` / ``robot0_eef_pos`` / ``robot0_eef_quat``, joints
  named ``robot0_<joint_names[i]>`` and ``gripper0_<finger_joints[i]>``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any

__all__ = [
    "PANDA_SPEC",
    "ROBOT_SPECS",
    "YAM_REAL_LEFT_SPEC",
    "YAM_REAL_RIGHT_SPEC",
    "YAM_ULTRA_SPEC",
    "GripperSpec",
    "RobotSpec",
    "WorkspaceSpec",
    "get_robot_spec",
    "robot_names",
]


@dataclass(frozen=True, kw_only=True)
class GripperSpec:
    """Two-finger gripper geometry + control contract.

    Attributes:
        finger_joints: robosuite gripper joint names (without the
            ``gripper0_`` prefix); ``finger_joints[0]`` is the joint whose
            qpos is read for the open fraction.
        qpos_closed / qpos_open: ``finger_joints[0]`` qpos at fully closed /
            fully open (fraction = (q - closed) / (open - closed)).
        max_opening_m: inner distance between the pads when fully open.
        hand_to_fingertip_m: distance from the ``ik_link`` origin to the
            grasp point along tool z (0 when the IK link IS the grasp point).
        pad_depth_m: usable pad depth along tool z (palm face -> pad end).
        finger_length_m: finger length used for pre-grasp clearances.
        slide_axis_in_tool: finger-slide axis expressed in the tool frame.
        open_settle_steps / close_settle_steps: connector settle budgets.
        action_dof: robosuite ``gripper.dof`` (action entries per gripper).
    """

    finger_joints: tuple[str, ...]
    qpos_closed: float
    qpos_open: float
    max_opening_m: float
    hand_to_fingertip_m: float
    pad_depth_m: float
    finger_length_m: float
    slide_axis_in_tool: tuple[float, float, float] = (0.0, 1.0, 0.0)
    open_settle_steps: int = 40
    close_settle_steps: int = 60
    action_dof: int = 1

    @property
    def travel(self) -> float:
        """Signed qpos travel from closed to open (``qpos_open - qpos_closed``)."""
        return float(self.qpos_open - self.qpos_closed)

    def fraction_from_qpos(self, q: float) -> float:
        """Open fraction (0 closed .. 1 open) of ``finger_joints[0]`` qpos.

        Unclamped on purpose: the Panda path historically reported the raw
        ``q / 0.04`` ratio (which can slightly exceed 1.0), and consumers
        rely on that exact number.
        """
        travel = self.travel
        if travel == 0.0:
            return 1.0
        return float((q - self.qpos_closed) / travel)

    def qpos_from_fraction(self, fraction: float) -> float:
        """Inverse of :meth:`fraction_from_qpos`."""
        return float(self.qpos_closed + fraction * self.travel)


@dataclass(frozen=True, kw_only=True)
class WorkspaceSpec:
    """Reachable-envelope numbers skills use for heights (robot base frame, m).

    Attributes:
        table_z: table-top height in the base frame.
        hover_z: pre-grasp hover height above the table.
        transport_z: carry / retract altitude.
        approach_z: approach-above height for pre-grasp alignment.
        max_topdown_z: highest TCP z at which a top-down (tool z = -Z) pose
            is still reachable over the usual object band (xy-agnostic ceiling
            for callers that do not know where they are going).
        reach_xy: max horizontal reach used for sanity boxes.
        wrist_raise_z: raised-wrist split height (perception re-centering).
        topdown_envelope: optional ``((r_xy, z_max), ...)`` knots, ascending
            in ``r_xy``: the highest top-down TCP z reachable at horizontal
            distance ``r_xy`` from the base. Empty = the ceiling is
            ``max_topdown_z`` everywhere (Panda). Consumed through
            :meth:`max_topdown_z_at` by callers that know the target xy.
    """

    table_z: float
    hover_z: float
    transport_z: float
    approach_z: float
    max_topdown_z: float
    reach_xy: float
    wrist_raise_z: float
    topdown_envelope: tuple[tuple[float, float], ...] = ()
    #: Pitched-tool transport (arms whose top-down ceiling is low, e.g. the
    #: YAM Ultra): tilting the tool forward (fingers pointing down-and-away
    #: from the base) by ``transport_pitch_deg`` makes ``transport_pitched_z``
    #: reachable at every radius. 0 / 0 = disabled (Panda: flat top-down
    #: transport, historical behaviour).
    transport_pitch_deg: float = 0.0
    transport_pitched_z: float = 0.0

    def max_topdown_z_at(self, r_xy: float) -> float:
        """Top-down ceiling at horizontal distance ``r_xy`` (base frame).

        Piecewise-linear over :attr:`topdown_envelope` (flat beyond the
        first / last knot); ``max_topdown_z`` when no envelope is given.
        """
        knots = self.topdown_envelope
        if not knots:
            return float(self.max_topdown_z)
        r = float(r_xy)
        if r <= knots[0][0]:
            return float(knots[0][1])
        for (r0, z0), (r1, z1) in zip(knots, knots[1:]):
            if r <= r1:
                t = (r - r0) / (r1 - r0) if r1 > r0 else 1.0
                return float(z0 + t * (z1 - z0))
        return float(knots[-1][1])

    def clamp_topdown_z(self, z: float, margin: float = 0.0,
                        r_xy: float | None = None) -> float:
        """Clamp a requested TCP height into ``[table_z, ceiling - margin]``
        (``ceiling`` = :meth:`max_topdown_z_at` when ``r_xy`` is given, else
        ``max_topdown_z``)."""
        hi = (self.max_topdown_z if r_xy is None else self.max_topdown_z_at(r_xy)) - margin
        return float(min(max(float(z), self.table_z), hi))


@dataclass(frozen=True, kw_only=True)
class RobotSpec:
    """Everything the env / connector / planner need to know about one arm."""

    name: str
    arm_dof: int
    joint_names: tuple[str, ...]
    robosuite_robot: str
    robosuite_gripper: str
    home_joints: tuple[float, ...]
    ik_link: str
    tcp_offset: tuple[float, float, float]
    tcp_rotation_z: float | None
    eef_to_ik_link_xyz: tuple[float, float, float]
    eef_to_ik_link_wxyz: tuple[float, float, float, float]
    robot_urdf_path: str | None
    planner_robot_file: str
    planner_tool_frame: str
    gripper: GripperSpec
    workspace: WorkspaceSpec
    top_down_quat_wxyz: tuple[float, float, float, float] = (0.0, 1.0, 0.0, 0.0)
    wrist_camera: str = "robot0_eye_in_hand"
    #: Cameras the sim env exposes as RGB-only: their observation entry
    #: carries ``images.rgb`` + intrinsics + pose but NO ``depth`` (and no
    #: depth-derived data). Mirrors the real-robot bridge contract, where
    #: the fixed exterior camera is RGB + calibration only and depth is
    #: never fabricated. Empty for the Panda (both LIBERO cameras RGB-D).
    rgb_only_cameras: tuple[str, ...] = ()
    #: The camera that is the metric authority AND the identification view
    #: for perception skills (``None`` = the historical exterior-first
    #: policy: identify on the exterior view, wrist only as a gated
    #: fallback). Must not be listed in ``rgb_only_cameras``.
    primary_camera: str | None = None
    #: Normalized ``(x1, y1, x2, y2)`` rectangles of the wrist image where the
    #: robot's own gripper / fingers are always visible; perception drops
    #: detector boxes centred inside them. Empty = no self-view masking.
    wrist_self_view_exclusion: tuple[tuple[float, float, float, float], ...] = ()
    #: Optional second observation pose (joint radians) for wrist-primary perception: when the
    #: Set-of-Marks pick says the target is not among the candidates at the home view (a part hidden
    #: behind the fingers), or the close-up verify rejects the picked candidate (its distinguishing
    #: feature self-occluded from that angle), the skill moves here, re-observes and retries once.
    #: Empty = no fallback.
    secondary_observe_joints: tuple[float, ...] = ()
    robot_link_prefixes: tuple[str, ...]
    joint_tolerance_rad: float
    move_max_steps: int
    post_move_settle_steps: int
    base_xpos_offset_empty: tuple[float, float, float]
    joint_ctrl_overrides: dict[str, Any] = field(default_factory=dict)
    osc_overrides: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if len(self.joint_names) != self.arm_dof:
            raise ValueError(
                f"RobotSpec {self.name!r}: {len(self.joint_names)} joint_names "
                f"but arm_dof={self.arm_dof}"
            )
        if len(self.home_joints) != self.arm_dof:
            raise ValueError(
                f"RobotSpec {self.name!r}: home_joints has {len(self.home_joints)} "
                f"entries but arm_dof={self.arm_dof}"
            )
        if self.primary_camera and self.primary_camera in self.rgb_only_cameras:
            raise ValueError(
                f"RobotSpec {self.name!r}: primary_camera {self.primary_camera!r} "
                f"is listed in rgb_only_cameras; the primary camera must carry "
                f"metric depth"
            )

    @property
    def hand_to_fingertip_m(self) -> float:
        """Shortcut for ``gripper.hand_to_fingertip_m``."""
        return self.gripper.hand_to_fingertip_m

    @property
    def is_panda(self) -> bool:
        return self.name == "panda"


# ---------------------------------------------------------------------------
# Table
# ---------------------------------------------------------------------------

# AgP vendoring note: upstream of this vendored copy the two YAM entries
# below carried
#     _GAP_YAM_ROOT = Path(__file__).resolve().parents[3]
#     _YAM_ULTRA_URDF = _GAP_YAM_ROOT / "custom_tasks" / "yam_ultra" /
#                       "urdf_combined" / "yam_ultra_linear_4310.urdf"
#     robot_urdf_path=str(_YAM_ULTRA_URDF)
# ``parents[3]`` climbed one level ABOVE the connector checkout into a
# private workspace directory, and nothing ever checked ``.exists()``, so a
# relocated copy silently carried a path that resolves nowhere. That asset
# is not part of this repository.
#
# Both YAM entries now use ``robot_urdf_path=None``. This is behaviour-
# preserving on the real-YAM path: IK there is remote, solved inside the
# hardware bridge by i2rt (``RemoteI2rtKinematicsBackend`` in
# ``gap.connector.real``, wired at construction so ``Connector._ik`` is
# never ``None``). ``robot_urdf_path`` is read ONLY by the in-process
# cuRobo / PyRoKi branches (``gap.connector.core.Connector.ik`` and
# ``make_pyroki_backend``), which that path never enters; both already
# treat a falsy value as 'no URDF override' — the comment on the right-arm
# entry below said as much ("Documentation only on the real path").
# ``gap.envs.registry.EnvConfig.robot_urdf_path`` is already ``str | None``.
# The ``RobotSpec.robot_urdf_path`` annotation was widened to ``str | None``
# to match, and the then-dead ``from pathlib import Path`` was dropped.
# Nothing on the YAM path reads the field; see UPSTREAM.md.

_RZ90_WXYZ = (math.cos(math.pi / 4.0), 0.0, 0.0, math.sin(math.pi / 4.0))

PANDA_SPEC = RobotSpec(
    name="panda",
    arm_dof=7,
    joint_names=tuple(f"joint{i}" for i in range(1, 8)),
    robosuite_robot="Panda",
    robosuite_gripper="PandaGripper",
    # Canonical Franka home (the historical ``_FRANKA_HOME_JOINTS``).
    home_joints=(0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785),
    ik_link="panda_hand",
    # panda_hand -> gripper0_grip_site is +0.097 m along hand z (negated).
    tcp_offset=(0.0, 0.0, -0.097),
    tcp_rotation_z=None,
    # gripper0_eef = right_gripper (Rz(-90) of panda_hand) + 0.097 m along z.
    # NOTE: FrankaLiberoEnv keeps its historical (0, 0, -0.107) literal for
    # the Panda so ``robot_cartesian_pos_0`` stays byte-identical; this is
    # the geometrically exact value (see YAM_MIGRATION_LOG.md).
    eef_to_ik_link_xyz=(0.0, 0.0, -0.097),
    eef_to_ik_link_wxyz=_RZ90_WXYZ,
    robot_urdf_path="panda_description",
    planner_robot_file="franka.yml",
    planner_tool_frame="panda_hand",
    gripper=GripperSpec(
        finger_joints=("finger_joint1", "finger_joint2"),
        qpos_closed=0.0,
        qpos_open=0.04,
        max_opening_m=0.08,
        hand_to_fingertip_m=0.1029,
        pad_depth_m=0.035 + 0.015,
        finger_length_m=0.058,
        slide_axis_in_tool=(0.0, 1.0, 0.0),
        open_settle_steps=40,
        close_settle_steps=60,
        action_dof=1,
    ),
    workspace=WorkspaceSpec(
        table_z=0.0,
        hover_z=0.20,
        transport_z=0.353,
        approach_z=0.35,
        max_topdown_z=0.60,
        reach_xy=0.85,
        wrist_raise_z=0.42,
    ),
    top_down_quat_wxyz=(0.0, 1.0, 0.0, 0.0),
    wrist_camera="robot0_eye_in_hand",
    # Historical behaviour: both LIBERO cameras RGB-D, exterior-first ID.
    rgb_only_cameras=(),
    primary_camera=None,
    # LiberoWorldAdapter's historical default prefix tuple.
    robot_link_prefixes=("robot", "panda_", "Robotiq", "finger_", "gripper"),
    joint_tolerance_rad=0.01,
    move_max_steps=120,
    post_move_settle_steps=0,
    base_xpos_offset_empty=(-0.6, 0.0, 0.0),
    joint_ctrl_overrides={},
    osc_overrides={},
)

YAM_ULTRA_SPEC = RobotSpec(
    name="yam_ultra",
    arm_dof=6,
    joint_names=tuple(f"joint{i}" for i in range(1, 7)),
    robosuite_robot="YamUltra",
    robosuite_gripper="YamLinearGripper",
    # HOME-A from the phase-1 kinematics study: grasp point at
    # (0.20, 0.00, 0.30) m, tool pitched 45 deg forward, collision-free.
    home_joints=(0.0, 0.80, 1.28, -1.27, 0.0, 0.0),
    # Fixed URDF link link_6 * [xyz 0 0 -0.1347, rpy 0 pi 0]: +z out of the
    # fingers, y = finger-slide axis, gap's canonical (0,1,0,0) <=> joint6~0.
    ik_link="tcp_gap",
    tcp_offset=(0.0, 0.0, 0.0),
    tcp_rotation_z=None,
    # The robosuite ``eef`` body is placed at tcp_gap -> identity.
    eef_to_ik_link_xyz=(0.0, 0.0, 0.0),
    eef_to_ik_link_wxyz=(1.0, 0.0, 0.0, 0.0),
    robot_urdf_path=None,  # remote i2rt IK in the bridge; see the note above
    planner_robot_file="yam_ultra.yml",
    planner_tool_frame="tcp_gap",
    gripper=GripperSpec(
        # linear_4310: joint7 / joint8 slides, 0 = closed, 0.0475 m = open
        # (both positive), inner pad opening 0.094 m when open.
        finger_joints=("joint7", "joint8"),
        qpos_closed=0.0,
        qpos_open=0.0475,
        max_opening_m=0.094,
        # tcp_gap already sits at the grasp point (grasp_site).
        hand_to_fingertip_m=0.0,
        pad_depth_m=0.05,
        finger_length_m=0.094,
        slide_axis_in_tool=(0.0, 1.0, 0.0),
        open_settle_steps=40,
        close_settle_steps=60,
        action_dof=1,
    ),
    # Top-down reach envelope measured through the connector's cuRobo IK
    # (custom_tasks/yam_ultra/probe_topdown_reach.py, canonical quat, base
    # at -0.30 on the floor; joint4/5 limited to +-90 deg): last reachable
    # TCP z per horizontal distance r_xy -> 0.19 @ r<=0.34, 0.18 @ 0.36,
    # 0.17 @ 0.38, 0.16 @ 0.41, 0.15 @ 0.43, 0.12 @ 0.45, 0.09 @ 0.47,
    # 0.03 @ 0.48. The knots below carry a 1 cm safety margin under those
    # measurements: cuRobo's seeded IK is not deterministic on the boundary
    # (a release at 0.1596 / r 0.411 planned in one run and failed in the
    # next). ``max_topdown_z`` (xy-agnostic) is the value good over the usual
    # object band r <= 0.41; xy-aware callers use the knots.
    workspace=WorkspaceSpec(
        table_z=0.0,
        hover_z=0.12,
        transport_z=0.15,
        approach_z=0.15,
        max_topdown_z=0.15,
        reach_xy=0.50,
        wrist_raise_z=0.17,
        topdown_envelope=(
            (0.34, 0.18), (0.36, 0.17), (0.38, 0.16), (0.41, 0.15),
            (0.43, 0.14), (0.45, 0.11), (0.47, 0.08), (0.48, 0.02),
        ),
        # Measured 2026-08-29 (probe_pitch, cuRobo, base at z=0.04): with the
        # tool pitched forward 20 deg the ceiling is 0.26 m for r <= 0.44
        # (0.22 @ 0.48); at 40 deg >= 0.34 m everywhere up to r = 0.48.
        transport_pitch_deg=35.0,
        transport_pitched_z=0.28,
    ),
    top_down_quat_wxyz=(0.0, 1.0, 0.0, 0.0),
    wrist_camera="robot0_eye_in_hand",
    # Wrist-primary perception (the real YAM paradigm: wrist RealSense D405
    # RGB-D is the metric authority + identification view; the fixed top
    # BRIO is RGB-only and only a cross-view check). The sim mirrors that
    # contract: ``agentview`` is exposed without a depth entry.
    rgb_only_cameras=("agentview",),
    # Fingertips + gripper body at the bottom of the wrist frame (HOME-B view,
    # open gripper; closed fingers stay inside the union of these bands).
    wrist_self_view_exclusion=((0.0, 0.76, 0.37, 1.0), (0.63, 0.76, 1.0, 1.0), (0.37, 0.88, 0.63, 1.0)),
    # HOME-B (63 deg down, wrist 0.40 m up; make_scene4_tasks.py): sees the near band that the fingers
    # hide at HOME-A. Used as the re-observe pose by perceiving-objects.
    secondary_observe_joints=(0.0, 0.90, 1.25, -1.45, 0.0, 0.0),
    primary_camera="robot0_eye_in_hand",
    robot_link_prefixes=("robot", "gripper", "link_", "tip_"),
    # 0.01 rad (was 0.03): with the kp-100 joint controller the closed-loop
    # move reaches norm < 0.01 in ~30 steps (vs ~26 for 0.03) and the wrist
    # joints settle to ~0.005 rad instead of ~0.02 (gap smoke: cartesian legs
    # land within 2 mm instead of 4 mm). Panda keeps 0.01 / 120 / 0.
    joint_tolerance_rad=0.01,
    move_max_steps=300,
    post_move_settle_steps=30,
    # Base moved ~0.30 m closer to the table centre than the Panda mount so
    # the LIBERO object region lands inside the top-down envelope.
    base_xpos_offset_empty=(-0.30, 0.0, 0.0),
    # JOINT_POSITION kp 100 (robosuite default 50): with kp 50 the closed-loop
    # home move leaves ~0.02 rad on joints 5/6 (norm just inside the 0.03 rad
    # tolerance); kp 100 tracks to ~1e-3 rad (I2's yam_ultra_joint_position.json).
    # The OSC controller is left at LIBERO's defaults.
    joint_ctrl_overrides={"kp": 100},
    osc_overrides={},
)

# ---------------------------------------------------------------------------
# Real LEFT YAM arm behind the hardware bridge (gap.envs.yam_real_env).
#
# TCP FRAME — grasp_site vs the sim tcp_gap:
#   The bridge's physical TCP is the station-model ``grasp_site`` frame, which
#   is the sim ``tcp_gap`` frame rotated Rz(pi) about the tool z axis (same
#   origin). That difference is owned ENTIRELY by the real connector
#   (gap.connector.real: ``sim_quat_wxyz_to_bridge`` /
#   ``bridge_quat_wxyz_to_sim``), which post-multiplies every orientation
#   crossing its boundary by q_z(pi) — uniformly, in both directions (the
#   step is self-inverse up to quaternion sign; positions are unchanged).
#   Skills, GraspSpecs and EVERY RobotSpec constant therefore stay in the
#   sim tcp_gap convention: canonical top-down is (w,x,y,z) = (0,1,0,0)
#   here, exactly as for the sim YAM; the grasp_site equivalent
#   (0,0,1,0) = (0,1,0,0) x q_z(pi) never appears outside the connector
#   boundary. (Historical note: this spec briefly carried the grasp_site
#   value directly, which split the convention across layers — reverted in
#   favor of the single connector-owned boundary.)
#
# Frames: bridge world == left_base == arm base at origin (the station model
# root IS left_base), hence base_xpos_offset_empty = (0, 0, 0).
# ---------------------------------------------------------------------------

YAM_REAL_LEFT_SPEC = RobotSpec(
    name="yam_real_left",
    arm_dof=6,
    joint_names=tuple(f"joint{i}" for i in range(1, 7)),
    # robosuite_* are sim-construction fields; unused by the real connector.
    # They keep the sim names so tooling that introspects the spec stays sane.
    robosuite_robot="YamUltra",
    robosuite_gripper="YamLinearGripper",
    # Accepted bridge home (first_acceptance.yaml home_joints_deg): folded
    # (0, 0, 0, 88, 90, 90) deg. joint4=88 keeps the 2 deg settle-bias
    # reserve below the model's 90 deg limit. The observe pose is separate
    # and NOT yet taught -> secondary_observe_joints empty below.
    home_joints=tuple(
        math.radians(v) for v in (0.0, 0.0, 0.0, 88.0, 90.0, 90.0)
    ),
    # The bridge's remote i2rt IK targets grasp_site directly; no offset.
    ik_link="grasp_site",
    tcp_offset=(0.0, 0.0, 0.0),
    tcp_rotation_z=None,
    eef_to_ik_link_xyz=(0.0, 0.0, 0.0),
    eef_to_ik_link_wxyz=(1.0, 0.0, 0.0, 0.0),
    # Documentation only on the real path (IK is remote, in the bridge).
    robot_urdf_path=None,  # see the note above the robot table
    # Local curobo/geometry tools (robot_file default injection) keep the sim
    # model; NOTE its tool frame is tcp_gap = grasp_site rotated Rz(pi).
    planner_robot_file="yam_ultra.yml",
    planner_tool_frame="tcp_gap",
    gripper=GripperSpec(
        # The real bridge exposes the gripper as an open FRACTION [0,1];
        # qpos_closed/open are carried over from the sim linear_4310 model
        # so fraction<->qpos helpers stay meaningful.
        finger_joints=("joint7", "joint8"),
        qpos_closed=0.0,
        qpos_open=0.0475,
        # Measured 2026-09-01 with calipers on the real left gripper (user):
        # max inner pad distance fully open = 95.5 mm.
        max_opening_m=0.0955,
        hand_to_fingertip_m=0.0,  # grasp_site IS the grasp point
        pad_depth_m=0.05,
        finger_length_m=0.094,
        slide_axis_in_tool=(0.0, 1.0, 0.0),
        open_settle_steps=40,
        close_settle_steps=60,
        action_dof=1,
    ),
    # Workspace: table_z=0.0 is a PLACEHOLDER (base frame; the real table
    # height under left_base must be measured). Per explicit user decision
    # there is NO safety box / table-z client-side limit — reach_xy is wide
    # open (the bridge config carries equally wide bounds; the user
    # supervises). The z heights / topdown envelope / pitched transport are
    # carried over from the SIM-probed YAM_ULTRA_SPEC as safe defaults and
    # are PENDING a V1 re-probe on the real arm via the bridge's motion-free
    # robot.solve_ik RPC.
    workspace=WorkspaceSpec(
        # Measured 2026-09-01: board-pattern plane fitted through the wrist
        # chain in 13 pair captures = -0.0424 m median (sd 0.95 mm), minus
        # the ~3 mm board plate -> table surface -0.0454. Known bias: the
        # wrist chain reads ~+4 mm high at far reach (r ~ 0.5 m).
        table_z=-0.045,
        hover_z=0.12,
        transport_z=0.15,
        approach_z=0.15,
        max_topdown_z=0.15,
        reach_xy=2.0,
        wrist_raise_z=0.17,
        # Measured 2026-09-01 by the V1 solve_ik probe on the REAL left arm
        # (v1/out/fan_reach_20260901_152940.json): min-over-bearing vertical
        # ceiling minus a 5 mm margin. Notably higher than the sim carryover
        # (no pedestal, table at -0.045). max_topdown_z above stays the
        # conservative xy-agnostic floor.
        topdown_envelope=(
            (0.25, 0.216), (0.275, 0.216), (0.30, 0.213), (0.325, 0.206),
            (0.34, 0.202), (0.35, 0.199), (0.36, 0.196), (0.375, 0.189),
            (0.38, 0.185), (0.40, 0.172), (0.41, 0.165), (0.425, 0.154),
            (0.43, 0.151), (0.45, 0.130), (0.47, 0.106), (0.475, 0.099),
            (0.48, 0.092), (0.50, 0.051),
        ),
        transport_pitch_deg=35.0,
        transport_pitched_z=0.28,
    ),
    # SIM tcp_gap convention (canonical top-down), like every other spec:
    # the real connector owns the Rz(pi) grasp_site boundary conversion —
    # see the TCP FRAME block above and gap/connector/real.py.
    top_down_quat_wxyz=(0.0, 1.0, 0.0, 0.0),
    wrist_camera="wrist_d405",
    # Fixed top BRIO is RGB + calibration only; depth is never fabricated.
    rgb_only_cameras=("top_brio",),
    primary_camera="wrist_d405",  # D405 RGB-D = metric authority
    # Measured 2026-09-01 on the observe_main / observe_secondary wrist
    # frames (v0/out/frames/): the two finger blobs at the bottom of the
    # 640x360 view, union over both poses with margin (rigid camera-gripper
    # mount -> pose-independent). Normalized (x1, y1, x2, y2), y down.
    wrist_self_view_exclusion=(
        (0.06, 0.70, 0.36, 1.00),
        (0.62, 0.68, 0.95, 1.00),
    ),
    # Taught 2026-09-01 (v0/out/observe_poses.json, i2rt command space =
    # corrected joints): observe_secondary — the second workspace vantage
    # for the perception re-observe rescue. observe_main lives in
    # OBSERVE_MAIN_JOINTS below for session/runbook use (graphs start from
    # wherever the arm is; the bridge's folded home sees nothing useful).
    secondary_observe_joints=(
        -0.454528, 1.743534, 2.20618, -1.565598, -0.419268, -0.354963,
    ),
    robot_link_prefixes=("robot", "gripper", "link_", "tip_"),
    # Matches the bridge's joint_settle_tolerance_rad envelope for go_home;
    # per-move tools pass their own tolerances explicitly.
    joint_tolerance_rad=0.01,
    move_max_steps=300,
    post_move_settle_steps=0,
    base_xpos_offset_empty=(0.0, 0.0, 0.0),
    joint_ctrl_overrides={},
    osc_overrides={},
)

#: Left-arm MAIN observe pose (taught 2026-09-01, i2rt command space, rad):
#: wrist camera overlooks the whole workspace. Sessions/graphs move here
#: before perceiving (the bridge's folded home sees nothing useful); the
#: perception re-observe rescue uses ``secondary_observe_joints`` instead.
OBSERVE_MAIN_JOINTS: tuple[float, ...] = (
    -0.036431, 1.627565, 2.166888, -1.510284, -0.165586, -0.015068,
)

# ---------------------------------------------------------------------------
# Real RIGHT YAM arm behind ITS OWN hardware bridge (default port 9022, CAN
# can_follower_r, wrist D405 353322271910). Identical hardware to the left
# arm, so the spec is the left spec with only the name changed: the bridge
# speaks the same grasp_site TCP convention (the connector's Rz(pi) boundary
# applies unchanged), the same gripper, the same speed caps.
#
# FRAMES: the right bridge reports and accepts poses in right_base — just as
# the left bridge does in left_base — so from this connector's point of view
# the right arm's world IS right_base and base_xpos_offset_empty stays
# (0, 0, 0). The left_base <-> right_base transform (nominal right_base at
# (0, -0.61, 0) in left_base, identity rotation) is owned by the free_agent
# right-arm server (right_base_in_left_base.json), NOT by GaP.
#
# CARRY-OVER CAVEAT: every MEASURED number in the left spec (table_z -0.045,
# max_opening_m 0.0955, the top-down envelope, wrist_self_view_exclusion,
# secondary_observe_joints, and OBSERVE_MAIN_JOINTS above) was measured or
# taught on the LEFT arm. They are carried over as the best available
# defaults for the right arm and are PENDING a right-arm re-measure; in
# particular the taught observe poses are NOT mirrored.
# ---------------------------------------------------------------------------

YAM_REAL_RIGHT_SPEC = replace(YAM_REAL_LEFT_SPEC, name="yam_real_right")

ROBOT_SPECS: dict[str, RobotSpec] = {
    "panda": PANDA_SPEC,
    "yam_ultra": YAM_ULTRA_SPEC,
    "yam_real_left": YAM_REAL_LEFT_SPEC,
    "yam_real_right": YAM_REAL_RIGHT_SPEC,
}


def robot_names() -> tuple[str, ...]:
    """Registered robot names (for CLI choices / error messages)."""
    return tuple(ROBOT_SPECS)


def get_robot_spec(name: str | None = None) -> RobotSpec:
    """Resolve a robot name (default: ``GAP_SIM_ROBOT`` / ``"panda"``).

    Matching is case-insensitive and tolerant of ``-`` vs ``_``
    (``"YAM-Ultra"`` -> ``"yam_ultra"``). Raises ``KeyError`` listing the
    known names for anything else.
    """
    if name is None or not str(name).strip():
        from gap import env_config

        name = env_config.sim_robot()
    key = str(name).strip().lower().replace("-", "_")
    try:
        return ROBOT_SPECS[key]
    except KeyError:
        raise KeyError(
            f"Unknown robot {name!r}. Known robots: {sorted(ROBOT_SPECS)}"
        ) from None
