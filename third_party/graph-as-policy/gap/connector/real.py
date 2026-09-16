# Modified by the Agent-as-Policy (AgP) authors, 2026.
# Original: graph-as-policy @ f09e37e7755d0a908c1b6cf136edf13292b730c0
#   https://github.com/graph-robots/graph-as-policy  (Apache-2.0, see
#   third_party/graph-as-policy/LICENSE)
# Changes: the entire YAM Ultra real-arm path — ``YamRealConnector``,
#   ``RemoteI2rtKinematicsBackend`` (IK delegated to the hardware
#   bridge), the sim<->bridge TCP quaternion conversions
#   (``bridge_quat_wxyz_to_sim`` / ``sim_quat_wxyz_to_bridge``),
#   min-jerk waypoint timing and densification, and the ``yam_left`` /
#   ``yam_right`` branches of ``real()``.  Upstream has no YAM support
#   at all: 285 lines there, 1395 here.  (+1123/-13)
#   Per-file diffstat and the full A/B/C classification in
#   third_party/graph-as-policy/UPSTREAM.md.
"""RealConnector — real-hardware backends + the ``real()`` factory.

Three robots:

- ``franka``: :class:`gap.envs.franka_real_env.FrankaRealEnv`. The env
  binds the msgpack server (pre-seeded with a hold-home command) and the
  vendored robots_realtime client connects back to it.
  ``rr_autostart=True`` (default) spawns that client via
  :class:`gap.connector.rr_launcher.RRSession`; ``rr_autostart=False``
  restores the two-terminal debug flow (run ``uv run --directory
  third_party/robots_realtime rr-session
  configs/franka/franka_robotiq_client.yaml`` yourself).
- ``ur_zed``: :class:`gap.envs.ur_zed_env.URZedEnv`. Direct pyzed capture
  + read-only RTDE joint state. **Perception-only**: the UR side has no
  motion interface wired (RTDE receive only), so the connector registers
  exclusively observation/camera tools — no ``robot.go_to_pose`` /
  gripper / trajectory tools exist in its registry, making accidental
  motion structurally impossible.
- ``yam_left`` / ``yam_right``: :class:`gap.envs.yam_real_env.YamRealEnv`
  — the LEFT (default 127.0.0.1:9021, spec ``yam_real_left``) or RIGHT
  (default 127.0.0.1:9022, spec ``yam_real_right``) YAM arm behind its own
  independent safety-owned hardware bridge. Both arms share this one code
  path; each bridge reports poses in ITS OWN base frame (left_base /
  right_base) and no cross-arm transform is applied here. The YAM surface
  exposes synchronized RGB-D, validated
  ``grasp_site`` state, bridge-owned joint and Cartesian motion, and
  explicit remote i2rt IK. Ported from the AgP GaP-Yam fork with
  the action-lease heartbeat and full protocol validation intact. The
  sim-``tcp_gap`` <-> bridge-``grasp_site`` frame convention (Rz(pi) about
  the tool z) is converted at this connector's boundary and nowhere else —
  see :class:`YamRealConnector` for the enumerated crossings.

No ``sim.*`` tools are ever registered on a real connector — there is no
reset, no scripted success check, and no ground-truth world state on
hardware (``capabilities`` reports all False).

Safety: ``robot.go_home`` is guarded in :meth:`gap.connector.core.
Connector.go_home` — with ``config.is_real`` set the call logs a warning
and returns without moving. The YAM connector deliberately OVERRIDES that
guard: the bridge is the safety owner there and go_home moves to the
accepted folded home.

    import gap
    conn = gap.connector.real("franka")
    result = gap.execute(graph, conn)   # open-robot-skills auto-discovered
"""

from __future__ import annotations

import logging
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
from gap_core.tools import ToolRegistry
from gap_core.types import JointState, Se3Pose, Vec3

from gap.connector.core import Capabilities, Connector

logger = logging.getLogger(__name__)

#: Default rr-session config for the Franka: ViserTeleop client agent
#: (FrankaOscClientCartesianAgent, robotiq_gripper=true, client_port 9000)
#: + RobotNode + ZED CameraNode. Resolved by rr-session relative to the
#: submodule checkout.
DEFAULT_FRANKA_RR_CONFIG = "configs/franka/franka_robotiq_client.yaml"

#: Observation/camera tool subset registered for perception-only robots.
_OBSERVATION_TOOL_PREFIXES = (
    "robot.get_observation",
    "robot.get_camera_pose",
    "robot.get_ee_pose",
    "robot.get_gripper",
    "robot.get_gripper_pose",
)

_YAM_TOOLS = frozenset({
    "robot.get_observation",
    "robot.get_camera",
    "robot.get_ee_pose",
    "robot.get_gripper",
    "robot.go_to_pose",
    "robot.go_to_pose_cartesian",
    "robot.move_to_joints",
    "robot.execute_trajectory",
    "robot.go_home",
    "robot.open_gripper",
    "robot.close_gripper",
    "robot.solve_ik",
    "robot.solve_position_ik",
})

_YAM_JOINT_SETTLE_TOLERANCE_RAD = 0.03
_YAM_GRIPPER_ENDPOINT_TOLERANCE_FRACTION = 0.02


# ---------------------------------------------------------------------------
# Frame-convention boundary: sim ``tcp_gap`` <-> bridge ``grasp_site``.
#
# Skills, GraspSpecs and every RobotSpec constant speak the SIM tcp_gap
# convention (canonical top-down (w,x,y,z) = (0,1,0,0)); the bridge validates
# and moves the physical ``grasp_site`` frame = tcp_gap rotated Rz(pi) about
# the tool z axis (same origin). The YAM real connector converts UNIFORMLY at
# its boundary, in both directions: positions are unchanged, every orientation
# crossing is post-multiplied by q_z(pi).
# ---------------------------------------------------------------------------


def _rotate_quat_wxyz_about_tool_z_pi(quat_wxyz: Any) -> np.ndarray:
    """Post-multiply a wxyz quaternion by q_z(pi) — the tcp_gap<->grasp_site step.

    Derivation. q_z(pi) = (cos(pi/2), 0, 0, sin(pi/2)) = (0, 0, 0, 1) in
    wxyz. The Hamilton product q ⊗ q_z(pi) for q = (w, x, y, z) is::

        w' = w*0 - x*0 - y*0 - z*1 = -z
        x' = w*0 + x*0 + y*1 - z*0 =  y
        y' = w*0 - x*1 + y*0 + z*0 = -x
        z' = w*1 + x*0 - y*0 + z*0 =  w

    so q ⊗ q_z(pi) = (-z, y, -x, w). Sanity check: the sim canonical
    top-down (0, 1, 0, 0) maps to (0, 0, -1, 0) ~ (0, 0, 1, 0), the
    grasp_site top-down (as matrices, Rx(pi) @ Rz(pi) = Ry(pi): tool z
    straight down, jaw line along world Y).

    q_z(pi) is self-inverse up to sign (q_z(pi) ⊗ q_z(pi) = (-1, 0, 0, 0)),
    so applying the raw product twice returns -q — the identical rotation
    with negated components. The result is therefore sign-canonicalized
    (largest-magnitude component made positive, a pure sign flip that keeps
    the rotation identical) so the round trip sim -> bridge -> sim returns a
    canonical input FLOAT-EXACTLY: the plan_directed_linear adapter's
    captured-then-resent LOCK orientation reaches the bridge as exactly its
    own reported quaternion (see its pure-translation consistency note).
    """
    q = np.asarray(quat_wxyz, dtype=np.float64).reshape(4)
    w, x, y, z = (float(v) for v in q)
    out = np.array([-z, y, -x, w], dtype=np.float64)
    if out[int(np.argmax(np.abs(out)))] < 0.0:
        out = -out
    return out


def sim_quat_wxyz_to_bridge(quat_wxyz: Any) -> np.ndarray:
    """INCOMING orientation crossing: skills/sim ``tcp_gap`` -> bridge ``grasp_site``.

    Apply to EVERY orientation sent to the bridge (IK targets, Cartesian
    move targets). See :func:`_rotate_quat_wxyz_about_tool_z_pi` for the
    algebra; positions cross unchanged.
    """
    return _rotate_quat_wxyz_about_tool_z_pi(quat_wxyz)


def bridge_quat_wxyz_to_sim(quat_wxyz: Any) -> np.ndarray:
    """OUTGOING orientation crossing: bridge ``grasp_site`` -> skills/sim ``tcp_gap``.

    Apply to EVERY orientation surfaced to skills (EE pose, observation arm
    pose, captured orientations inside adapters). Same Rz(pi) step as the
    incoming direction — it is self-inverse up to quaternion sign.
    """
    return _rotate_quat_wxyz_about_tool_z_pi(quat_wxyz)

# ---------------------------------------------------------------------------
# Bridge speed caps → waypoint-time synthesis (mirrors the GaP-Yam fork's
# time_joint_path skill: minimum-jerk peak factors + a 1.05 duration margin).
# The bridge enforces 20 deg/s / 40 deg/s^2 joint caps at 50 Hz; a min-jerk
# profile over displacement d in time T peaks at 1.875*d/T velocity and
# 5.7735*d/T^2 acceleration, so a segment is feasible when
#   T >= 1.875 * d / v_max   and   T >= sqrt(5.7735 * d / a_max).
# ---------------------------------------------------------------------------
_YAM_MAX_JOINT_VELOCITY_RAD_S = math.radians(20.0)
_YAM_MAX_JOINT_ACCELERATION_RAD_S2 = math.radians(40.0)
_MIN_JERK_PEAK_VELOCITY = 1.875
_MIN_JERK_PEAK_ACCELERATION = 5.773502691896258
_TIME_SYNTH_DURATION_MARGIN = 1.05
#: Per-segment floor for dense (already interpolated) plans: one 50 Hz tick.
_TIME_SYNTH_MIN_SEGMENT_S = 0.02
#: The first waypoint's absolute time: budget for the controller to sync onto
#: the trajectory start (assumed within ~5 deg of the current configuration).
_TIME_SYNTH_START_SYNC_RAD = math.radians(5.0)
_TIME_SYNTH_START_MIN_S = 0.20


def _min_jerk_duration(
    displacement_rad: float, *, min_duration_s: float = _TIME_SYNTH_MIN_SEGMENT_S
) -> float:
    """Minimum-jerk-feasible duration for the worst-joint displacement."""
    velocity_duration = (
        _MIN_JERK_PEAK_VELOCITY * displacement_rad / _YAM_MAX_JOINT_VELOCITY_RAD_S
    )
    acceleration_duration = math.sqrt(
        _MIN_JERK_PEAK_ACCELERATION * displacement_rad
        / _YAM_MAX_JOINT_ACCELERATION_RAD_S2
    )
    return _TIME_SYNTH_DURATION_MARGIN * max(
        min_duration_s, velocity_duration, acceleration_duration
    )


def synthesize_min_jerk_waypoint_times(
    waypoints: Any, *, initial_joints: Any | None = None
) -> list[float]:
    """Absolute waypoint times (s) respecting the YAM bridge joint caps.

    General adapter for callers (skills, the plan adapter) that hand
    ``robot.execute_trajectory`` a bare joint path: each segment gets the
    minimum-jerk-feasible duration for its worst-joint displacement under
    the 20 deg/s / 40 deg/s^2 bridge caps (with a 1.05 margin), and the
    first waypoint gets a start-sync budget sized for a <= 5 deg offset —
    or, when ``initial_joints`` (the measured current configuration) is
    given, for the ACTUAL initial-to-first displacement when that is larger.
    Returns a strictly increasing list of ``len(waypoints)`` times.
    """
    arr = np.asarray(waypoints, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] < 1:
        raise ValueError("waypoints must be a non-empty (N, dof) array")
    if not np.isfinite(arr).all():
        raise ValueError("waypoints contain non-finite values")
    start_displacement = _TIME_SYNTH_START_SYNC_RAD
    if initial_joints is not None:
        initial = np.asarray(initial_joints, dtype=np.float64).reshape(-1)
        start_displacement = max(
            start_displacement, float(np.max(np.abs(arr[0] - initial)))
        )
    times = [
        _min_jerk_duration(
            start_displacement, min_duration_s=_TIME_SYNTH_START_MIN_S
        )
    ]
    for previous, current in zip(arr, arr[1:]):
        displacement = float(np.max(np.abs(current - previous)))
        times.append(times[-1] + _min_jerk_duration(displacement))
    return times


#: Densification threshold: the bridge fails closed on any adjacent-waypoint
#: (and initial-to-first) worst-joint jump > 0.5 rad; subdividing at 0.4 rad
#: keeps every streamed jump comfortably under that limit.
_YAM_BRIDGE_MAX_JOINT_JUMP_RAD = 0.5
_DENSIFY_MAX_JOINT_JUMP_RAD = 0.4


def densify_waypoint_jumps(
    waypoints: Any,
    waypoint_times_s: Any | None = None,
    *,
    initial_joints: Any | None = None,
    max_jump_rad: float = _DENSIFY_MAX_JOINT_JUMP_RAD,
) -> tuple[list[list[float]], list[float] | None]:
    """Subdivide waypoint segments so no worst-joint jump exceeds ``max_jump_rad``.

    The bridge rejects any adjacent-waypoint worst-joint jump > 0.5 rad, and
    likewise the initial-to-first jump against the measured configuration.
    Every segment whose worst-joint delta exceeds the 0.4 rad threshold is
    subdivided by LINEAR interpolation (mirroring the GaP-Yam fork's 50 Hz
    min-jerk sampling in spirit, at waypoint rather than tick granularity):

    - interior segments get ``ceil(delta / max_jump_rad) - 1`` evenly spaced
      inserted waypoints;
    - when ``initial_joints`` is given and the initial-to-first jump exceeds
      the threshold, intermediate points from the current configuration to
      the first waypoint are PREPENDED (the current configuration itself is
      not — the bridge prepends it at t=0);
    - when ``waypoint_times_s`` is given, inserted waypoints get linearly
      interpolated times at the same fractions (subdividing a segment this
      way preserves its per-segment velocity, so a cap-compliant timing
      stays cap-compliant); prepended head points subdivide ``times[0]``.

    Returns ``(dense_waypoints, dense_times_or_None)``; callers synthesize
    times AFTER densification when none were provided.
    """
    arr = np.asarray(waypoints, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] < 1:
        raise ValueError("waypoints must be a non-empty (N, dof) array")
    times = (
        None
        if waypoint_times_s is None
        else np.asarray(waypoint_times_s, dtype=np.float64).reshape(-1)
    )
    if times is not None and times.shape != (len(arr),):
        raise ValueError("waypoint_times_s must have one value per waypoint")

    out_waypoints: list[list[float]] = []
    out_times: list[float] | None = None if times is None else []

    def _emit(joints: np.ndarray, t: float | None) -> None:
        out_waypoints.append([float(v) for v in joints])
        if out_times is not None:
            assert t is not None
            out_times.append(float(t))

    if initial_joints is not None:
        initial = np.asarray(initial_joints, dtype=np.float64).reshape(-1)
        jump = float(np.max(np.abs(arr[0] - initial)))
        if jump > max_jump_rad:
            segments = math.ceil(jump / max_jump_rad)
            for k in range(1, segments):
                fraction = k / segments
                _emit(
                    initial + fraction * (arr[0] - initial),
                    None if times is None else float(times[0]) * fraction,
                )
    _emit(arr[0], None if times is None else float(times[0]))
    for index in range(1, len(arr)):
        previous, current = arr[index - 1], arr[index]
        jump = float(np.max(np.abs(current - previous)))
        if jump > max_jump_rad:
            segments = math.ceil(jump / max_jump_rad)
            for k in range(1, segments):
                fraction = k / segments
                _emit(
                    previous + fraction * (current - previous),
                    None
                    if times is None
                    else float(times[index - 1])
                    + fraction * float(times[index] - times[index - 1]),
                )
        _emit(current, None if times is None else float(times[index]))
    return out_waypoints, out_times


class RemoteI2rtKinematicsBackend:
    """GaP-side adapter for the bridge's validated i2rt ``grasp_site`` IK."""

    trajectory_needs_joint_reverse = False

    def __init__(self, env: Any) -> None:
        self._env = env

    def solve_ik(
        self,
        target_world_pose: dict[str, Any],
        *,
        arm_id: int = 0,
        seed_joints: list[float] | None = None,
        tcp_offset: np.ndarray | None = None,
    ) -> list[float] | None:
        if arm_id != 0:
            raise ValueError("YAM has exactly one arm with arm_id=0")
        if seed_joints is None:
            raise ValueError("YAM i2rt IK requires the current six-joint seed")
        if tcp_offset is not None and not np.allclose(tcp_offset, 0.0, atol=0.0):
            raise ValueError("YAM IK targets the real grasp_site and accepts no TCP offset")
        position = target_world_pose["position"]
        rotation = target_world_pose["rotation"]
        # POSE CROSSING (incoming): callers (skills via robot.solve_ik) hand
        # sim-tcp_gap-convention orientations; the bridge IK targets the
        # physical grasp_site frame. Positions cross unchanged (same origin).
        quat_bridge = sim_quat_wxyz_to_bridge(
            (rotation["w"], rotation["x"], rotation["y"], rotation["z"])
        )
        pose = np.concatenate(
            (
                np.array(
                    [position["x"], position["y"], position["z"]],
                    dtype=np.float64,
                ),
                quat_bridge,
            )
        ).astype(np.float32)
        solution = self._env.solve_ik(pose, seed_joints=seed_joints)
        return None if solution is None else solution.astype(float).tolist()


class RealConnector(Connector):
    """Connector over a real-hardware environment.

    Args:
        env: Real env instance (FrankaRealEnv / URZedEnv).
        config: The env's ``EnvConfig`` (``is_real=True``).
        camera_names: Camera override; defaults to ``config.default_cameras``.
        ik: Optional pre-built IK backend.
        rr_session: Optional :class:`~gap.connector.rr_launcher.RRSession`
            whose lifetime this connector owns (terminated on ``close()``).
        motion_enabled: When False (perception-only robots), register only
            the observation/camera tools — no motion, gripper, trajectory,
            or IK tools.
    """

    def __init__(
        self,
        env: Any,
        config: Any,
        *,
        camera_names: list[str] | None = None,
        ik: Any | None = None,
        rr_session: Any | None = None,
        motion_enabled: bool = True,
    ) -> None:
        super().__init__(env, config, camera_names=camera_names, ik=ik)
        self._rr_session = rr_session
        self._motion_enabled = bool(motion_enabled)

    # ------------------------------------------------------------------
    # Capabilities
    # ------------------------------------------------------------------

    @property
    def capabilities(self) -> Capabilities:
        """Real hardware: no scripted reset, success check, video, or world state.

        The env classes expose ``reset``/``enable_video_capture`` shims for
        interface parity, but on hardware "reset" only means "wait for an
        observation" and there is no ground truth — the benchmark harness
        must not branch into sim-only flows here.
        """
        return Capabilities(
            reset=False, success_check=False, video=False, world_state=False,
        )

    # ------------------------------------------------------------------
    # Tool registration
    # ------------------------------------------------------------------

    def _register_robot_tools(self, reg: ToolRegistry) -> None:
        if self._motion_enabled:
            super()._register_robot_tools(reg)
            return
        # Perception-only (ur_zed): observation/camera getters exclusively.
        rc = reg.register_callable
        rc("robot.get_observation", self.get_observation,
           summary="Capture the current observation: all cameras + arm states.")
        rc("robot.get_camera_pose", self.get_camera_pose,
           summary="Get one camera's world pose by name.")
        rc("robot.get_ee_pose", self._tool_get_ee_pose,
           summary="Get the end-effector pose in world frame.")
        rc("robot.get_gripper", self._tool_get_gripper,
           summary="Get the gripper open fraction (0 closed, 1 open).")
        rc("robot.get_gripper_pose", self._tool_get_gripper_pose,
           summary="Get the gripper (end-effector) pose in world frame.")

    # No _register_extra_tools override: the base hook is a no-op, so no
    # sim.* tools are ever registered on a real connector.

    # ------------------------------------------------------------------
    # Readiness
    # ------------------------------------------------------------------

    def wait_ready(self, timeout_s: float = 60.0, poll_s: float = 0.5) -> None:
        """Block until the first RGB observation arrives from the hardware.

        Polls ``env.get_observation()`` until the primary camera carries an
        RGB frame. On timeout, raises ``TimeoutError`` carrying the env's
        ``_diagnose_missing_rgb`` diagnosis (which wire stage is silent)
        when the env provides one.

        Envs that define their own ``wait_ready`` (the YAM bridge env, which
        waits for ``bridge_health.state == "ok"``) get it called instead.
        """
        env_wait_ready = getattr(self.env, "wait_ready", None)
        if callable(env_wait_ready):
            env_wait_ready(timeout_s=timeout_s, poll_s=poll_s)
            logger.info(
                "real connector ready: environment observation contract satisfied"
            )
            return

        cam_name = self.camera_names[0] if self.camera_names else None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                obs = self.env.get_observation()
            except Exception:
                logger.debug("wait_ready: get_observation failed", exc_info=True)
                obs = {}
            for key in ([cam_name] if cam_name else list(obs.keys())):
                cam = obs.get(key) if key else None
                if isinstance(cam, dict) and cam.get("images", {}).get("rgb") is not None:
                    logger.info("real connector ready: camera %r streaming", key)
                    return
            time.sleep(poll_s)

        diagnosis = ""
        diagnose = getattr(self.env, "_diagnose_missing_rgb", None)
        if callable(diagnose):
            try:
                diagnosis = " " + diagnose(cam_name or "")
            except Exception:
                pass
        rr_hint = ""
        if self._rr_session is not None and self._rr_session.poll() is not None:
            rr_hint = (
                f" rr-session exited with code {self._rr_session.poll()}"
                f" (see {self._rr_session.log_path})."
            )
        raise TimeoutError(
            f"No RGB observation from the real robot within {timeout_s:.0f}s."
            f"{rr_hint}{diagnosis}"
        )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Terminate the owned rr-session (if any), then close the env."""
        if self._closed:
            return
        if self._rr_session is not None:
            try:
                self._rr_session.terminate()
            except Exception:
                logger.warning("rr-session terminate failed", exc_info=True)
        super().close()


class YamRealConnector(RealConnector):
    """YAM surface backed only by bridge-owned motion and remote i2rt IK.

    Every motion tool maps 1:1 onto a validated bridge action; the base
    class's IK/planning orchestration (cuRobo, servo, linear plans) is
    never engaged. ``curobo.plan_directed_linear`` is provided as a
    hardware adapter tool via :meth:`register_tool_overrides` so the
    directed-grasp skills keep their sim contract unchanged.

    FRAME-CONVENTION BOUNDARY (the ONE place it lives): skills, GraspSpecs
    and RobotSpec constants stay entirely in the sim ``tcp_gap`` convention
    (canonical top-down (w,x,y,z) = (0,1,0,0)); the bridge speaks the
    physical ``grasp_site`` frame = tcp_gap rotated Rz(pi) about the tool z.
    This connector converts uniformly at its boundary via
    :func:`sim_quat_wxyz_to_bridge` / :func:`bridge_quat_wxyz_to_sim`.
    Every pose crossing, enumerated:

    INCOMING (sim -> bridge): ``robot.go_to_pose`` targets
    (:meth:`_tool_go_to_pose`), ``robot.go_to_pose_cartesian`` targets
    (:meth:`_tool_go_to_pose_cartesian`), ``robot.solve_ik`` targets
    (:class:`RemoteI2rtKinematicsBackend.solve_ik`), and the
    ``curobo.plan_directed_linear`` adapter's outbound goal (IK precheck +
    Cartesian move). ``robot.solve_position_ik`` carries no orientation and
    IK seeds are joint vectors — nothing to convert there.

    OUTGOING (bridge -> sim): :meth:`get_ee_pose` (serves
    ``robot.get_ee_pose``), the observation's arm ``ee_pose``
    (:meth:`get_observation`), and the plan adapter's captured current
    orientation. Camera poses (``camera_to_world`` extrinsics) are camera
    frames, not tool frames — never converted. Bridge action results carry
    only joints (``final_joint_pos_0``), never poses.
    """

    # ------------------------------------------------------------------
    # Video (independent second bridge reader)
    # ------------------------------------------------------------------

    def start_video(self, output_path: str, *, fps: float = 10.0) -> None:
        """Start continuous top/wrist recording on an independent read client."""
        if getattr(self, "_video_recorder", None) is not None:
            raise RuntimeError("YAM video recording is already active")
        from gap.connector.yam_video import YamVideoRecorder
        from gap.envs.yam_real_env import YamRealEnv

        client = self.env._client
        reader = YamRealEnv(
            host=client.host,
            port=client.port,
            request_timeout_s=client.request_timeout_s,
            stale_after_s=client.stale_after_ns / 1e9,
            camera_stale_after_s=client.camera_stale_after_ns / 1e9,
            max_camera_joint_skew_s=client.max_camera_joint_skew_ns / 1e9,
            heartbeat_interval_s=client._heartbeat_interval_s,
            camera_names=list(self.camera_names),
        )
        recorder = YamVideoRecorder(reader, output_path, fps=fps)
        self._video_recorder = recorder
        recorder.start()

    def save_video(
        self,
        output_path: str,
        fps: int = 10,
        clear: bool = False,
    ) -> dict[str, Any]:
        """Finalize continuous camera files started by :meth:`start_video`."""
        del output_path, fps, clear
        recorder = getattr(self, "_video_recorder", None)
        if recorder is None:
            return {
                "success": False,
                "file_path": "",
                "num_frames": 0,
                "frame_counts": {},
                "camera_files": {},
                "error": "YAM video recording was not started",
            }
        result = recorder.stop(capture_final=True)
        self._video_recorder = None
        return result

    def close(self) -> None:
        recorder = getattr(self, "_video_recorder", None)
        if recorder is not None:
            recorder.stop(capture_final=True)
            self._video_recorder = None
        super().close()

    # ------------------------------------------------------------------
    # Tool registration
    # ------------------------------------------------------------------

    def _register_robot_tools(self, reg: ToolRegistry) -> None:
        tools = {
            "robot.get_observation": (
                self.get_observation,
                "Capture calibrated camera and arm state.",
                (),
            ),
            "robot.get_camera": (
                self.get_camera,
                "Capture one named calibrated camera.",
                (),
            ),
            "robot.get_ee_pose": (self._tool_get_ee_pose, "Get grasp_site in world frame.", ()),
            "robot.get_gripper": (self._tool_get_gripper, "Get gripper open fraction.", ()),
            "robot.go_to_pose": (
                self._tool_go_to_pose,
                "Move the grasp point to a world-frame pose (bridge IK + min-jerk joint move).",
                ("sim_step",),
            ),
            "robot.go_to_pose_cartesian": (
                self._tool_go_to_pose_cartesian,
                "Move the real grasp_site to a world-frame Cartesian pose.",
                ("sim_step",),
            ),
            "robot.move_to_joints": (self._tool_move_to_joints, "Move to six absolute joints.", ("sim_step",)),
            "robot.execute_trajectory": (self._tool_execute_trajectory, "Execute a timed six-joint trajectory.", ("sim_step",)),
            "robot.go_home": (self.go_home, "Move to the accepted YAM home pose.", ("sim_step",)),
            "robot.open_gripper": (self.open_gripper, "Open the linear gripper.", ("sim_step",)),
            "robot.close_gripper": (self.close_gripper, "Close the linear gripper.", ("sim_step",)),
            "robot.solve_ik": (self._tool_solve_ik, "Solve grasp_site IK with the bridge i2rt model.", ("planning",)),
            "robot.solve_position_ik": (
                self._tool_solve_position_ik,
                "Solve grasp_site position with unconstrained orientation.",
                ("planning",),
            ),
        }
        for name in _YAM_TOOLS:
            callable_, summary, tags = tools[name]
            reg.register_callable(name, callable_, summary=summary, tags=tags)

    def register_tool_overrides(self, reg: ToolRegistry) -> None:
        """Shadow bundle tools that cannot run against real hardware.

        Called by ``gap.execute`` AFTER the skill bundles' ``@tool``
        registrations were drained into the registry, so the hardware
        adapter deterministically replaces the sim/GPU implementation of
        the same name (registering earlier would make the bundle drain
        collide instead).
        """
        reg.register_callable(
            "curobo.plan_directed_linear",
            self._tool_plan_directed_linear,
            summary=(
                "Directed straight-line move of the real grasp_site (bridge "
                "IK precheck, then an orientation-held minimum-jerk linear "
                "move). Same contract as the sim planner tool, but the "
                "returned trajectory is already executed."
            ),
            tags=("planning", "sim_step"),
            replace=True,
        )

    # ------------------------------------------------------------------
    # Observation / IK tools
    # ------------------------------------------------------------------

    def get_observation(self) -> Any:
        """Observation with the arm EE orientation in the SIM convention.

        POSE CROSSING (outgoing): ``robot_cartesian_pos_0[3:7]`` arrives in
        the bridge's grasp_site convention; convert it so the assembled
        ``arms[0].ee_pose`` reaches skills in the sim tcp_gap convention.
        Positions and camera poses (camera extrinsics, not tool frames)
        cross unchanged.
        """
        obs = dict(self.env.get_observation())
        cart = obs.get("robot_cartesian_pos_0")
        if cart is not None and len(cart) >= 7:
            cart = np.array(cart, copy=True)
            cart[3:7] = bridge_quat_wxyz_to_sim(cart[3:7])
            obs["robot_cartesian_pos_0"] = cart
        return self._build_observation(obs)

    def get_ee_pose(self, arm_id: int = 0) -> Se3Pose:
        """grasp point pose with the orientation in the SIM convention.

        POSE CROSSING (outgoing): serves ``robot.get_ee_pose`` (and any
        internal reader) — the bridge-reported grasp_site quaternion is
        converted to the sim tcp_gap convention at this boundary.
        """
        pose = super().get_ee_pose(arm_id=arm_id)
        rotation = pose["rotation"]
        w, x, y, z = bridge_quat_wxyz_to_sim(
            (rotation["w"], rotation["x"], rotation["y"], rotation["z"])
        )
        pose["rotation"] = {
            "w": float(w), "x": float(x), "y": float(y), "z": float(z),
        }
        return pose

    def get_camera(self, camera_name: str = "wrist_d405") -> dict[str, Any]:
        for camera in self.get_observation()["cameras"]:
            if camera["name"] == camera_name:
                return {"camera": camera}
        from gap_core.errors import ToolError

        raise ToolError("robot.get_camera", f"Camera {camera_name!r} not found")

    def _tool_solve_position_ik(
        self,
        position: Vec3,
        seed_joints: JointState,
    ) -> dict[str, Any]:
        from gap_core.errors import ToolError

        from gap.connector.core import _as_positions, _as_vec3

        try:
            target = _as_vec3(position)
            seed = _as_positions(seed_joints)
            if target is None or not np.isfinite(target).all():
                raise ValueError("position must contain three finite values")
            if len(seed) != 6 or not np.isfinite(seed).all():
                raise ValueError("seed_joints must contain six finite joint radians")
            joints = self.env.solve_position_ik(target, seed_joints=seed)
        except (TypeError, ValueError) as exc:
            raise ToolError("robot.solve_position_ik", str(exc)) from exc
        if joints is None:
            raise ToolError("robot.solve_position_ik", "Target position is unreachable")
        try:
            solution = np.asarray(joints, dtype=np.float64)
        except (TypeError, ValueError) as exc:
            raise ToolError(
                "robot.solve_position_ik",
                "IK backend must return six finite joint radians",
            ) from exc
        if solution.shape != (6,) or not np.isfinite(solution).all():
            raise ToolError(
                "robot.solve_position_ik",
                "IK backend must return six finite joint radians",
            )
        return {"joint_config": {"positions": solution.tolist()}, "success": True}

    # ------------------------------------------------------------------
    # Motion tools
    # ------------------------------------------------------------------

    def _tool_move_to_joints(
        self,
        joint_config: Any,
        tolerance: float = 0.01,
        timeout_s: float = 10.0,
    ) -> dict[str, Any]:
        from gap.connector.core import _as_positions

        positions = _as_positions(joint_config) if joint_config is not None else []
        if len(positions) != 6:
            from gap_core.errors import ToolError

            raise ToolError("robot.move_to_joints", "six joint positions are required")
        return self.env.move_to_joints_blocking(
            positions, tolerance=tolerance, timeout_s=timeout_s, arm_id=0
        )

    @staticmethod
    def _parse_sim_pose7(pose: Se3Pose, tool_name: str) -> np.ndarray:
        """Validate an Se3Pose and return float64 ``[xyz, wxyz]`` (sim convention)."""
        from gap_core.errors import ToolError

        try:
            position = pose["position"]
            rotation = pose["rotation"]
            target = np.asarray(
                [
                    position["x"],
                    position["y"],
                    position["z"],
                    rotation["w"],
                    rotation["x"],
                    rotation["y"],
                    rotation["z"],
                ],
                dtype=np.float64,
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ToolError(
                tool_name,
                "pose must contain finite position xyz and rotation wxyz",
            ) from exc
        if target.shape != (7,) or not np.isfinite(target).all():
            raise ToolError(
                tool_name,
                "pose must contain finite position xyz and rotation wxyz",
            )
        quaternion_norm = float(np.linalg.norm(target[3:]))
        if not np.isclose(quaternion_norm, 1.0, atol=1e-4):
            raise ToolError(tool_name, "rotation wxyz must be a unit quaternion")
        return target

    @staticmethod
    def _sim_pose7_to_bridge(target_sim: np.ndarray) -> np.ndarray:
        """POSE CROSSING (incoming): sim-convention 7-vector -> bridge float32."""
        return np.concatenate(
            (target_sim[:3], sim_quat_wxyz_to_bridge(target_sim[3:7]))
        ).astype(np.float32)

    def _joint_move_timeout_s(self, displacement_rad: float) -> float:
        """Timeout for a bridge min-jerk joint move, from the worst-joint
        displacement: double the feasible duration plus settle margin, never
        below the 10 s bridge action default."""
        return max(10.0, 2.0 * _min_jerk_duration(displacement_rad) + 5.0)

    def _tool_go_to_pose(
        self,
        pose: Se3Pose,
        tolerance: float = 0.0,
        max_steps: int = 0,
        tcp_offset: Any | None = None,
        arm_id: int = 0,
        z_approach: float = 0.0,
        timeout_s: float = 0.0,
    ) -> dict[str, Any]:
        """Free-space pose move: bridge ``solve_ik`` -> min-jerk joint move.

        Same input contract as the sim connector's ``robot.go_to_pose`` (the
        directed-grasp skills call it per candidate and catch exceptions):
        the target is prechecked with the bridge's motion-free ``solve_ik``
        RPC seeded with the CURRENT joints, and an IK failure raises
        ``ToolError("robot.go_to_pose", "IK failed for target pose")``; on
        success the solution is executed as a bridge-owned min-jerk
        ``absolute_joints`` move. ``z_approach > 0`` first moves to a point
        that far above the target (sim contract parity).

        ``max_steps`` is a sim stepping budget and is ignored; ``tcp_offset``
        must be omitted or zero (the bridge IK targets the physical grasp
        point itself). ``timeout_s <= 0`` auto-sizes each move's timeout from
        its worst-joint displacement.

        POSE CROSSING (incoming): ``pose`` arrives in the sim tcp_gap
        convention and is converted to the bridge grasp_site convention here.
        """
        from gap_core.errors import ToolError

        from gap.connector.core import _as_vec3

        del max_steps  # sim stepping budget; the bridge owns motion timing
        if arm_id != 0:
            raise ToolError("robot.go_to_pose", "YAM has exactly one arm with arm_id=0")
        offset = _as_vec3(tcp_offset)
        if offset is not None and not np.allclose(offset, 0.0, atol=0.0):
            raise ToolError(
                "robot.go_to_pose",
                "YAM IK targets the real grasp point and accepts no TCP offset",
            )
        if not pose:
            raise ToolError("robot.go_to_pose", "pose required")
        target_sim = self._parse_sim_pose7(pose, "robot.go_to_pose")
        tolerance = tolerance if tolerance > 0 else self._joint_tolerance

        current = np.asarray(
            self.env.get_observation()["robot_joint_pos_0"][:6], dtype=np.float64
        )

        def _solve_and_move(
            pose_sim: np.ndarray, seed: np.ndarray, failure: str
        ) -> dict[str, Any]:
            solution = self.env.solve_ik(
                self._sim_pose7_to_bridge(pose_sim), seed_joints=seed
            )
            if solution is None:
                raise ToolError("robot.go_to_pose", failure)
            joints = np.asarray(solution, dtype=np.float64)[:6]
            displacement = float(np.max(np.abs(joints - seed)))
            return self.env.move_to_joints_blocking(
                joints,
                tolerance=tolerance,
                timeout_s=(
                    timeout_s
                    if timeout_s > 0
                    else self._joint_move_timeout_s(displacement)
                ),
                arm_id=0,
            )

        if z_approach > 0:
            approach_sim = target_sim.copy()
            approach_sim[2] += float(z_approach)
            result = _solve_and_move(
                approach_sim, current, "IK failed for approach pose"
            )
            current = np.asarray(
                result["final_joint_pos_0"][:6], dtype=np.float64
            )
        return _solve_and_move(target_sim, current, "IK failed for target pose")

    def _tool_go_to_pose_cartesian(
        self,
        pose: Se3Pose,
        timeout_s: float = 10.0,
    ) -> dict[str, Any]:
        target_sim = self._parse_sim_pose7(pose, "robot.go_to_pose_cartesian")
        # POSE CROSSING (incoming): sim tcp_gap -> bridge grasp_site.
        return self.env.move_to_cartesian_blocking(
            self._sim_pose7_to_bridge(target_sim),
            timeout_s=timeout_s,
            arm_id=0,
        )

    def _tool_execute_trajectory(
        self,
        trajectory: Any,
        waypoint_times_s: list[float] | None = None,
        tolerance: float = 0.01,
        timeout_s: float = 0.0,
    ) -> dict[str, Any]:
        """Execute a six-joint trajectory through the bridge.

        Adaptations over the ported fork tool:

        - ``waypoint_times_s`` is OPTIONAL: when omitted, feasible times are
          synthesized from the bridge speed caps (20 deg/s / 40 deg/s^2)
          via :func:`synthesize_min_jerk_waypoint_times` — skills that call
          ``robot.execute_trajectory(trajectory=...)`` bare (the sim
          contract) keep working.
        - A single-waypoint trajectory (the real
          ``curobo.plan_directed_linear`` adapter returns one equal to the
          already-reached joints) is executed as a blocking absolute-joints
          settle — effectively a no-op hold — instead of being rejected by
          the bridge's N >= 2 streaming contract; its timeout auto-sizes
          from the worst-joint displacement to the hold target.
        - The bridge fails closed on any adjacent-waypoint (and
          initial-to-first) worst-joint jump > 0.5 rad, so the path is
          densified via :func:`densify_waypoint_jumps` BEFORE time
          synthesis (segments over 0.4 rad get linearly interpolated
          waypoints; the measured current configuration anchors the
          initial-to-first segment).
        - ``timeout_s <= 0`` auto-sizes the timeout from the final waypoint
          time.
        """
        from gap_core.errors import ToolError

        from gap.connector.core import _as_positions

        if not trajectory or not trajectory.get("waypoints"):
            raise ToolError("robot.execute_trajectory", "trajectory required")
        waypoints = [_as_positions(waypoint) for waypoint in trajectory["waypoints"]]
        if any(len(waypoint) < 6 for waypoint in waypoints):
            raise ToolError(
                "robot.execute_trajectory",
                "every waypoint needs six joint positions",
            )
        waypoints = [waypoint[:6] for waypoint in waypoints]
        current = np.asarray(
            self.env.get_observation()["robot_joint_pos_0"][:6], dtype=np.float64
        )
        if len(waypoints) == 1:
            # The bridge's absolute_joints action min-jerk-samples the move
            # itself (no waypoint-jump limit applies); auto-size its timeout
            # from the worst-joint displacement instead of a flat 10 s.
            displacement = float(
                np.max(np.abs(np.asarray(waypoints[0], dtype=np.float64) - current))
            )
            return self.env.move_to_joints_blocking(
                waypoints[0],
                tolerance=tolerance,
                timeout_s=(
                    timeout_s
                    if timeout_s > 0
                    else self._joint_move_timeout_s(displacement)
                ),
                arm_id=0,
            )
        original_count = len(waypoints)
        waypoints, waypoint_times_s = densify_waypoint_jumps(
            waypoints, waypoint_times_s, initial_joints=current
        )
        if len(waypoints) != original_count:
            logger.info(
                "robot.execute_trajectory: densified %d -> %d waypoints "
                "(bridge rejects worst-joint jumps > %.2f rad)",
                original_count, len(waypoints), _YAM_BRIDGE_MAX_JOINT_JUMP_RAD,
            )
        if waypoint_times_s is None:
            waypoint_times_s = synthesize_min_jerk_waypoint_times(
                waypoints, initial_joints=current
            )
            logger.info(
                "robot.execute_trajectory: synthesized %d waypoint times "
                "(%.2f s total) from bridge speed caps",
                len(waypoint_times_s), waypoint_times_s[-1],
            )
        if timeout_s <= 0:
            timeout_s = max(30.0, float(np.max(np.asarray(waypoint_times_s))) + 10.0)
        return self.env.stream_joint_trajectory(
            waypoints,
            waypoint_times_s=waypoint_times_s,
            timeout_s=timeout_s,
            position_tolerance_rad=tolerance,
            arm_id=0,
        )

    def go_home(self, timeout_s: float = 30.0) -> dict[str, Any]:
        # Deliberately overrides the base is_real no-op guard: the bridge is
        # the safety owner on the YAM and home is the accepted folded pose.
        if self._home_joints is None:
            raise RuntimeError("YAM home joints are not configured")
        return self.env.move_to_joints_blocking(
            self._home_joints,
            timeout_s=timeout_s,
            tolerance=_YAM_JOINT_SETTLE_TOLERANCE_RAD,
            arm_id=0,
        )

    def open_gripper(self, timeout_s: float = 5.0) -> dict[str, Any]:
        result = self.env._set_gripper(
            1.0,
            timeout_s=timeout_s,
            tolerance=_YAM_GRIPPER_ENDPOINT_TOLERANCE_FRACTION,
            arm_id=0,
            stop_on_contact=False,
        )
        return {**result, "position": float(result["final_joint_pos_0"][6])}

    def close_gripper(self, timeout_s: float = 5.0) -> dict[str, Any]:
        result = self.env._set_gripper(
            0.0,
            timeout_s=timeout_s,
            tolerance=_YAM_GRIPPER_ENDPOINT_TOLERANCE_FRACTION,
            arm_id=0,
            stop_on_contact=True,
        )
        return {**result, "position": float(result["final_joint_pos_0"][6])}

    # ------------------------------------------------------------------
    # curobo.plan_directed_linear hardware adapter
    # ------------------------------------------------------------------

    def _tool_plan_directed_linear(
        self,
        start_joint_position: JointState,
        start_pose: Se3Pose | None = None,
        target_pose: Se3Pose | None = None,
        allowed_axes: list[str] | None = None,
        explicit_direction: Vec3 | None = None,
        distance: float | None = None,
        endpoint_mode: str = "PROJECT_TO_TARGET",
        orientation_mode: str = "LOCK",
        orientation_target: Any | None = None,
        robot_file: str = "",
    ) -> dict[str, Any]:
        """Real-hardware ``curobo.plan_directed_linear`` (same input contract).

        Instead of planning a joint trajectory it (1) computes the endpoint
        from the CURRENT measured grasp_site pose (+ ``distance`` along
        ``explicit_direction`` / projected onto ``target_pose``), (2)
        prechecks reachability with the bridge's motion-free ``solve_ik``
        RPC — an unreachable endpoint returns ``{"success": False,
        "trajectory": None, "failure_reason": "ik_none"}`` WITHOUT raising —
        and (3) on success EXECUTES the move immediately via the bridge's
        ``cartesian_pose`` action, which realizes an orientation-held
        minimum-jerk straight-line move under the bridge speed caps. The
        execution is fully non-throwing: a mid-move bridge fault returns
        ``{"success": False, "trajectory": None, "failure_reason":
        str(exc)}`` and a non-"completed" action status returns
        ``failure_reason = "bridge_" + status``.

        The successful result carries a one-waypoint trajectory equal to the
        REACHED joints, so the skills' follow-up
        ``robot.execute_trajectory(res["trajectory"])`` is a no-op hold (see
        ``_tool_execute_trajectory``); ``note`` documents that execution
        already happened. ``start_pose`` and ``robot_file`` are accepted for
        contract parity and ignored (the bridge owns FK and the model).
        """
        del start_pose, robot_file

        def _fail(reason: str) -> dict[str, Any]:
            logger.info("[yam plan_directed_linear] infeasible: %s", reason)
            return {"success": False, "trajectory": None, "failure_reason": reason}

        from gap.connector.core import _as_positions, _as_vec3

        # --- current state (bridge-validated) --------------------------
        obs = self.env.get_observation()
        current_joints = np.asarray(
            obs["robot_joint_pos_0"][:6], dtype=np.float64
        )
        cart = np.asarray(obs["robot_cartesian_pos_0"][:7], dtype=np.float64)
        start_position = cart[:3].copy()
        # POSE CROSSING (outgoing): the bridge reports the grasp_site
        # orientation; this adapter's orientation algebra (LOCK, targets
        # from skills) runs in the sim tcp_gap convention.
        start_quat = bridge_quat_wxyz_to_sim(cart[3:7])

        # --- start_joint_position: contract parity + safety check ------
        # The sim tool plans from FK(start_joint_position); the real adapter
        # has no client-side FK and moves the PHYSICAL arm, so the start must
        # be the arm's actual configuration (skills read it from the current
        # observation immediately before calling).
        try:
            requested = np.asarray(
                _as_positions(start_joint_position)[:6], dtype=np.float64
            )
        except (TypeError, ValueError, KeyError):
            return _fail("start_joint_position must contain six joint radians")
        if requested.shape != (6,) or not np.isfinite(requested).all():
            return _fail("start_joint_position must contain six joint radians")
        if float(np.max(np.abs(requested - current_joints))) > 0.05:
            return _fail(
                "start_joint_position differs from the measured configuration "
                "(> 0.05 rad); the real adapter can only move from the "
                "current state"
            )

        # --- endpoint --------------------------------------------------
        axes_free = (
            ("X", "Y", "Z")
            if allowed_axes is None
            else tuple(str(axis).upper() for axis in allowed_axes)
        )
        if endpoint_mode == "DISTANCE":
            if explicit_direction is None or distance is None:
                return _fail("DISTANCE mode requires explicit_direction + distance")
            direction = _as_vec3(explicit_direction)
            norm = float(np.linalg.norm(direction))
            if not np.isfinite(direction).all() or norm <= 0.0:
                return _fail("explicit_direction must be a finite non-zero vector")
            goal_position = start_position + float(distance) * direction / norm
        elif endpoint_mode == "PROJECT_TO_TARGET":
            if target_pose is None:
                return _fail("PROJECT_TO_TARGET needs target_pose")
            tp = target_pose["position"]
            goal_position = np.array(
                [float(tp["x"]), float(tp["y"]), float(tp["z"])], dtype=np.float64
            )
        elif endpoint_mode == "ORIENT_IN_PLACE":
            goal_position = start_position.copy()
        else:
            return _fail(f"unknown endpoint_mode {endpoint_mode!r}")
        if endpoint_mode != "ORIENT_IN_PLACE":
            # Axes not free to move are held at their start values. In
            # DISTANCE mode this DELIBERATELY diverges from the sim planner:
            # sim keeps the full ``distance * direction`` offset in the goal
            # (held axes are constrained along the PATH via the
            # hold_partial_pose cost, which fights any held-axis component of
            # the direction), while this adapter PINS the held axes of the
            # endpoint itself — the bridge's Cartesian move has no partial-
            # pose path cost, so pinning the endpoint is the only way to
            # honor ``allowed_axes`` here. Directed skills pass directions
            # aligned with the free axes, where both behaviors coincide.
            for index, axis in enumerate(("X", "Y", "Z")):
                if axis not in axes_free:
                    goal_position[index] = start_position[index]

        # --- goal orientation ------------------------------------------
        if orientation_mode == "LOCK":
            goal_quat = start_quat
        elif orientation_mode in ("TARGET_AT_END", "SLERP"):
            rot = orientation_target
            if rot is None and target_pose is not None:
                rot = target_pose.get("rotation")
            if rot is None:
                return _fail(
                    f"orientation_mode {orientation_mode} requires "
                    "orientation_target or target_pose"
                )
            try:
                goal_quat = np.array(
                    [
                        float(rot["w"]),
                        float(rot["x"]),
                        float(rot["y"]),
                        float(rot["z"]),
                    ],
                    dtype=np.float64,
                )
            except (KeyError, TypeError, ValueError):
                return _fail("orientation_target must be a wxyz quaternion")
            quat_norm = float(np.linalg.norm(goal_quat))
            if not np.isfinite(goal_quat).all() or quat_norm <= 0.0:
                return _fail("orientation_target must be a finite wxyz quaternion")
            goal_quat = goal_quat / quat_norm
        else:
            return _fail(f"unknown orientation_mode {orientation_mode!r}")

        # POSE CROSSING (incoming): the goal was composed in the sim tcp_gap
        # convention; convert once at the outbound edge for both the IK
        # precheck and the executed move. ROUND-TRIP CONSISTENCY: for a
        # pure-translation move (LOCK), goal_quat == bridge_quat_wxyz_to_sim
        # (captured) and the conversions cancel (Rz(pi) is self-inverse up
        # to sign, and the helpers sign-canonicalize), so the bridge is sent
        # exactly its own current orientation — no spurious 180 deg tool
        # flip on a straight descend/lift.
        goal_pose_bridge = np.concatenate(
            (goal_position, sim_quat_wxyz_to_bridge(goal_quat))
        ).astype(np.float32)

        # --- bridge IK precheck (motion-free RPC, seeded with current) --
        solution = self.env.solve_ik(goal_pose_bridge, seed_joints=current_joints)
        if solution is None:
            return _fail("ik_none")

        # --- execute: bridge-owned orientation-held linear move ---------
        translation = float(np.linalg.norm(goal_position - start_position))
        rotation_angle = 2.0 * math.acos(
            min(1.0, abs(float(np.dot(start_quat, goal_quat))))
        )
        # Bridge caps: 0.03 m/s translation, 10 deg/s rotation; double the
        # estimate plus settle margin, never below the 10 s action default.
        estimate_s = translation / 0.03 + rotation_angle / math.radians(10.0)
        timeout_s = max(10.0, 2.0 * estimate_s + 5.0)
        try:
            result = self.env.move_to_cartesian_blocking(
                goal_pose_bridge, timeout_s=timeout_s, arm_id=0
            )
        except Exception as exc:  # noqa: BLE001
            # Mid-move bridge faults (path IK_FAILED, tracking error,
            # convergence timeout, disconnects) must not raise out of the
            # planner contract. NOTE: the arm may have PARTIALLY moved along
            # the segment — the directed skills re-verify the reached pose
            # (robot.get_ee_pose / observation) before acting on the result.
            logger.warning(
                "[yam plan_directed_linear] bridge execution failed mid-move: %s",
                exc,
            )
            return {"success": False, "trajectory": None, "failure_reason": str(exc)}
        status = result.get("status")
        if status != "completed":
            # The bridge reported a non-terminal-success outcome (e.g.
            # "cancelled" — lease lost or operator stop). Non-throwing, like
            # every other infeasible/failed plan result.
            return _fail(f"bridge_{status}")
        final_joints = [float(v) for v in result["final_joint_pos_0"][:6]]
        logger.info(
            "[yam plan_directed_linear] executed %s move: %.3f m / %.1f deg "
            "(status=%s)",
            endpoint_mode, translation, math.degrees(rotation_angle),
            result.get("status"),
        )
        return {
            "success": True,
            # One waypoint == the reached joints: the skills' follow-up
            # robot.execute_trajectory becomes a no-op hold.
            "trajectory": {"waypoints": [{"positions": final_joints}]},
            "failure_reason": "",
            "note": (
                "executed on hardware by the real adapter (bridge "
                "orientation-held minimum-jerk linear move); the returned "
                "single-waypoint trajectory equals the reached joints and "
                "re-executing it is a no-op hold"
            ),
            "action_status": result.get("status"),
        }


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------


def real(
    robot: str = "franka",
    *,
    cameras: list[str] | None = None,
    rr_config: str | Path | None = None,
    rr_autostart: bool = True,
    rr_log_path: str | Path | None = None,
    port: int | None = None,
    wait_timeout_s: float = 60.0,
    **env_kwargs: Any,
) -> RealConnector:
    """Build a :class:`RealConnector` for real hardware.

    Args:
        robot: ``"franka"`` (robots_realtime msgpack bridge, full motion),
            ``"ur_zed"`` (UR + ZED, perception-only), ``"yam_left"`` (LEFT
            YAM arm behind the safety-owned hardware bridge, spec
            ``yam_real_left``) or ``"yam_right"`` (RIGHT YAM arm behind its
            own bridge, spec ``yam_real_right``).
        cameras: Camera-name override (default: the env's
            ``EnvConfig.default_cameras``).
        rr_config: Franka only — rr-session yaml, resolved relative to the
            ``third_party/robots_realtime`` checkout. Defaults to
            :data:`DEFAULT_FRANKA_RR_CONFIG`.
        rr_autostart: Franka only — spawn the rr-session client
            automatically. Pass False to drive it from a second terminal
            (debug flow); the connector then still blocks in
            ``wait_ready`` until your client connects.
        rr_log_path: Franka only — rr-session log tee destination.
        port: Franka msgpack-server port (default 9000), or the YAM bridge
            port (default 9021 for ``yam_left``, 9022 for ``yam_right``).
        wait_timeout_s: Seconds ``wait_ready`` blocks for the robot-specific
            observation contract (RGB for Franka; healthy bridge state for
            YAM) before raising.
        **env_kwargs: Extra env-factory kwargs (e.g. ``host=`` for franka /
            yam_left / yam_right; ``robot_ip=``, ``calibration_path=`` for
            ur_zed). ``robot_spec`` is NOT accepted for the YAM robots — it
            is implied by ``robot``.

    Order of operations for franka (matters): the env constructor binds
    the msgpack server and pre-seeds a hold-home action *before* the
    rr-session client is spawned — the client retries until the server is
    up, and its very first action request must see a valid hold command
    (an empty reply makes it fall back to its Viser IK gizmo and jolt the
    arm).
    """
    from gap.envs.registry import resolve

    if robot == "franka":
        factory, key = resolve("franka_real")
        if port is not None:
            env_kwargs["port"] = port
        env, config = factory(key, 0, camera_names=cameras, **env_kwargs)

        rr_session = None
        if rr_autostart:
            from gap.connector.rr_launcher import RRSession

            rr_session = RRSession(
                rr_config or DEFAULT_FRANKA_RR_CONFIG, log_path=rr_log_path,
            )
        conn = RealConnector(
            env, config, camera_names=cameras, rr_session=rr_session,
        )
        try:
            conn.wait_ready(timeout_s=wait_timeout_s)
        except BaseException:
            conn.close()
            raise
        return conn

    if robot == "ur_zed":
        if rr_config is not None or port is not None:
            raise ValueError(
                "rr_config/port only apply to robot='franka' — ur_zed has no "
                "rr-session (direct pyzed + RTDE capture)"
            )
        factory, key = resolve("ur_zed")
        env, config = factory(key, 0, camera_names=cameras, **env_kwargs)
        # Perception-only: no motion tools (see RealConnector docstring).
        conn = RealConnector(
            env, config, camera_names=cameras, motion_enabled=False,
        )
        try:
            conn.wait_ready(timeout_s=wait_timeout_s)
        except BaseException:
            conn.close()
            raise
        return conn

    if robot in ("yam_left", "yam_right"):
        if rr_config is not None:
            raise ValueError("rr_config only applies to robot='franka'")
        if "robot_spec" in env_kwargs:
            raise ValueError(
                "robot_spec is implied by robot= ('yam_left' -> yam_real_left, "
                "'yam_right' -> yam_real_right); do not pass it"
            )
        factory, key = resolve("yam_real")
        if robot == "yam_right":
            # The left path stays byte-identical: it relies on make_env's own
            # defaults (port 9021, spec yam_real_left) exactly as before.
            env_kwargs["robot_spec"] = "yam_real_right"
            if port is None:
                port = 9022
        if port is not None:
            env_kwargs["port"] = port
        env, config = factory(key, 0, camera_names=cameras, **env_kwargs)
        conn = YamRealConnector(
            env,
            config,
            camera_names=cameras,
            ik=RemoteI2rtKinematicsBackend(env),
        )
        try:
            conn.wait_ready(timeout_s=wait_timeout_s)
        except BaseException:
            conn.close()
            raise
        return conn

    raise ValueError(
        f"unknown real robot {robot!r}; expected 'franka', 'ur_zed', "
        "'yam_left', or 'yam_right'"
    )


__all__ = [
    "DEFAULT_FRANKA_RR_CONFIG",
    "RealConnector",
    "RemoteI2rtKinematicsBackend",
    "YamRealConnector",
    "bridge_quat_wxyz_to_sim",
    "densify_waypoint_jumps",
    "real",
    "sim_quat_wxyz_to_bridge",
    "synthesize_min_jerk_waypoint_times",
]
