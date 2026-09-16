"""Safety-owned motion execution for the YAM hardware bridge."""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from agp_yam_bridge.config import BridgeConfig
from agp_yam_bridge.kinematics import I2rtKinematicsBackend, TrajectoryValidationError


@dataclass(frozen=True)
class MotionState:
    position: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray
    monotonic_ns: int
    sequence: int
    motor_errors: tuple[str, ...]


class MotionFault(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


class MotionController:
    def __init__(
        self,
        actuator: Any,
        kinematics: I2rtKinematicsBackend,
        config: BridgeConfig,
        *,
        action_log_path: Path | None = None,
        heartbeat_timeout_s: float | None = None,
        feedback_stale_after_s: float | None = None,
        max_tracking_error_rad: float | None = None,
        tracking_error_timeout_s: float | None = None,
        gripper_max_effort_nm: float | None = None,
        gripper_contact_min_effort_nm: float | None = None,
        gripper_contact_release_effort_nm: float | None = None,
        gripper_contact_squeeze_fraction: float | None = None,
        gripper_contact_confirmation_s: float | None = None,
        gripper_contact_max_velocity_fraction_s: float | None = None,
        gripper_contact_min_open_fraction: float | None = None,
        gripper_stall_velocity_fraction_s: float | None = None,
        gripper_stall_error_fraction: float | None = None,
        gripper_stall_timeout_s: float | None = None,
    ) -> None:
        safety = config.safety
        heartbeat_timeout_s = (
            safety.command_heartbeat_timeout_s
            if heartbeat_timeout_s is None
            else heartbeat_timeout_s
        )
        feedback_stale_after_s = (
            safety.feedback_stale_after_s
            if feedback_stale_after_s is None
            else feedback_stale_after_s
        )
        if max_tracking_error_rad is None and safety.max_tracking_error_deg is not None:
            max_tracking_error_rad = float(
                np.radians(safety.max_tracking_error_deg)
            )
        tracking_error_timeout_s = (
            safety.tracking_error_timeout_s
            if tracking_error_timeout_s is None
            else tracking_error_timeout_s
        )
        gripper_max_effort_nm = (
            safety.gripper_max_effort_nm
            if gripper_max_effort_nm is None
            else gripper_max_effort_nm
        )
        gripper_contact_min_effort_nm = (
            safety.gripper_contact_min_effort_nm
            if gripper_contact_min_effort_nm is None
            else gripper_contact_min_effort_nm
        )
        gripper_contact_release_effort_nm = (
            safety.gripper_contact_release_effort_nm
            if gripper_contact_release_effort_nm is None
            else gripper_contact_release_effort_nm
        )
        gripper_contact_squeeze_fraction = (
            getattr(safety, "gripper_contact_squeeze_fraction", 0.0)
            if gripper_contact_squeeze_fraction is None
            else gripper_contact_squeeze_fraction
        )
        gripper_contact_confirmation_s = (
            safety.gripper_contact_confirmation_s
            if gripper_contact_confirmation_s is None
            else gripper_contact_confirmation_s
        )
        gripper_contact_max_velocity_fraction_s = (
            safety.gripper_contact_max_velocity_fraction_s
            if gripper_contact_max_velocity_fraction_s is None
            else gripper_contact_max_velocity_fraction_s
        )
        gripper_contact_min_open_fraction = (
            safety.gripper_contact_min_open_fraction
            if gripper_contact_min_open_fraction is None
            else gripper_contact_min_open_fraction
        )
        gripper_stall_velocity_fraction_s = (
            safety.gripper_stall_velocity_fraction_s
            if gripper_stall_velocity_fraction_s is None
            else gripper_stall_velocity_fraction_s
        )
        gripper_stall_error_fraction = (
            safety.gripper_stall_error_fraction
            if gripper_stall_error_fraction is None
            else gripper_stall_error_fraction
        )
        gripper_stall_timeout_s = (
            safety.gripper_stall_timeout_s
            if gripper_stall_timeout_s is None
            else gripper_stall_timeout_s
        )
        self._actuator = actuator
        self._kinematics = kinematics
        self._control_period_s = 1.0 / config.acceptance.speed_limits.control_frequency_hz
        self._gripper_rate_fraction_s = config.acceptance.speed_limits.gripper_fraction_s
        self._max_joint_velocity_rad_s = np.radians(
            config.acceptance.speed_limits.joint_velocity_deg_s
        )
        self._max_joint_acceleration_rad_s2 = np.radians(
            config.acceptance.speed_limits.joint_acceleration_deg_s2
        )
        self._max_cartesian_translation_m_s = (
            config.acceptance.speed_limits.cartesian_translation_m_s
        )
        self._max_cartesian_rotation_rad_s = np.radians(
            config.acceptance.speed_limits.cartesian_rotation_deg_s
        )
        self._action_log_path = (
            safety.action_log_path if action_log_path is None else action_log_path
        )
        self._action_log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_lock = threading.Lock()
        if heartbeat_timeout_s <= 0:
            raise ValueError("heartbeat_timeout_s must be positive")
        if min(
            feedback_stale_after_s,
            tracking_error_timeout_s,
            gripper_max_effort_nm,
            gripper_contact_min_effort_nm,
            gripper_contact_release_effort_nm,
            gripper_contact_confirmation_s,
            gripper_contact_max_velocity_fraction_s,
            gripper_contact_min_open_fraction,
            gripper_stall_velocity_fraction_s,
            gripper_stall_error_fraction,
            gripper_stall_timeout_s,
        ) <= 0 or (
            max_tracking_error_rad is not None and max_tracking_error_rad <= 0
        ):
            raise ValueError("motion safety thresholds must be positive")
        if gripper_contact_min_effort_nm >= gripper_max_effort_nm:
            raise ValueError("gripper contact effort must be below the overcurrent limit")
        if gripper_contact_release_effort_nm >= gripper_contact_min_effort_nm:
            raise ValueError(
                "gripper contact release effort must be below its acquisition effort"
            )
        if gripper_contact_min_open_fraction <= gripper_stall_error_fraction:
            raise ValueError(
                "gripper contact open fraction must exceed the stall error fraction"
            )
        self._heartbeat_timeout_s = heartbeat_timeout_s
        self._feedback_stale_after_s = feedback_stale_after_s
        self._max_tracking_error_rad = max_tracking_error_rad
        self._tracking_error_timeout_s = tracking_error_timeout_s
        self._joint_settle_tolerance_rad = safety.joint_settle_tolerance_rad
        self._cartesian_position_tolerance_m = safety.cartesian_position_tolerance_m
        self._cartesian_orientation_tolerance_rad = np.radians(
            safety.cartesian_orientation_tolerance_deg
        )
        self._cartesian_settle_gain_s_inv = safety.cartesian_settle_gain_s_inv
        self._cartesian_settle_max_bias_rad = np.radians(
            safety.cartesian_settle_max_bias_deg
        )
        self._cartesian_settle_miss = getattr(safety, "cartesian_settle_miss", "idle")
        self._gripper_max_effort_nm = gripper_max_effort_nm
        self._gripper_contact_min_effort_nm = gripper_contact_min_effort_nm
        self._gripper_contact_release_effort_nm = gripper_contact_release_effort_nm
        self._gripper_contact_confirmation_s = gripper_contact_confirmation_s
        self._gripper_contact_max_velocity_fraction_s = (
            gripper_contact_max_velocity_fraction_s
        )
        self._gripper_contact_min_open_fraction = gripper_contact_min_open_fraction
        self._gripper_contact_squeeze_fraction = float(gripper_contact_squeeze_fraction)
        # Last gripper command that established a contact grasp (None when open / no
        # contact). Arm motions keep commanding THIS value instead of the measured
        # gripper position, otherwise the squeeze force is released on the first move.
        self._gripper_hold_target: float | None = None
        self._gripper_stall_velocity_fraction_s = gripper_stall_velocity_fraction_s
        self._gripper_stall_error_fraction = gripper_stall_error_fraction
        self._gripper_stall_timeout_s = gripper_stall_timeout_s
        self._state_lock = threading.Lock()
        self._cancel_event = threading.Event()
        self._stop_event = threading.Event()
        self._lease_request_id: int | None = None
        self._last_heartbeat = 0.0
        self._active = False
        self._holding = False
        self._cancel_reason = ""
        self._fault_detail = ""
        # Per-thread flag: False from execute() start until _begin() takes the lease,
        # i.e. until the action has commanded anything (see execute()'s fault handling).
        self._exec_phase = threading.local()
        self._last_state: MotionState | None = None
        self._closed = False
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="yam_command_watchdog",
        )
        self._watchdog_thread.start()

    def _watchdog_loop(self) -> None:
        check_period = min(0.02, self._heartbeat_timeout_s / 4.0)
        while not self._stop_event.wait(check_period):
            with self._state_lock:
                leased = self._active or self._holding
                holding = self._holding
                lease_request_id = self._lease_request_id
                expired = (
                    leased
                    and time.monotonic() - self._last_heartbeat
                    > self._heartbeat_timeout_s
                )
                if expired:
                    self._cancel_reason = "command heartbeat timed out"
                    self._fault_detail = self._cancel_reason
                    self._cancel_event.set()
                    self._active = False
                    self._holding = False
                    self._lease_request_id = None
            if expired:
                try:
                    self._actuator.enter_safe_idle()
                except Exception as exc:
                    with self._state_lock:
                        self._fault_detail += f"; safe idle failed: {exc}"
                continue
            if holding and lease_request_id is not None:
                try:
                    self._read_feedback()
                except MotionFault as exc:
                    with self._state_lock:
                        if (
                            self._holding
                            and self._lease_request_id == lease_request_id
                        ):
                            self._fault_detail = f"{exc.code}: {exc}"
                            self._cancel_reason = str(exc)
                            self._cancel_event.set()
                            self._holding = False
                            self._lease_request_id = None
                        else:
                            continue
                    try:
                        self._actuator.enter_safe_idle()
                    except Exception as idle_exc:
                        with self._state_lock:
                            self._fault_detail += f"; safe idle failed: {idle_exc}"
                    self._log(
                        "result",
                        lease_request_id,
                        kind="hold_monitor",
                        status="fault",
                        code=exc.code,
                        detail=str(exc),
                    )

    def heartbeat(self, request_id: int) -> None:
        with self._state_lock:
            if request_id != self._lease_request_id:
                raise MotionFault(
                    "ACTION_MISMATCH",
                    "heartbeat does not own the active motion lease",
                )
            self._last_heartbeat = time.monotonic()

    def validate_action_timestamp(self, monotonic_ns: int) -> None:
        age_s = (time.monotonic_ns() - monotonic_ns) / 1e9
        if age_s < 0:
            raise MotionFault("INVALID_TIMESTAMP", "action timestamp is in the future")
        if age_s > self._heartbeat_timeout_s:
            raise MotionFault(
                "STALE_ACTION",
                f"action age {age_s:.3f}s exceeds {self._heartbeat_timeout_s:.3f}s",
            )

    def cancel(self, request_id: int) -> dict[str, Any]:
        with self._state_lock:
            if request_id != self._lease_request_id:
                raise MotionFault(
                    "ACTION_MISMATCH",
                    "cancel request does not own the active motion lease",
                )
            self._cancel_reason = "cancelled by request"
            self._cancel_event.set()
            self._active = False
            self._holding = False
            self._lease_request_id = None
        self._actuator.enter_safe_idle()
        self._log("cancel", request_id, status="cancelled")
        return {
            "request_id": request_id,
            "status": "cancelled",
            "detail": "cancelled by request",
        }

    def _gripper_command_for_arm_motion(self, measured: float) -> float:
        """Gripper element for arm-motion commands: keep the squeezed contact command
        established by the last close (so the grip force survives transport); fall
        back to the measured position when nothing is held."""
        held = self._gripper_hold_target
        if held is not None and abs(float(measured) - held) < 0.1:
            return held
        return float(measured)

    def _begin(self, request_id: int) -> None:
        with self._state_lock:
            if self._active:
                raise MotionFault(
                    "ACTION_BUSY", "another motion action is already active"
                )
            self._lease_request_id = request_id
            self._last_heartbeat = time.monotonic()
            self._active = True
            self._holding = False
            self._cancel_reason = ""
            self._fault_detail = ""
            self._cancel_event.clear()
        self._exec_phase.begun = True

    def health(self) -> dict[str, Any]:
        with self._state_lock:
            if self._active:
                safety_state = "moving"
            elif self._holding:
                safety_state = "holding"
            elif self._fault_detail:
                safety_state = "fault"
            else:
                safety_state = "idle"
            return {
                "state": "error" if safety_state == "fault" else "ok",
                "source_connected": not self._closed,
                "motion_enabled": not self._closed,
                "safety_state": safety_state,
                "active_action_request_id": self._lease_request_id,
                "detail": self._fault_detail,
            }

    def _finish(self, *, hold: bool) -> None:
        with self._state_lock:
            self._active = False
            self._holding = hold

    def _cancelled_result(
        self,
        *,
        request_id: int,
        kind: str,
        started_ns: int,
        state: MotionState,
        max_tracking_error_rad: float,
    ) -> dict[str, Any]:
        with self._state_lock:
            detail = self._cancel_reason or "motion cancelled"
        result = {
            "request_id": request_id,
            "kind": kind,
            "status": "cancelled",
            "started_monotonic_ns": started_ns,
            "finished_monotonic_ns": time.monotonic_ns(),
            "final_joint_pos_0": state.position.astype(np.float32),
            "max_tracking_error_rad": max_tracking_error_rad,
            "detail": detail,
        }
        self._finish(hold=False)
        self._log(
            "result",
            request_id,
            **{key: value for key, value in result.items() if key != "request_id"},
        )
        return result

    @staticmethod
    def _json_value(value: Any) -> Any:
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, dict):
            return {key: MotionController._json_value(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [MotionController._json_value(item) for item in value]
        return value

    def _log(self, event: str, request_id: int, **payload: Any) -> None:
        record = {
            "event": event,
            "request_id": request_id,
            "monotonic_ns": time.monotonic_ns(),
            "wall_time_ns": time.time_ns(),
            **payload,
        }
        encoded = json.dumps(self._json_value(record), separators=(",", ":"), sort_keys=True)
        with self._log_lock, self._action_log_path.open("a", encoding="utf-8") as stream:
            stream.write(encoded + "\n")
            stream.flush()

    def _check_feedback(self, state: MotionState) -> None:
        vectors = {
            "position": state.position,
            "velocity": state.velocity,
            "effort": state.effort,
        }
        for name, value in vectors.items():
            array = np.asarray(value)
            if array.shape != (7,) or not np.isfinite(array).all():
                raise MotionFault(
                    "INVALID_FEEDBACK",
                    f"motion feedback {name} must be a finite seven-vector",
                )
        age_s = (time.monotonic_ns() - state.monotonic_ns) / 1e9
        if age_s < 0:
            raise MotionFault("INVALID_FEEDBACK", "motion feedback timestamp is in the future")
        if age_s > self._feedback_stale_after_s:
            raise MotionFault(
                "STALE_FEEDBACK",
                f"motion feedback age {age_s:.3f}s exceeds "
                f"{self._feedback_stale_after_s:.3f}s",
            )
        if state.motor_errors:
            raise MotionFault(
                "MOTOR_ERROR",
                "motor feedback reported errors: " + "; ".join(state.motor_errors),
            )
        self._last_state = state

    def _read_feedback(self) -> MotionState:
        try:
            state = self._actuator.read_motion_state()
        except Exception as exc:
            raise MotionFault(
                "CONTROL_SOURCE",
                f"could not read live motor feedback: {exc}",
            ) from exc
        self._check_feedback(state)
        return state

    def _command(self, target: np.ndarray) -> None:
        try:
            self._actuator.command_joint_positions(target)
        except Exception as exc:
            raise MotionFault(
                "CONTROL_SOURCE",
                f"could not dispatch joint command: {exc}",
            ) from exc

    def _execute_joint_trajectory(
        self,
        *,
        request_id: int,
        action: dict[str, Any],
        result_kind: str = "joint_trajectory",
        initial_state: MotionState | None = None,
        cartesian_target_pose: np.ndarray | None = None,
    ) -> dict[str, Any]:
        started_ns = time.monotonic_ns()
        if initial_state is None:
            initial = self._read_feedback()
        else:
            # The planner (per-step IK for a Cartesian move) can run for >100 ms. Re-checking
            # the age of the snapshot it planned from measured our own planning time and
            # raised spurious STALE_FEEDBACK faults (15 on 2026-09-03, each idling the chain
            # and dropping whatever was held). A fresh, validated read proves the feedback
            # stream is alive; the arm must simply not have moved since the snapshot the
            # trajectory starts from.
            fresh = self._read_feedback()
            drift = float(np.max(np.abs(fresh.position[:6] - initial_state.position[:6])))
            if drift > self._joint_settle_tolerance_rad:
                raise MotionFault(
                    "INVALID_FEEDBACK",
                    f"arm moved {drift:.4f} rad while the trajectory was being planned; "
                    "re-plan from the current state",
                )
            initial = initial_state
        waypoints = np.asarray(action["waypoints"], dtype=np.float64)
        if "waypoint_times_s" in action:
            times = np.asarray(action["waypoint_times_s"], dtype=np.float64)
        else:
            times = np.arange(len(waypoints), dtype=np.float64) / float(action["control_hz"])
        self._kinematics.validate_trajectory(
            waypoints,
            times,
            initial_joints=initial.position[:6],
        )
        self._begin(request_id)
        gripper = self._gripper_command_for_arm_motion(float(initial.position[6]))
        deadline = time.monotonic() + float(action["timeout_s"])
        last = initial
        segment_start = initial.position[:6].astype(np.float64, copy=True)
        segment_time = 0.0
        started = time.monotonic()
        tracking_error_since: float | None = None
        max_tracking_error = 0.0
        for target, target_time in zip(waypoints, times, strict=True):
            duration = float(target_time - segment_time)
            steps = max(1, int(np.ceil(duration / self._control_period_s)))
            for step in range(1, steps + 1):
                if self._cancel_event.is_set():
                    return self._cancelled_result(
                        request_id=request_id,
                        kind=result_kind,
                        started_ns=started_ns,
                        state=last,
                        max_tracking_error_rad=float(
                            np.max(np.abs(last.position[:6] - target))
                        ),
                    )
                if time.monotonic() > deadline:
                    raise MotionFault(
                        "ACTION_TIMEOUT", "joint trajectory exceeded its hard timeout"
                    )
                fraction = step / steps
                arm_target = segment_start + fraction * (target - segment_start)
                command = np.append(arm_target, gripper)
                self._command(command)
                wake_at = started + segment_time + duration * fraction
                remaining = wake_at - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                last = self._read_feedback()
                tracking_error = float(
                    np.max(np.abs(last.position[:6] - arm_target))
                )
                max_tracking_error = max(max_tracking_error, tracking_error)
                if (
                    self._max_tracking_error_rad is not None
                    and tracking_error > self._max_tracking_error_rad
                ):
                    if tracking_error_since is None:
                        tracking_error_since = time.monotonic()
                    elif (
                        time.monotonic() - tracking_error_since
                        > self._tracking_error_timeout_s
                    ):
                        raise MotionFault(
                            "TRACKING_ERROR",
                            f"joint tracking error {tracking_error:.6f} rad exceeded "
                            f"{self._max_tracking_error_rad:.6f} rad for "
                            f"{self._tracking_error_timeout_s:.3f}s",
                        )
                else:
                    tracking_error_since = None
                self._log(
                    "feedback",
                    request_id,
                    command=command,
                    feedback=last.position,
                )
                if self._cancel_event.is_set():
                    return self._cancelled_result(
                        request_id=request_id,
                        kind=result_kind,
                        started_ns=started_ns,
                        state=last,
                        max_tracking_error_rad=float(
                            np.max(np.abs(last.position[:6] - target))
                        ),
                    )
            segment_start = target.copy()
            segment_time = float(target_time)

        final_arm_target = waypoints[-1]
        final_command = np.append(final_arm_target, gripper)
        previous_arm_command = final_arm_target.copy()
        settle_bias = np.zeros(6, dtype=np.float64)
        settle_bias_velocity = np.zeros(6, dtype=np.float64)
        joint_lower, joint_upper = self._kinematics.joint_limits
        settle_bias_lower = np.maximum(
            -self._cartesian_settle_max_bias_rad,
            joint_lower - final_arm_target,
        )
        settle_bias_upper = np.minimum(
            self._cartesian_settle_max_bias_rad,
            joint_upper - final_arm_target,
        )
        tolerance = float(action["position_tolerance_rad"])
        joint_error = float(np.max(np.abs(last.position[:6] - final_arm_target)))
        position_error_m = 0.0
        orientation_error_rad = 0.0

        def has_converged() -> bool:
            nonlocal position_error_m, orientation_error_rad
            if cartesian_target_pose is None:
                return joint_error <= tolerance
            actual_pose = self._kinematics.forward(last.position[:6])
            position_error_m = float(
                np.linalg.norm(actual_pose[:3] - cartesian_target_pose[:3])
            )
            actual_rotation = Rotation.from_quat(actual_pose[[4, 5, 6, 3]])
            target_rotation = Rotation.from_quat(cartesian_target_pose[[4, 5, 6, 3]])
            orientation_error_rad = float(
                (actual_rotation.inv() * target_rotation).magnitude()
            )
            return (
                position_error_m <= self._cartesian_position_tolerance_m
                and orientation_error_rad <= self._cartesian_orientation_tolerance_rad
            )

        consecutive_converged_samples = int(has_converged())
        converged = consecutive_converged_samples >= 2
        detail = ""
        while not converged:
            if self._cancel_event.is_set():
                return self._cancelled_result(
                    request_id=request_id,
                    kind=result_kind,
                    started_ns=started_ns,
                    state=last,
                    max_tracking_error_rad=max_tracking_error,
                )
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if cartesian_target_pose is not None and self._cartesian_settle_miss == "hold":
                    # 2026-09-03: the trajectory is complete and the arm is stationary at
                    # its last command; report the miss instead of faulting. Faulting used
                    # to gravity-comp the whole chain (gripper included) over a few-mm
                    # settle miss and dropped whatever was held.
                    detail = (
                        "SETTLE_MISS: Cartesian pose did not converge within its hard "
                        f"timeout; position error {position_error_m:.6f} m exceeds "
                        f"{self._cartesian_position_tolerance_m:.6f} m or orientation "
                        f"error {orientation_error_rad:.6f} rad exceeds "
                        f"{self._cartesian_orientation_tolerance_rad:.6f} rad"
                    )
                    break
                if cartesian_target_pose is not None:
                    raise MotionFault(
                        "ACTION_TIMEOUT",
                        "Cartesian pose did not converge within its hard timeout; "
                        f"position error {position_error_m:.6f} m exceeds "
                        f"{self._cartesian_position_tolerance_m:.6f} m or orientation "
                        f"error {orientation_error_rad:.6f} rad exceeds "
                        f"{self._cartesian_orientation_tolerance_rad:.6f} rad",
                    )
                detail = (
                    "joint settle tolerance not reached before timeout; "
                    f"final error {joint_error:.6f} rad exceeds tolerance "
                    f"{tolerance:.6f} rad"
                )
                break
            desired_bias_velocity = np.clip(
                self._cartesian_settle_gain_s_inv
                * (final_arm_target - last.position[:6]),
                -self._max_joint_velocity_rad_s,
                self._max_joint_velocity_rad_s,
            )
            max_velocity_change = (
                self._max_joint_acceleration_rad_s2 * self._control_period_s
            )
            settle_bias_velocity += np.clip(
                desired_bias_velocity - settle_bias_velocity,
                -max_velocity_change,
                max_velocity_change,
            )
            previous_bias = settle_bias.copy()
            proposed_bias = settle_bias + settle_bias_velocity * self._control_period_s
            bounded_bias = np.clip(
                proposed_bias,
                settle_bias_lower,
                settle_bias_upper,
            )
            proposed_arm_command = final_arm_target + bounded_bias
            arm_command: np.ndarray | None = None
            validation_error: TrajectoryValidationError | None = None
            for backtrack in range(13):
                scale = 0.0 if backtrack == 12 else 0.5**backtrack
                candidate = previous_arm_command + scale * (
                    proposed_arm_command - previous_arm_command
                )
                try:
                    self._kinematics.validate_trajectory(
                        np.vstack((previous_arm_command, candidate)),
                        np.array([0.0, self._control_period_s]),
                        initial_joints=previous_arm_command,
                    )
                except TrajectoryValidationError as exc:
                    validation_error = exc
                    continue
                arm_command = candidate
                break
            if arm_command is None:
                assert validation_error is not None
                raise MotionFault(
                    "SETTLE_LIMIT",
                    "final-target settle correction remained unsafe after "
                    f"backtracking: {validation_error}",
                ) from validation_error
            settle_bias = arm_command - final_arm_target
            settle_bias_velocity = (
                settle_bias - previous_bias
            ) / self._control_period_s
            hit_limit = proposed_bias != bounded_bias
            settle_bias_velocity[hit_limit] = 0.0
            final_command = np.append(arm_command, gripper)
            previous_arm_command = arm_command
            self._command(final_command)
            time.sleep(min(self._control_period_s, remaining))
            last = self._read_feedback()
            joint_error = float(np.max(np.abs(last.position[:6] - final_arm_target)))
            tracking_error = float(
                np.max(np.abs(last.position[:6] - final_command[:6]))
            )
            max_tracking_error = max(max_tracking_error, tracking_error)
            if (
                self._max_tracking_error_rad is not None
                and tracking_error > self._max_tracking_error_rad
            ):
                if tracking_error_since is None:
                    tracking_error_since = time.monotonic()
                elif (
                    time.monotonic() - tracking_error_since
                    > self._tracking_error_timeout_s
                ):
                        raise MotionFault(
                            "TRACKING_ERROR",
                            f"joint tracking error {tracking_error:.6f} rad exceeded "
                            f"{self._max_tracking_error_rad:.6f} rad for "
                        f"{self._tracking_error_timeout_s:.3f}s",
                    )
            else:
                tracking_error_since = None
            self._log(
                "feedback",
                request_id,
                command=final_command,
                feedback=last.position,
            )
            if has_converged():
                consecutive_converged_samples += 1
            else:
                consecutive_converged_samples = 0
            converged = consecutive_converged_samples >= 2
        result = {
            "request_id": request_id,
            "kind": result_kind,
            "status": "completed",
            "started_monotonic_ns": started_ns,
            "finished_monotonic_ns": time.monotonic_ns(),
            "final_joint_pos_0": last.position.astype(np.float32),
            "max_tracking_error_rad": max_tracking_error,
            "detail": detail,
        }
        self._finish(hold=True)
        self._log(
            "result",
            request_id,
            **{key: value for key, value in result.items() if key != "request_id"},
        )
        return result

    def _execute_absolute_joints(
        self, *, request_id: int, action: dict[str, Any]
    ) -> dict[str, Any]:
        initial = self._read_feedback()
        start = initial.position[:6].astype(np.float64, copy=True)
        target = np.asarray(action["joint_target"], dtype=np.float64)
        largest_delta = float(np.max(np.abs(target - start)))
        velocity_duration = 1.875 * largest_delta / self._max_joint_velocity_rad_s
        acceleration_duration = np.sqrt(
            5.7736 * largest_delta / self._max_joint_acceleration_rad_s2
        )
        duration = max(self._control_period_s, velocity_duration, acceleration_duration)
        duration *= 1.05
        steps = max(2, int(np.ceil(duration / self._control_period_s)))
        times = np.linspace(0.0, steps * self._control_period_s, steps + 1)
        unit_time = times / times[-1]
        blend = 10.0 * unit_time**3 - 15.0 * unit_time**4 + 6.0 * unit_time**5
        waypoints = start[None, :] + blend[:, None] * (target - start)[None, :]
        trajectory_action = {
            "kind": "joint_trajectory",
            "waypoints": waypoints.astype(np.float32),
            "waypoint_times_s": times.astype(np.float32),
            "timeout_s": action["timeout_s"],
            "position_tolerance_rad": action["position_tolerance_rad"],
        }
        return self._execute_joint_trajectory(
            request_id=request_id,
            action=trajectory_action,
            result_kind="absolute_joints",
            initial_state=initial,
        )

    def _execute_gripper(
        self, *, request_id: int, action: dict[str, Any]
    ) -> dict[str, Any]:
        started_ns = time.monotonic_ns()
        initial = self._read_feedback()
        arm_target = initial.position[:6].astype(np.float64, copy=True)
        self._kinematics.validate_trajectory(
            np.vstack((arm_target, arm_target)),
            np.array([0.0, self._control_period_s]),
            initial_joints=arm_target,
        )
        target = float(action["open_fraction"])
        stop_on_contact = bool(action["stop_on_contact"])
        self._gripper_hold_target = None          # any new gripper action ends the previous hold
        delta = target - float(initial.position[6])
        steps = max(
            1,
            int(
                np.ceil(
                    abs(delta)
                    / (self._gripper_rate_fraction_s * self._control_period_s)
                )
            ),
        )
        duration = abs(delta) / self._gripper_rate_fraction_s
        deadline = time.monotonic() + float(action["timeout_s"])
        started = time.monotonic()
        self._begin(request_id)
        last = initial
        stall_started: float | None = None
        contacted = False
        contact_probe_armed = True
        contact_probe_cooldown_steps = 0
        for step in range(1, steps + 1):
            if self._cancel_event.is_set():
                return self._cancelled_result(
                    request_id=request_id,
                    kind="gripper",
                    started_ns=started_ns,
                    state=last,
                    max_tracking_error_rad=0.0,
                )
            if time.monotonic() > deadline:
                raise MotionFault("ACTION_TIMEOUT", "gripper action exceeded its hard timeout")
            fraction = step / steps
            gripper_target = float(initial.position[6]) + fraction * delta
            command = np.append(arm_target, gripper_target)
            self._command(command)
            wake_at = started + duration * fraction
            remaining = wake_at - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            last = self._read_feedback()
            if abs(float(last.effort[6])) > self._gripper_max_effort_nm:
                raise MotionFault(
                    "GRIPPER_OVERCURRENT",
                    f"gripper effort {float(last.effort[6]):.6f} Nm exceeds "
                    f"{self._gripper_max_effort_nm:.6f} Nm",
                )
            remaining_error = abs(float(last.position[6]) - target)
            stalled = (
                remaining_error > self._gripper_stall_error_fraction
                and abs(float(last.velocity[6]))
                < self._gripper_stall_velocity_fraction_s
            )
            contact_effort = float(last.effort[6])
            contact_probe_allowed = contact_probe_cooldown_steps == 0
            if contact_probe_cooldown_steps > 0:
                contact_probe_cooldown_steps -= 1
            if (
                stop_on_contact
                and delta < 0.0
                and contact_probe_armed
                and contact_probe_allowed
                and remaining_error > self._gripper_stall_error_fraction
                and float(last.position[6])
                > self._gripper_contact_min_open_fraction
                and contact_effort >= self._gripper_contact_min_effort_nm
            ):
                contact_probe_armed = False
                confirmation_started = time.monotonic()
                hold_command = command.copy()
                contacted = True
                observed_contact_slowdown = (
                    abs(float(last.velocity[6]))
                    < self._gripper_contact_max_velocity_fraction_s
                )
                while (
                    time.monotonic() - confirmation_started
                    < self._gripper_contact_confirmation_s
                ):
                    if self._cancel_event.is_set():
                        return self._cancelled_result(
                            request_id=request_id,
                            kind="gripper",
                            started_ns=started_ns,
                            state=last,
                            max_tracking_error_rad=0.0,
                        )
                    if time.monotonic() > deadline:
                        raise MotionFault(
                            "ACTION_TIMEOUT", "gripper action exceeded its hard timeout"
                        )
                    self._command(hold_command)
                    time.sleep(self._control_period_s)
                    last = self._read_feedback()
                    if abs(float(last.effort[6])) > self._gripper_max_effort_nm:
                        raise MotionFault(
                            "GRIPPER_OVERCURRENT",
                            f"gripper effort {float(last.effort[6]):.6f} Nm exceeds "
                            f"{self._gripper_max_effort_nm:.6f} Nm",
                        )
                    self._log(
                        "feedback",
                        request_id,
                        command=hold_command,
                        feedback=last.position,
                        effort=last.effort,
                    )
                    if (
                        float(last.position[6])
                        <= self._gripper_contact_min_open_fraction
                        or float(last.effort[6])
                        < self._gripper_contact_release_effort_nm
                    ):
                        contacted = False
                        break
                    observed_contact_slowdown = observed_contact_slowdown or (
                        abs(float(last.velocity[6]))
                        < self._gripper_contact_max_velocity_fraction_s
                    )
                contacted = contacted and observed_contact_slowdown
                started += time.monotonic() - confirmation_started
                stall_started = None
                if contacted:
                    # SQUEEZE (2026-09-03): stopping exactly at the contact position holds the
                    # object with only the ~contact effort, which let taped 5 cm cubes slip
                    # under lift/yaw. Close a further configurable fraction past the contact
                    # position (position-controlled, so the grip force comes from the
                    # controller stiffness), still bounded by the overcurrent limit.
                    squeeze = float(self._gripper_contact_squeeze_fraction)
                    if squeeze > 0.0:
                        squeeze_target = max(
                            self._gripper_contact_min_open_fraction,
                            float(hold_command[6]) - squeeze,
                        )
                        n_sq = max(
                            1,
                            int(
                                np.ceil(
                                    (float(hold_command[6]) - squeeze_target)
                                    / (self._gripper_rate_fraction_s * self._control_period_s)
                                )
                            ),
                        )
                        start_g = float(hold_command[6])
                        for k in range(1, n_sq + 1):
                            if self._cancel_event.is_set():
                                return self._cancelled_result(
                                    request_id=request_id,
                                    kind="gripper",
                                    started_ns=started_ns,
                                    state=last,
                                    max_tracking_error_rad=0.0,
                                )
                            if time.monotonic() > deadline:
                                raise MotionFault(
                                    "ACTION_TIMEOUT", "gripper action exceeded its hard timeout"
                                )
                            hold_command[6] = start_g + (squeeze_target - start_g) * k / n_sq
                            self._command(hold_command)
                            time.sleep(self._control_period_s)
                            last = self._read_feedback()
                            if abs(float(last.effort[6])) > self._gripper_max_effort_nm:
                                raise MotionFault(
                                    "GRIPPER_OVERCURRENT",
                                    f"gripper effort {float(last.effort[6]):.6f} Nm exceeds "
                                    f"{self._gripper_max_effort_nm:.6f} Nm during squeeze",
                                )
                        self._log(
                            "feedback",
                            request_id,
                            command=hold_command,
                            feedback=last.position,
                            effort=last.effort,
                        )
                    self._gripper_hold_target = float(hold_command[6])
                    break
                contact_effort = float(last.effort[6])
                contact_probe_cooldown_steps = max(
                    1,
                    int(np.ceil(0.1 / self._control_period_s)),
                )
            if contact_effort < self._gripper_contact_min_effort_nm:
                contact_probe_armed = True
            pending_contact_confirmation = (
                stop_on_contact
                and delta < 0.0
                and contact_probe_cooldown_steps > 0
                and contact_effort >= self._gripper_contact_min_effort_nm
            )
            if stalled and not pending_contact_confirmation:
                if stall_started is None:
                    stall_started = time.monotonic()
                elif time.monotonic() - stall_started > self._gripper_stall_timeout_s:
                    raise MotionFault(
                        "GRIPPER_STALL",
                        f"gripper remained {remaining_error:.6f} from target with "
                        f"velocity below {self._gripper_stall_velocity_fraction_s:.6f} "
                        f"for {self._gripper_stall_timeout_s:.3f}s",
                    )
            else:
                stall_started = None
            self._log(
                "feedback",
                request_id,
                command=command,
                feedback=last.position,
                effort=last.effort,
            )
        error = abs(float(last.position[6]) - target)
        if not contacted and error > float(action["position_tolerance_fraction"]):
            raise MotionFault(
                "GRIPPER_STALL",
                f"final gripper error {error:.6f} exceeds tolerance "
                f"{float(action['position_tolerance_fraction']):.6f}",
            )
        result = {
            "request_id": request_id,
            "kind": "gripper",
            "status": "completed",
            "started_monotonic_ns": started_ns,
            "finished_monotonic_ns": time.monotonic_ns(),
            "final_joint_pos_0": last.position.astype(np.float32),
            "max_tracking_error_rad": 0.0,
            "detail": "gripper contact" if contacted else "",
        }
        self._finish(hold=True)
        self._log(
            "result",
            request_id,
            **{key: value for key, value in result.items() if key != "request_id"},
        )
        return result

    def _execute_cartesian_pose(
        self, *, request_id: int, action: dict[str, Any]
    ) -> dict[str, Any]:
        initial = self._read_feedback()
        start_pose = self._kinematics.forward(initial.position[:6])
        target_pose = np.asarray(action["pose"], dtype=np.float64)
        start_rotation = Rotation.from_quat(start_pose[[4, 5, 6, 3]])
        target_rotation = Rotation.from_quat(target_pose[[4, 5, 6, 3]])
        translation_distance = float(np.linalg.norm(target_pose[:3] - start_pose[:3]))
        rotation_distance = float((start_rotation.inv() * target_rotation).magnitude())
        duration = max(
            2.0 * self._control_period_s,
            1.875 * translation_distance / self._max_cartesian_translation_m_s,
            1.875 * rotation_distance / self._max_cartesian_rotation_rad_s,
        )
        duration *= 1.05
        if duration > float(action["timeout_s"]):
            raise MotionFault(
                "ACTION_TIMEOUT",
                f"Cartesian speed limits require {duration:.3f}s but timeout is "
                f"{float(action['timeout_s']):.3f}s",
            )
        steps = max(2, int(np.ceil(duration / self._control_period_s)))
        times = np.linspace(0.0, steps * self._control_period_s, steps + 1)
        unit_time = times / times[-1]
        blend = 10.0 * unit_time**3 - 15.0 * unit_time**4 + 6.0 * unit_time**5
        positions = start_pose[:3] + blend[:, None] * (
            target_pose[:3] - start_pose[:3]
        )
        rotations_xyzw = Slerp(
            [0.0, 1.0],
            Rotation.concatenate([start_rotation, target_rotation]),
        )(blend).as_quat()
        poses = np.column_stack(
            (
                positions,
                rotations_xyzw[:, 3],
                rotations_xyzw[:, 0],
                rotations_xyzw[:, 1],
                rotations_xyzw[:, 2],
            )
        )
        previous = initial.position[:6].astype(np.float64, copy=True)
        solved = [previous.copy()]
        for pose in poses[1:]:
            solution = self._kinematics.solve_ik(pose, seed_joints=previous)
            if solution is None:
                raise MotionFault(
                    "IK_FAILED",
                    "world-frame grasp_site target has no safe continuous IK trajectory",
                )
            solved.append(solution)
            previous = solution
        waypoints = np.asarray(solved)
        while True:
            try:
                self._kinematics.validate_trajectory(
                    waypoints,
                    times,
                    initial_joints=initial.position[:6],
                )
                break
            except TrajectoryValidationError as exc:
                if exc.code != "JOINT_ACCELERATION":
                    raise
                times *= 1.5
                if times[-1] > float(action["timeout_s"]):
                    raise MotionFault(
                        "ACTION_TIMEOUT",
                        "Cartesian IK path cannot satisfy joint dynamics within timeout",
                    ) from exc
        trajectory_action = {
            "kind": "joint_trajectory",
            "waypoints": np.asarray(waypoints, dtype=np.float32),
            "waypoint_times_s": times.astype(np.float32),
            "timeout_s": action["timeout_s"],
            "position_tolerance_rad": self._joint_settle_tolerance_rad,
        }
        return self._execute_joint_trajectory(
            request_id=request_id,
            action=trajectory_action,
            result_kind="cartesian_pose",
            initial_state=initial,
            cartesian_target_pose=target_pose,
        )

    def execute(self, *, request_id: int, action: dict[str, Any]) -> dict[str, Any]:
        if self._closed:
            raise RuntimeError("motion controller is closed")
        self._log("start", request_id, action=action)
        self._exec_phase.begun = False
        try:
            kind = action.get("kind")
            if kind == "joint_trajectory":
                return self._execute_joint_trajectory(request_id=request_id, action=action)
            if kind == "absolute_joints":
                return self._execute_absolute_joints(request_id=request_id, action=action)
            if kind == "gripper":
                return self._execute_gripper(request_id=request_id, action=action)
            if kind == "cartesian_pose":
                return self._execute_cartesian_pose(request_id=request_id, action=action)
            raise ValueError(f"unsupported motion kind {kind!r}")
        except TrajectoryValidationError as exc:
            self._log(
                "result",
                request_id,
                kind=str(action.get("kind", "unknown")),
                status="rejected",
                code=exc.code,
                detail=str(exc),
            )
            raise
        except MotionFault as exc:
            if exc.code == "ACTION_BUSY":
                self._log(
                    "result",
                    request_id,
                    kind=str(action.get("kind", "unknown")),
                    status="rejected",
                    code=exc.code,
                    detail=str(exc),
                )
                raise
            if not getattr(self._exec_phase, "begun", True):
                # 2026-09-03: the fault happened while planning/validating, before this
                # action commanded anything (IK_FAILED, speed-limit timeout, feedback
                # drift...). The arm is still exactly where the previous action left it
                # (its lease-monitored hold, or idle): leave it there. Gravity-comp idling
                # here used to drop a held object over a pure planning failure.
                self._log(
                    "result",
                    request_id,
                    kind=str(action.get("kind", "unknown")),
                    status="rejected",
                    code=exc.code,
                    detail=str(exc),
                    phase="planning",
                )
                raise
            self._actuator.enter_safe_idle()
            self._finish(hold=False)
            with self._state_lock:
                self._fault_detail = f"{exc.code}: {exc}"
                self._lease_request_id = None
            self._log(
                "result",
                request_id,
                kind=str(action.get("kind", "unknown")),
                status="fault",
                code=exc.code,
                detail=str(exc),
                feedback=None if self._last_state is None else self._last_state.position,
            )
            raise

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        self._cancel_event.set()
        self._watchdog_thread.join(timeout=1.0)
        self._actuator.enter_safe_idle()
