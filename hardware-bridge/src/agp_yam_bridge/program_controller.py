"""Buffered joint/gripper programs on top of MotionController.

program_controller.py of the reference throw runtime (not part of this repo)
brought into this bridge. Kept from the reference: the execution lock
(one program or ordinary action at a time), the dispatch lock that serializes every motor exchange with cancel/idle, the
per-program report file (program_<id>.json: program, initial position, per-tick trace,
measured status), planning rejections that never move the arm and keep the current hold,
faults during dispatch that cancel, latch and idle the chain. What differs here:

  * attached by main.py ONLY when the bridge config names program J4 limits
    (config/left_arm_throw.yaml); every other config keeps the plain MotionController, so
    the tasks recorded before this date run exactly the code they ran before;
  * ordinary kinds (absolute_joints, cartesian_pose, gripper, joint_trajectory) run the
    unmodified base paths: a new action still clears a latched fault (our _begin) and a
    cancelled ordinary move still returns "cancelled" instead of latching a fault;
  * the cancellation interlock (CANCELLED from the next exchange, fault latched by a cancel)
    applies while a program is in flight.
"""

from __future__ import annotations

import json
import re
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

from agp_yam_bridge.kinematics import TrajectoryValidationError
from agp_yam_bridge.motion import MotionController, MotionFault, MotionState
from agp_yam_bridge.program import compile_program, run_program
from agp_yam_bridge.protocol import PROGRAM_ID_PATTERN


class GuardedActuator:
    """Serialize every motor exchange (program ticks, ordinary moves, watchdog idle) on one lock."""

    def __init__(self, source: Any, lock: threading.RLock) -> None:
        self.source, self.lock = source, lock

    def read_motion_state(self) -> MotionState:
        with self.lock:
            return self.source.read_motion_state()

    def command_joint_positions(self, target: np.ndarray) -> None:
        with self.lock:
            self.source.command_joint_positions(target)

    def enter_safe_idle(self) -> None:
        with self.lock:
            self.source.enter_safe_idle()


class ProgramController(MotionController):
    def __init__(self, actuator: Any, kinematics: Any, config: Any, *, log_dir: Path, **kwargs: Any) -> None:
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.execution_lock = threading.Lock()
        self.dispatch_lock = threading.RLock()
        self.config = config
        self._program_in_flight = False
        super().__init__(GuardedActuator(actuator, self.dispatch_lock), kinematics, config, **kwargs)

    # ---- interlocks used by the program loop -------------------------------------------
    def check_running(self) -> None:
        if self._cancel_event.is_set() or self._closed:
            raise MotionFault("CANCELLED", self._cancel_reason or "motion cancelled")

    def _program_command(self, target: np.ndarray) -> None:
        with self.dispatch_lock:
            self.check_running()
            self._command(target)

    def _program_feedback(self) -> MotionState:
        with self.dispatch_lock:
            self.check_running()
            return self._read_feedback()

    def cancel(self, request_id: int) -> dict[str, Any]:
        # A completed session releases its held lease through the same cancel request as an
        # interrupted program. Only the completed case is reusable; a cancel that lands while a
        # program is in flight latches a fault so nothing can revive that program's hold.
        owns_execution = self.execution_lock.acquire(blocking=False)
        try:
            with self.dispatch_lock:
                with self._state_lock:
                    release_completed = (
                        owns_execution
                        and self._holding
                        and not self._active
                        and not self._closed
                        and not self._fault_detail
                    )
                    in_flight = self._program_in_flight
                result = super().cancel(request_id)
                with self._state_lock:
                    if release_completed and not self._fault_detail:
                        self._cancel_reason = ""
                        self._cancel_event.clear()
                    elif in_flight:
                        self._fault_detail = self._fault_detail or "cancelled by request"
                return result
        finally:
            if owns_execution:
                self.execution_lock.release()

    def close(self) -> None:
        self._cancel_event.set()
        acquired = self.execution_lock.acquire(timeout=5.0)   # an in-flight program exits within a tick
        try:
            super().close()
        finally:
            if acquired:
                self.execution_lock.release()

    # ---- programs ----------------------------------------------------------------------
    @staticmethod
    def checked_id(program_id: Any) -> str:
        if not isinstance(program_id, str) or not re.fullmatch(PROGRAM_ID_PATTERN, program_id):
            raise ValueError("invalid program_id")
        return program_id

    def report(self, program_id: str) -> dict[str, Any]:
        path = self.log_dir / f"program_{self.checked_id(program_id)}.json"
        if not path.is_file():
            raise MotionFault("PROGRAM_NOT_FOUND", f"no report for program_id {program_id!r}")
        return json.loads(path.read_text())

    def _program_rate_limits(self) -> tuple[np.ndarray, np.ndarray]:
        limits = self.config.acceptance.speed_limits
        velocity_limits = np.full(6, np.radians(limits.joint_velocity_deg_s))
        acceleration_limits = np.full(6, np.radians(limits.joint_acceleration_deg_s2))
        if limits.program_j4_velocity_deg_s is not None:
            velocity_limits[3] = np.radians(limits.program_j4_velocity_deg_s)
            acceleration_limits[3] = np.radians(limits.program_j4_acceleration_deg_s2)
        return velocity_limits, acceleration_limits

    def prepare(self, program: Any, state: MotionState):
        initial = state.position.copy()
        initial[6] = self._gripper_command_for_arm_motion(float(initial[6]))
        plan = compile_program(program, initial)
        velocity_limits, acceleration_limits = self._program_rate_limits()
        self._kinematics.validate_trajectory(
            plan.commands[:, :6],
            plan.times,
            initial_joints=initial[:6],
            max_joint_velocity_rad_s=velocity_limits,
            max_joint_acceleration_rad_s2=acceleration_limits,
        )
        # Include transitions from/to the held endpoint, not just internal knots.
        exceeded = np.flatnonzero(plan.peak_acceleration > acceleration_limits + 1e-9)
        if len(exceeded):
            joint = int(exceeded[0])
            raise TrajectoryValidationError(
                "JOINT_ACCELERATION",
                f"J{joint + 1} acceleration including start/stop "
                f"{plan.peak_acceleration[joint]:.6f} rad/s^2 exceeds "
                f"{acceleration_limits[joint]:.6f} rad/s^2",
            )
        return plan

    def preview(self, program: Any) -> dict[str, Any]:
        if not self.execution_lock.acquire(blocking=False):
            raise MotionFault("ACTION_BUSY", "controller busy")
        try:
            plan = self.prepare(program, self._read_feedback())
            poses = np.array([self._kinematics.forward(q[:6]) for q in plan.commands])
            return {
                "valid": True,
                "times_s": plan.times.tolist(),
                "commands": plan.commands.tolist(),
                "tcp_poses_wxyz": poses.tolist(),
                "tcp_velocity_m_s": np.gradient(poses[:, :3], plan.times, axis=0).tolist(),
                "events": plan.events,
                "peak_joint_velocity_rad_s": plan.peak_velocity.tolist(),
                "peak_joint_acceleration_rad_s2": plan.peak_acceleration.tolist(),
            }
        finally:
            self.execution_lock.release()

    def execute(self, *, request_id: int, action: dict[str, Any]) -> dict[str, Any]:
        if not self.execution_lock.acquire(blocking=False):
            raise MotionFault("ACTION_BUSY", "another motion action is already active")
        try:
            if action.get("kind") == "joint_program":
                return self.execute_program(request_id, action)
            return super().execute(request_id=request_id, action=action)
        finally:
            self.execution_lock.release()

    def execute_program(self, request_id: int, action: dict[str, Any]) -> dict[str, Any]:
        program_id = self.checked_id(action["program_id"])
        path = self.log_dir / f"program_{program_id}.json"
        if path.exists():
            raise ValueError(
                "program_id already executed; retrieve its report instead of replaying it"
            )
        report: dict[str, Any] = {
            "program_id": program_id,
            "request_id": request_id,
            "started_wall_time_ns": time.time_ns(),
            "program": action["program"],
            "trace": [],
            "status": "rejected",
        }
        try:
            json.dumps(report["program"], allow_nan=False)
        except (TypeError, ValueError):
            report["invalid_program_repr"] = repr(report.pop("program"))
        knots = len(action["program"].get("times_s", [])) if isinstance(action["program"], dict) else None
        self._log("start", request_id, action={"kind": "joint_program", "program_id": program_id, "knots": knots})
        self._exec_phase.begun = False
        self._program_in_flight = True
        dispatched = False
        started = time.monotonic_ns()
        try:
            self._begin(request_id)
            initial = self._read_feedback()
            plan = self.prepare(action["program"], initial)
            report["initial_position"] = initial.position.tolist()
            report["events"] = plan.events
            self.check_running()
            dispatched = True
            result = run_program(
                plan,
                command=self._program_command,
                feedback=self._program_feedback,
                check=self.check_running,
                trace=report["trace"],
                max_effort_nm=self.config.safety.gripper_max_effort_nm,
            )
            with self.dispatch_lock:
                self.check_running()
                report.update(result)
                self._gripper_hold_target = float(plan.commands[-1, 6])
                self._finish(hold=True)
            self._log(
                "result",
                request_id,
                kind="joint_program",
                status="completed",
                program_id=program_id,
                measured_status=report["status"],
                max_tracking_error_rad=result["max_tracking_error_rad"],
            )
            # Transport completed means the bounded program returned. The richer
            # report carries measured settle_miss and never asserts task success.
            return {
                "request_id": request_id,
                "kind": "joint_program",
                "status": "completed",
                "started_monotonic_ns": started,
                "finished_monotonic_ns": time.monotonic_ns(),
                "final_joint_pos_0": np.array(result["final_joint_pos_0"], dtype=np.float32),
                "max_tracking_error_rad": result["max_tracking_error_rad"],
                "detail": json.dumps(
                    {"program_id": program_id, "measured_status": report["status"]}
                ),
            }
        except Exception as exc:
            rejected = not dispatched and isinstance(exc, (ValueError, TrajectoryValidationError))
            code = getattr(exc, "code", "INVALID_ARGUMENT" if rejected else "CONTROL_SOURCE")
            report.update(
                status="rejected" if rejected else "fault",
                error_code=code,
                error=str(exc),
            )
            if not rejected:
                with self.dispatch_lock:
                    self._cancel_event.set()
                    self._fault_detail = f"{code}: {exc}"
                    self._lease_request_id = None
                    self._finish(hold=False)
                    self._actuator.enter_safe_idle()
                self._log(
                    "result", request_id, kind="joint_program", status="fault",
                    program_id=program_id, code=code, detail=str(exc),
                )
                raise MotionFault(code, str(exc)) from exc
            # A planning rejection: nothing was commanded, the arm keeps holding where the
            # previous action left it, now under this request's lease.
            self.check_running()
            self._finish(hold=True)
            self._log(
                "result", request_id, kind="joint_program", status="rejected",
                program_id=program_id, code=code, detail=str(exc), phase="planning",
            )
            state = self._last_state or self._read_feedback()
            return {
                "request_id": request_id,
                "kind": "joint_program",
                "status": "completed",
                "started_monotonic_ns": started,
                "finished_monotonic_ns": time.monotonic_ns(),
                "final_joint_pos_0": state.position.astype(np.float32),
                "max_tracking_error_rad": 0.0,
                "detail": json.dumps({"program_id": program_id, "measured_status": "rejected"}),
            }
        finally:
            self._program_in_flight = False
            report["finished_monotonic_ns"] = time.monotonic_ns()
            temporary = path.with_suffix(".tmp")
            temporary.write_text(json.dumps(report, allow_nan=False) + "\n")
            temporary.replace(path)
