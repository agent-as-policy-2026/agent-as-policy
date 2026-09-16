"""Task-neutral joint/hand programs: schema, 50 Hz compilation and the dispatch loop.

motion_program.py of the reference throw runtime (not part of this repo) brought into
this bridge, unchanged in behaviour. No robot imports and no task decisions live here; agp_yam_bridge.program_controller owns the hardware side.

A program is relative to the freshly measured start: `times_s` (strictly increasing from 0),
one six-vector of joint deltas per knot (the first all zeros) and optional `gripper_events`
({time_s, fraction}) that each start a 0.25 fraction/s ramp at the first 50 Hz tick at/after
their time. The last arm knot is held while the final jaw ramp finishes.
"""

from dataclasses import dataclass
import time

import numpy as np
from pydantic import BaseModel, ConfigDict, Field


class Event(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    time_s: float = Field(ge=0)
    fraction: float = Field(ge=0, le=1)


class Program(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    times_s: list[float] = Field(min_length=2)
    joint_deltas_rad: list[list[float]]
    gripper_events: list[Event] = Field(default_factory=list)


class ProgramFault(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


@dataclass
class CompiledProgram:
    times: np.ndarray
    commands: np.ndarray
    events: list[dict]
    peak_velocity: np.ndarray
    peak_acceleration: np.ndarray


def compile_program(value, initial, *, period=0.02):
    program = Program.model_validate(value)
    times = np.asarray(program.times_s)
    delta = np.asarray(program.joint_deltas_rad, dtype=float)
    initial = np.asarray(initial, dtype=float)
    if (
        initial.shape != (7,)
        or not np.isfinite(initial).all()
        or not 0 <= initial[6] <= 1
    ):
        raise ValueError("initial must be six joints and a gripper fraction")
    if (
        delta.shape != (len(times), 6)
        or not np.isfinite(delta).all()
        or times[0] != 0
        or np.any(delta[0] != 0)
        or np.any(np.diff(times) <= 0)
        or not times[-1] > 0
    ):
        raise ValueError(
            "program needs increasing times from zero, zero initial delta, and positive duration"
        )
    # Gripper events use the first 50 Hz tick at/after the requested time.
    # Never insert short extra intervals into the motor exchange schedule.
    events = []
    jaw_times, jaw_values = [0.0], [float(initial[6])]
    start_time, start_value, target_value = 0.0, float(initial[6]), float(initial[6])
    previous_tick = -1
    for event in program.gripper_events:
        tick = int(np.ceil(event.time_s / period - 1e-9))
        if event.time_s > times[-1] or tick <= previous_tick:
            raise ValueError(
                "gripper events must be ordered, within the program, and occupy distinct ticks"
            )
        scheduled = tick * period
        reached = start_time + abs(target_value - start_value) / 0.25
        if jaw_times[-1] < reached < scheduled:
            jaw_times.append(reached)
            jaw_values.append(target_value)
        value = start_value + float(
            np.clip(
                target_value - start_value,
                -0.25 * (scheduled - start_time),
                0.25 * (scheduled - start_time),
            )
        )
        jaw_times.append(scheduled)
        jaw_values.append(value)
        start_time, start_value, target_value = scheduled, value, event.fraction
        events.append(
            {
                "requested_s": event.time_s,
                "scheduled_s": scheduled,
                "fraction": event.fraction,
            }
        )
        previous_tick = tick
    reached = start_time + abs(target_value - start_value) / 0.25
    if reached > jaw_times[-1]:
        jaw_times.append(reached)
        jaw_values.append(target_value)
    duration = max(times[-1], reached)
    ticks = np.arange(int(np.ceil(duration / period - 1e-9)) + 1) * period
    joints = np.column_stack([np.interp(ticks, times, delta[:, j]) for j in range(6)])
    commands = np.column_stack(
        (joints + initial[:6], np.interp(ticks, jaw_times, jaw_values))
    )
    velocity = np.diff(commands[:, :6], axis=0) / period
    # Programs begin and finish at a held position. Include both transitions.
    acceleration = np.diff(np.vstack((np.zeros(6), velocity, np.zeros(6))), axis=0) / period
    return CompiledProgram(
        ticks,
        commands,
        events,
        np.max(np.abs(velocity), axis=0),
        np.max(np.abs(acceleration), axis=0) if len(acceleration) else np.zeros(6),
    )


def run_program(
    plan,
    *,
    command,
    feedback,
    check,
    trace,
    clock=time,
    max_effort_nm=1.2,
):
    """Dispatch once per tick, exchange immediately, then wait for the next tick.

    No I/O for diagnostics occurs in this loop. Caller persists even partial
    traces. `command` and `check` must enforce ownership/cancellation.
    """
    started = clock.monotonic()
    final = None
    peak_error = 0.0
    # Same bounded timeout style as ordinary moves; a terminal miss
    # is recoverable, whereas an unfinished path exceeding the timeout is not.
    timeout = max(10.0, 2.0 * float(plan.times[-1]) + 5.0)
    index = 0
    settled = False
    while True:
        scheduled = index * 0.02
        if clock.monotonic() - started >= timeout:
            if index < len(plan.commands):
                raise ProgramFault(
                    "ACTION_TIMEOUT", "program trajectory exceeded its timeout"
                )
            break
        check()
        clock.sleep(max(0.0, started + scheduled - clock.monotonic()))
        check()
        dispatch = clock.monotonic()
        target = plan.commands[min(index, len(plan.commands) - 1)]
        # Retain dispatch evidence even when the subsequent feedback fails.
        row = {
            "scheduled_s": float(scheduled),
            "dispatch_s": dispatch - started,
            "dispatch_monotonic_s": dispatch,
            "command": target.tolist(),
            "command_accepted": False,
        }
        trace.append(row)
        command(target)
        row["command_accepted"] = True
        row["command_return_s"] = clock.monotonic() - started
        final = feedback()
        now = clock.monotonic()
        row.update(
            feedback_s=now - started,
            exchange_s=now - dispatch,
            feedback=np.asarray(final.position).tolist(),
            velocity=np.asarray(final.velocity).tolist(),
            effort=np.asarray(final.effort).tolist(),
            feedback_monotonic_ns=int(final.monotonic_ns),
        )
        check()
        if abs(float(final.effort[6])) > max_effort_nm:
            raise ProgramFault(
                "GRIPPER_EFFORT", "gripper effort exceeded hardware limit"
            )
        if now - final.monotonic_ns / 1e9 > 0.1:
            raise ProgramFault("STALE_FEEDBACK", "motor feedback older than 100ms")
        error = float(np.max(np.abs(final.position[:6] - target[:6])))
        peak_error = max(peak_error, error)
        if index >= len(plan.commands) - 1:
            settled = (
                error <= 0.03 and abs(float(final.position[6]) - target[6]) <= 0.02
            )
            if settled:
                break
        index += 1
    return {
        "status": "completed" if settled else "settle_miss",
        "final_gripper_fraction": float(final.position[6]),
        "commanded_gripper_fraction": float(target[6]),
        "final_joint_pos_0": np.asarray(final.position).tolist(),
        "max_tracking_error_rad": peak_error,
        "events": plan.events,
        "motion_complete_is_task_success": False,
    }
