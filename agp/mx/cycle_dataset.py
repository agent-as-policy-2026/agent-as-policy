"""Ordered assembly-cycle evidence.

The reference multiple-experience implementation's cycle_dataset.py (progress/mark; not part
of this repo) brought into this framework. Differences, both forced by the task: one stage per cycle ("assemble"; the original cycles had disassemble + assemble),
and no cycle-0 "setup" stage. The row fields, attempt ids, ordering rules and error texts are
those of the original. The dataset export (per-attempt slices of the session recording) is not part of this module.
"""

import json
import math
from pathlib import Path
import time

STAGES = ("assemble",)
CYCLES = 5


def _jsonable(o):
    # the original geometry._jsonable: numpy scalars / arrays / paths -> plain JSON (duck-typed, no numpy import)
    if hasattr(o, "tolist"):
        return o.tolist()
    if hasattr(o, "item"):
        return o.item()
    if isinstance(o, Path):
        return str(o)
    raise TypeError(f"{type(o).__name__} is not JSON serializable")


def _finite(value):
    # allow_nan=False below: a NaN telemetry field would otherwise abort the marker (the original runtime
    # validated its observations finite; this bridge does not), so non-finite floats become null
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: _finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite(v) for v in value]
    return value


def _write(path, value):
    path = Path(path)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, default=_jsonable, allow_nan=False) + "\n")
    temporary.replace(path)


def progress(session):
    path = Path(session) / "episodes.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []
    segments = []
    for row in rows:
        if row["event"] == "start":
            segments.append(
                {
                    "id": row["id"],
                    "cycle": row["cycle"],
                    "stage": row["stage"],
                    "start": row,
                    "end": None,
                    "outcome": "incomplete",
                }
            )
        else:
            segments[-1].update(end=row, outcome=row["outcome"])
    successes = [s for s in segments if s["cycle"] > 0 and s["outcome"] == "success"]
    return {
        "completed_cycles": len(successes) // len(STAGES),
        "successful_stages": len(successes),
        "segments": segments,
        "open_segment": segments[-1]["id"] if segments and segments[-1]["end"] is None else None,
    }


def mark(
    session,
    *,
    actor,
    cycle,
    stage,
    event,
    outcome=None,
    capture=None,
    note,
    state,
    revision,
    wall_ns=None,
    mono_ns=None,
):
    """Called under the robot server after permission and capture checks."""
    session = Path(session)
    current = progress(session)
    if type(cycle) is not int or not 1 <= cycle <= CYCLES or stage not in STAGES:
        raise ValueError("cycle must be 1..5 and stage assemble")
    if not isinstance(note, str) or not note.strip() or event not in {"start", "end"}:
        raise ValueError("nonempty note and start/end event required")
    if event == "start":
        if current["open_segment"]:
            raise ValueError("end the open segment before beginning another")
        n = current["successful_stages"]
        expected = (n // len(STAGES) + 1, STAGES[n % len(STAGES)])
        if (cycle, stage) != expected or n >= CYCLES * len(STAGES):
            raise ValueError("complete the current assembly stage before advancing")
        attempt = 1 + sum(s["cycle"] == cycle and s["stage"] == stage for s in current["segments"])
        identity = f"cycle_{cycle:02d}_{stage}_attempt_{attempt:02d}"
    else:
        if not current["open_segment"]:
            raise ValueError("no open segment")
        segment = current["segments"][-1]
        if (cycle, stage) != (segment["cycle"], segment["stage"]):
            raise ValueError("end must match the open segment")
        if actor not in {segment["start"]["actor"], "coordinator"}:
            raise ValueError("only the segment actor or coordinator may end it")
        if outcome not in {"success", "failure", "uncertain"} or type(capture) is not int:
            raise ValueError("end requires assessed outcome and capture evidence")
        identity = segment["id"]
    row = dict(
        id=identity,
        cycle=cycle,
        stage=stage,
        event=event,
        actor=actor,
        outcome=outcome,
        capture=capture,
        note=note,
        state=_finite(state),
        revision=revision,
        wall_time_ns=time.time_ns() if wall_ns is None else wall_ns,
        monotonic_ns=time.monotonic_ns() if mono_ns is None else mono_ns,
    )
    encoded = json.dumps(row, default=_jsonable, allow_nan=False)
    with (session / "episodes.jsonl").open("a") as stream:
        stream.write(encoded + "\n")
    result = progress(session)
    _write(session / "episodes.json", result)
    return {
        "ok": True,
        "segment": identity,
        "completed_cycles": result["completed_cycles"],
        "successful_stages": result["successful_stages"],
    }
