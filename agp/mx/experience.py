"""Immutable experience checkpoints for repeated work by one physical operator.

The reference multiple-experience implementation's experience.py (not part of this repo) brought into this framework, 2026-09-11.
Validation, snapshot layout, memory fields and metrics columns are those of the original. Differences:
  * one stage per cycle: commit() requires the cycle's successful stages to be ["assemble"]
    (original: ["disassemble", "assemble"]);
  * every cycle runs in its own session; mx/carry.py copies the memory state into each session
    before its agent starts, so load()/commit() see what the original session-local store held;
  * metrics(): the original intervals, except that for cycles 2..5 the stretch between the previous agent's
    exit and this session's start (teardown, the scatter session, setup) is not counted: between
    = previous agent after its checkpoint (rounds.jsonl exit row) + this agent before its first start;
  * MOTION lists this framework's motion commands (home instead of run_program).
"""

import csv
import hashlib
import json
from pathlib import Path
import shutil
import time
import uuid

from .cycle_dataset import STAGES, progress, _write

MOTION = {"move_ee", "move_delta", "move_joints", "gripper", "home"}


def load(session):
    root = Path(session) / "experience"
    history = [json.loads(p.read_text()) for p in sorted(root.glob("cycle_*/memory.json"))]
    return {"ok": True, "latest": history[-1] if history else None, "history": history}


def _rounds(session):
    path = Path(session) / "rounds.jsonl"
    out = {}
    if path.exists():
        for line in path.read_text().splitlines():
            if line.strip():
                row = json.loads(line)
                out.setdefault(int(row["cycle"]), {}).update(row)   # ready row (server) + exit row (carry.py save)
    return out


def metrics(session):
    """Report cycle intervals including retries and reflection overhead."""
    history = load(session)["history"]
    path = Path(session) / "command_metrics.jsonl"
    commands = [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []
    rounds = _rounds(session)
    rows = []
    for index, memory in enumerate(history):
        segments = memory["segments"]
        start = segments[0]["start"]["monotonic_ns"]
        end = segments[-1]["end"]["monotonic_ns"]
        boundary = history[index - 1]["committed_monotonic_ns"] if index else start
        committed = memory["committed_monotonic_ns"]
        # the carried command_metrics.jsonl holds only this sequence's cycle sessions, so the original window
        # (previous checkpoint -> this checkpoint) selects the same commands it does there
        calls = [c for c in commands if boundary <= c["started_monotonic_ns"] < committed]
        between = (start - boundary) / 1e9
        own = rounds.get(memory["cycle"], {})
        if index and "ready_monotonic_ns" in own:
            # the original interval from the previous checkpoint holds the previous agent's work after its checkpoint
            # and this agent's work before its first start marker; count those two parts and leave out what
            # only exists here (session teardown, the scatter session, session setup)
            previous = history[index - 1]
            exit_ns = rounds.get(previous["cycle"], {}).get("agent_exit_wall_time_ns")
            after_previous = max(0, int(exit_ns) - previous["committed_wall_time_ns"]) if exit_ns else 0
            between = (after_previous + max(0, start - int(own["ready_monotonic_ns"]))) / 1e9
        row = dict(
            cycle=memory["cycle"],
            available_experience_version=memory["available_experience_version"],
            outcome="success",
            task_s=(end - start) / 1e9,
            between_cycle_s=between,
            reflection_s=(committed - end) / 1e9,
            total_s=between + (committed - start) / 1e9,
            failed_stage_attempts=sum(s["outcome"] != "success" for s in segments),
            motion_calls=sum(c["command"] in MOTION for c in calls),
            capture_calls=sum(c["command"] == "frames" for c in calls),
            unsuccessful_commands=sum(not c["ok"] for c in calls),
            motion_command_wait_s=sum(
                (min(c["finished_monotonic_ns"], committed) - c["started_monotonic_ns"]) / 1e9
                for c in calls
                if c["command"] in MOTION
            ),
        )
        row["speedup_vs_cycle_1"] = rows[0]["total_s"] / row["total_s"] if rows else 1.0
        rows.append(row)
    result = {
        "cycles": rows,
        "completed_checkpoints": len(history),
        "timing": "monotonic host clock; intervals from first assembly start through each checkpoint; later cycles also count the previous agent's time after its checkpoint and this agent's time before its first start marker",
        "excluded": "scene measurement before cycle 1, session teardown and setup, the scatter between cycles, recording finalization",
        "interpretation": "Within-sequence order trend. Physical placements, strategy changes and practice can affect times. This alone does not isolate a causal memory benefit.",
        "setup_segments": [],
    }
    root = Path(session) / "experience"
    root.mkdir(exist_ok=True)
    _write(root / "metrics.json", result)
    if rows:
        temporary = root / "metrics.csv.tmp"
        with temporary.open("w") as out:
            writer = csv.DictWriter(out, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        temporary.replace(root / "metrics.csv")
    return result


def commit(session, *, cycle, lesson, artifacts, mono_ns=None, wall_ns=None):
    session = Path(session).resolve()
    current = progress(session)
    history = load(session)["history"]
    if any(m["cycle"] == cycle for m in history):
        raise ValueError("cycle experience already saved")
    if (
        type(cycle) is not int
        or not 1 <= cycle <= 5
        or cycle != len(history) + 1
        or current["completed_cycles"] != cycle
        or current["open_segment"]
    ):
        raise ValueError("commit the latest completed cycle in order")
    if not isinstance(lesson, dict) or any(
        not isinstance(lesson.get(k), str) or not lesson[k].strip()
        for k in ["geometry", "recipe", "corrections", "next_cycle"]
    ):
        raise ValueError("lesson needs geometry, recipe, corrections and next_cycle evidence")
    if not isinstance(artifacts, list) or not artifacts:
        raise ValueError("include at least one validated scratch artifact")
    snapshots = []
    for name in artifacts:
        if not isinstance(name, str):
            raise ValueError("artifacts must be relative scratch paths")
        relative = Path(name)
        source = (session / relative).resolve()
        if (
            relative.is_absolute()
            or ".." in relative.parts
            or relative.parts[:1] != ("scratch",)
            or not source.is_relative_to(session / "scratch")
            or not source.is_file()
        ):
            raise ValueError("artifacts must be files within session scratch")
        snapshots.append((relative, source.read_bytes()))
    segments = [s for s in current["segments"] if s["cycle"] == cycle]
    if [s["stage"] for s in segments if s["outcome"] == "success"] != list(STAGES):
        raise ValueError("the assembly stage must be completed successfully")
    root = session / "experience"
    root.mkdir(exist_ok=True)
    folder = root / f"cycle_{cycle:02d}"
    temporary = root / f".pending_{uuid.uuid4().hex}"
    temporary.mkdir()
    try:
        records = []
        for relative, data in snapshots:
            destination = temporary / "artifacts" / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(data)
            records.append(
                dict(
                    source=str(relative),
                    sha256=hashlib.sha256(data).hexdigest(),
                    snapshot=str((folder / "artifacts" / relative).relative_to(session)),
                )
            )
        info_path = session / "session.json"
        info = json.loads(info_path.read_text()) if info_path.exists() else {}
        memory = dict(
            cycle=cycle,
            available_experience_version=cycle - 1,
            lesson=lesson,
            artifacts=records,
            source_segments=[s["id"] for s in segments if s["outcome"] == "success"],
            segments=segments,
            reference_sha256=info.get("reference_sha256", {}),
            task_prompt_sha256=info.get("task_prompt_sha256"),
            committed_monotonic_ns=time.monotonic_ns() if mono_ns is None else mono_ns,
            committed_wall_time_ns=time.time_ns() if wall_ns is None else wall_ns,
        )
        if memory["committed_monotonic_ns"] < segments[-1]["end"]["monotonic_ns"]:
            raise ValueError("checkpoint time precedes completed cycle")
        _write(temporary / "memory.json", memory)
        temporary.rename(folder)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    _write(root / "latest.json", memory)
    report = metrics(session)
    return {
        "ok": True,
        "cycle": cycle,
        "memory": str((folder / "memory.json").relative_to(session)),
        "metrics": report["cycles"][-1],
    }
