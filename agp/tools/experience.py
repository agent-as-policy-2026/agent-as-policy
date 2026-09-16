#!/usr/bin/env python3
"""Immutable experience checkpoints — a faithful port of the multiple-experience method's
experience.py (reference implementation, not part of this repo) to this framework.

Same contract as the original version:
  * one checkpoint per completed cycle, committed in order (cycle == len(history) + 1),
    a cycle may be saved only once;
  * `lesson` is a dict with FOUR required non-empty strings:
    geometry / recipe / corrections / next_cycle;
  * `artifacts` is a NON-EMPTY list of files under the session's scratch/; each is
    snapshotted byte-for-byte with its sha256, so the procedure is preserved exactly
    as used even if the working copy is edited later;
  * the checkpoint folder is immutable and written atomically (.pending_<uuid> -> rename);
  * the next agent is handed the LATEST checkpoint only and rewrites it, carrying forward
    what still applies (rolling summary, not an append-only log).

Deviations forced by this framework, documented deliberately:
  * the original's cycles live inside ONE session; ours are separate sessions, so the store lives in
    the batch directory (paper_runs/<task>_<batch>/experience/) and the latest checkpoint is
    copied INTO each session as experience/cycle_NN/, reproducing what the original agent can reach
    (the original agent's cwd is the session, which holds every checkpoint, and its no-arg experience call
    returns the full history);
  * the original validates against graded episode stages (disassemble+assemble both successful) before
    allowing a commit; this framework has no stage machinery, so a checkpoint records the
    agent's own claim and `verified_stages` is absent. The trial gate (see run_paper_pyramid.sh)
    still refuses to start cycle N+1 without checkpoint N, mirroring the original verify_handoff.

Agent CLI (run from the session directory, like robot_client.py):
    python3 experience.py show                 -> {assignment, latest, history}; history is every
                                                  checkpoint committed so far, as the original load() returns
    python3 experience.py save '<json>'        -> validate + write scratch/experience_checkpoint/
        json = {"cycle": N, "lesson": {"geometry":..., "recipe":..., "corrections":..., "next_cycle":...},
                "artifacts": ["scratch/....py", ...]}

Harness:
    python3 tools/experience.py install <store> <session> <cycle>   copy latest checkpoint in
    python3 tools/experience.py commit <store> <session>            move the session's checkpoint into the store
    python3 tools/experience.py history <store>                     one line per checkpoint
"""
import hashlib
import json
import os
import shutil
import sys
import time
import uuid
from pathlib import Path

LESSON_KEYS = ["geometry", "recipe", "corrections", "next_cycle"]
PENDING = "scratch/experience_checkpoint"


def _write(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    tmp.replace(path)


def load(store):
    """All committed checkpoints of a store, oldest first."""
    root = Path(store)
    history = [json.loads(p.read_text()) for p in sorted(root.glob("cycle_*/memory.json"))]
    return {"ok": True, "latest": history[-1] if history else None, "history": history}


def save(session, payload):
    """Validate the agent's checkpoint and stage it in the session. Raises ValueError."""
    session = Path(session).resolve()
    if not isinstance(payload, dict):
        raise ValueError("payload must be a JSON object")
    cycle = payload.get("cycle")
    lesson = payload.get("lesson")
    artifacts = payload.get("artifacts")
    assign_path = session / "experience" / "assignment.json"
    assigned = json.loads(assign_path.read_text())["cycle"] if assign_path.exists() else None
    if type(cycle) is not int or cycle < 1:
        raise ValueError("cycle must be a positive integer")
    if assigned is not None and cycle != assigned:
        raise ValueError(f"this session is assigned cycle {assigned}")
    if not isinstance(lesson, dict) or any(
        not isinstance(lesson.get(k), str) or not lesson[k].strip() for k in LESSON_KEYS
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
    folder = session / PENDING
    if folder.exists():
        shutil.rmtree(folder)
    temporary = session / f"scratch/.pending_{uuid.uuid4().hex}"
    temporary.mkdir(parents=True)
    try:
        records = []
        for relative, data in snapshots:
            destination = temporary / "artifacts" / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(data)
            records.append(
                dict(source=str(relative), sha256=hashlib.sha256(data).hexdigest(),
                     snapshot=str(Path("artifacts") / relative))
            )
        memory = dict(
            cycle=cycle,
            available_experience_version=cycle - 1,
            lesson={k: lesson[k] for k in LESSON_KEYS},
            artifacts=records,
            session=session.name,
            committed_monotonic_ns=time.monotonic_ns(),
            committed_wall_time_ns=time.time_ns(),
        )
        _write(temporary / "memory.json", memory)
        temporary.rename(folder)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    return {"ok": True, "cycle": cycle, "checkpoint": PENDING,
            "artifacts": [r["source"] for r in records],
            "note": "staged; the harness commits it to the batch store when this session stops"}


def commit(store, session):
    """Move a session's staged checkpoint into the immutable batch store."""
    store, session = Path(store), Path(session).resolve()
    staged = session / PENDING
    if not (staged / "memory.json").exists():
        return {"ok": False, "error": "no staged checkpoint in this session"}
    memory = json.loads((staged / "memory.json").read_text())
    history = load(store)["history"]
    cycle = memory["cycle"]
    if any(m["cycle"] == cycle for m in history):
        return {"ok": False, "error": f"cycle {cycle} experience already saved"}
    if cycle != len(history) + 1:
        return {"ok": False, "error": f"commit the latest completed cycle in order (expected {len(history) + 1}, got {cycle})"}
    store.mkdir(parents=True, exist_ok=True)
    folder = store / f"cycle_{cycle:02d}"
    temporary = store / f".pending_{uuid.uuid4().hex}"
    shutil.copytree(staged, temporary)
    try:
        temporary.rename(folder)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    _write(store / "latest.json", memory)
    return {"ok": True, "cycle": cycle, "checkpoint": str(folder),
            "artifacts": [r["source"] for r in memory["artifacts"]]}


def install(store, session, cycle):
    """Copy EVERY committed checkpoint (with its artifacts) into a session, plus the assignment.

    The original store lives inside the session, so the original agent's working directory holds every checkpoint
    committed so far and `call("experience")` with no args returns the full history. Ours lives in
    the batch directory, so all of it is copied in to reproduce exactly what the original agent can reach."""
    store, session = Path(store), Path(session).resolve()
    target = session / "experience"
    target.mkdir(parents=True, exist_ok=True)
    history = load(store)["history"]
    latest = history[-1] if history else None
    _write(target / "assignment.json",
           {"cycle": int(cycle), "available_experience_version": latest["cycle"] if latest else 0})
    installed = []
    for memory in history:
        name = f"cycle_{memory['cycle']:02d}"
        src, dst = store / name, target / name
        if dst.exists():
            shutil.rmtree(dst)
        shutil.copytree(src, dst)
        # snapshot paths rewritten session-relative so each copied memory.json stays self-consistent
        copied = json.loads((dst / "memory.json").read_text())
        for record in copied["artifacts"]:
            record["snapshot"] = str(Path("experience") / name / "artifacts" / record["source"])
        _write(dst / "memory.json", copied)
        installed.append(name)
    return {"ok": True, "installed": installed,
            "available_experience_version": latest["cycle"] if latest else 0,
            "latest_artifacts": [r["source"] for r in latest["artifacts"]] if latest else []}


def _agent_main(argv):
    session = Path.cwd()
    if not argv or argv[0] not in ("show", "save"):
        print(__doc__.split("Agent CLI")[1].strip(), file=sys.stderr)
        return 2
    if argv[0] == "show":
        a = session / "experience" / "assignment.json"
        state = load(session / "experience")
        out = {"ok": True,
               "assignment": json.loads(a.read_text()) if a.exists() else None,
               "latest": state["latest"], "history": state["history"]}
        print(json.dumps(out, indent=2))
        return 0
    try:
        payload = json.loads(argv[1]) if len(argv) > 1 else json.load(sys.stdin)
    except Exception as exc:
        print(json.dumps({"ok": False, "error": f"invalid JSON: {exc}"}))
        return 1
    try:
        print(json.dumps(save(session, payload), indent=2))
        return 0
    except ValueError as exc:
        print(json.dumps({"ok": False, "error": str(exc)}))
        return 1


def main():
    argv = sys.argv[1:]
    if argv and argv[0] in ("install", "commit", "history"):
        if argv[0] == "install":
            print(json.dumps(install(argv[1], argv[2], argv[3])))
        elif argv[0] == "commit":
            print(json.dumps(commit(argv[1], argv[2])))
        else:
            for m in load(argv[1])["history"]:
                print(f"cycle {m['cycle']:02d}  from {m.get('session', '?')}  "
                      f"artifacts {len(m['artifacts'])}  lesson chars "
                      f"{ {k: len(v) for k, v in m['lesson'].items()} }")
        return 0
    return _agent_main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
