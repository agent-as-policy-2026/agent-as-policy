#!/usr/bin/env python3
"""Carry the multiple-experience memory state between this framework's per-cycle sessions.

    python3 mx/carry.py install <store> <session> <cycle>   before the session's server starts
    python3 mx/carry.py save    <store> <session> <cycle>   after the session stopped
    python3 mx/carry.py check   <store> <cycle>             handoff gate: exit 0 = cycle may start

Read-only reuse (2026-09-11, another model inherits a finished sequence, e.g. terra <- astra):
    python3 mx/carry.py seed       <source store> <store>   copy a finished store once (SEEDED_FROM.txt)
    python3 mx/carry.py check-ro   <store>                  exit 0 = the store holds checkpoints
    python3 mx/carry.py install-ro <store> <session>        every checkpoint + the shared scratch; a fresh
                                                           stage record; nothing is ever saved back

The original multiple-experience runtime ran all five cycles in ONE session, so every fresh agent saw that session's
experience/ (all checkpoints), the shared scratch/ (attempts.md, scripts, measurements, earlier
round reports) and the stage records (episodes.jsonl / episodes.json). Here every cycle has its
own session: install copies the state the last committed cycle left into the new session, save
copies it back out only when this session committed its checkpoint (a cycle without a checkpoint
leaves the carried state untouched, so a re-run of that cycle starts from the same state).
frames/ is not carried: later agents never opened an earlier cycle's frames (checked on the reference
five-fresh-agent session, 2026-09-11).

Store layout (<store> = paper_runs/<task>_<batch>/mx):
  experience/            cycle_NN/{memory.json, artifacts/}, latest.json, metrics.json, metrics.csv
  carry/scratch/         the shared scratch as the last committed cycle left it
  carry/episodes.jsonl   stage records of all committed cycles (incl. their failed attempts)
  carry/command_metrics.jsonl, carry/rounds.jsonl   inputs of experience/metrics.json
"""

import json
from pathlib import Path
import shutil
import sys
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from mx.cycle_dataset import progress, _write  # noqa: E402

CARRIED = ("episodes.jsonl", "command_metrics.jsonl", "rounds.jsonl")


def _checkpoints(experience):
    return sorted(p.name for p in Path(experience).glob("cycle_[0-9][0-9]") if (p / "memory.json").is_file())


def install(store, session, cycle):
    store, session, cycle = Path(store), Path(session), int(cycle)
    exp, carry = store / "experience", store / "carry"
    target = session / "experience"
    target.mkdir(parents=True, exist_ok=True)
    if exp.is_dir():
        for p in sorted(exp.iterdir()):
            if p.name.startswith("."):
                continue
            if p.is_dir():
                shutil.copytree(p, target / p.name)
            else:
                shutil.copy2(p, target / p.name)
    if (carry / "scratch").is_dir():
        shutil.copytree(carry / "scratch", session / "scratch", dirs_exist_ok=True)
    for name in CARRIED:
        if (carry / name).is_file():
            shutil.copy2(carry / name, session / name)
    state = progress(session)
    if (session / "episodes.jsonl").exists():
        _write(session / "episodes.json", state)
    out = {
        "ok": True,
        "cycle": cycle,
        "checkpoints": _checkpoints(target),
        "completed_cycles": state["completed_cycles"],
        "scratch_files": sum(1 for f in (session / "scratch").rglob("*") if f.is_file()),
    }
    print(json.dumps(out))
    return 0


def save(store, session, cycle):
    store, session, cycle = Path(store), Path(session), int(cycle)
    folder = session / "experience" / f"cycle_{cycle:02d}"
    if not (folder / "memory.json").is_file():
        print(json.dumps({"ok": False, "cycle": cycle,
                          "note": "no checkpoint committed in this session; carried state left unchanged"}))
        return 0
    exp = store / "experience"
    exp.mkdir(parents=True, exist_ok=True)
    have = _checkpoints(exp)
    if f"cycle_{cycle:02d}" in have or len(have) != cycle - 1:
        print(json.dumps({"ok": False, "cycle": cycle, "error": f"store holds {have}; cycle {cycle} not saved"}))
        return 3
    pending = exp / f".pending_{uuid.uuid4().hex}"
    shutil.copytree(folder, pending)
    pending.rename(exp / folder.name)
    for name in ("latest.json", "metrics.json", "metrics.csv"):
        src = session / "experience" / name
        if src.is_file():
            shutil.copy2(src, exp / name)
    carry = store / "carry"
    carry.mkdir(exist_ok=True)
    fresh = carry / f".scratch_{uuid.uuid4().hex}"
    shutil.copytree(session / "scratch", fresh, ignore=shutil.ignore_patterns("__pycache__"))
    if (carry / "scratch").exists():
        shutil.rmtree(carry / "scratch")
    fresh.rename(carry / "scratch")
    # the agent's last event = its exit; the next cycle's between_cycle_s counts this agent's time after
    # its checkpoint, as the original contiguous interval does (mx/experience.py metrics)
    events = session / "agent_events.jsonl"
    if events.is_file():
        with (session / "rounds.jsonl").open("a") as stream:
            stream.write(json.dumps({"cycle": cycle, "session": session.name,
                                     "agent_exit_wall_time_ns": events.stat().st_mtime_ns}) + "\n")
    for name in CARRIED:
        if (session / name).is_file():
            shutil.copy2(session / name, carry / name)
    print(json.dumps({"ok": True, "cycle": cycle, "checkpoints": _checkpoints(exp),
                      "scratch_files": sum(1 for f in (carry / "scratch").rglob("*") if f.is_file())}))
    return 0


def check(store, cycle):
    """The original verify_handoff(session, cycle - 1), on the carried state."""
    store, cycle = Path(store), int(cycle)
    completed = cycle - 1
    have = _checkpoints(store / "experience")
    state = progress(store / "carry")          # reads carry/episodes.jsonl
    history = [json.loads((store / "experience" / n / "memory.json").read_text())["cycle"] for n in have]
    ok = (state["completed_cycles"] == completed and state["successful_stages"] == completed
          and not state["open_segment"] and history == list(range(1, completed + 1)))
    msg = (f"cycle {cycle}: checkpoints {history}, completed stages {state['successful_stages']}, "
           f"open segment {state['open_segment']}")
    if not ok:
        print(f"cycle {completed} requires completed stages and a saved experience checkpoint ({msg})")
        return 3
    print(f"handoff ok for {msg}")
    return 0


def seed(source, store):
    """Copy a finished store (its checkpoints and carried scratch) once into a read-only batch."""
    source, store = Path(source), Path(store)
    have = _checkpoints(source / "experience")
    if not have:
        print(f"[seed] no committed checkpoints at {source / 'experience'}")
        return 2
    if _checkpoints(store / "experience"):
        print(f"[seed] target store already has checkpoints: {store}")
        return 3
    store.mkdir(parents=True, exist_ok=True)
    for name in ("experience", "carry"):
        if (source / name).is_dir():
            shutil.copytree(source / name, store / name, dirs_exist_ok=True)
    (store / "SEEDED_FROM.txt").write_text(
        f"seeded_from: {source}\ndate: {time.strftime('%Y-%m-%d %H:%M:%S')}\ncheckpoints: {' '.join(have)}\n")
    print(f"[seed] {store} <- {source} ({len(have)} checkpoint(s))")
    return 0


def check_ro(store):
    have = _checkpoints(Path(store) / "experience")
    if not have:
        print(f"no checkpoints in {Path(store) / 'experience'}: seed it first (--knowledge-from <batch>)")
        return 3
    print(f"read-only reuse of {len(have)} checkpoint(s) {have}")
    return 0


def install_ro(store, session):
    """Every checkpoint and the shared scratch the finished sequence left. The stage record starts fresh
    (this session's own assembly is its cycle 1) and nothing is saved back on stop."""
    store, session = Path(store), Path(session)
    exp, carry = store / "experience", store / "carry"
    target = session / "experience"
    target.mkdir(parents=True, exist_ok=True)
    if exp.is_dir():
        for p in sorted(exp.iterdir()):
            if p.name.startswith("."):
                continue
            if p.is_dir():
                shutil.copytree(p, target / p.name)
            else:
                shutil.copy2(p, target / p.name)
    if (carry / "scratch").is_dir():
        shutil.copytree(carry / "scratch", session / "scratch", dirs_exist_ok=True)
    print(json.dumps({"ok": True, "read_only": True, "checkpoints": _checkpoints(target),
                      "scratch_files": sum(1 for f in (session / "scratch").rglob("*") if f.is_file())}))
    return 0


if __name__ == "__main__":
    COMMANDS = {"install": install, "save": save, "check": check,
                "seed": seed, "check-ro": check_ro, "install-ro": install_ro}
    if len(sys.argv) < 2 or sys.argv[1] not in COMMANDS:
        print(__doc__)
        raise SystemExit(2)
    cmd, args = sys.argv[1], sys.argv[2:]
    raise SystemExit(COMMANDS[cmd](*args))
