#!/usr/bin/env python3
"""Prompt, interface section and session.json for multiple-experience sessions.

    python3 mx/prompt.py session-json <session> <cycle> <task_prompt_file> <goal_dir>
    python3 mx/prompt.py prompt <cycle>        -> the sections appended to PROMPT.md
    python3 mx/prompt.py readme                -> the section appended to README_interface.md
    read-only reuse of a finished sequence (2026-09-11):
    python3 mx/prompt.py session-json <session> 1 <task_prompt_file> <goal_dir> --read-only
    python3 mx/prompt.py prompt-ro <session>   -> the same sections without the saving / handoff parts
    python3 mx/prompt.py readme-ro

Every memory-related instruction of the original rendered cycle prompt (agents/cycle_NN/
PROMPT.md: shared operator instructions + task + experience section + assigned round) is here,
worded as in the original. Dropped, per the decision to keep this framework: the session-spanning
launcher, home ownership, the original runtime interface (run_program, solve_ik, tcp_gap), the
original 1200-command / 180-minute shared caps and the dataset export. Changed where the task differs:
one assembly stage per cycle (no disassembly, no cycle-0 setup), the left arm and the left set.
"""

import hashlib
import json
from pathlib import Path
import sys

OPERATOR = """
## Operator record (all cycles)

Write and execute your own Python programs in scratch/. Use your image tool for
analysis and visualization. Keep and revise your scripts, measurements and
experiments during this same session.

Keep scratch/attempts.md with commands, evidence and what you learned. When
finished, capture final evidence and write scratch/RESULT.md with your
interpretation, implementation, measurements, experiments, actual outcomes,
relevant image/report paths and an honest judgement of success, failure or
uncertainty. Distinguish direct observations from estimates and command feedback.
A normal agent exit, endpoint tolerance or successful command is not physical
task success.

Reuse measured part geometry, validated transforms and successful scripts across
cycles to reduce repeated work. Revalidate actual poses, grasp
and visibility for each reuse. Do not replay coordinates after objects move.
Complete the measurement, planning and execution for your assigned cycle.

# Required cycle boundaries and training evidence

The robot server owns structured cycle markers. Every physical stage must be
inside its own start/end pair, including retries. Begin before planning or action
for that stage. Call through the session robot interface.

```
python3 robot_client.py . episode '{"cycle": 1, "stage": "assemble", "event": "start", "note": "Assemble both pairs of the left set as demonstrated"}'
# Measure, grasp, lift, align, insert, release, withdraw and inspect.
python3 robot_client.py . frames
python3 robot_client.py . episode '{"cycle": 1, "stage": "assemble", "event": "end", "outcome": "success", "capture": <that capture>, "note": "Both pairs engaged and stable; open gripper withdrawn"}'
```

Each cycle has one stage, assemble. End requires the latest server capture with
no intervening motion, under 30 seconds old. Success requires a verified empty
open gripper. Inspect the evidence and supply a precise note. Record failure or
uncertain when justified, then a new start for that same stage if a revised
attempt is feasible. Never advance the number to hide a failure. After recovery
and fresh observation, close an unresolved attempt honestly before starting the
next one.

Read episodes.json for progress. Every end is saved immediately in episodes.jsonl
and episodes.json. Detailed scripts and findings go in scratch with paths relative
to the session. Save grasp estimates, receiver poses, chosen joint targets,
corrections and the supporting capture IDs so later training can distinguish plan
from execution.

Stop after five verified cycles. Report overall success only when completed_cycles
is 5 and the last assembly is still stable. RESULT.md reports initial state, each
cycle's outcome, failure attempts, final evidence and timing. If blocked by safety,
uncertain fit, budget or hardware fault, stop honestly with the completed count and
retain all data. Five successful cycles are the goal, not a reason to exceed
measured clearances or hide failed attempts.

# Experience shared between fresh cycle agents

Each cycle runs in its own session with a fresh agent process and context.
Agents share persistent experience and validated artifacts across these sessions.
Cycle 1 starts with empty task experience and establishes a successful procedure.
On later cycles reuse the verified procedure with freshly observed object poses.
Avoid repeating solved geometry analysis. Prefer compact parameterized scripts
with clear inputs for current part poses, receiver poses and safe clearances.
Keep the same motion limits. Efficiency should come from fewer avoidable analysis
steps, captures, corrections and moves while preserving outcome verification.

After each successful assembly end marker, including cycle 5, save an experience
checkpoint before the next cycle. Read the prior checkpoint before planning a new
cycle. After saving your checkpoint and round report, exit so the next fresh agent
can be started. The server enforces the assigned cycle. The next cycle is started
only after its completed stages and saved experience are verified.

```
python3 robot_client.py . experience
# "latest" contains the last immutable checkpoint, or null for cycle 1.
# After a full cycle ends with verified release and withdrawal:
python3 robot_client.py . experience '{"cycle": 1,
  "lesson": {
    "geometry": "Measured dimensions and grasp/receiver estimates and units",
    "recipe": "Successful script entry point, exact parameters and execution evidence",
    "corrections": "Failed attempts and changes that actually worked, or explicitly none",
    "next_cycle": "What will be reused, what must be rechecked and the proposed time saving"
  },
  "artifacts": ["scratch/your_actual_successful_script.py", "scratch/your_measurements.json"]}'
```

Replace example text and paths with this cycle's observed results and real files.
Frames and capture numbers stay in this cycle's session and are not carried to later
cycles, so do not cite capture IDs in the lesson; state what the evidence showed.
Keep the checkpoint concise and self-contained. Carry forward useful geometry,
procedure and corrections from prior experience, updating them with current evidence.
Include dependencies needed to execute the saved scripts. The server snapshots these files and successful
stage evidence under experience/cycle_NN. Snapshots preserve the exact procedure
as used even when you edit your working scripts later. geometry may include
uncertainty. recipe must distinguish commands from confirmed physical results.
corrections must include unsuccessful alternatives rather than calling them
successful reusable actions. next_cycle must say which earlier version you used,
what current observations validated reuse and what changed. The automatic field
available_experience_version identifies the prior saved checkpoint. Your lesson
identifies which procedure you actually reused. A checkpoint records
your evidence-based assessment. It does not automatically certify a physical fit.

Experience, the shared scratch and the stage records are carried from each cycle's
session into the next. Read the latest checkpoint and its referenced artifacts
for prior task knowledge. Use fresh observations to establish current object poses.
Previous agent transcripts remain archived for analysis. The task uses the left
arm and the four parts of the left set.

Start each assembly marker before that cycle's planning or measurements. Keep
all retries inside marked attempts. Do not do next-cycle motion while writing the
previous checkpoint. Checkpointing and between-cycle planning time are included
in experience/metrics.csv and metrics.json. Timings show task time, reflection,
between-cycle overhead, command waits, captures and failed attempts.

The cycle 5 agent must, after checkpoint 5, verify the final assembly evidence is
saved. In RESULT.md report each of the five measured times, outcomes, retries and
actual changes to the procedure. A decreasing time trend is evidence from this one
ordered sequence of cycles. Distinguish it from a controlled comparison against
five cycles without experience checkpointing.
"""

README = """
## Cycle experience and stage markers

Two extra commands in this session, both free (not counted):

| Command | Arguments | Result |
|---|---|---|
| `experience` | `{}` | immutable checkpoint history and the latest version: `{"ok", "latest", "history"}` |
| `experience` | `{"cycle": n, "lesson": {...}, "artifacts": [...]}` | commits the latest completed cycle |
| `episode` | `{"cycle": n, "stage": "assemble", "event": "start", "note": "..."}` | opens a stage attempt |
| `episode` | `{"cycle": n, "stage": "assemble", "event": "end", "outcome": "success"/"failure"/"uncertain", "capture": N, "note": "..."}` | closes it |

lesson requires nonempty geometry, recipe, corrections and next_cycle strings.
artifacts contains real relative scratch file paths. The server supplies successful
evidence, timestamps and file hashes. Every cycle requires one checkpoint. The next
cycle marker requires the previous checkpoint. The server rejects other cycle
numbers, and motion after the checkpoint of cycles 1 through 4. Recover before
committing a checkpoint when needed. Metrics and snapshots are in experience/.
An end marker requires the latest `frames` capture with no motion since it and less
than 30 s old; a success end additionally requires an empty open gripper
(fraction at least 0.95). Progress is in episodes.json.
"""


def assigned_round(cycle):
    """The original cycle_agents.prepare_agent text; disassembly, home and shared caps removed."""
    cycle = int(cycle)
    previous = f"experience/cycle_{cycle - 1:02d}/memory.json" if cycle > 1 else None
    assignment = dict(cycle=cycle, previous_experience=previous)
    text = f"\n\n# Your assigned round\n\nYou are a fresh agent assigned ONLY cycle {cycle} of 5.\n"
    text += (
        "Read your assignment below and the latest experience checkpoint. Reuse its validated "
        "scripts and accumulated lessons, then verify current poses from fresh observations. "
        "Shared scratch files are working artifacts. The checkpoint identifies validated versions. "
        "Use experience files for prior task knowledge. Keep previous agent transcripts and "
        "Codex session logs out of your context. Save a self-contained cumulative lesson, "
        "including still-useful prior knowledge, corrections and required script dependencies.\n"
    )
    if cycle == 1:
        text += "Task experience starts empty. Perform cycle 1 assembly.\n"
    else:
        text += (
            f"Read {previous} and its referenced artifacts. Start from the parts as they now "
            "lie and perform your assigned assembly.\n"
        )
    if cycle < 5:
        text += (
            f"After verified assembly, save experience for cycle {cycle}, write "
            f"scratch/cycle_{cycle:02d}_RESULT.md, leave the open empty gripper withdrawn, "
            "and exit.\n"
        )
    else:
        text += (
            "After verified assembly, save experience for cycle 5, verify fresh final evidence, "
            "and write scratch/RESULT.md using the structured stage records and experience "
            "metrics. Then exit.\n"
        )
    text += (
        "If blocked, record the observed outcome and a round report, recover safely if feasible, "
        "and exit. The next cycle is not started on an incomplete handoff.\n\n"
        + json.dumps(assignment, indent=2)
        + "\n"
    )
    return text


# ---- read-only reuse (2026-09-11): a fresh agent of another model is handed the memory a finished
# five-cycle sequence left (every checkpoint + the shared scratch) and performs one assembly with it.
# The text is the sequence's own text above; only the parts about saving checkpoints, the five-cycle
# handoff and the cycle 5 report are replaced. Each replaced passage must occur exactly once.
_RO_READ = """Read the latest checkpoint before planning. In this session the experience is
read-only: do not save a checkpoint (the server refuses commits).

```
python3 robot_client.py . experience
# "latest" contains the last immutable checkpoint, "history" all of them.
```

The server snapshotted each checkpoint's files and successful stage evidence under
experience/cycle_NN. Snapshots preserve the exact procedure as used even when the
working scripts were edited later. The field available_experience_version identifies
the checkpoint each cycle had available; each lesson identifies which procedure its
cycle actually reused. A checkpoint records its agent's evidence-based assessment.
It does not automatically certify a physical fit.

"""

_RO_REPLACE = (
    ("""Stop after five verified cycles. Report overall success only when completed_cycles
is 5 and the last assembly is still stable. RESULT.md reports initial state, each
cycle's outcome, failure attempts, final evidence and timing. If blocked by safety,
uncertain fit, budget or hardware fault, stop honestly with the completed count and
retain all data. Five successful cycles are the goal, not a reason to exceed
measured clearances or hide failed attempts.""",
     # 2026-09-11 after terra t03: the first wording "Stop after one verified assembly" was read as
     # "one pair"; this is the sequence's own wording with five cycles -> one cycle
     """Stop after one verified cycle. Report overall success only when completed_cycles
is 1 and the last assembly is still stable. RESULT.md reports initial state, the
cycle's outcome, failure attempts, final evidence and timing. If blocked by safety,
uncertain fit, budget or hardware fault, stop honestly with the completed count and
retain all data. A successful cycle is the goal, not a reason to exceed
measured clearances or hide failed attempts."""),
    ("""Cycle 1 starts with empty task experience and establishes a successful procedure.
On later cycles reuse the verified procedure with freshly observed object poses.""",
     """You are handed the experience that earlier cycle agents accumulated on this task.
Reuse the verified procedure with freshly observed object poses."""),
    ("""Experience, the shared scratch and the stage records are carried from each cycle's
session into the next.""",
     """The experience and the shared scratch of those cycles are copied into this
session."""),
    ("""Start each assembly marker before that cycle's planning or measurements. Keep
all retries inside marked attempts. Do not do next-cycle motion while writing the
previous checkpoint. Checkpointing and between-cycle planning time are included
in experience/metrics.csv and metrics.json. Timings show task time, reflection,
between-cycle overhead, command waits, captures and failed attempts.

The cycle 5 agent must, after checkpoint 5, verify the final assembly evidence is
saved. In RESULT.md report each of the five measured times, outcomes, retries and
actual changes to the procedure. A decreasing time trend is evidence from this one
ordered sequence of cycles. Distinguish it from a controlled comparison against
five cycles without experience checkpointing.
""",
     """Start the assembly marker before planning or measurements. Keep all retries inside
marked attempts. experience/metrics.csv and metrics.json hold the earlier cycles'
timings: task time, reflection, between-cycle overhead, command waits, captures and
failed attempts.
"""),
)


def operator_ro():
    head, rest = OPERATOR.split("After each successful assembly end marker", 1)
    _, tail = rest.split("Experience, the shared scratch and the stage records are carried", 1)
    text = head + _RO_READ + "Experience, the shared scratch and the stage records are carried" + tail
    for old, new in _RO_REPLACE:
        if text.count(old) != 1:
            raise SystemExit(f"prompt-ro: passage not found exactly once: {old[:60]!r}")
        text = text.replace(old, new)
    return text


def readme_ro():
    commit_row = ('| `experience` | `{"cycle": n, "lesson": {...}, "artifacts": [...]}` | commits the latest '
                  'completed cycle |\n')
    rules = """lesson requires nonempty geometry, recipe, corrections and next_cycle strings.
artifacts contains real relative scratch file paths. The server supplies successful
evidence, timestamps and file hashes. Every cycle requires one checkpoint. The next
cycle marker requires the previous checkpoint. The server rejects other cycle
numbers, and motion after the checkpoint of cycles 1 through 4. Recover before
committing a checkpoint when needed. Metrics and snapshots are in experience/.
"""
    text = README
    for old, new in ((commit_row, ""), (rules, "The experience is read-only in this session: the server refuses "
                     "checkpoint commits.\nMetrics and snapshots of the earlier cycles are in experience/.\n")):
        if text.count(old) != 1:
            raise SystemExit(f"readme-ro: passage not found exactly once: {old[:60]!r}")
        text = text.replace(old, new)
    return text


def assigned_round_ro(session):
    cycles = sorted(int(p.parent.name[6:]) for p in Path(session, "experience").glob("cycle_[0-9][0-9]/memory.json"))
    if not cycles:
        raise SystemExit("prompt-ro: the session holds no checkpoints")
    previous = f"experience/cycle_{cycles[-1]:02d}/memory.json"
    assignment = dict(cycle=1, previous_experience=previous, read_only=True)
    return (
        "\n\n# Your assigned round\n\nYou are a fresh agent assigned ONLY cycle 1.\n"
        "Read your assignment below and the latest experience checkpoint. Reuse its validated "
        "scripts and accumulated lessons, then verify current poses from fresh observations. "
        "Shared scratch files are working artifacts. The checkpoint identifies validated versions. "
        "Use experience files for prior task knowledge. Keep previous agent transcripts and "
        "Codex session logs out of your context.\n"
        f"Read {previous} and its referenced artifacts. The experience is read-only: do not save a "
        "checkpoint. Start from the parts as they now lie and perform your assigned assembly.\n"
        "After verified assembly, verify fresh final evidence, write scratch/RESULT.md, leave the open "
        "empty gripper withdrawn, and exit.\n"
        "If blocked, record the observed outcome and a report in scratch/RESULT.md, recover safely if "
        "feasible, and exit.\n\n"
        + json.dumps(assignment, indent=2)
        + "\n"
    )


def session_json(session, cycle, task_prompt, goal, read_only=False):
    session, goal = Path(session), Path(goal)
    text = Path(task_prompt).read_text()
    refs = {str(p.relative_to(goal)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(goal.rglob("*")) if p.is_file()} if goal.is_dir() else {}
    info = {
        "method": "multiple_experience",
        "cycle": int(cycle),
        "task_prompt": Path(task_prompt).name,
        "task_prompt_sha256": hashlib.sha256(text.encode()).hexdigest(),
        "reference_sha256": refs,
    }
    if read_only:
        info["read_only"] = True
    (session / "session.json").write_text(json.dumps(info, indent=2) + "\n")
    print(json.dumps({"ok": True, "cycle": int(cycle), "references": len(refs), **({"read_only": True} if read_only else {})}))


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else ""
    if what == "prompt":
        sys.stdout.write(OPERATOR + assigned_round(sys.argv[2]))
    elif what == "readme":
        sys.stdout.write(README)
    elif what == "session-json":
        session_json(*sys.argv[2:6], read_only="--read-only" in sys.argv[6:])
    elif what == "prompt-ro":
        sys.stdout.write(operator_ro() + assigned_round_ro(sys.argv[2]))
    elif what == "readme-ro":
        sys.stdout.write(readme_ro())
    else:
        print(__doc__)
        raise SystemExit(2)
