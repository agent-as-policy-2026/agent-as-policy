# Agent as Policy — Real Dual-Arm Manipulation with LLM Agents

Code for the paper *Agent as Policy for Robotic Manipulation* ([arXiv:2609.12541](https://arxiv.org/abs/2609.12541)). An LLM agent acts directly as the policy on a
real bimanual YAM robot: it reads camera observations through a tool interface,
issues Cartesian and joint commands, and judges its own completion. No
teleoperation, no learned low-level controller.

**Project page**: https://agent-as-policy-2026.github.io/

**Data**: videos, per-step frames, joint trajectories and full agent traces live on
Hugging Face, not in this repository.
→ https://huggingface.co/datasets/Agent-as-Policy/agent-as-policy

The dataset is public (CC BY 4.0). `side` camera video is
published separately: it has no per-frame alignment key, so it cannot be joined
to the joint trajectories the way the other three cameras can. `DATASET_b01.md`
maps the published files onto this repository and says how to regenerate them
with the scripts in `scripts/`.

## What is here

| Path | Contents |
|---|---|
| `agp/` | Experiment harness: session server (`server_real.py`), the agent's CLI (`robot_client.py`), prompts and interface contracts, the runners, and the bridge launcher `start_bridges.sh` |
| `agp/tools/` | Session helpers and the metrics scripts that produce the paper tables |
| `agp/knowledge/`, `agp/paper_runs/` | The note/tool stores carried between trials, and the per-batch `results.csv` with before/after photos |
| `hardware-bridge/` | Safety-owned bridge that exclusively owns the arms and cameras — its own uv project, documented in `hardware-bridge/README.md` |
| `calib/` | Hand-eye and overhead-camera calibration procedures (`calib/README_runbook_zh.md`) |
| `scripts/` | The Hugging Face export/verification pipeline, plus `data_sync.sh` for this repo's own large files |
| `snapshots/b01/` | The frozen session lists the published batch was cut from |
| `third_party/graph-as-policy/` | The vendored connector package (`gap`, `gap_core`), Apache-2.0, pinned to one upstream commit (see its `UPSTREAM.md`) |
| `third_party/i2rt/` | Patch + pinned upstream commit for the robot SDK (see its `UPSTREAM.md`) |
| `make_config_sha.sh`, `make_left_config_sha.sh` | Recompute the i2rt fingerprint that a bridge config pins |

Deliberately not in git: `i2rt/` (your patched SDK checkout, see below),
`agp/sessions/`, `agp/goal_sets/`, `agp/bridge_recordings*/`,
`hardware-bridge/logs/`, `calib/out/`, and every `*.mp4` / `*.npz` / `*.npy`.
`scripts/data_sync.sh` moves those between the working tree and Drive; the
`data_manifest.md5` index it uses is written by `push` and is not part of a
checkout.

The bridge ships as the Python package `agp_yam_bridge` with three console
scripts: `agp-yam-preflight`, `agp-yam-bridge`, `agp-yam-camera-acceptance`.

**Vendored connector.** The harness's session server reaches the bridge through
the connector package `gap` (upstream project `graph-as-policy`, Apache-2.0),
vendored at `third_party/graph-as-policy/`: the sixteen modules the real-arm
path needs, pinned to one upstream commit and modified. The bridge's uv project
depends on it as an editable path source, so `uv sync` in `hardware-bridge/` is
what makes `import gap.connector` work — no second checkout, no `PYTHONPATH`,
nothing to repoint per machine. Provenance, the per-file list of changes and
what was deliberately left out are in
`third_party/graph-as-policy/UPSTREAM.md`.

## Getting the robot SDK

The bridge depends on a patched i2rt checkout at `<repo>/i2rt` (its uv source is
`../i2rt`). Clone it there, pin the commit, apply our patch **without committing
it**, then copy the untracked additions; the exact commands are in
`third_party/i2rt/UPSTREAM.md`. The bridge's preflight fingerprints
`git diff HEAD` of that tree and refuses to serve when the hash does not match
the config's `tracked_diff_sha256` — which is why the patch must stay
uncommitted.

## Running

```bash
# 0. once per checkout, after the SDK step above: build the bridge environment. It is
#    also the interpreter the harness runs on, and it is what makes `gap` importable.
(cd hardware-bridge && uv sync --locked)

# 1. bring up a bridge (per arm; --source fake needs no hardware — see hardware-bridge/README.md)
bash agp/start_bridges.sh start left
bash agp/start_bridges.sh status

# 2. one supervised trial: it takes the before-photo, then waits for Enter before the arm moves
bash agp/run_trial.sh <task> <trial_no> [--effort high] [--model <slug>] \
     [--bare] [--right | --dual] [--knowledge task|ckpt|mx]
PAPER_DRYRUN=1 bash agp/run_trial.sh <task> 1   # print the resolved config, start nothing

# 3. the paper tables, from the batch results
python3 agp/tools/paper_table1_metrics.py --out /tmp/table1.csv
python3 agp/tools/paper_table2_metrics.py --out /tmp/table2.csv
```

Calibration comes before the first real trial: `calib/README_runbook_zh.md`.
`agp/README_zh.md` is the operator's manual for the harness
(Chinese): scene setup per task, knowledge modes, dual-arm sessions, failure
table. The two table scripts can cross-check against CSVs from the
paper-analysis workspace (`--detailed-runs` / `--per-run`, or the env vars
`AGP_DETAILED_RUNS_CSV` / `AGP_PER_RUN_CSV`); those files are not in this
repository and the cross-check is skipped when they are absent.

Large artefacts referenced by these steps (goal sets, demo videos, session
recordings) come from the Hugging Face dataset or from Drive.

## Safety

The arms move at full speed within a shared workspace. Every runner assumes a
human is present with the e-stop. The bridge enforces motion limits and fails
closed; do not bypass its preflight checks. The physical first-motion checklist
is in `hardware-bridge/README.md`.

## License

Code in this repository is Apache-2.0 (`LICENSE`); `NOTICE` carries the
third-party attributions that come with it. The dataset on Hugging Face is
CC BY 4.0. The vendored connector under `third_party/graph-as-policy/` is
Apache-2.0 and modified, upstream `LICENSE` and `NOTICE.md` preserved. The
vendored SDK under `third_party/i2rt/` remains MIT, upstream notice preserved.

## Citation

See `CITATION.cff`.
