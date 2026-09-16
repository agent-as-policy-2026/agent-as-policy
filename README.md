# Agent as Policy — Real Dual-Arm Manipulation with LLM Agents

<p align="center">
  <a href="https://agent-as-policy-2026.github.io/"><img alt="Project page" src="https://img.shields.io/badge/%F0%9F%8C%90%20Project-Page-1f6feb?style=for-the-badge"></a>
  <a href="https://arxiv.org/abs/2609.12541"><img alt="Paper" src="https://img.shields.io/badge/%F0%9F%93%84%20arXiv-2609.12541-b31b1b?style=for-the-badge"></a>
  <a href="https://huggingface.co/datasets/Agent-as-Policy/agent-as-policy"><img alt="Hugging Face dataset" src="https://img.shields.io/badge/%F0%9F%A4%97%20Dataset-Agent--as--Policy-ffd21e?style=for-the-badge"></a>
</p>

Code for the paper *Agent as Policy for Robotic Manipulation*. An LLM agent acts
directly as the policy on a real bimanual YAM robot: it reads camera
observations through a tool interface, issues Cartesian and joint commands, and
judges its own completion. No teleoperation, no learned low-level controller.

Videos, per-step frames, joint trajectories and full agent traces live on the
Hugging Face dataset linked above, not in this repository.

## What is here

| Path | Contents |
|---|---|
| `agp/` | Experiment harness: session server (`server_real.py`), the agent's CLI (`robot_client.py`), prompts and interface contracts, the runners, and the bridge launcher `start_bridges.sh` |
| `agp/tools/` | Session helpers and the metrics scripts that produce the paper tables |
| `agp/knowledge/`, `agp/paper_runs/` | The note/tool stores carried between trials, and the per-batch `results.csv` with before/after photos |
| `hardware-bridge/` | Safety-owned bridge that exclusively owns the arms and cameras — its own uv project, documented in `hardware-bridge/README.md` |
| `calib/` | Hand-eye and overhead-camera calibration procedures (`calib/README_runbook_zh.md`) |
| `third_party/graph-as-policy/` | The vendored connector package (`gap`, `gap_core`), Apache-2.0, pinned to one upstream commit (see its `UPSTREAM.md`) |
| `third_party/i2rt/` | Patch + pinned upstream commit for the robot SDK (see its `UPSTREAM.md`) |

The bridge ships as the Python package `agp_yam_bridge` with three console
scripts: `agp-yam-preflight`, `agp-yam-bridge`, `agp-yam-camera-acceptance`.

## Getting the robot SDK

The bridge depends on a patched i2rt checkout at `<repo>/i2rt` (its uv source is
`../i2rt`). Clone it there, pin the commit, apply our patch **without committing
it**, then copy the untracked additions; the exact commands are in
`third_party/i2rt/UPSTREAM.md`.

## Running

```bash
# 0. once per checkout, after the SDK step above: build the bridge environment. It is
#    also the interpreter the harness runs on, and it is what makes `gap` importable.
(cd hardware-bridge && uv sync --locked)

# 1. bring up a bridge (per arm; --source fake needs no hardware — see hardware-bridge/README.md)
bash agp/start_bridges.sh start left
bash agp/start_bridges.sh status

# 2. a goal set for the task: a photo of the goal state, or a recorded human demonstration
G=agp/goal_sets/$(date +%Y%m%d)_<task>; mkdir -p $G
cp <photo>.jpg $G/test_1.jpg && bash agp/capture_top.sh $G/top_camera.png
bash agp/record_demo.sh <task>

# 3. one supervised trial: it takes the before-photo, then waits for Enter before the arm moves
bash agp/run_trial.sh <task> <trial_no> [--effort high] [--model <slug>] \
     [--bare] [--right | --dual] [--knowledge task|ckpt|mx]
PAPER_DRYRUN=1 bash agp/run_trial.sh <task> 1   # print the resolved config, start nothing

# 4. results in agp/paper_runs/<task>_<batch>/ (results.csv, before/after photos);
#    the session (videos, joints, agent events) in agp/sessions/
bash agp/start_bridges.sh stop left
```

Large artefacts referenced by these steps (goal sets, demo videos, session
recordings) come from the Hugging Face dataset or from Drive.

## License

Code in this repository is Apache-2.0 (`LICENSE`); `NOTICE` carries the
third-party attributions that come with it. The dataset on Hugging Face is
CC BY 4.0. The vendored connector under `third_party/graph-as-policy/` is
Apache-2.0 and modified, upstream `LICENSE` and `NOTICE.md` preserved. The
vendored SDK under `third_party/i2rt/` remains MIT, upstream notice preserved.

## Citation

```bibtex
@article{jia2026agent,
  title   = {Agent as Policy for Robotic Manipulation},
  author  = {Jia, Mengzhao and Lin, Yang and Zhang, Xixin and
             Zhang, Zhihan and Liu, Xiaobai and Jiang, Meng},
  journal = {arXiv preprint arXiv:2609.12541},
  year    = {2026},
  doi     = {10.48550/arXiv.2609.12541},
  url     = {https://arxiv.org/abs/2609.12541}
}
```
