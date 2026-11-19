<div align="center">

<img src="docs/assets/agp-logo.png" alt="Agent as Policy logo" width="128" height="128">

# Agent as Policy
### Agent as Policy for Robotic Manipulation

**General-purpose agents observe, program, act, and verify on real robots.**

Mengzhao Jia · Yang Lin · Xixin Zhang · Zhihan Zhang · Meng Jiang

<a href="https://agent-as-policy-2026.github.io/"><img alt="Project page" src="https://img.shields.io/badge/Project-Page-187764?style=for-the-badge"></a>
<a href="https://arxiv.org/abs/2609.12541"><img alt="Paper on arXiv" src="https://img.shields.io/badge/arXiv-2609.12541-b31b1b?style=for-the-badge"></a>
<a href="https://github.com/user-attachments/assets/788696b7-7461-4011-b58c-dce6ee930342"><img alt="Watch project video" src="https://img.shields.io/badge/Watch-Video-187764?style=for-the-badge"></a>
<a href="https://huggingface.co/datasets/Agent-as-Policy/agent-as-policy"><img alt="Hugging Face dataset" src="https://img.shields.io/badge/Hugging_Face-Dataset-ffd21e?style=for-the-badge"></a>

</div>

<p align="center">
  <a href="https://github.com/user-attachments/assets/788696b7-7461-4011-b58c-dce6ee930342"><img src="docs/assets/project-video-cover.jpg" alt="Watch Agent as Policy demonstrations, including bimanual towel folding, block construction, and assembly" width="100%"></a>
</p>
<p align="center"><b><a href="https://github.com/user-attachments/assets/788696b7-7461-4011-b58c-dce6ee930342">▶ Watch the project video</a></b> · 5 min 49 sec · Full task demonstrations and recorded agent execution</p>

<details>
<summary><b>▶ Play the project video here</b></summary>

https://github.com/user-attachments/assets/788696b7-7461-4011-b58c-dce6ee930342

</details>

**Agent as Policy (AgP)** uses a general-purpose agent as the policy for real-world robotic manipulation. Given a task and a documented robot interface, the agent interprets camera observations, writes executable programs, commands the robot, and revises its actions from physical feedback. Model parameters remain fixed during execution.

[**Demos**](#real-robot-demonstrations) · [**Method**](#how-it-works) · [**Results**](#results-from-the-paper) · [**Setup**](#getting-the-robot-sdk) · [**Running**](#running) · [**RoboDojo**](#robodojo-simulation) · [**Citation**](#citation)

## Real robot demonstrations

From deformable objects to precision assembly and dynamic motion. Click a preview to watch the corresponding task video on the project website.

<table>
  <tr>
    <td align="center" width="50%">
      <a href="https://agent-as-policy-2026.github.io/media/towel-simultaneous-run-8x-views.mp4"><img src="docs/assets/towel-preview.gif" alt="Two robot arms folding a blue towel, 4x excerpt" width="100%"></a><br>
      <b>Bimanual towel folding</b><br><sub>Coordinate two arms to fold a deformable object · 4× preview</sub><br>
      <sub><a href="https://agent-as-policy-2026.github.io/media/towel-sequential-run-8x-views.mp4">Method 1</a> · <a href="https://agent-as-policy-2026.github.io/media/towel-simultaneous-run-8x-views.mp4">Method 2</a></sub>
    </td>
    <td align="center" width="50%">
      <a href="https://agent-as-policy-2026.github.io/media/blocks-pyramid-8x-views.mp4"><img src="docs/assets/pyramid-preview.gif" alt="Robot building a six-block pyramid, 4x excerpt" width="100%"></a><br>
      <b>Block construction</b><br><sub>Build a six-block pyramid from a goal image · 4× preview</sub>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://agent-as-policy-2026.github.io/media/assembly-trial4-timelapse-8x-views.mp4"><img src="docs/assets/assembly-preview.gif" alt="Robot assembling matching parts, 4x excerpt" width="100%"></a><br>
      <b>Precision assembly</b><br><sub>Match and assemble four pairs of parts · 4× preview</sub>
    </td>
    <td align="center">
      <a href="https://agent-as-policy-2026.github.io/media/throw-probe-then-throw-8x-views.mp4"><img src="docs/assets/throw-preview.gif" alt="Robot throwing an object toward a target bowl at original speed" width="100%"></a><br>
      <b>Targeted throwing</b><br><sub>Release an object toward a target bowl · 1× preview</sub>
    </td>
  </tr>
</table>

Previews are selected excerpts from the task recordings. Linked task recordings play at 8× and may show a different trial of the same task. The [project page](https://agent-as-policy-2026.github.io/) also includes dice reorientation, alternative towel-folding strategies, and recorded agent sessions. Videos, per-step frames, joint trajectories, and full agent traces are available in the [dataset](https://huggingface.co/datasets/Agent-as-Policy/agent-as-policy).

## How it works

<p align="center">
  <a href="docs/assets/method-overview.webp"><img src="docs/assets/method-overview.webp" alt="Code as Policy compared with Agent as Policy, the agent robot interface, and real world manipulation tasks with success rates." width="100%"></a>
</p>

1. **Prepare the task.** A preparation agent turns the request and video, image, or language references into a reusable definition with goals, constraints, and completion criteria.
2. **Observe and program.** The execution agent reads camera images and robot state, estimates geometry, and writes programs to choose poses and compose actions.
3. **Act and verify.** The robot bridge executes joint or Cartesian targets. Fresh observations and motion feedback guide the agent's next decision.

Saved procedures and programs can be reused across trials. The [paper](https://arxiv.org/abs/2609.12541) describes the task setup, controller, and evaluation protocol.

## Results from the paper

- **Real-world task coverage.** Assembly from human videos, block construction from goal images, dice flipping, targeted throwing, and bimanual towel folding.
- **Task completion.** At least 8 successful trials out of 10 for each evaluated assembly, block-construction, and dice-flipping configuration.
- **Experience reuse.** Reusing saved procedures and programs shortens execution time across repeated trials.

See the [paper](https://arxiv.org/abs/2609.12541) for per-task results and experimental conditions.

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
| `robodojo/AGP/` | AgP on the RoboDojo simulation benchmark: a self-contained XPolicyLab policy adapter, independent of the code above (see [RoboDojo simulation](#robodojo-simulation)) |
| `robodojo/results/` | The RoboDojo results of that adapter: the `summarize` output of one complete seed |

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

## RoboDojo simulation

`robodojo/AGP/` runs AgP on the [RoboDojo](https://robodojo-benchmark.com)
simulation benchmark: codex with `gpt-6-astra` drives both ARX X5 arms through
MCP tools, from the official RGB images and proprioception only. It holds the
same files as the policy adapter submitted to XPolicyLab in
[XPolicyLab#182](https://github.com/XPolicyLab/XPolicyLab/pull/182) and uses none
of the code above.

Requirements: Ubuntu 22.04 x86_64, an NVIDIA GPU with RT cores (not A100/H100)
and at least 16 GB of memory, NVIDIA driver 570 or 580, conda, and either a
ChatGPT account with codex access to `gpt-6-astra` (device code login enabled in
its security settings) or an OpenAI API key with access to it.

```bash
# 1. RoboDojo, as in its install guide (https://robodojo-benchmark.com/doc/usage/install-and-download/)
sudo apt install libvulkan1 mesa-vulkan-drivers vulkan-tools git-lfs python3-yaml
git lfs install
git clone https://github.com/RoboDojo-Benchmark/RoboDojo.git && cd RoboDojo
bash scripts/install.sh -i                  # the conda environment RoboDojo
bash scripts/init_assets.sh                 # robot, object and layout assets into ./Assets
conda activate RoboDojo && python utils/update_embodiment_config_path.py

# 2. AgP as an XPolicyLab policy, and a codex login
cp -r <agent-as-policy>/robodojo/AGP XPolicyLab/policy/
cd XPolicyLab/policy/AGP
bash install.sh                             # venvs under ~/.agp-robodojo and the pinned codex
CODEX_HOME="$HOME/.codex" ~/.codex/packages/standalone/releases/0.160.1-x86_64-unknown-linux-musl/bin/codex \
    -c 'cli_auth_credentials_store="file"' login --device-auth
# or, with an OpenAI API key:
# printenv OPENAI_API_KEY | CODEX_HOME="$HOME/.codex" ~/.codex/packages/standalone/releases/0.160.1-x86_64-unknown-linux-musl/bin/codex \
#     -c 'cli_auth_credentials_store="file"' login --with-api-key

# 3. a check without the model, then one episode of the agent
EVAL_NUM=1 AGP_SCRIPTED=smoke bash eval.sh RoboDojo stack_blocks no-checkpoint arx_x5 joint 0 0 0 default RoboDojo
EVAL_NUM=1 bash eval.sh RoboDojo stack_blocks no-checkpoint arx_x5 joint 0 0 0 default RoboDojo
```

Results go to RoboDojo's `eval_result/`. [robodojo/AGP/README.md](robodojo/AGP/README.md)
has the command for a complete seed and the other options.

### Results (seed 0)

One complete seed of the official evaluation: all 54 configurations at their
native counts, 2,100 episodes, scored with RoboDojo's `scripts/robodojo.sh summarize`
([robodojo/results/seed0/_summary.md](robodojo/results/seed0/_summary.md)).

| | Generalization | Precision | Long-Horizon | Memory | Open | Average |
|---|---|---|---|---|---|---|
| success rate (%) | 54.67 | 44.00 | 28.75 | 91.00 | 62.25 | **56.13** |
| score | 59.79 | 50.90 | 49.48 | 92.22 | 65.75 | **63.63** |

Every episode, with its recordings, the agent's tool calls and the images it
received, is in the dataset
[Agent-as-Policy/agent-as-policy-robodojo](https://huggingface.co/datasets/Agent-as-Policy/agent-as-policy-robodojo).

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
             Zhang, Zhihan and Jiang, Meng},
  journal = {arXiv preprint arXiv:2609.12541},
  year    = {2026},
  doi     = {10.48550/arXiv.2609.12541},
  url     = {https://arxiv.org/abs/2609.12541}
}
```
