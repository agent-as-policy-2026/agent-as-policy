# vendored graph-as-policy (connector subset)

This directory is **not** a complete copy of graph-as-policy. It contains only
the 16 Python modules the AgP session server (`agp/server_real.py`) needs in
order to talk to the hardware bridge, plus the upstream licence notices and the
packaging shim that makes them importable. Everything else upstream ships —
the simulation stack, the agent/codegen runtime, the IK stack, the visualiser,
the four git submodules — is deliberately absent (§10).

- Upstream: https://github.com/graph-robots/graph-as-policy
- Pinned commit: `f09e37e7755d0a908c1b6cf136edf13292b730c0`
  ("Delete scripts directory", 2026-07-07, Kaiyuan Eric Chen, branch `main`)
- Licence: Apache-2.0 (see `LICENSE`, upstream's file preserved byte-for-byte,
  sha256 `cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30`).
  `NOTICE.md` is upstream's file, also unmodified.
- **This copy is modified.** 7 of the 16 files differ from upstream
  (6 of them already diverged in the working copy this was vendored from,
  1 was trimmed by the vendoring itself); 2 more do not exist upstream at all;
  7 are byte-identical to upstream. Per-file detail in §8.
- Upstream is self-declared BETA: "APIs, the workflow schema, and skill
  interfaces may change without notice between releases"
  (upstream `README.md`). That is a second reason the commit is pinned.

## Why source is vendored here, and not a patch like `third_party/i2rt/`

`third_party/i2rt/` ships a patch because the bridge's preflight fingerprints
`git diff HEAD` of the i2rt checkout and refuses to serve when the hash moves;
committing those changes would empty the diff. The connector has no such
constraint — it is an ordinary imported Python package. Vendoring the source
directly is what makes this repository clone-and-run, which was the point of
the exercise. What is copied from the i2rt precedent is the *documentation and
verification* structure, not the distribution mechanism.

## How it is wired in

`hardware-bridge/pyproject.toml` depends on `agp-vendored-gap-connector` and
resolves it from `[tool.uv.sources]` as an editable path dependency on this
directory. After a plain `uv sync` in `hardware-bridge/`,
`hardware-bridge/.venv/bin/python -c "import gap.connector"` works with no
`PYTHONPATH` and no `sys.path` manipulation.

It is a **separate distribution**, not part of `agp_yam_bridge`: the vendored
Apache-2.0 code never becomes part of AgP's own package, and its attribution
stays separable. The import names are upstream's (`gap`, `gap_core`) so this
copy remains diffable against upstream and so the harness's import lines are
unchanged. The on-disk layout is upstream's too — `gap/` at this directory's
root, `gap_core/` under `gap-core/src/` — for the same reason.

## Licence status, recorded as found (two upstream contradictions)

Both are upstream's, and neither is "corrected" here. They are reported so a
reader is not surprised by them:

1. `NOTICE.md` line 3 says "gap and open-robot-skills are released under the
   **MIT** License (see LICENSE)" — but the `LICENSE` file it points at is the
   Apache-2.0 text, and upstream's `README.md` and `pyproject.toml`
   (`license = { text = "Apache-2.0" }` plus the Apache classifier) all say
   Apache-2.0. Three sources to one, and the licence *file* is authoritative:
   this copy is treated as Apache-2.0.
2. Upstream's `gap-core/pyproject.toml` declares `license = { text = "MIT" }`
   for the `gap_core` sub-package, which has no LICENSE file of its own; the
   repository-level LICENSE is Apache-2.0. `gap_core.errors.ToolError` — which
   AgP imports — belongs to that sub-package. Both readings permit vendoring;
   this copy follows the stricter one (Apache-2.0) for all of it.

**Copyright holder:** upstream's LICENSE appendix is the unfilled template
(`Copyright [yyyy] [name of copyright owner]`), and not one of upstream's 126
Python files carries a per-file copyright or SPDX header. So there is no
upstream copyright line to reproduce and none is invented here. The only
attributions upstream offers are `pyproject.toml`'s
`authors = [{ name = "graph-as-policy authors" }]` and the commit author of the
pinned revision.

## Upstream f09e37e has no YAM support at all

`git ls-tree -r --name-only f09e37e | grep -i yam` matches six `.yaml`
filenames and no YAM source file. Upstream's `gap/connector/real.py` is 285
lines and contains no `YamRealConnector`, no `bridge_quat_wxyz_to_sim`, no
bridge client; the file here is 1395 lines. The entire YAM Ultra capability —
the bridge protocol client, the remote i2rt IK backend, the TCP-frame
conventions, dual-arm routing — was added by the AgP authors (§8 C).
Read that way, "vendored connector" understates
it: most of what runs on the real path is ours, sitting on upstream's
`Connector` base.

## Attribution for the files that are not upstream's

- `gap/envs/yam_real_env.py` — the P4/P5 bridge client — was written by the
  AgP authors (not upstream code), adapted for this tree, and is published
  here under Apache-2.0.
- `gap/envs/robot_specs.py` was written by the AgP authors (not upstream
  code).
- Both live under `third_party/` only because they must be importable as
  `gap.envs.*` for upstream's connector to find them. Each carries a header
  saying it is not upstream code.

## 8. Per-file classification

### A — upstream files, modified (7)

Each of these carries the Apache-2.0 §4(b) modification header above its module
docstring. Diffstats are against `f09e37e`.

| file | vs upstream | what changed |
|---|---|---|
| `gap/connector/real.py` | +1123 / −13 (285 → 1395 lines) | the whole YAM real-arm path: `YamRealConnector`, `RemoteI2rtKinematicsBackend`, `bridge_quat_wxyz_to_sim` / `sim_quat_wxyz_to_bridge`, min-jerk waypoint timing and densification, the `yam_left` / `yam_right` branches of `real()` |
| `gap/connector/core.py` | +240 / −69 | robot identity plumbed through `EnvConfig` / `RobotSpec`, tool-registry overrides and default kwargs, real-arm accommodations in the shared `Connector` base |
| `gap-core/src/gap_core/tools/_registry.py` | +72 / −2 | `register_callable(replace=True)`, prefix default kwargs, a `robot_spec` slot, idempotent bundle drains |
| `gap/env_config.py` | +53 / −0 | `sim_robot()` (`GAP_SIM_ROBOT`) plus the codegen-backend knobs `GAP_BACKEND`, `GAP_LLM_REASONING_EFFORT`, `GAP_CODEX_BIN`, `GAP_CODEX_TRACE_DIR` |
| `gap/envs/registry.py` | +52 / −2 | `EnvConfig` gained the robot-identity fields mirrored from `robot_specs` (Panda-equivalent defaults) and the `yam_real` registration |
| `gap-core/src/gap_core/types.py` | +7 / −2 | `CameraFrame.depth` became `NotRequired` — the bridge contract for RGB-only exterior cameras |
| `gap/connector/__init__.py` | +0 / −5 | **introduced by the vendoring itself**: the eager `collector` and `sim` imports and their `__all__` entries (`DataCollector`, `SimConnector`, `sim`) were removed so this subset imports without h5py or the sim stack. The module docstring is upstream's and still shows a `gap.connector.sim(...)` example; it was left untouched because these docstrings are runtime documentation. |

### B — upstream files, byte-identical (7)

No modification header on these; adding one would be a false statement.

- `gap/__init__.py`
- `gap/envs/__init__.py`
- `gap/envs/base_env.py`
- `gap-core/src/gap_core/__init__.py`
- `gap-core/src/gap_core/errors.py`
- `gap-core/src/gap_core/tools/__init__.py`
- `gap-core/src/gap_core/tools/schema.py`

`gap/__init__.py` and `gap/envs/__init__.py` keep their full upstream
`_LAZY_ATTRS` tables, so `gap.agent`, `gap.benchmark`, `gap.viz`,
`gap.envs.libero_env` … raise `ModuleNotFoundError` here instead of resolving.
Likewise `gap/envs/registry.py` keeps its upstream `register_env` rows for the
LIBERO / Franka / UR envs, so `resolve("libero")` raises `ModuleNotFoundError`
rather than the registry's own `KeyError`. Both were left alone on purpose:
trimming them would have meant editing two more upstream files for cosmetics.
The only name the harness resolves is `yam_real`.

### C — written by this project, absent from upstream (2)

| file | lines | origin |
|---|---|---|
| `gap/envs/yam_real_env.py` | 1348 | the AgP authors (not upstream code), adapted here |
| `gap/envs/robot_specs.py` | 629+ | the AgP authors (not upstream code) |

`gap/envs/robot_specs.py` is the one file changed during vendoring beyond the
copy: its `_YAM_ULTRA_URDF` path arithmetic was removed. Upstream of this copy
it computed `Path(__file__).resolve().parents[3] / "custom_tasks" / …`, which
climbed one directory **above** the connector checkout into a private
workspace, was never `.exists()`-checked, and would have shipped a dangling
absolute path in a public repository. Both YAM specs now set
`robot_urdf_path=None`; the field's annotation was widened to `str | None` and
the then-dead `from pathlib import Path` dropped. This is behaviour-preserving
on the real path: YAM IK is solved remotely inside the hardware bridge
(`RemoteI2rtKinematicsBackend`, wired at construction so `Connector._ik` is
never `None`), and `robot_urdf_path` is read only by the in-process cuRobo /
PyRoKi branches that the YAM path never enters — both of which already treat a
falsy value as "no URDF override". The reasoning is repeated in a comment at
the change site.

## 9. Verification

- `file_digest.txt` — sha256 of every file in this directory, one per line,
  relative paths, sorted. `file_digest_aggregate.txt` is the sha256 of that
  file. `./verify_gap.sh` recomputes both and diffs them.
- What this proves: that the bytes here are the bytes that were reviewed and
  published, and that nothing was edited afterwards. Digest checks are
  content-only and carry no git dependency, so unlike `third_party/i2rt`'s
  patch fingerprint they do not depend on your git version or configuration.
- What it does **not** prove: nothing here can be re-derived from upstream.
  Two of the files do not exist at `f09e37e`, and a clean upstream clone is
  missing 1123 lines of `gap/connector/real.py` — `git clone` + `git checkout
  f09e37e` produces a tree that cannot run this harness. The pinned commit
  lets you reconstruct the *delta*, not the copy; `git diff` it against the
  A-class files to see exactly what was added on top.
- `verify_gap.sh` does not check the licence files' hashes against upstream
  for you. The expected `LICENSE` sha256 is quoted near the top of this file.

## 10. Deliberately left out

- **The sim connector path** — `gap/connector/sim.py`, `collector.py`,
  `world_adapter.py`, `gap/runtime/verify/**`. Nothing in this repository
  constructs a `SimConnector` or a `DataCollector`; dropping them also drops
  an eager `import h5py`, which in this venv is a dev-group-only package.
- **The IK stack** — `gap/connector/ik.py`, `_ik_jax.py`. This is the
  decisive omission: the YAM connector is always built with
  `ik=RemoteI2rtKinematicsBackend(env)`, so `Connector.ik`'s lazy import never
  fires and every YAM IK call goes to the bridge. Keeping it would have
  dragged in jax, pyroki, jaxls, jaxlie, yourdfpy, robot_descriptions and an
  optional cuRobo import.
- `gap/connector/rr_launcher.py` — reached only from the `robot == "franka"`
  branch of `real()`.
- `gap/connector/yam_video.py` — a connector-side video recorder written by
  this project (not upstream either). `agp/server_real.py` forbids its use in
  so many words ("Exactly ONE bridge observation client at a time; never
  `conn.start_video`"), and no call site exists in this repository.
- **The other ~110 upstream modules** — agent/codegen, benchmark, viz,
  builder, runtime/execute, the skill bundles, the LIBERO / Franka / UR envs.
  None is on the real-YAM import path.
- **Upstream's `pyproject.toml` / `gap-core/pyproject.toml`** — they declare
  torch, mujoco, robosuite, LIBERO and the rest for a framework that is not
  here. The local `pyproject.toml` declares only what these 16 modules import
  (numpy, scipy, msgpack, msgpack-numpy), each of which was already a
  hardware-bridge dependency.
- **`uv.lock` (1.3 MB), `.venv/`, `docs/`, `examples/`, `benchmark_runs/`,
  `outputs/`, `gap/viz/frontend/dist/`, and the four git submodules**
  (LIBERO-PRO, robosuite, Variational-Automation-Benchmark,
  robots_realtime) — build artefacts, documentation for absent code, or a
  simulation stack this repository does not use. Upstream's `NOTICE.md` is
  kept whole anyway, so its third-party table still names them even though
  they are not vendored here.
