# AgP 真机操作手册（`agp/`）

LLM agent 直接作为策略操作真机双臂 YAM。agent 通过文件桥 CLI（`robot_client.py`）下命令：
`frames`（腕 D405 RGB-D + 顶 BRIO RGB + 标定）、`state`、`deproject`（深度 / 平面射线）、
`move_ee` / `move_delta` / `move_joints` / `home` / `gripper`。服务端（`server_real.py`）只做物理护栏
（可达环、桌面下限、单步位移），不含任何任务脚手架。

本目录不改 `hardware-bridge/`、`calib/` 与 `third_party/` 下的内容（对会话而言它们是只读依赖）。
命令若无特别说明都在 **`agp/` 目录**下执行；仓库 checkout 放在哪里都可以。

```
agp/
  server_real.py            真机服务端（默认观察专用；--allow-motion 才开运动）
  robot_client.py           agent 用的 CLI
  README_interface*.md      给 agent 的接口契约，复制进会话为 README_interface.md
                            （real / bare / dual / dual_bare / programs 五份）
  PROMPT_*.md               各任务的任务书（含 _bare、_dual、左右臂与 mx 变体）
  run_trial.sh              有人值守跑一次试验：所有运行方式的入口
  run_paper_pyramid.sh      单次试验的正式流程（前后顶视照、结果写 results.csv）
  run_experiment.sh         会话 + agent + 三路录像
  run_real_probe.sh         会话机：start / stop / status / rec-start / rec-stop / merge
  run_paper_batch.sh        无人值守批次（试验 → 机器人打散 → 检查 → 下一次）
  run_model_chain.sh        无人值守串联（组装 → agent 打散 → 组装 …）
  run_scatter.sh            机器人自己打散场景（不计入试验）
  record_demo.sh            用桥内录像器录一段人手示范 → goal_sets/
  capture_top.sh            经桥拍一张顶视照（只读，不动臂）
  consolidate_task_knowledge.sh   离线把已有会话整理成任务知识库
  start_bridges.sh          起 / 停 / 查看左右两个桥
  knowledge/                全局笔记与工具库（RIG_NOTES.md、playbooks/、tools/）
  mx/                       多经验模式（carry.py / prompt.py / experience.py）
  tools/                    会话与分析脚本（拍照、对焦门、清故障、指标、基座标定 …）
  config/                   top_calib.json、right_base_in_left_base.json
  paper_runs/<批次>/        results.csv、前后照片、本批工具库与知识库
  sessions/<ts>_<name>/     每次会话（不进 git）
  goal_sets/                目标集（不进 git；来自数据集或自己录）
```

## 0. 硬规则

1. **同一时刻只能有一个桥观测客户端。** 本服务端、`capture_top.sh`、`calib/` 的采集脚本互斥；
   启动脚本会 `pgrep` 拒绝，但只认得已知名字。
2. **桥独占该臂的顶相机与腕相机**，整个会话期间别的进程都打不开它们。agent 的 `frames` 只是按需经桥取单帧，
   不是连续录像；顶 / 腕全程录像由桥内录像器完成，侧视录像用第三台相机（USB2）。
   如果 `calib/preview_camera.py`（:8768）正占着侧视相机，会话机会自动改从预览的 MJPEG HTTP 流录。
3. 左桥 `--config config/left_arm.yaml`（端口 9021），右桥 `config/right_arm.yaml`（9022）；
   端口可用 `BRIDGE_PORT` / `BRIDGE_PORT_RIGHT` 改。
4. **每次运动会话都有人握急停。** 双臂运动会话要求左右两个急停都有人在手边。

## 1. 新机器部署

按顺序：

1. **机器人 SDK**：按 `third_party/i2rt/UPSTREAM.md` 在仓库根目录建 `i2rt/`（clone、pin commit、打 patch 但不提交）。
   桥的 preflight 会对 i2rt 工作树做指纹校验，与 yaml 里的 `tracked_diff_sha256` 不符就拒绝服务；
   i2rt 一变就要重算：`bash make_config_sha.sh hardware-bridge/config/<arm>.yaml`。
2. **桥环境**：`(cd hardware-bridge && uv sync --locked)`。这个 `.venv` 同时是本目录脚本用的解释器，
   也是让 `gap`（vendored 连接器）可 import 的唯一途径。
3. **硬件身份**：`hardware-bridge/config/left_arm.yaml` / `right_arm.yaml` 里的 CAN 通道、腕相机序列号与 USB 物理口、
   顶相机标定文件都是**本工位专有**的，换工位必须改；`agp-yam-preflight` 会逐项核对。
4. **标定**：按 `calib/README_runbook_zh.md` 做手眼与顶相机外参；顶相机标定结果同时写到
   `hardware-bridge/acceptance/top_left/`（桥用）和 `agp/config/top_calib.json`（打散检查等离线工具用）。
   双臂协同前还要标基座变换（第 9 节）。
5. **agent CLI**：`codex`（默认后端）、`claude`（Claude Code）或 `agy`（Antigravity）至少装一个并登录；`ffmpeg` / `ffprobe` 在 PATH。
6. **顶相机对焦**：左顶 BRIO 必须 autofocus=0 / focus=0 / zoom=100。`tools/check_top_focus.sh` 在各入口脚本开头自动检查，
   `FA_FIX_TOP_FOCUS=1 bash tools/check_top_focus.sh` 可自动恢复；右顶相机用 `--arm right`。
7. **第一次真机运行走第 3 节的只读验证**，通过后再开运动。

## 2. 起桥

```bash
bash start_bridges.sh start [left|right|both]   # i2rt 源、开运动、录像器待命（--record-dir 按臂分开）
bash start_bridges.sh start left throw          # 投掷专用左桥（config/left_arm_throw.yaml），见第 10 节
bash start_bridges.sh stop  [left|right|both]   # 干净的 SIGINT 退出（重力补偿 → 电机断电）
bash start_bridges.sh status
```
它自己算路径、查端口、检查顶相机对焦，日志在 `bridge_logs/bridge_<arm>_<ts>.log`；等到 `SERVING ... recording: armed` 即可。
有会话在跑时拒绝 stop。手工等价命令（cwd = `hardware-bridge`）：
```bash
uv run --locked agp-yam-preflight --config config/left_arm.yaml          # 期望 status: READY
uv run --locked agp-yam-bridge --config config/left_arm.yaml --source i2rt --acknowledge-i2rt-startup-motion \
    --enable-motion --acknowledge-first-motion-checklist --record-dir ../agp/bridge_recordings
```
只读桥去掉 `--enable-motion` 与 `--acknowledge-first-motion-checklist`。注意：只读桥仍会让夹爪在启动时自校准动一下；
臂处于重力补偿（软），可能缓慢下沉，起桥前先托住。桥的详细说明与首次运动检查表在 `hardware-bridge/README.md`。

## 3. 只读验证（新工位第一次）

1. CAN 上电，preflight 通过，起**只读**桥。
2. 手托臂到能俯视桌面的姿态，桌上只放一个已知边长的方块。
3. `bash run_real_probe.sh start ro_check` → 期望 `READY ... motion_enabled=False bridge_state=ok`。
4. 在会话目录逐条：`status` / `state` / `frames '{}'` → 看两张图 → 腕 `deproject` 方块顶面（z ≈ 桌面 + 边长，±6 mm）
   → 顶相机 `deproject` + `plane_z` → 与腕估计 xy 差 < 1.5 cm（> 3 cm 提示顶相机外参问题）
   → `move_delta` 期望被 `READ_ONLY` 拒绝且服务端存活。
5. `bash run_real_probe.sh stop`，确认录像可播放。托住臂，Ctrl-C 停桥。

`run_real_probe.sh` 的完整用法：
```bash
bash run_real_probe.sh start <name> [--allow-motion] [--prompt FILE] [--goal DIR] [--no-video] [--arms left,right|right] [-- <服务端参数>]
bash run_real_probe.sh status | rec-start | rec-stop | stop [SESSION]
bash run_real_probe.sh merge <workspace>       # 离线合并一个工作区的 knowledge_delta
```

## 4. 跑一次试验：`run_trial.sh`

```bash
bash run_trial.sh <任务名> <试验号> [选项]
PAPER_DRYRUN=1 bash run_trial.sh <任务名> 1 [选项]    # 只打印解析出的配置，不拍照不起会话；每次改动后先看这一行
```
它按任务名找最新的 `goal_sets/*_<任务名>`，先拍 before 顶视照，回车后才起 agent，跑完写
`paper_runs/<任务名>_<批次>/results.csv`（时长、超时、命令数、夹爪次数、token、cost、agent 自评；`operator_verdict` 由人填）。
两次试验之间由人打散场景、停好手臂，或用第 6 节的自动打散。

| 选项 | 含义 |
|---|---|
| `--effort low\|medium\|high\|xhigh` | 推理强度；给了它时批次名默认 `<full\|bare>_<effort>` |
| `--model <slug>` | 模型；批次名加标签（gpt-6-astra → `_6astra`，gpt-5.6-sol → `_56sol`，claude-opus-5 → `_copus5`，claude-fable-5-1 → `_cfable51`，gemini-3.1-pro-high → `_g31prohigh` …） |
| `--backend codex\|claude\|agy` | agent CLI；默认 codex |
| `--bare` | 裸接口消融（第 11 节）；批次名 `bare…` |
| `--harness yield\|yield,batch` | 给接口文档追加 "tool harness" 段（第 11 节）；批次尾缀 `_yield` / `_yieldbatch` |
| `--knowledge task\|task-ro\|ckpt\|ckpt-ro\|mx\|mx-ro` | 知识 / 经验模式（第 7 节） |
| `--knowledge-from <批次>` | 用另一批次的库播种本批次（只在本批次库不存在时复制一次，来源记在 `SEEDED_FROM.txt`） |
| `--batch NAME` | 显式批次名 |
| `--right` | 右臂单独试验（右桥 9022、右顶相机、位姿在 right_base）；批次尾缀 `_right`；可与左臂试验同时跑 |
| `--dual` | 双臂会话（第 9 节）；批次尾缀 `_dual` |
| `--nocount` | 场景重置运行：一切照录，但会话名前缀 `scatter_`，从不计入统计 |

环境变量透传：`PAPER_TIMEOUT_MIN`（默认 90）、`PAPER_MODEL`、`PAPER_FAST`（codex service tier fast）、`PAPER_PROMPT`、`PAPER_BATCH`、
`PAPER_ARMS`、`PAPER_KNOWLEDGE`、`PAPER_HARNESS`。

**任务书与目标集的解析**：任务书优先 `PROMPT_<task>_left|_right.md`（单臂）/ `PROMPT_<task>_dual[_bare].md`（双臂），
再 `PROMPT_<task>[_bare].md`，都没有就用通用的 `PROMPT_stack_blocks[_bare].md`。目标集目录要有 `test_1.jpg`、`top_camera.png`
或 `demo_*.mp4` 之一。

**后端**：
- `codex`：`codex exec --json`，`--effort` 对应 model_reasoning_effort，`PAPER_FAST=1` 对应 service_tier fast；可用模型以 `~/.codex/models_cache.json` 为准。
- `claude`：Claude Code print 模式（`claude -p --output-format stream-json --no-session-persistence --disable-slash-commands --dangerously-skip-permissions`），
  任务书从 stdin 喂入。会话目录对它是一个全新项目，不加载任何 memory / skill。
- `agy`：Antigravity，模型名自带强度（`agy models`），`--effort` 不传。它每轮重传所有看过的图片，可能很慢。

**会话里的 agent 文件**（与后端无关）：`agent.pid`、`agent_events.jsonl`、`agent_stderr.log`、`agent_last_message.txt`；
`session.env` 记 `AGENT_BACKEND/AGENT_MODEL/AGENT_START_S/AGENT_END_S/INTERFACE/HARNESS/ARMS`。
`python3 tools/agent_transcript.py <会话目录>` 把任一后端的事件流整理成 `agent_transcript.md`；
`tools/agent_usage.py` 按后端归一 token 用量（codex 取最后一个 `turn.completed`，claude / agy 取 result 事件）；
被超时杀掉的 codex 会话没有 `turn.completed`，可到 `~/.codex/sessions/<日期>/rollout-*.jsonl` 找最后一条 `token_count`。

**底层入口**（run_trial.sh 之下）：
```bash
FA_YES=1 bash run_experiment.sh <名字> --allow-motion --prompt PROMPT_x.md --goal goal_sets/<集合> \
    [--timeout-min 150] [--backend codex|claude|agy] [--model M] [--effort E] [--fast]
```
它依次：起服务端（带 knowledge/ 与 prompt）→ 发射 agent → 三路录像同时开始 → 等 agent 退出或超时 → 三路同时停止
→ 停服务端（租约掉落 → **臂变软、夹爪中的物体会掉**）→ 合并知识 → 打印摘要。
`FA_KNOWLEDGE=all|notes|tools|none` 控制复制进会话的全局笔记 / 工具库；`FA_TOOLS_DIR` 指定工具库目录。

## 5. 目标集

**照片目标集**（叠方块等）：手机照 `test_1.jpg` + 顶相机同场照：
```bash
G=goal_sets/$(date +%Y%m%d)_<名字>; mkdir -p $G && cp <手机照>.jpg $G/test_1.jpg && bash capture_top.sh $G/top_camera.png
```
`capture_top.sh` 只读观测、不动臂，要求没有别的桥观测客户端在跑；`--wrist` 同时存腕相机。

**示范视频目标集**（叠毛巾、装配等）：
```bash
bash record_demo.sh <任务名>      # 回车拍 demo_start.png 并开录 → 人在顶相机下操作 → 回车停录并拍 top_camera.png
```
产出 `goal_sets/<日期>_<任务名>/{demo_top.mp4, demo_start.png, top_camera.png}`（同名已存在自动加 `_v2_`；
录像器同时录下的腕相机视频、帧时间戳与 joints.csv 放在 `raw/`，不会复制进会话）。要求：桥带 `--record-dir`、无会话在跑、
**两只臂都停在顶相机视野之外**（脚本不动臂）。同一任务的第二种做法就换个任务名（如 `towel2`）：批次、results.csv、知识库全部独立，
`goal_sets/*_towel` 的通配不会匹配 `_towel2`。

agent 在会话里用 ffmpeg / Pillow 从示范视频截帧（会话里有 `/usr/bin/ffmpeg` 与系统 python3 的 Pillow，没有 cv2 / numpy）。

## 6. 无人值守

**串联 `run_model_chain.sh`**：一臂上"组装 → agent 打散 → 组装 → …"，每轮都是普通的 `run_trial.sh` 运行（全部录像、results.csv），
打散用 `twopairsreset` 任务、`--nocount`，编号自动取该打散批次的下一个空号。打散 agent 没报成功或某次运行异常退出就停下等人（回车继续）；
最后一轮之后也打散一次（`--no-final-reset` 关掉）。日志 `paper_runs/<task>_<batch>/chain_<时间>.log`。
```bash
bash run_model_chain.sh <task> <起始轮> <轮数> --model <slug> --effort <lvl> [--backend codex|claude|agy] [--right] [--knowledge mx] [--no-final-reset]
```
打散任务书 `PROMPT_twopairsreset_left|_right.md`，目标集 `goal_sets/*_twopairsreset`（人拆开摊平后录的示范）。

**方块批次 `run_paper_batch.sh <from> <to>`**：每次 = `run_paper_pyramid.sh n` → `run_scatter.sh n`（机器人拆塔并打散，不录像）→ 顶视自动检查 → 下一次。
打散目标布局由 `tools/scatter_layout.py` 按批次 + 试验号的种子随机采样；检查 `tools/check_scatter.py`（顶相机照 + `config/top_calib.json`，
HSV 分块反投到方块顶面）**假设 3 青 + 3 灰共六块**，换物体要改颜色阈值、期望数量与任务书里的物体描述。不通过或打散失败则停下等人修场景后回车。
```bash
PAPER_TASK=<任务> PAPER_GOAL=goal_sets/<集合> PAPER_FAST=0 BATCH_SCATTER_FIRST=0|1 BATCH_SCATTER_LAST=1 \
  setsid nohup bash run_paper_batch.sh <from> <to> </dev/null >> bridge_logs/batch_<任务>.log 2>&1 &
```
`FIRST=1` = 桌上是搭好的成品，先让机器人拆散。可调：`PAPER_TIMEOUT_MIN`、`SCATTER_TIMEOUT_MIN`、`PAPER_MODEL/PAPER_EFFORT`、`PAPER_BATCH`。

监控进程若用 `pgrep -f` 匹配自身命令行会永不触发，用 PID 文件。

## 7. 知识与经验模式

| 模式 | 库的位置 | 每轮开始 | 每轮结束 |
|---|---|---|---|
| `FA_KNOWLEDGE=notes/tools/all`（全局） | `knowledge/{RIG_NOTES.md,playbooks/,tools/}` | 复制进会话并附"前人内容是假设"的元说明 | 合并事实与 playbook；通过静态检查的工具入库 |
| `--knowledge task` | `paper_runs/<task>_<batch>_kn/knowledge/` | 整库复制进会话 | agent 写的 knowledge_delta 机械追加 |
| `--knowledge task-ro` | 同上 | 同上 | 不注入"写 delta"段，什么都不合并 |
| `--knowledge ckpt` | `paper_runs/<batch>/experience/cycle_NN/` | 全部已提交检查点拷进会话 + `experience.py` | 必须存一份检查点（`lesson` 四段 + 至少一个 `scratch/` 脚本快照），否则下一轮不许开跑 |
| `--knowledge ckpt-ro` | 同上 | 同上 | 不注入"存检查点"段，不提交 |
| `--knowledge mx` | `paper_runs/<batch>/mx/` | `mx/carry.py install`：检查点 + 共享 scratch + episodes / command_metrics | 只有确实存了检查点 `mx/carry.py save` 才写回；没存则交接闸拦下下一轮 |
| `--knowledge mx-ro` | 同上 | 同上 | `experience` 只能读，提交被拒（`READ_ONLY_EXPERIENCE`） |

- 只读模式必须先有库（`--knowledge-from <批次>` 播种，或库已存在）。
- 工具入库的静态检查：七行固定头（tool / category / purpose / usage / inputs-outputs / assumptions / verified）齐全、类别合法、
  不 import `gap`、无 `/home/` 绝对路径、无会话 / 帧引用、≤ 32 KB；同名不同内容存为 `_v2`、`_v3`，从不覆盖。
- `mx` 模式下服务端多两条命令 `episode`（阶段起止标记）和 `experience`（读 / 存检查点），都不计数；end 必须带最新一次 `frames` 的 capture
  且之后没有运动、不超过 30 s，success 还要求空爪张开；第 1–4 轮存完检查点后不许再运动。任务书用 `PROMPT_<task>_left_mx.md`。
  进度与交接闸：`cat paper_runs/<batch>/mx/experience/metrics.csv`、`python3 mx/carry.py check paper_runs/<batch>/mx <n>`。
- 离线整理已有会话成初始任务知识（codex 只读报告 / 脚本 / 命令日志，不碰机器人）：
  `bash consolidate_task_knowledge.sh <task> paper_runs/<task>_<batch>_kn sessions/<s1> sessions/<s2> ...`

## 8. 录像与数据

会话目录 `sessions/<ts>_<name>/`：
```
bridge/  frames/  server.log  server.pid  server_boot.log  record/           服务端与 agent 命令 / 帧
README_interface.md  PROMPT.md  goal/  scratch/  knowledge/  session.env
run_video_side.mp4  run_video_top.mp4  run_video_wrist.mp4                   三路录像
run_joints.csv  run_video_top_frames.csv  run_video_wrist_frames.csv          关节 50 Hz + 帧时间戳
agent_events.jsonl  agent_transcript.md  agent_last_message.txt
```
桥在录像窗口（SIGUSR1 → SIGUSR2）内写 `joints.csv`（50 Hz：wall_time_ns、monotonic_ns、sequence、q0–q6、v0–v6、eff0–eff6，q6 为夹爪开度分数）
和 `top_frames.csv` / `wrist_frames.csv`（每帧送入 ffmpeg 的帧序号与时间），`rec-stop` 移入会话；桥没带 `--record-dir` 时顶 / 腕退化为服务端 2 s 间隔的 timelapse。
agent 的高层动作在 `bridge/req_*.json`（带时间戳），桥侧每个动作的起止与 20 ms 反馈在 `hardware-bridge/logs/actions[_right].jsonl`。
双臂会话另有 `bridge_right/ frames_right/ server_right.* record_right/ run_joints_right.csv run_video_right_wrist.mp4`。

论文表：`python3 tools/paper_table1_metrics.py --out t1.csv`、`tools/paper_table2_metrics.py --out t2.csv`。

## 9. 双臂与右臂

**概念**
- 左臂：桥 `config/left_arm.yaml` :9021，顶相机外参在 left_base。右臂：桥 `config/right_arm.yaml` :9022，手眼取共享 station 模型的
  `right_gripper -> right_camera`；桥 schema 强制要有顶相机，所以配了右工位自己的顶 BRIO（`hardware-bridge/acceptance/top/`）。
- **agent 只看到一个世界系 = left_base。** 右桥在 right_base 里说话；`server_real.py --arm right` 对每个进出的位姿做换算，
  r / z / 单步护栏在 right_base 里评估。`T_left_right` 存在 `config/right_base_in_left_base.json`
  （`{"translation":[x,y,z],"rotation_wxyz":[w,x,y,z],"source":"...","date":"..."}`，示例见 `.example.json`）时以文件为准，
  否则用名义值平移 (0, −0.61, 0)、单位旋转；服务端**启动时**读取。
- 双臂会话里右服务端只暴露 `wrist`；要 `top` 会被拒并提示用左臂。右臂单独会话（`--right`）则把右顶相机作为该会话的 `top`。
- agent 侧语法 `python3 robot_client.py . --arm right <cmd> ['<json>']`，省略 `--arm` = left。两臂预算分别计数；
  系统**不做任何双臂协调**，防撞是 agent 的责任（写在双臂 README 里）。

**命令**
```bash
bash run_real_probe.sh start dual_ro --arms left,right                    # 只读彩排：两臂 status/state/frames
bash run_real_probe.sh start dual_t1 --arms left,right --allow-motion --prompt PROMPT_x_dual.md --goal goal_sets/<集合> [-- --bare]
bash run_trial.sh <task> 1 --dual [--bare]                                # 正式试验；README_interface_dual[_bare].md + PROMPT_<task>_dual[_bare].md
bash run_trial.sh <task> 1 --right --effort high                          # 右臂单独试验，可与左臂同时跑
```
守卫按端口区分：`run_real_probe.sh` 与 `capture_top.sh` 的"另一个观察者"检查只看同一端口的 server_real；
`capture_top.sh`、`record_demo.sh` 只走左桥（顶相机在左）。

**基座标定 `tools/solve_base_transform.py`**：推荐用现成的顶相机外参标定链跑 `--rig cross`（右臂 + 左顶相机，见 `calib/README_runbook_zh.md` §8e），
得到 `calib/out/top_brio_<serial>_in_right_base_calibration.json`，然后
```bash
../hardware-bridge/.venv/bin/python tools/solve_base_transform.py --top-in-right-base <该文件> --check   # 先看与名义值的差
../hardware-bridge/.venv/bin/python tools/solve_base_transform.py --top-in-right-base <该文件>           # 写 config/right_base_in_left_base.json
```
平移应在几 cm 内、旋转 1–2° 内；差得离谱先查四元数顺序 wxyz。重开右服务端生效，`--arm right status` 的 `base_in_world` 应等于文件值；
只读会话里比对 `--arm right state` 的 ee_pose 与尺量位置再开运动。两臂各干各的时名义值够用，两臂要碰同一物体就要标。

## 10. 投掷任务 `throw`

需要带缓冲关节程序的左桥变体（`config/left_arm_throw.yaml`：只多了程序内 J4 的速度 / 加速度上限），接口文档 `README_interface_programs.md`。
```bash
bash start_bridges.sh stop left && bash start_bridges.sh start left throw   # 换桥时臂会短暂失力，先扶好
bash run_trial.sh throw 1                                                    # --programs 自动加上；桥不支持会直接 BOOT_ERROR
bash start_bridges.sh stop left && bash start_bridges.sh start left          # 结束后换回普通左桥
```
第一次真机运行只让 agent 做低速 J4 探测，确认跟踪和夹爪响应正常再放开到 180°/s。

## 11. 裸接口 `--bare` 与 `--harness`

`--bare` 只改四样：① 服务端 `--bare`：`deproject`、`home`、`move_delta`、`reset` 不存在，help / 状态 / 错误文本不带任何解释与建议；
② 接口文档换成 `README_interface_bare.md`（只有调用约定、坐标系、包络、命令参数与返回字段、相机与标定文件格式、预算、安全一句话）；
③ 任务书用 `PROMPT_<task>_bare.md`，只留任务陈述、完整性规则、预算与交付物；④ 不继承工具。
不变的：桥及其全部安全行为、护栏（r 0.12–0.65 m / z ≥ 桌面 −5 mm / 单步 0.25 m，静默执行）、命令与时间预算、录像与关节记录、目标集。
`results.csv` 的 `interface` 列记 full / bare。

`--harness yield` 给接口文档追加"机器人命令会阻塞，每条含机器人命令的 exec 调用都设 yield_time_ms 90000，不要轮询"；
`yield,batch` 再加"一次调用可串多条命令并看图"。默认 none 时文档逐字节不变；`results.csv` 记 `harness` 与 `harness_version`。

## 12. 服务端参数（物理护栏，均可覆盖）

`--r-min 0.12 --r-max 0.65 --z-min -0.050 --z-max 0.60 --max-step-m 0.25 --budget 500 --observe-joints <6 rad>`，
通过 `run_real_probe.sh start <name> -- --max-step-m 0.15` 传入（双臂时两臂都吃）。

## 13. 中途终止与故障

**正确的终止方式**：Ctrl-C 跑 `run_experiment.sh` / `run_trial.sh` 的那个终端（trap 会 rec-stop + stop），
或 `bash run_real_probe.sh rec-stop <会话> && bash run_real_probe.sh stop <会话>`。桥内录像器只认 SIGUSR1/2，
删会话目录不会让它停；`rec-start` 若发现桥仍在为旧会话录会先 SIGUSR2 再重试。不要在录像器可能还在录时删会话目录。

| 现象 | 含义 / 处理 |
|---|---|
| `SOURCE_ERROR: camera sequence ... did not advance` | 有第二个观测客户端在读桥 |
| `BOOT_ERROR: --allow-motion requested but the bridge is read-only` | 桥没开 `--enable-motion` |
| 桥启动 `could not open fixed RGB camera ...` | 有进程占着顶相机（preview / 上一次桥没退干净） |
| 响应 `error_code: DISCONNECTED` | 桥进程断开；服务端下一条命令会尝试重连 |
| `recorder.log` 报 `Device or resource busy` | 侧视相机被 preview 占用 |
| `YAM bridge was not ready within 30.0s`（桥明明在 SERVING） | 会话在动作进行中被 Ctrl-C，桥看门狗锁存了 `command heartbeat timed out`。臂在重力补偿怠速。`bash tools/clear_bridge_fault.sh` 先看状态，确认后加 `--yes`（右臂再加 `--right`），不用重启桥 |
| `WARN: bridge did not confirm recording` | 桥仍在为旧会话录像，见上面的终止方式 |
| 多次 `SETTLE_MISS:` | 笛卡尔到位容差偏紧；见 yaml `cartesian_position_tolerance_m`，右臂默认比左臂松 |
