# 已发布实验清单（真机试次、判定口径与发布边界）

本文件记录随本仓库公开的真机实验到底包含哪些内容：有哪些任务 / 模型格子、每个格子实际跑了多少条、
每条的成败是怎么判定的、哪些会话没有进入数据集发布。

所有数字只来自三处可核对的文件，本文件不引用任何仓库之外的工作区：

| 来源 | 内容 | 覆盖范围 |
|---|---|---|
| `<batch>/results.csv` | 逐条试次的运行记录：会话名、模型、档位、接口、时长、是否超时、token、自评与人工判定 | 全部批次，含数据集冻结之后的试次 |
| `table1_metrics.csv` / `table2_metrics.csv` | 论文表格口径的指标，由 `../tools/paper_table1_metrics.py` / `paper_table2_metrics.py` 生成 | 只覆盖进入论文表格的那部分试次 |
| `../../snapshots/b01/{batch.json,sessions.txt,released.txt,excluded.txt,labels.csv}` | b01 数据集发布的会话清单与每条试次的最终判定 | 冻结于 2026-09-13T17:31 之前的会话 |

三者的覆盖面不同，看数时必须先确认用的是哪一份（见第 7 节）。

## 1. 判定是如何给出的

- **自评 `agent_verdict`**：智能体自己写在 RESULT.md 里的结论，取值 `success (agent)` / `unclear (agent)` /
  `fail (agent)` / `no RESULT.md`（最后一种表示超时且没有产出报告）。
- **人工判定 `operator_verdict`**：人工复核结束照片（毛巾、抛掷、mx-ro 三组另看录像）后的判定，
  只有在需要覆盖自评时才填，所以 `results.csv` 里这一列大部分为空。
- **最终判定 `snapshots/b01/labels.csv`**：每条试次一行，`verdict` 为最终判定，`source` 说明它从哪里来。
  158 行的来源分布是：
  - `paper_code` 57 行 —— 取自两个指标脚本里写死的人工判定表（Table 1 批次 2026-09-09 复核，twopairs 批次 2026-09-10 复核）；
  - `astra_self` 71 行 —— 直接采用自评；
  - `results_csv` 17 行 —— 采用 `results.csv` 里已填的 `operator_verdict`（毛巾 5、毛巾2 5、mx-ro Terra 7）；
  - `claude_review` 10 行 —— 由模型复核结束照片/录像后给出（Luna 2、Fable 5.1 4、ckpt-ro Terra 4）；
  - `timed_out` 3 行 —— 超时且无报告，记为失败。
  158 行合计 139 成功、18 失败、1 条基础设施事故（`infra`）；`counts_toward_paper` 为真的 145 行，
  为假的 13 行（12 条复位试次 + 那 1 条事故）。
- **自评与最终判定不一致的共 27 条**（占 158 条的 17%），分四类：
  - 自评 `unclear`、最终判为成功 22 条：pyramid_tools1 七条、twopiles 六条、onebigpile 两条、
    twopairs Astra low 两条、assembly_bare_medium 两条、pyramid 一条、twopairs Astra high 右臂一条、
    twopairs 任务笔记组一条；
  - 自评 `unclear`、最终判为失败 2 条：Terra t03、Fable 5.1 右臂 t04；
  - **自评 `success`、最终判为失败 1 条**：`twopairs_full_high_56terra` t02；
  - 自评 `no RESULT.md`、按超时判失败 2 条：四对组装右臂 t05、Luna 右臂 t02。
  另有 `throw_tools1` t05、t08 自评 `unclear`、人工确认为成功（该批次不在 b01 中，见第 7 节）。
- **指标定义**（详见两个脚本的文件头）：成功率 = 人工确认的完整成功 / 参与评估的试次；
  Table 1 的时间是成功试次的中位分钟数（任务下发 → 最终物理确认），Table 2 的时间是成功试次的均值 [min, max]
  （任务开始 → 最后一次写 RESULT.md 的服务器端时间戳）；token = 输入（含缓存）+ 输出（含推理）；
  成本按各模型 list price 计，单个请求输入超过 272K token 时套用长上下文倍率。
- **双臂格子的合并**：twopairs 任务由左、右臂两个批次合并成一个格子，两批次的试验号互不重复；
  判定只看该臂自己那一套零件。
- **服务档位**：`pyramid` 与 `pyramid_tools1` 跑在 fast 档，因此 `table1_metrics.csv` 的 `cost_tier_usd`
  是标准价的两倍；其余批次都是 default 档，两列相等。
- `table1_metrics.csv` / `table2_metrics.csv` 的 `rollout` 列指向生成指标的原始 rollout / agent_events 文件，
  路径已脱敏为 `<HOME>` / `<REPO>`。

## 2. 主任务（GPT-6 Astra，high 档）

| 任务 | 批次 | `results.csv` 条数 | b01 判定（成功/条数） | 进入 `table1_metrics.csv` |
|---|---|---|---|---|
| 金字塔 | `pyramid_tools1` | 10 | 10/10 | 10 行，10 成功 |
| 金字塔（早期批次） | `pyramid` | 2 | 2/2 | 无 |
| 两座塔 | `twopiles_tools1` | 10 | 10/10 | 10 行，10 成功 |
| 六块塔 | `onebigpile_tools1` | 10（t01–t10） | 9/10（t02 失败） | 只有 t01–t05，4 成功 |
| 骰子翻面 | `dice_tools1` | 10（t01–t10） | 10/10 | 只有 t01–t02，2 成功 |
| 四对组装（右臂） | `assembly_full_high_right` | 10 | 8/10（t03、t05 超时无报告） | 无 |
| 毛巾折叠 towel | `towel_full_high_yieldbatch_v2_kn_dual` | 5（双臂） | 5/5（t04 自评失败、人工判成功） | 无 |
| 毛巾折叠 towel2 | `towel2_full_high_yieldbatch_v2_kn_dual` | 5（双臂） | 3/5（t01、t03 人工判失败） | 无 |
| 毛巾折叠 medium 变体 | `towel_full_medium_yieldbatch_v2_kn_dual` | 1 | 0/1 | 无 |
| 抛掷（左臂） | `throw_tools1` | 10 | 不在 b01 | 无 |

`table1_metrics.csv` 一共只有 27 行、覆盖 4 个批次（金字塔 10、两座塔 10、六块塔 5、骰子 2），其中 26 行判为成功；
六块塔和骰子补到 10 条的部分、以及四对组装 / 毛巾 / 抛掷三个任务，都没有进入这张表。
`throw_tools1` 的 10 条按 `results.csv` 全部成功（8 条自评成功，t05、t08 自评 `unclear` 经人工确认成功）。

## 3. 双对组装的模型对比（twopairs，high 档除另注）

| 格子 | 批次 | `table2_metrics.csv` | `results.csv` 条数 | b01 判定（成功/条数） |
|---|---|---|---|---|
| GPT-5.6 Sol | `..._56sol`（左） | 5 行，5 成功 | 5 | 5/5 |
| GPT-5.6 Terra | `..._56terra`（左） | 5 行，1 成功 | 5 | 1/5 |
| GPT-6 Astra low | `..._low_6astra{,_right}` | 5 行（左 01/03/04 + 右 02/05），5 成功 | 左 8 + 右 2 = 10 | 10/10 |
| GPT-6 Astra medium | `..._medium_6astra{,_right}` | 5 行（左 01/03/04/05 + 右 02），5 成功 | 左 4 + 右 6 = 10 | 8/8（右 09/10 未进 b01） |
| GPT-6 Astra high | `..._high_6astra{,_right}` | 5 行（左 03/05 + 右 01/02/04），5 成功 | 左 5 + 右 5 = 10 | 5/5（另 5 条未进 b01） |
| Claude Opus 5 | `..._copus5{,_right}` | 5 行（左 01/03/05 + 右 02/04），5 成功 | 3 + 2 = 5 | 5/5 |
| Claude Fable 5.1 | `..._cfable51_right`（右） | 无 | 4 | 3/4 |
| GPT-5.6 Luna | `..._56luna{,_right}` | 无 | 3（左 05、右 01、右 02） | 0/3 |

`table2_metrics.csv` 共 30 行 = 6 个格子 × 5 条，是论文表格口径；Fable 5.1 与 Luna 两个格子在这张表里没有任何行，
它们的判定只存在于 `labels.csv`（Fable 4 条由模型复核，Luna 2 条模型复核 + 1 条超时）。
Astra 三档在 `results.csv` 里已各自跑到 10 条，比表格口径多出的 5 条不在表里。

## 4. 经验复用与知识来源（twopairs，GPT-6 Astra / GPT-5.6 Terra，high 档）

| 组 | 批次 | 知识来源 `notes_used` | `results.csv` 条数 | b01 判定 |
|---|---|---|---|---|
| 任务笔记 | `..._kn_6astra` | `task` | 5 | 5/5 |
| 经验检查点（累积） | `..._ckpt_6astra` | `ckpt` | 5 | 5/5 |
| mx 经验循环 | `..._mx_6astra` | `mx` | 5 | 5/5 |
| Terra 读取 Astra 检查点 | `..._ckptro_56terra` | `ckpt`（只读） | 5 行，但 t02 有两行 | 4 行，2/4 |
| Terra 读取 mx 经验 | `..._mxro_56terra` | `mx-ro` | 7（含 t01x、t03x） | 7 行：4 成功、2 失败、1 事故 |

- `..._ckptro_56terra` 的 `results.csv` 里试验号 02 出现两次（会话 `20260910_231736` 与 `20260910_233513`）；
  b01 只发布并标注了后者，前者不在 b01 的任何清单中。该组的第 5 个会话在发布时被排除（第 7 节）。
- `..._mxro_56terra` 的 t01x 是服务器守卫拒绝全部运动指令的基础设施事故，`counts_toward_paper=false`，
  其会话也不在 b01；t03x 是提示词串用导致只完成一对的试次，判为失败且计入。

## 5. 接口与调度消融（GPT-6 Astra）

| 组 | 批次 | 接口 / 调度 | `results.csv` 条数 | b01 判定 |
|---|---|---|---|---|
| 四对组装 bare 接口 | `assembly_bare_low` / `_medium` / `_high` | 无工具的裸接口 | 1 / 2 / 1 | 1/1、2/2、1/1（medium 两条均为自评 `unclear` 转判成功） |
| 单次插入 | `singleinsert_bare_medium_yield` | yield | 1 | 1/1 |
| 单次插入 | `..._yieldbatch` | yield + batch | 1 | 1/1 |
| 单次插入 | `..._yieldbatch_v2` | yield + batch（v2） | 3 | 3/3 |
| 单次插入 | `..._yieldbatch_v2_kn` | v2 + 任务笔记 | 3 | 3/3 |

## 6. 复位会话（不计入任何指标）

- `twopairsreset_full_medium_6astra`：12 行（试验号 01–07、09–13），b01 判定 12/12，
  但 `counts_toward_paper` 全为 false —— 这些是每条正式试次之后把零件打散的动作，不是实验试次。
- b01 另发布了 14 个 `scatter_tools1_*` 会话（金字塔 / 两座塔的自动复位）和 1 个
  `20260911_041509_scatter_twopairsreset_full_medium_6astra_08` 会话，它们在 `results.csv` 和 `labels.csv`
  里都没有对应行。

## 7. 发布边界：b01 数据集里有什么、没有什么

`batch.json`：196 个会话（paper 148、scatter 27、其他 21），冻结于 2026-09-13T17:31，
158 个带关节数据、159 个带 frames.csv、537 段根视频、17905 张帧图，合计 89,161,998,493 字节。

- **已发布** `released.txt` 169 个会话 = 142 个 paper + 27 个 scatter。
- **已排除** `excluded.txt` 27 个：21 个辅助会话（标定、只读检查、正式批次之前的手动试跑），
  6 个"已评估但没有批次号"的会话 —— `towel_tools1` 01/02、Luna 右臂 03、Luna 04、Fable 5.1 右臂 05、
  ckpt-ro Terra 05。这 6 个会话都没有 `results.csv` 行，因此也不参与任何统计；
  `towel_tools1/` 目录下只留了工具快照，没有 `results.csv`。
- **判定表与发布清单的差**：`labels.csv` 158 行中 154 行对应已发布会话；另外 4 行对应未发布的会话 ——
  mx-ro Terra t01x（事故会话），以及 Astra low t10、Astra medium 右臂 t07 / t08
  （这三条在 09-13 冻结时刻前后结束，未收进 b01）。反过来，15 个已发布会话没有判定行（第 6 节的复位会话）。
- **冻结之后的试次**：`results.csv` 里有 17 条 2026-09-14 的试次不在 b01 —— `throw_tools1` 10 条、
  Astra high 左 06/07/09 与右 08/10、Astra medium 右 09/10。引用抛掷任务或 Astra 各档的 10 条口径时，
  需要说明这部分数据不在 b01 发布内。
- `assembly_full_high/`、`assembly_full_low/`、`assembly_full_medium/` 三个目录只保留了工具快照，
  没有 `results.csv`，也没有对应的试次。

## 8. 引用这些数字时的注意事项

1. 论文表格的口径是"每格 5 条"，仓库里的 `results.csv` 则包含之后补跑的试次；两者数值不同不是矛盾，
   但必须说明用的是哪一份。
2. 成功率一律以人工判定为准。自评在 158 条里有 27 条与最终判定不同，其中 1 条是自评成功、实为失败，
   直接统计 `agent_verdict` 会高估成功率。
3. 超时且无报告的试次（四对组装 t05、Luna 右臂 t02）计为失败并计入分母，不是"未完成、不计"。
4. 只有 `counts_toward_paper=true` 的行进入指标；复位试次和基础设施事故行始终排除。
