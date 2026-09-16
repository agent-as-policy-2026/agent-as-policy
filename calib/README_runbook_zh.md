# 左臂手眼标定操作手册（LEFT ARM, can_follower_l）

本手册给操作员使用：**只需人工完成采集部分**，其余（求解、零位偏置拟合、
写回模型、验证）由 Claude 离线完成。

工具与右臂机位的流程一致，但采集工具是**独立脚本**，不需要启动 bridge。

本手册的命令若无特别说明都在**仓库根目录**下执行；`cd hardware-bridge` 之后的
命令用 `../calib/...` 引用本目录的脚本（仓库 checkout 放在哪里都可以）。

---

## 1. 准备工作（开始前逐项确认）

1. **CAN**：确认 `can_follower_l` 已经 UP（udev 规则已验证过）。检查：

   ```bash
   ip -br link show can_follower_l
   ```

   显示 `UP` 即可。若未 UP，用平时 lerobot 遥操作前的 canup 方式带起
   （bitrate 1000000）。可用下面命令确认总线上有数据（Ctrl+C 退出）：

   ```bash
   candump can_follower_l -n 5
   ```

2. **相机**：确认左腕 D405（序列号 `353322271204`）USB 线插好、没有松动。

   注意：本工具运行期间会**独占**左腕 D405（pyrealsense2 同一时间只允许
   一个进程打开该相机）。运行期间**不要**对序列号 `353322271204` 同时开
   realsense-viewer 或另一套桥的 camera_preview.py —— 想看画面请用本工具
   **自带的浏览器预览**（见第 2 节）。顶部相机（TOP）本工具**既不使用
   也不占用**，不受任何影响。

3. **标定板**：22 mm 方格、9x7 内角点的棋盘格，**已放在左臂前方**。
   必须**刚性固定**（压紧/贴住，整个采集过程中绝对不能动）；
   左臂底座同样不能动。整个 session 中如果标定板或底座被碰动，
   全部姿态需要重采。

4. **急停**：手边放好急停开关，全程监督。

5. **手臂初始姿态**：启动前把手臂摆到一个正常的、不顶关节限位的姿态
   （平时的收拢休息位即可）。若某个关节压在限位外，i2rt 会在初始化时
   报错退出，但报错**之前已经先把手臂锁定在当前位置（HOLD）**，电机会
   继续保持不动（脚本会打印 "position hold IS active on the motors" 和
   "The motors keep holding the last position onboard."，不会下坠）。
   为保险起见，启动时仍请扶好手臂；随后按急停或扶住手臂断电，
   把手臂摆回正常姿态重新来。

6. **夹爪**：本工具**跳过了夹爪自动标定的开合动作**
   （使用 i2rt 官方支持的 `gripper_limits_override` 参数），
   启动时夹爪不会动。代价是记录的夹爪开合度数值无意义 ——
   对手眼标定完全无影响（只用 1~6 关节）。

---

## 2. 启动采集工具

```bash
cd hardware-bridge
uv run --locked python ../calib/capture_left_handeye.py
```

流程：

1. 脚本先打开相机，把左 D405 的出厂内参/畸变写到
   `calib/out/left_d405_intrinsics.json`（自动完成，不用管）。
2. 随后启动**浏览器实时预览**（MJPEG，默认端口 8766，避开另一套桥
   camera_preview.py 的 8765），并醒目打印访问地址，形如：

   ```
   http://<本机 IP 或 tailscale IP>:8766/
   ```

   在浏览器打开（远程机器用 tailscale IP）即可看到左腕相机实时画面，
   画面顶部有棋盘格检测横幅：

   * 绿色 `board OK: 63/63 corners` —— 标定板**完整**入画，可以采；
   * 红色 `board NOT fully detected (n/63)` —— 不完整/检测失败，调整姿态。

   角点检测约 2 Hz、画面约 8 fps，只用于预览，**不影响采集质量**。
   加 `--no-preview` 可关闭预览；`--preview-port N` 可换端口。
3. 然后提示即将上电左臂：**手臂上电后会立刻锁定在当前位置（HOLD），
   不会有任何运动**。输入 `yes` 回车确认。
4. 进入交互命令行。

### 交互命令（输完按回车）

| 命令 | 含义 |
|------|------|
| `g` | 重力补偿模式：可以**用手拖动手臂**摆姿态。从 HOLD 切换时会先提示"请先扶住手臂，按回车后释放为重力补偿模式"，回车确认后才释放 |
| `h` | 锁定（HOLD）：手臂**冻结在当前位置**。若某个关节太靠近软限位，会**拒绝锁定**并提示把该关节稍微拖离限位后重试（保持在重力补偿模式） |
| `c` 或 `c 标签` | 采集：只在 HOLD 下允许；抓 30 帧（约 3 秒），存为 `calib/out/left_poseNN[_标签].npz` |
| `p` | 在终端打印当前棋盘格检测状态（`board OK: 63/63 corners` 或 `board NOT fully detected (n/63)`）。随时可用，只读相机，不碰手臂 |
| `q` | 退出：手臂**保持 HOLD**，之后你可以按急停或接住手臂再断电 |

标准节奏：`g` → 用手拖到一个姿态 → `h` → 等 1~2 秒稳定 → `c tilt_left` →
`g` → 下一个姿态 → …

每次 `c` 之后脚本会打印这次采集的抖动（stillness）数值：
要求 **≤ 2 mm / 0.5°**。超了会警告，请在同一姿态重新 `c` 一次。

**安全说明**：脚本只会发送"保持当前测量位置"这一种位置指令，
不会有任何轨迹/移动；任何异常都会保持当前模式退出，不会命令运动。
退出后电机会继续保持最后指令（HOLD 会一直保持住），
所以最后请**先按急停或扶住手臂，再断电**。

---

## 3. 姿态怎么摆（关键！）

共采 **8~10 个姿态**，每个姿态必须：

* 标定板**完整**出现在画面里（9x7=63 个内角点全部可见、不贴边）——
  用浏览器预览确认横幅是绿色 `board OK: 63/63 corners`（或按 `p`）；
* 相机距标定板 **0.25 ~ 0.45 m**；
* 板面尽量占画面 1/3 以上。

姿态之间要有**明显的角度差异**（这是求解精度的决定因素）：

1. 距离变化：近（~0.25 m）、中（~0.35 m）、远（~0.45 m）各来几个；
2. 左右倾：让相机从板的左侧、右侧斜着看（±20~30°）；
3. 前后俯仰：从更高俯视、从更低仰视（±20~30°）;
4. 手腕滚转：同一视角下把腕部左右拧一拧（±20° 以上）各采一个；
5. **避免**只平移不转动的姿态组合（几乎平行的视角对求解没贡献）。

推荐清单（10 个）：
近-正视、中-左倾、中-右倾、中-俯视、中-仰视、
远-正视、近-左倾+滚转、中-右倾+滚转、远-俯视、中-仰视+反向滚转。

打标签方便核对，如：`c near_center`、`c mid_tilt_left`、`c roll_plus` 等。

采完 `q` 退出即可。

---

## 4. 之后会发生什么（Claude 来做，供了解）

1. **求解**：`bash calib/solve_left.sh`
   跑 hardware-bridge 自带的离线 calibrate-checkerboard 求解器，
   产出 `calib/out/left_hand_eye_report.json`。
   通过门限：重投影 **≤ 3 px**、固定板一致性 **≤ 15 mm / ≤ 2° (RMS)**。
   右臂当时做到 ~2 mm 量级，左臂预期同水平。
2. **零位偏置诊断**（右臂当时 joint4 差了 +5.47°，左臂也要查）：
   `fit_joint_offsets.py` 用同一批 npz 拟合 2~5 关节的常数零位偏差。
   如发现稳定的 >0.3° 偏置，会把它写入
   `i2rt/i2rt/robots/config/yam_v1.yml` 的
   `motor_offsets_deg_by_channel: can_follower_l: [...]`，
   然后**需要你再采一轮**（旧数据的关节读数已经变了）。
3. **写回**：把解出的手眼外参写入 station XML 的左相机分支。
4. **验证**：请你再采 1 个**全新姿态**做 held-out 检查
   （门限同样 15 mm / 2°），加 evaluate-jitter 静止性检查。

---

## 5. 命令速查（复制粘贴）

采集（唯一需要你跑的）：

```bash
cd hardware-bridge
uv run --locked python ../calib/capture_left_handeye.py
```

以下由 Claude 跑（列出备查）：

```bash
# 求解手眼
bash calib/solve_left.sh

# 单个采集的静止性检查
cd hardware-bridge
uv run --locked agp-yam-camera-acceptance --config config/left_calib.yaml \
    evaluate-jitter --capture ../calib/out/left_pose01_xxx.npz

# 零位偏置拟合（含合成数据自检：--self-test）
uv run --locked python ../calib/fit_joint_offsets.py \
    --captures ../calib/out/left_pose*.npz

# held-out 新姿态验证（写回后）
uv run --locked agp-yam-camera-acceptance --config config/left_calib.yaml \
    evaluate-checkerboard-pose \
    --capture ../calib/out/left_poseXX_holdout.npz \
    --reference-report ../calib/out/left_hand_eye_report.json \
    --output ../calib/out/left_held_out.json

# 采集工具的离线自检（不上电，验证 FK 数学）
uv run --locked python ../calib/capture_left_handeye.py --dry-run
```

---

## 6. 常见问题

* **启动时报 joint limit violation** → 手臂初始姿态顶到限位了。i2rt 在
  报错前已装好当前位置 HOLD（脚本会打印 "position hold IS active" /
  "motors keep holding"），电机保持不动、不会下坠；为保险仍建议扶住。
  按急停或扶住手臂后断电，把手臂摆回正常姿态，重新来。
* **某次 `c` 抖动超标** → 手没完全松开或地面震动，保持 HOLD 再 `c` 一次。
* **画面里找不到棋盘格/求解时报 corner 检测失败** → 该姿态板不完整或
  反光过强，换角度重采该姿态。采集前先看浏览器预览的横幅（或按 `p`）
  是否为绿色 `board OK: 63/63 corners`，可以避免采到废姿态。
* **浏览器打不开预览页面** → 确认用的是脚本启动时打印的地址和端口
  （默认 8766），远程机器要用 tailscale IP。若启动时打印过 "preview
  server could not bind" 警告（端口被占），预览不可用，但 `p` 命令
  仍然能在终端里报告检测状态。
* **中途碰到了标定板或底座** → 停止，重新固定，从第一个姿态重采整轮。

---

## 7. 标定已完成（2026-08-31）

左臂手眼标定**已完成并写回**，本节记录结果与后续注意事项。

* **数据**：11 个姿态（`calib/out/left_pose02..12.npz`，RGB 640x360，
  D405 序列号 `353322271204`），采集时 `can_follower_l` 的软件零位偏置
  为**全零**。
* **零位偏置**：`fit_joint_offsets.py` 拟合出 joint 3/4/5 三个稳定零位
  修正（q_true = q_measured + delta，delta_deg =
  `[0, 0, -0.813493705088, -1.9574947249, +0.359152777503, 0]`），
  已按 `reported = raw - offset` 约定写入
  `i2rt/i2rt/robots/config/yam_v1.yml` 的 `can_follower_l` 行
  `[0, 0, +0.8135, +1.9575, -0.3592, 0]`。
* **一致性**：基线（零修正、CAD 外参）8.32 mm / 1.156° RMS →
  修正后重解 **1.37 mm / 0.266° RMS**（`resolve_with_offsets.py`；
  官方 CLI 闭环复核 `left_hand_eye_report_corrected.json`：PASS，
  solved_from_nominal ≈ 0）。
* **写回**：解得的 `left_gripper → left_camera` 已装入 station 模型三个
  文件（`yam_station_linear_4310_d405` 的 `.xml` 叶子 body、`.urdf` 的
  `left_camera_joint` origin、`README.md` 表格；与右臂安装模式一致）。
  平移 `(-0.072824, 0.002148, -0.073374)` m，
  四元数 wxyz `(0.160345, 0.701747, 0.677653, 0.150422)`；
  与 CAD 差 4.85 mm / 2.15°。
* **dry-run**：`capture_left_handeye.py --dry-run` 中 station-vs-CAD 的
  对比已改为**信息性输出**（写回后二者本来就应有 ~4.85 mm / ~2.15° 差
  异）；FK 自检仍按 CAD 常量把关，PASS。
* **重要 — 以后再采集/再拟合**：新一轮采集将在**已安装偏置**下记录关节
  读数，因此给 `fit_joint_offsets.py` 必须传：

  ```bash
  --installed-offsets-deg 0 0 0.813493705088 1.9574947249 -0.359152777503 0
  ```

  （即当前 yam_v1.yml `can_follower_l` 行；不传会把已装偏置当成零，
  打印出错误的安装行。）旧的 `left_pose02..12.npz` 内嵌的是**旧读数**，
  不能与新一轮混用；离线修正副本在 `calib/out/corrected/`。

## 8. 顶部相机（BRIO 178B0DAE）标定

目标：左工位**固定顶部相机**（Logitech BRIO，序列号 178B0DAE，
`/dev/v4l/by-id/usb-046d_Logitech_BRIO_178B0DAE-video-index0`）的内参 + 外参
（`camera_to_world`，world = left_base）。流配置固定为 **1920x1080@30（2026-09-01 起；640x360 时代的数据与标定已归档为 *_640x360） V4L2
MJPG**——与桥/客户端将来实际流出的完全一致，工具打开相机后会校验协商结果
（分辨率、帧率、MJPG FourCC），不一致会直接拒绝。棋盘仍是 22 mm、9x7 内角
点那块。共两场采集，之后求解由 Claude 运行 `solve_top.sh` 完成。

### 8.1 第一场：内参采集（手持棋盘，**不上电、不碰机械臂**；双相机一场约 15 分钟）

两台顶部 BRIO（左 178B0DAE → `calib/out/top_intr/`，右机位 B8C7F203 →
`calib/out/top_intr_right/`）**一次同时采**：一个进程开两台相机，同一块棋盘举
一次、两台各存各的。单台仍可 `--rig left` / `--rig right`（行为与之前完全一致）。

```bash
cd hardware-bridge
# 离线自检（不开相机；单台 / 双台都应 PASS）
uv run --locked python ../calib/capture_top_intrinsics.py --rig left right --dry-run
# 双相机一场（预览 :8767）
uv run --locked python ../calib/capture_top_intrinsics.py --rig left right
# 某台确实盖不到的格子（例如左机位最右一列）可声明为"接受不覆盖"（只影响 DONE 提示）：
#   ... --rig left right --ignore-cells left:c6
#   写法：r4c6 单格 / r4 整行 / c6 整列；前缀 left: 或 right: 限定一台，不加前缀两台都算
```

启动前提：**两台桥都不能在跑**（9021 左桥、9020 右臂桥——任一端口有监听即拒绝
启动、不碰硬件）；对焦按各自 rig 规则处理（左 verify 只读、右 lock 后回读，见
§8d）。启动时会**重新检测目录里已有的 calib_NN/val_NN PNG** 并计入覆盖统计（每张
检 3 次，约 0.4 s/张），所以中断后可以直接续采；检不出角点或分辨率不对的旧图会被点
名警告且**不计数**（求解器会在它上面硬失败，求解前把它移走）；3 次里有漏检的会标
**unstable**（计数，但求解器只检一次、可能整场求解失败——建议删掉重拍，编号接着排）。

浏览器打开预览 **http://<本机IP>:8767/**：两台相机**并排**（各缩到 960 px 宽；单台
时保持 1920 原尺寸），每台画面上叠 6×4 覆盖网格——**绿色半透明 = 该格已有标定图
角点，红框（标 rNcM）= 还空着，灰框 = 已声明忽略**；横幅显示该台 "board OK /
NOT detected" 和已存张数。

命令（输完按回车）：

* `c` 对**每台**相机各存一张**标定**图 → 各自目录 `calib_NN.png`（每台独立编号）
* `v` 对每台各存一张**验证**图 → `val_NN.png`（求解时单独留出）
* `p` 打印各台检测状态 + 三张覆盖表；`q` 退出

每次 `c`/`v` 只在**那台画面里完整检出 9x7 角点**时才存；没检出的那台打印
`<rig>: no board — skipped`、不存，另一台照常存。存图门槛是"同一帧**连续 3 次都检出**"：
求解器用的 `findChessboardCornersSB(EXHAUSTIVE)` 对同一张图会**随机漏检**（实测旧的 25
张里有 4 张漏检率 6~44%，其余从不漏），而求解器只检一次、漏一张就整场失败，所以 3 次里
有一次没检出的帧会打印 `board UNSTABLE — skipped`——把板端稳/拿近/避开反光再按。每次按键后打印每台张数 + 三张
覆盖表（**只统计标定图**，验证图留出）：

| 表 | 含义 | DONE 条件 |
|----|------|-----------|
| grid 6×4 | 画面分 6 列 × 4 行（r1 = 顶行，c1 = 左列），格内数字 = 有角点落入该格的标定图张数，`.` = 空，`x` = 忽略；下方列出空格（如 `row4: all 6 empty; col6: rows1-3 empty`） | 24 格全部非空（`--ignore-cells` 声明的除外） |
| scale | 每张图的 px/square（相邻角点间距，取透视压缩较小的那条轴的中值）：**near ≥ 60 px**（板离镜头 < ~0.43 m）、**mid 40–60 px**（~0.43–0.64 m）、**far < 40 px**（> ~0.64 m；板平放桌面约 28 px）。换算按 22 mm 方格、f ≈ 1160 px @1080p，规则会随表打印 | 三档各 ≥ 3 张 |
| tilt | 用 63 个角点的单应（板平面 → 图像，按名义焦距分解）算板法线与**视线**的夹角：**flat ≤ 10°**、**medium 10–25°**、**steep > 25°** | 三档各 ≥ 3 张 |

表后一行 `next: ...` 给出**最空的格、最少的距离档、最少的倾角档**，下一张就往那
儿举。最后一行状态：**`TODO`**（未到求解最低量）→ **`minimum met (20 calib + 5
val), keep going for coverage`**（已达求解硬最低，但覆盖未满）→ **`DONE <rig>`**
（标定 ≥ 30、验证 ≥ 6、格子全覆盖、六个档各 ≥ 3）。**两台都 DONE 再 `q`**。

怎么举：每台目标 **30~40 张标定 + 6~8 张验证**（双相机一场约 **40~50 次按键**，
两台常常同时存）。24 格**都要**盖到——四角和**下边缘**（r4 一行最容易漏）；距离
从 ~0.4 m 铺到 ~1.0 m（near：板举到离镜头 40 cm 内、占画面大半；mid：半臂距离；
far：放到/贴近桌面）；倾角到 ~35°（三档都要，别全是斜的也别全是平的）；存图瞬间
**端稳、画面清晰**——动态模糊的角点会拉高重投影误差，宁可慢。若该帧没检出完整
角点，工具会拒存并提示（反光/太斜/太远最常见），调整后再按一次即可。

### 8.2 第二场：外参 pair 采集（机械臂 + 双相机，无桥）

```bash
cd hardware-bridge
uv run --locked python ../calib/capture_top_pairs.py
```

安全事项与 §2 完全相同（同一套 g/h 保持/重力模式、软限位拒绝、CAN 链路活
性检查、手不离急停；退出后手臂保持 HOLD）。预览：手腕相机 **:8766**，顶部
相机 **:8767**。

流程：棋盘**平放在桌面上**，在工作区内选 **12~16 个不同位置**（位置怎么选
见 §8b 重采协议——首轮"包含工作区边缘/四角"的做法已被 §8b 取代），顶部相机
**全程不动**。每个位置：

1. `g` 拖动手臂，让**手腕相机也能看到整块棋盘**（看 :8766 预览的绿色横幅；
   距离与倾角要求见 §8b）；
2. `h` 保持；
3. 先按 `p` 看 ADVICE 行（见 §8b），姿态没问题再按
4. `c`：抓 **`--frames`（默认 20）帧**（约 1 秒，**保持完全静止**），逐像素
   取**时间中值**后存为一对 → `calib/out/top_pairs/pairNN.npz`；原始帧栈另
   存旁路文件 `calib/out/top_pairs/frames/pairNN_frames.npz`（求解器**永远
   不读**它，只读中值 npz；schema 与旧版完全一致）。

门限（不达标会**拒存**，调整后重按 `c`）：两个相机在**每一帧保留帧**（以及
中值图）都要完整检出 9x7 角点；帧对时间差 > **0.1 s** 的个别帧对会被**自动
丢弃**（BRIO 偶发掉帧/缓冲所致，场景静止时无害；工具会打印丢弃数量），只有
同步良好的帧对少于 **60%**（20 帧默认下少于 12 对）才整体拒存；采集期间
棋盘/手臂不能动（帧间角点漂移 ≤ **0.5 px** rms，超了拒存——手离开棋盘、
HOLD 稳定后再按）。每次保存后工具还会打印本次的质量数字 + ADVICE 行（不拦
截保存）以及 board z 一致性行（见 §8b）。npz 内嵌的手腕
`camera_to_world` = FK(上报关节，已含零位修正) x 实测版 station 左手眼变换
（启动时从 MJCF 现读），格式与官方 `capture-top-pair` 完全一致。

### 8b. 重采协议（RECAPTURE，2026-09-01 之后适用）

背景：第一轮 16 对（2026-09-01）解出外参 PASS 但标定 RMS **9.4 mm**，逼近
15 mm 门限。残差分析（`calib/analysis/out/report.txt`）定位了三个原因：
(1) 顶部图像里板只有 ~85 px 宽，单帧角点噪声贡献 ~5 mm；(2) 手腕伸到远处
（腕距板 0.45~0.58 m、板离底座 r>0.55 m）的陡姿态下手腕链朝向散布，经
0.9 m 顶部相机杠杆臂放大；(3) 部分位置贴近顶部图像边缘。据此重采：

1. **内参保留**（0.14 px，无需重跑 §8.1）。
2. 把旧 pair **移走**（不要混用，编号会接着排、solve_top.sh 会混采）：

   ```bash
   mkdir -p calib/out/top_pairs_v1
   mv calib/out/top_pairs/pair*.npz \
      calib/out/top_pairs_v1/
   # 若存在原始帧旁路目录，一并移走：
   mv calib/out/top_pairs/frames \
      calib/out/top_pairs_v1/frames 2>/dev/null || true
   ```

   （工具启动时若发现目录里已有 pair 会醒目提醒这一步。）
3. **位置**：12~16 个点全部放在**工作区内**——板中心距左臂底座水平距离
   **r 0.25 ~ 0.50 m**（物体实际会出现的区域），在这个区域内尽量铺开；
   **避开上一轮用过的远端 −y 方向长伸展**（那批 r>0.55 m 的点正是残差
   最大的来源）。
4. **手腕姿态**：腕相机距板 **0.25 ~ 0.35 m**、倾角 **20 ~ 35°**，让板占
   腕相机画面相当大一部分。
5. **顶部画面位置**：尽量让板落在顶部图像**中央 ~60%** 区域（中心偏移
   ≤ 180 px）。
6. 每次 `c` 是 **20 帧时间中值（约 1 秒）**：按之前手离开棋盘、HOLD 稳定
   1~2 秒，按下后**保持完全静止**；帧间角点漂移 > 0.5 px 会拒存。
7. **按 `c` 之前先按 `p` 读 ADVICE 行**（ADVICE 不拦截保存，但请当作要求
   对待）。ADVICE 触发条件：

   | 指标 | 允许范围 |
   |------|----------|
   | 腕→板距离 | 0.22 ~ 0.38 m |
   | 腕相机倾角（板法线 vs 光轴） | 15 ~ 40° |
   | 顶部图像板尺度 | ≥ 10 px/格 |
   | 顶部图像板中心偏移主点 | ≤ 180 px |
   | 板中心距左底座水平距离 r | ≤ 0.50 m |

8. 每存一对后看 **board z 一致性行**：板一直平放在同一张桌上，各对解出的
   板面 z（世界系，经手腕链）应当很紧。若某对 **|dz| > 5 mm**（相对此前
   各对的中位数）会打印 wrist-chain warning——该姿态的手腕链不可信，换一
   个更近/更平的姿态重采这个点（替换时手动删掉坏的 pairNN.npz 及其
   `frames/pairNN_frames.npz`）。
9. 采满后求解 `bash calib/solve_top.sh`，
   **目标：calibration RMS < 4 mm**（门限仍是 15 mm，但按残差分析，去掉
   上述三个误差源后应能到 4 mm 以内；显著高于此值说明还有姿态没达标）。

### 8.3 求解（Claude 来做，供了解）

```bash
bash calib/solve_top.sh
```

* 第一步 `calibrate-top-intrinsics`：`top_intr/` 的 20+5 张图 →
  `calib/out/top_brio_178B0DAE_intrinsics.json`；门限：标定与验证重投影
  RMS 均 ≤ 1 px。
* 第二步 `calibrate-top-extrinsics`：`top_pairs/` 的 pair 按文件名排序，
  **每第 4 个（第 4、8、12…个）留作验证**，其余参与标定（≥3 标定 + ≥1 验
  证）；手腕畸变从 `left_d405_intrinsics.json` 读取（两个采集工具启动时都
  会重新导出）→ `calib/out/top_brio_178B0DAE_calibration.json`；门限：标定
  RMS ≤ 15 mm / 2°，验证误差 ≤ 15 mm / 2°。
* 注意：报告里的 `solved_from_nominal_translation_m` 会在 **0.61 m** 左右
  ——官方求解器的"名义位姿"取自 MJCF 的 `right_base` 帧（左工位 world =
  left_base，两基座相距 0.61 m），该项仅以 1e-4 权重参与朝向消歧、不影响
  PASS/FAIL，属预期现象。
* 已用合成数据端到端验证过整条链路（渲染棋盘 → 两步求解 PASS，恢复真值
  2.3 mm / 0.32°）。

### 8.4 之后接线（**默认不启用**，需用户拍板后再跑）

`calibrate-top-extrinsics` 输出的 JSON **就是**桥启动时
`agp_yam_bridge.camera.load_fixed_rgb_calibration` 直接消费的格式，无需转
换。启用步骤已写成脚本（本次**不运行**）：

```bash
bash calib/install_top_calibration.sh
```

它会：用桥自己的加载器校验 `top_brio_178B0DAE_calibration.json`（PASS、
name=top_brio、serial=178B0DAE、1920x1080@30、rigid camera_to_world、
metrics.validation_translation_max_m>0 均为桥的硬性要求）→ 拷到
`hardware-bridge/acceptance/top_left/top_brio_calibration.json` → 解开
`config/left_arm.yaml` 里注释掉的 `top_camera:` 块（精确匹配、可重复运
行）→ 离线验证 left_arm.yaml 能整体加载。**不会**启动桥。

### 8c. 2026-09-01 补记：顶部相机档位改为 1920×1080@30
- 原顶部 BRIO 178B0DAE 位于 USB3 口，支持 1920x1080@30 MJPG；桥从标定 JSON 的 camera{width,height,fps} 协商流档位，
  因此**只需在新档位重做内参 + 外参**（分辩率改变使旧标定失效），桥/客户端无需改代码；装完标定后重启桥即生效。
- 旧 640x360 产物（内参图、pair、标定 JSON，以及当时装机的那份 acceptance 副本）**归档在本仓库之外**，
  留在机器上的 `calib/out/*_640x360/` 里；仓库只随代码附带当前 1080p 的装机标定。采集命令与 §8 完全相同（工具默认档位已改）。
- 配对工具的两个像素阈值随宽度 ×3（px/square ≥ 30、离主点 ≤ 540 px）。
- 新增的 BRIO D0CF5843（USB2 口，对面视角）**仅作观察预览**，不标定、不进桥：`calib/preview_camera.py --device <by-id> --port 8768 --width 1920 --height 1080 --grid`。

### 8d. 右臂机位顶部相机标定（`--rig right` / `RIG=right`，2026-09-02 起）

目的：用**同一套工具**标定右工位的固定顶部相机（BRIO **B8C7F203**，
`/dev/v4l/by-id/usb-046d_Logitech_BRIO_B8C7F203-video-index0`，同样 1920x1080@30
MJPG），产出由右臂桥仓库自行安装的 JSON。**本机不安装任何东西**，交接见第 5 步。
所有 left/right 差异集中在 `calib/left_handeye_common.py` 的 `RIGS` 表：

| 项目 | left（默认，行为不变） | right（右臂机位） |
|------|------------------------|---------------|
| CAN 通道 | `can_follower_l` | `can_follower_r`（i2rt 按通道自动施加右臂的 joint-4 零位偏置） |
| 手腕 D405 | `353322271204` | `353322271910`（出厂畸变**现读该设备**，不复用左臂的）|
| station 体（手眼） | `left_gripper` → `left_camera` | `right_gripper` → `right_camera`（我们 i2rt 副本里右臂的实测值） |
| 世界系 | 左臂底座 | **右臂底座**（单臂 FK 模型原点；**不**加 station 的 0.61 m 左右偏移） |
| 顶部 BRIO | `178B0DAE` | `B8C7F203` |
| 拒绝启动的桥端口 | 9021（我们的左桥） | **9020（右臂桥）** |
| 对焦处理 | verify（只读校验 autofocus=0） | **lock**：`focus_automatic_continuous=0, focus_absolute=0, zoom_absolute=100` 后回读，autofocus 仍为 1 则拒绝 |
| 内参图 | `calib/out/top_intr/` | `calib/out/top_intr_right/` |
| pair | `calib/out/top_pairs/` | `calib/out/top_pairs_right/` |
| 手腕内参 dump | `calib/out/left_d405_intrinsics.json` | `calib/out/right_d405_intrinsics.json` |
| 产物 | `top_brio_178B0DAE_{intrinsics,calibration}.json` | `top_brio_B8C7F203_{intrinsics,calibration}.json` |
| 求解配置 | `config/left_calib.yaml` | `config/right_calib.yaml`（**仅离线求解用**，不是桥运行配置） |
| 安装 | 装进我们的 `left_arm.yaml` | **不安装**，写 `calib/out/RIGHT_RIG_HANDOFF.md` 交接 |

**前提（缺一不可）**

1. **右臂桥必须先停掉**（9020 上那份桥的 serve 进程独占
   `can_follower_r` + 右臂的两台相机）。工具启动时检测到 9020 有监听会**直接拒绝**、
   不碰任何硬件；**不要**从本工具这边 kill 那个桥进程，用桥自己的方式停。检查：`ss -ltn | grep 9020`
   应为空。
2. `can_follower_r` 已 UP（`ip -br link show can_follower_r`）；右腕 D405
   `353322271910` 没被 realsense-viewer / 另一套桥的 camera_preview.py 占用。
3. BRIO B8C7F203 在 USB3 口（否则协商不出 1080p30 MJPG，工具会拒绝）。
4. 棋盘仍是 22 mm、9x7 内角点那块。

**安全（比左臂更谨慎）**

* 急停放在**操作员**手边，采 pair 时**最好有第二个人在场**；`g` 后是**重力补偿**，手臂由操作
  员用手拖动摆姿态（切换前会提示"先扶住手臂"），拖动时另一只手不离急停。
* 其余与 §2/§8.2 完全相同：`h` 保持、软限位拒绝、CAN 链路活性检查、任何异常保持
  当前模式退出不命令运动、`q` 后手臂**保持 HOLD**——退出后**先按急停或扶住再断电**。
* 工具只会发"保持当前测量位置"这一种指令，没有任何轨迹。

**命令**

```bash
cd hardware-bridge
# 离线自检（不开相机、不上电，两个都应 PASS）
uv run --locked python ../calib/capture_top_intrinsics.py --rig right --dry-run   # 或 --rig left right --dry-run
uv run --locked python ../calib/capture_top_pairs.py --rig right --dry-run

# 第一场：内参（手持棋盘，不上电）30~40 标定 + 6~8 验证 -> calib/out/top_intr_right/，预览 :8767
#   建议与左机位一起一场采完（--rig left right，覆盖表/DONE 提示见 §8.1）；单独采右机位那台：
uv run --locked python ../calib/capture_top_intrinsics.py --rig right
# 第二场：外参 pair（右臂 + 双相机）12~16 对，按 §8b 协议 -> calib/out/top_pairs_right/
#   （ADVICE 里的 r ≤ 0.50 m 是相对**右臂底座**；预览手腕 :8766、顶部 :8767）
uv run --locked python ../calib/capture_top_pairs.py --rig right

# 求解（Claude 来做）-> top_brio_B8C7F203_{intrinsics,calibration}.json
RIG=right DRY=1 bash ../calib/solve_top.sh    # 只打印解析出的路径
RIG=right bash ../calib/solve_top.sh
# 交接（不安装）：用桥的加载器只读校验 + 写 calib/out/RIGHT_RIG_HANDOFF.md
RIG=right DRY=1 bash ../calib/install_top_calibration.sh
RIG=right bash ../calib/install_top_calibration.sh
```

对焦：`--rig right` 启动时先用 `v4l2-ctl` 把 B8C7F203 锁到 autofocus=0 /
focus_absolute=0 / zoom_absolute=100 并回读（autofocus 读回 1 即拒绝启动；
`--focus-lock verify|skip` 可改，不建议）。**内参只在这个对焦状态下有效**——之后右机位
那边不要再动 BRIO 的对焦/变焦。

**交接（第 5 步）**：把 `RIGHT_RIG_HANDOFF.md` 里列出的三个文件（calibration JSON
是唯一必须安装的；intrinsics JSON 与 `right_d405_intrinsics.json` 为信息）连同该
说明一起交接给右臂桥仓库。在右臂桥仓库那边：备份旧的 640x360 JSON → 拷入该仓库的
`hardware-bridge/acceptance/top/` → 改 `first_acceptance.yaml` `top_camera:`
块里的 `calibration_path`（分辨率**不写 yaml**，桥从 JSON 的 camera.width/height/fps
协商 1920x1080@30）→ 离线 `load_config` 验证 → 重启右臂桥。求解报告里的
`solved_from_nominal_*` 对右工位是**有意义的**参考（求解器名义位姿本来就相对
`right_base`），不再像左工位那样固定在 0.61 m 附近。

### 8e. 双臂世界坐标系：左顶相机对右臂的外参（`--rig cross` / `RIG=cross`，2026-09-08 起）

目的：求 **T_left_right**（右臂基座在左臂基座坐标系里的位姿），供 agp 右臂服务端把右臂的一切位姿换算到
left_base。方法是用**同一套 pair 采集 + 求解链**，只是机位组合换成 **右臂 + 我们的左顶相机 178B0DAE**：
求得左顶相机在 **right_base** 下的 camera_to_world，再和已装机的左标定（同一台相机在 **left_base** 下）相除：
`T_lb_rb = T_lb_top · inv(T_rb_top)`。差异全部集中在 `left_handeye_common.py` 的 `RIGS["cross"]`：

| 项目 | cross |
|------|-------|
| 机械臂 / CAN | 右臂 `can_follower_r`（i2rt 按通道施加 joint-4 零位偏置） |
| 手腕 D405 | `353322271910`（出厂内参现读，写 `calib/out/right_d405_intrinsics.json`） |
| station 体 | `right_gripper` → `right_camera` |
| 顶部 BRIO | **`178B0DAE`（左工位那台，1920x1080@30）**，对焦只 verify，不动 |
| 内参 | **复用** `calib/out/top_brio_178B0DAE_intrinsics.json`（09-02，同一台相机同一档位；`solve_top.sh` 自动跳过第 1 步） |
| pair | `calib/out/top_pairs_cross/` |
| 产物 | `calib/out/top_brio_178B0DAE_in_right_base_calibration.json`（**不覆盖**左标定，**不安装**进任何桥） |
| 求解配置 | `config/right_calib.yaml`（腕相机序列号门 + 右臂 station 体） |
| 拒绝启动的端口 | **9020、9021、9022 任一有监听即拒绝**（9021 的左桥独占顶相机，9020/9022 独占右臂 CAN） |

**前提**：三个桥都停（`ss -ltn | grep -E '902[012]'` 为空）；`can_follower_r` UP；右臂急停在手边（最好有第二个人在场）；
棋盘仍是 22 mm、9x7 内角点；左顶相机的对焦/变焦自 09-02 起没动过（工具会 verify，动过则要按 §8.1 重采内参）。

**姿态**：棋盘平放桌上，12~16 个位置，**在右臂的工作区内**（ADVICE 的 r ≤ 0.50 m 现在是相对右臂基座），同时要落在左顶相机画面里
（它能看到整张桌子，通常不是问题）；每个位置 `g` 拖臂让右腕相机从 0.25~0.35 m、20~35° 看板，`h` 保持，`c` 采集。其余安全规则同 §2/§8d。

**命令**
```bash
cd hardware-bridge
uv run --locked python ../calib/capture_top_pairs.py --rig cross --dry-run     # 离线自检（不开相机不上电）
uv run --locked python ../calib/capture_top_pairs.py --rig cross               # 采 12~16 对 -> calib/out/top_pairs_cross/
RIG=cross DRY=1 bash ../calib/solve_top.sh                                      # 只打印路径
RIG=cross bash ../calib/solve_top.sh                                            # 求解 -> top_brio_178B0DAE_in_right_base_calibration.json
# 基座变换（Claude 来做）：先 --check 看与名义 (0,-0.61,0) 的差，再写文件
# 用桥的 venv python（已装 numpy；连接器 vendored 在 third_party/graph-as-policy/，不用另找 checkout）
../hardware-bridge/.venv/bin/python ../agp/tools/solve_base_transform.py \
    --top-in-right-base ../calib/out/top_brio_178B0DAE_in_right_base_calibration.json --check
../hardware-bridge/.venv/bin/python ../agp/tools/solve_base_transform.py \
    --top-in-right-base ../calib/out/top_brio_178B0DAE_in_right_base_calibration.json \
    --source "cross rig $(date +%F)"        # -> agp/config/right_base_in_left_base.json，右臂服务端启动时自动加载
```
求解报告里 `solved_from_nominal_*` 对 cross 机位**没有意义**（求解器的名义位姿是 station 中线上那台顶相机），只看
calibration/validation 的平移旋转误差是否 PASS。左顶相机的 T_lb_top 默认取 `agp/config/top_calib.json`（09-02 装机版的快照），
也可用 `--left-top-pose hardware-bridge/acceptance/top_left/top_brio_calibration.json` 直接读装机文件。
合成数据自检（09-08）：已知偏移 (12,-8,3) mm、1.5° 可被精确还原。

**已完成（2026-09-08 15:20）**：采 13 对（15:00–15:11），求解器对 pair11 的顶相机图检不出棋盘（EXHAUSTIVE 检测器的已知不稳定，直接调用同一函数 3/3 能检出），
已挪到 `top_pairs_cross/excluded/`，12 对（9 标定 + 3 验证）求解 PASS：标定平移 RMS 13.5 mm（最大 21.9）、旋转 RMS 0.88°；验证最大 18.3 mm / 1.38°。
基座变换：right_base 在 left_base 中 t = (−0.0086, −0.6249, −0.0141) m，绕 z 约 −2.3°（相对名义 (0,−0.61,0) 偏 22 mm / 2.4°），
已写入 `agp/config/right_base_in_left_base.json`，右臂服务端启动时加载（假桥验证通过）。用装机文件与快照两种左顶位姿求解结果相同。
另：左顶 BRIO 当天发现自动对焦被打开（原因不明，疑似上电复位），已关闭；`agp/tools/check_top_focus.sh` 现在在每次会话/试验/拍照/录示范前检查 autofocus=0、focus=0、zoom=100。

