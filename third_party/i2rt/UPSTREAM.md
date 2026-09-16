# vendored i2rt

本目录**不是** i2rt 的完整副本，只包含把上游还原成我们实验所用状态需要的最小集合。

- 上游: https://github.com/i2rt-robotics/i2rt
- 固定 commit: `2c2f3ae7a9040e3ce892873eb9aa744eb286063f`
- 许可: MIT（见 LICENSE，上游原文保留）
- **本副本已修改**：10 个已跟踪文件被改动，另有 7 个新增文件。

## 为什么不直接 fork 并提交改动

桥的 preflight 会校验 `git diff HEAD` 的 sha256（见 `hardware-bridge/src/agp_yam_bridge/preflight.py` 与 `make_config_sha.sh`）。
一旦把改动 commit 进上游 fork，diff 变空、sha 改变，桥会拒绝启动。因此改动以 patch 形式分发，**应用后不要提交**。

## 复现步骤

```bash
git clone https://github.com/i2rt-robotics/i2rt.git
cd i2rt && git checkout 2c2f3ae7a9040e3ce892873eb9aa744eb286063f
git apply /path/to/third_party/i2rt/tracked_diff.patch      # 不要 commit
cp -r /path/to/third_party/i2rt/untracked/. .               # 7 个新增文件
/path/to/third_party/i2rt/verify_i2rt.sh "$PWD"
```

## 校验

- `expected_sha256.txt` = `27f1ec6f94218dd651d1826a30fbc44b8e52f577102d9f2413a9a3f5d116c440`
  这是**本机发布当日**算出的值。`git diff` 的字节输出依赖使用者的 git 版本与配置
  （diff.algorithm / renames / core.autocrlf / .gitattributes），所以不承诺逐字节可复现。
- 若 sha 不符，改用 `file_digest.txt` 逐文件比对内容摘要，这一项与 git 环境无关。

## 改动的已跟踪文件（10）

- i2rt/motor_drivers/dm_driver.py
- i2rt/robot_models/station/yam_station_linear_4310_d405/README.md
- i2rt/robot_models/station/yam_station_linear_4310_d405/yam_station_linear_4310_d405.urdf
- i2rt/robot_models/station/yam_station_linear_4310_d405/yam_station_linear_4310_d405.xml
- i2rt/robots/config/yam_v1.yml
- i2rt/robots/get_robot.py
- i2rt/robots/motor_chain_robot.py
- i2rt/robots/tests/test_robot_variants.py
- i2rt/robots/utils.py
- i2rt/utils/viser_control_interface.py

## 新增的未跟踪文件（7）

- examples/command_joint_angles/command_joint_angles.py
- i2rt/robots/tests/test_command_joint_angles_example.py
- i2rt/robots/tests/test_gravity_idle_barrier.py
- i2rt/robots/tests/test_render_yam_joint_angles.py
- i2rt/robots/tests/test_viser_control_interface.py
- scripts/render_yam_joint_angles.py
- scripts/yam_mujoco_passive_preview.py

> 上游的 `artifacts/`（23 个渲染产物）与 `MUJOCO_LOG.TXT` 是一次性输出，未随本副本分发。
> 79 MB 的 STL 模型也不再 vendor，固定的上游 commit 已能提供。
