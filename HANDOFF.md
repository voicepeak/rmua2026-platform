# HANDOFF — 2026-11-06

把这份文档当作接手的第一入口。它记录了当前代码、官方路线更正、远端
runtime 实测结果、作废项和下一步优先级。**旧代理结果不能当官方成绩。**

## 0. 一句话

平台骨架和 `sim-fast` 已完成，前一版 1→3 策略是**代理赛道**，已被官方
runtime 证明不匹配；现在必须按官方 shuttle 路线和官方 gate 事件重建
stage 1 再训练。远端官方模拟器已经能跑。

## 1. 工作区与远端

- 本地工作区：`D:\RMUA2026-Platform`
- GitHub：https://github.com/voicepeak/rmua2026-platform
- 当前提交：`7382180`（文档/issue 更正后）
- Issue #2（v2.1）：https://github.com/voicepeak/RMUA2026-re/issues/2
- HF 数据：https://huggingface.co/datasets/Vociepeak/rmua2026-platform-data
- HF 权重：https://huggingface.co/Vociepeak/rmua2026-platform-policies
- 远端 SSH：`100.120.200.58`，用户 `tianbot`
- SSH 密码/本地敏感信息：见 `HANDOFF.local.md`（已 gitignore，不要入库）

## 2. 已完成的节点

### Node 0：规则/坐标/数据契约

- `configs/rules_2026.yaml`：六段、时限、30s、风、工厂、传感器、控制、
  Docker 规则；**段配置当前标记为 `proxy_invalid`**。
- `src/rmua/rules/model.py`：YAML 加载与 G0 检查。
- `src/rmua/core/frames.py`：UE cm/m、UE/NED、yaw/门法向。
- `docs/rules.md`、`docs/data_contract.md`。
- `tests/test_rules.py`、`test_frames.py`、`test_map_data.py`。

### Node 1：state-only sim-fast

- `src/rmua/envs/sim_fast.py`：
  - kinematic + 6DOF quad 两种动力学；
  - 门触发体、30s 出发、段计时、力场、风、线障/车辆、碰撞、工厂前置；
  - vectorized reset/step；最近点用 SciPy cKDTree；
  - `reset_indices` 支持 PPO 自动 reset。
- `src/rmua/scene/`：map_data、track、route（Catmull-Rom）、obstacles、
  wind、forcefield。
- `src/rmua/policies/`：obs/action/safety 契约。
- `src/rmua/training/ppo.py`：CPU PPO + BC checkpoint 加载。
- `scripts/train_bc_ppo.py`：scripted demos + DAgger + BC，`--skip-ppo`。
- `scripts/eval_policy.py`：成功率/时间/平均速度。
- `scripts/benchmark.py`：raw sim 吞吐。
- 29 个测试全过；raw sim 256 env 约 214k env-steps/s（本机 20 线程）。

## 3. 关键更正（必须读）

### 3.1 官方 stage 是 shuttle，不是单条 route

官方六段：`1-3, 3-5, 5-8, 8-12, 12-10, 10-7`。

- stage 1：1 → 枢纽 → route 3 末端，触发 stage 2；
- stage 2：沿 route 3 返回枢纽 → route 5；
- stage 3：route 5 返回 → route 8；
- stage 4：route 8 返回 → route 12；
- stage 5：route 12 返回 → route 10；
- stage 6：route 10 返回 → route 7 终点。

每段 = **上一段终点路线反向返回 + 下一段去程**。不是 `1_1+1_2`，也不是
`1_1+2_1`。

### 3.2 官方 stage 1 路线

远端 `route_1_3.yaml` 注释：**`spline0 + reverse(spline2)`**。
路线文件在远端：
`/home/tianbot/RUMA-by-helinjun/beta/rmua_ws/src/route_follower/config/route_1_3.yaml`
（370 个点）。

### 3.3 门数未定

- 地图 UAsset 只有 12 个 `*_Gate` actor（`1_1_Gate`..`6_2_Gate`），更像
  枢纽/路线闸门。
- 远端 `gates_vision_1_3.yaml` 有 10 个门，但这是视觉观测文件，不是
  官方全量门表。
- 用户记忆的 60–70 更可能是**整场检测门总数**；单段是否更多尚未确认。
- **不要用 2、10、12 当官方门数。** 先做官方 gate 审计。

### 3.4 旧本地结果作废

`runs/route_seg{1,2,3}/best.pt` 和 HF `route_spline_20261106` 是在
`1_1+2_1` / 两门代理赛道上训的，**不能用于官方 stage 1**；只保留作
pipeline smoke test。HF 模型卡已加警告。

### 3.5 工厂逻辑

- stage 1–3 无工厂。
- 8–12：枢纽前 1 次。
- 12–10、10–7：枢纽前后各 1 次。
- 入口/上报触发尚未确认；先做可配置状态机，不硬编码。

## 4. 官方 runtime 实测

远端：Ubuntu 22.04，i7-12700H，20 逻辑核，38GB RAM，RTX 3060 Laptop 6GB。

### 4.1 容器修复

- 旧 `rmua_noetic` 容器有 NVML 驱动不匹配。
- 已停止并重命名旧容器为 `rmua_noetic_old_20261106`。
- 新 `rmua_noetic` 用 `rmua/noetic-xal:latest` + `--gpus all` 重建。
- 新容器 `nvidia-smi`、`/opt/conda/envs/xal` 的 torch CUDA 均正常。
- 新容器缺 `scipy`，已安装 `python3-scipy`（route_follower 需要）。

### 4.2 官方模拟器

- `run_sim.sh 123 render`（`DISPLAY=:1`）启动成功；OL1 加载。
- 话题正常：`/airsim_node/drone_1/debug/pose_gt`、
  `/airsim_node/initial_pose`、`/airsim_node/end_goal`、
  `/airsim_node/drone_1/vel_body_cmd`。
- stage 1 `end_goal=(545.56, 519.85, -30.21)`。
- 运行日志：`/tmp/sim_render.log`。

### 4.3 route_follower smoke test

- 用静态 `gates_vision_1_3.yaml` + `guides_seed123_recorded.yaml`，
  `use_gate_map=false`、`wait_for_first_map=false` 启动。
- 约 10 m/s 能飞；通过 gate 1/2/3/6；gate 0/4/5 因横向 1.89–2.48m
  超过默认 `gate_pass_half_width=1.5m` 报 MISSED。
- 日志：`/tmp/route_static.log`；run 目录：
  `beta/frames/race_20261106_152055/`。
- 这只是 runtime smoke test，不是官方成绩。

### 4.4 当前远端状态

- `rmua_noetic` 容器仍 Up；sim/rosmaster/route_follower 已被停止。
- 旧容器 `rmua_noetic_old_20261106` 保留备份。
- 主机 Xorg 在 `:1`，容器有 `/tmp/.X11-unix/X1`。
- 新的 latest 镜像没有 Xvfb；`run_sim.sh background` 会失败。
  用 `DISPLAY=:1 ./run_sim.sh 123 render`，或先在容器安装 xvfb。

## 5. 下一步优先级

1. **官方 gate 审计**：用 `route_1_3.yaml` + 官方 runtime gate 事件
   （`/rmua/controller/events`、`/airsim_node/custom_msgs`、
   `RouteGateConfigs`）确认 stage 1 门数和门序；不要用视觉 10 门当全量。
2. **重建本地 stage 1**：把 `src/rmua/scene/track.py` 从代理 route pair
   改成官方 `route_1_3`（370 点 + shuttle 返回逻辑），坐标对齐
   `end_goal`/`pose_gt`。
3. **重训 stage 1**：BC/DAgger + ≥15 m/s；旧 `best.pt` 不沿用。
4. **stage 2/3**：按 shuttle 生成 3→5、5→8，再训练。
5. **动态障碍/风**：先 smoke，再扩规模。
6. **工厂/感知**：8→12、12→10、10→7；可配置状态机，不硬编码。
7. **官方校准**：OL1、≤4 实例、新进程/容器；官方只做校准，不做吞吐。

## 6. 常用命令

本地：

    cd D:\RMUA2026-Platform
    .\.venv\Scripts\python.exe -m pytest -q
    .\.venv\Scripts\python.exe scripts\benchmark.py --envs 64,256 --steps 200
    .\.venv\Scripts\python.exe scripts\eval_policy.py --checkpoint runs\route_seg1\best.pt

训练（**旧代理，仅流程验证**）：

    .\.venv\Scripts\python.exe scripts\train_bc_ppo.py --segment 1 --skip-ppo --demo-steps 50000 --dagger-rounds 6 --dagger-steps 30000 --bc-epochs 12 --dagger-bc-epochs 6 --save-dir runs\route_seg1

远端：

    ssh tianbot@100.120.200.58
    # 密码见 HANDOFF.local.md
    docker start rmua_noetic
    cd /home/tianbot/RUMA-by-helinjun/beta
    DISPLAY=:1 ./run_sim.sh 123 render
    # 或静态 route_follower:
    # docker exec -d rmua_noetic bash -lc 'source /opt/ros/noetic/setup.bash; ... roslaunch route_follower ...'

## 7. 已知坑

- 远端旧容器驱动不匹配；不要再用 `rmua_noetic_old_20261106` 做 GPU 测试。
- `run_sim.sh background` 在新容器缺 Xvfb，会失败；用 `render` + `DISPLAY=:1`。
- `gates_vision_1_3.yaml` 10 门是视觉子集；`gates_seed123_recorded.yaml`
  18 门是记录文件；都不等于官方完整门表。
- `route_1_3.yaml` 的坐标 frame 还需与官方 `end_goal`/`pose_gt` 对齐；
  不要直接按本地 UE 坐标硬搬。
- 旧训练 checkpoint 是 1.47km 代理 + 2 门，不能用于官方 stage 1。
- 本地 `configs/rules_2026.yaml` 的 gate/route pair 已标 `proxy_invalid`。
- HF/GitHub token 不要入库；`HANDOFF.local.md` 已 gitignore。
- 远端 `route_follower` 默认 `gate_pass_half_width=1.5`，容易 MISSED；
  如需 smoke，可临时调到 3–4m 并记录为开发配置。

## 8. 相关文档

- `docs/official_stage_logic.md`：官方 shuttle/门数/工厂更正
- `docs/training_results.md`：旧代理结果 + 失效警告
- `docs/rules.md`、`docs/data_contract.md`
- `docs/19`/`docs/20`（上游逆向仓库）
- `README.md`

## 9. 交接检查清单

- [ ] 读 `docs/official_stage_logic.md`
- [ ] 确认 stage 1 官方门表（数量/顺序）
- [ ] 重建 `track.py` 的 stage 1 官方路线
- [ ] 重训 stage 1，目标 ≥15 m/s
- [ ] 按 shuttle 生成 stage 2/3
- [ ] 工厂逻辑等 8→12 再确认
- [ ] 更新 issue #2 和 HF 权重说明
