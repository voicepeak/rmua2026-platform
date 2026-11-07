# HANDOFF — 2026-11-06

把这份文档当作接手的第一入口。它记录了当前代码、官方路线更正、远端
runtime 实测结果、作废项和下一步优先级。**旧代理结果不能当官方成绩。**

## 0. 一句话

平台骨架和 `sim-fast` 已完成；OL1 stage 1 的近似开发环境已接入训练和评估，
并用 BC+DAgger 在本地从头训练。留出 5 seed × 64 env 验收为 100% 成功、
63/63 门、67.06 s、21.99 m/s（**仅本地开发环境成绩**）。前一版 1→3 策略
使用**代理赛道**，不能沿用。新环境的枢纽、门洞和官方 stage 切换仍需校准；
远端官方模拟器能运行。

## 1. 工作区与远端

- 本地工作区：`D:\RMUA2026-Platform`
- GitHub：https://github.com/voicepeak/rmua2026-platform
- 当前 HEAD：`e5f2ae9`；stage 1 开发环境和审计材料为**未提交的工作区改动**，
  交接时应保留并读取这些改动，不能只检出该提交或覆盖工作区。
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
- `src/rmua/training/ppo.py`：CPU PPO + BC checkpoint 加载；`PPOConfig`
  现在贯通 map/track_mode/门尺寸；保存 `track` 身份元数据。
- `scripts/train_bc_ppo.py`：scripted demos + DAgger + BC，`--skip-ppo`；
  默认 `stage1_audited`，逐 episode 记录成功/门数/时间/失败原因。
- `scripts/eval_policy.py`：按 checkpoint 元数据选择赛道，拒绝无元数据或
  赛道不匹配的 checkpoint；报告成功率、63 门率、时间、速度（含失败样本）
  和失败原因。
- `scripts/benchmark.py`：raw sim 吞吐。
- 当前 41 个测试全过；旧代理 raw sim 256 env 约 214k env-steps/s。

### Node 2：stage 1 训练接入与本地训练（2026-11-06）

- `SimFastConfig` 已有 `track_mode="stage1_audited"`；训练、DAgger、PPO 和
  评估均显式传递 map/track_mode/门尺寸，旧代理路径需显式选择。
- 新增 `src/rmua/training/checkpoint.py`：保存/校验赛道身份（地图、模式、
  segment、长度、63 门数、门尺寸、观测来源），禁止新旧赛道静默混用。
- 修复训练期评估 horizon（原 20 s 截断）和自动 reset 时 episode 统计丢失。
- 在 `runs/stage1_audited_seed7_20261106/` 从头训练 BC+DAgger（seed 7，
  64 env × 30000 demo steps，4 轮 DAgger × 15000 steps，`--skip-ppo`）。
- 留出 seeds 1201/1302/1403/1504/1605 × 64 env：100% 成功、100% 63/63 门、
  平均 67.061 s、21.986 m/s；失败 0。权重和记录已上传 HF
  `policies/stage1_audited_seed7_20261106`。

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
（`route_1_3` 实际 269 点；文件另有 `route_road1` 的 101 点，合计
370 点，先前误将文件总点数当成 stage 1 点数）。其中前后各 101 点与
`splines.txt` 的 spline0 和反向 spline2 逐点相同；中间 67 点由远端
`splines_to_yaml.py` 的 `connect_routes` 生成，采用门口延长线和贝塞尔曲线，
是开发者构造的连接段，官方一致性尚未核验。

### 3.3 OL1 计分组件与限制

- 地图 UAsset 只有 12 个 `*_Gate` actor（`1_1_Gate`..`6_2_Gate`），更像
  枢纽/路线闸门。
- 远端 `gates_vision_1_3.yaml` 有 10 个门，但这是视觉观测文件，不是
  官方全量门表。
- 用户记忆是约 67 分后进入 stage 2，尚待验证；此前推测 60–70 是整场
  总数缺乏证据，应撤回。
- OL1 seed 123 的运行时审计读取到 33+30 个计分组件，顺序是 32→0、
  0→29；原生计分函数和硬件观察点确认了顺序触发与 Score 自增的联系。
  后半段有些事件发生在 `Finished` 后，63 不是已完成比赛的有效分数，
  stage 2 切换尚未验证。
- 静态 `SegmentInterval=2100`；远端 spline0/spline2 采样折线长度约
  683.03/624.45 m。若间隔为 21 m 且向上取整，候选为 33+30 段；单位、
  取整算法尚未全部确认，不能将 OL1 的结果推广到其他地图。规则 YAML 中
  `detection_gates_total: 12` 也是旧代理校验值，不是官方门数。
- **不要用 2、10、12 当官方检测门数。**

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

- 上次检查时 `rmua_noetic` 容器 Up；sim/rosmaster/route_follower 已停止。
  新接手时应重新检查当前状态。
- 旧容器 `rmua_noetic_old_20261106` 保留备份。
- 主机 Xorg 在 `:1`，容器有 `/tmp/.X11-unix/X1`。
- 新的 latest 镜像没有 Xvfb；`run_sim.sh background` 会失败。
  用 `DISPLAY=:1 ./run_sim.sh 123 render`，或先在容器安装 xvfb。

## 5. 下一步优先级

2026-11-06 全链路静态审计及破局顺序见 `docs/fidelity_audit_20261106.md`。
重点：kinematic 分支没有将风/力场因子作用于运动；力场计时器返回赛道会
清零（与规则 v1 文本“累计”需核对）；终止 env 仍更新位置/进度；checkpoint
未实际保存几何哈希；`docs/data_contract.md` 的 30 维已落后于当前 33 维。
先完善安全停止与可复现实验，校验接口和官方动力学，再决定模型调整/重训。

官方随机 seed 首次策略诊断：seed `283441`，同 seed 三次新进程，前两次
是桥接器 vz 反号导致的无效尝试；修正后加规则式垂直起飞前置，模型接管后
仍停留起点附近，内部 Score 0、约 29 s 后 Finished。一次同 seed 旧
route_follower 对照可起飞并达 x≈102.8 m。桥接用 `debug/pose_gt`、估计
速度、推断门状态、20 Hz 和受限 yaw，不能归因于模型本身或视作官方成绩。
完整结果及原始记录见 `docs/official_policy_smoke_283441.md`；下一步先
校验观测/控制适配、起飞和动力学，不要直接宣称策略通过官方验收。

后续同 seed、单实例受控实验：规则起飞+actor 影子到 x≈12.01 m（State1，
Score 1）；水平受限 actor 到 x≈30.15 m（Score 2）；再开放受限垂直
到 x≈30.04 m（Score 2）；参考近似路线的高度包络到 x≈60.12 m（Score 3）。
均为短程诊断，使用真值、监测内存及保护器，不是完整成绩；探针记录到启动
阶段碰撞回调，含义未解码。官方起步状态和本地推断有短暂差异。探针、安全
停止和原始日志见 `docs/official_policy_smoke_283441.md`。尚需验证状态、
碰撞、估计和动力学后再扩展，不要把这视为模型独立完成赛道。
随后受限诊断到达本地路径进度约 200 m（State1，Score 10），桥退出后模拟器
继续运行，约 15 s 后碰撞并进入 Finished。复测在检查点后连续发布 20 个带
时间戳的零速度命令，0.95 s 内仍移动约 2.98 m，未证明完全停止；GDB 附着时
普通 TERM 也未关闭模拟器，已解除调试并强制清理。后续必须验证进程退出，
并在诊断桥退出后立即关闭模拟器。无完整赛道成绩或 stage 2 验证。

OL1 stage 1 开发环境：`SimFastConfig(track_mode="stage1_audited")`，使用
两段地图 Hermite spline、近似枢纽连接线和运行时审计的 63 个有序检测组件，
起终点触发区独立。仅支持 OL1；旧 `proxy` 保留作旧流程比较，必须显式选择。
训练、DAgger、PPO 和评估已全部接入新模式并写入 `track` 元数据。门洞用
可配置近似平面，枢纽、触发体、传感器部署和官方 stage 切换仍待校准；详情见
`docs/stage1_environment.md`。
代理线障和车辆在该模式默认关闭，可通过 `stage1_enable_obstacles` 开启。
本次验证 pytest 41 项通过。8 m/s 脚本 smoke 是接入过程的历史验证，不是
训练或验收成绩。

本地训练（`runs/stage1_audited_seed7_20261106/`，seed 7，BC+DAgger，
`--skip-ppo`）：64 env × 30000 demo steps，4 轮 DAgger × 15000 steps。
留出 seeds 1201/1302/1403/1504/1605 × 64 env：100% 成功、100% 63/63 门、
平均 67.061 s、21.986 m/s，失败 0。该结果只代表本地开发环境，不代表官方
等价成绩。权重、配置、episode 记录和验收 JSON 在 HF
`policies/stage1_audited_seed7_20261106`。

最新 gate 审计：OL1 seed 123 中读取到第一条路 33 个、第二条路 30 个
计分组件；顺序分别为 32→0、0→29。原生 `0x2fc8d80` 按待触发队首
检查并增加 Score，硬件观察点记录了对应增分。运行时组件位置和日志在
`data/audits/ol1_seed123/`。第二次实验在 43 分进入 Finished（原因未
确认），后续回调仍继续到 63；不能称为有效成绩，也尚未验证 stage 2
切换或坐标/门洞。完整证据及局限见 `docs/official_stage_logic.md`。
第三场 15 m/s 诊断飞行在 State1 完成首路 33 个增分，随后在枢纽附近以
33 分结束；回溯是通用事件分发器，具体失败原因和 stage 2 切换未确认。

2026-11-06 审计补充：`/rmua/controller/events` 是 route_follower 自己的
诊断事件，源码明确说明不是独立比赛计分；不能据此确认官方门表。OL1 seed
123 静止时 `/airsim_node/custom_msgs` 未出现在话题列表，尚未测试过门后是否
出现。`RouteGateConfigs` 只列出 12 个命名路线闸门引用，不证明检测门全量。
完整比赛的有效分数和 stage 切换仍未确认。模拟器检查后已停止，容器仍运行。详见
`docs/official_stage_logic.md` 的审计记录。
后续用 `pose_gt`、视觉门子集和记录 guides 做了单实例诊断飞行；控制器到达
其路线终点，但没有确认官方 stage 切换。周期采样未观察到 `custom_msgs`，
不能排除短暂话题。模拟器和控制器已停止，容器仍运行。

1. ~~接通新环境~~：训练、评估和 PPO 构造器均已显式选择并记录
   `stage1_audited`，checkpoint 校验赛道身份，旧代理必须显式选择。
2. **校准与验证**：复核门触发、坐标、枢纽近似和成功判定；本地策略已能
   100% 完成 63 门，但官方 stage 2 切换和有效比赛仍未验证。
3. ~~重训 stage 1~~：已从头 BC/DAgger，留出 seed 验收 100% 成功、21.99 m/s；
   结果只称为本地开发环境成绩。
4. **stage 2/3**：按 shuttle 生成 3→5、5→8，再训练。
5. **动态障碍/风**：先 smoke，再扩规模。
6. **工厂/感知**：8→12、12→10、10→7；可配置状态机，不硬编码。
7. **官方校准**：OL1、≤4 实例、新进程/容器；官方只做校准，不做吞吐。

## 6. 常用命令

本地：

    cd D:\RMUA2026-Platform
    .\.venv\Scripts\python.exe -m pytest -q
    .\.venv\Scripts\python.exe scripts\benchmark.py --envs 64,256 --steps 200
    .\.venv\Scripts\python.exe scripts\eval_policy.py --checkpoint runs\stage1_audited_seed7_20261106\best.pt --seeds 1201,1302,1403,1504,1605 --envs 64

训练（**新环境，本地开发**）：

    .\.venv\Scripts\python.exe scripts\train_bc_ppo.py --track-mode stage1_audited --map RMUA2024OL1 --segment 1 --demo-envs 64 --demo-steps 30000 --bc-epochs 12 --dagger-rounds 4 --dagger-steps 15000 --dagger-expert-prob 0.0 --dagger-bc-epochs 8 --hidden 128 --seed 7 --skip-ppo --save-dir runs\stage1_audited_seed7_20261106

训练（**旧代理，仅流程验证**，必须显式加 `--track-mode proxy`）：

    .\.venv\Scripts\python.exe scripts\train_bc_ppo.py --track-mode proxy --segment 1 --skip-ppo --demo-steps 50000 --dagger-rounds 6 --dagger-steps 30000 --bc-epochs 12 --dagger-bc-epochs 6 --save-dir runs\route_seg1

旧代理 checkpoint 评估需显式确认：

    .\.venv\Scripts\python.exe scripts\eval_policy.py --checkpoint runs\route_seg1\best.pt --allow-legacy-proxy

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
- `eval_policy.py` 会拒绝没有 `track` 元数据的旧 checkpoint；旧代理评估需
  `--allow-legacy-proxy`，不要绕过该检查来混用新旧赛道。
- 本地 `configs/rules_2026.yaml` 的 gate/route pair 已标 `proxy_invalid`。
- HF/GitHub token 不要入库；`HANDOFF.local.md` 已 gitignore。
- 远端 `route_follower` 默认 `gate_pass_half_width=1.5`，容易 MISSED；
  如需 smoke，可临时调到 3–4m 并记录为开发配置。

## 8. 相关文档

- `docs/official_stage_logic.md`：官方 shuttle/门数/工厂更正
- `docs/stage1_environment.md`：新环境入口、训练/评估接入、校准边界
- `docs/training_results.md`：新 stage 1 本地结果 + 旧代理失效警告
- `docs/rules.md`、`docs/data_contract.md`
- `docs/19`/`docs/20`（上游逆向仓库）
- `README.md`

## 9. 交接检查清单

- [ ] 读 `docs/official_stage_logic.md`
- [x] 确认 OL1 seed 123 的 63 个计分组件和顺序（完整比赛切换仍待验证）
- [x] 建立 stage 1 开发环境（枢纽与门洞仍为近似）
- [x] 将训练、评估及 PPO 路径显式接入新环境并记录模型元数据
- [x] 重训 stage 1，留出 seed 验收 100% 成功、21.99 m/s（本地开发环境）
- [ ] 用官方模拟器校准门洞/枢纽并验证 stage 2 切换（≤4 实例）
- [ ] 按 shuttle 生成 stage 2/3
- [ ] 工厂逻辑等 8→12 再确认
- [x] 更新 HF 权重说明和 Issue #2
