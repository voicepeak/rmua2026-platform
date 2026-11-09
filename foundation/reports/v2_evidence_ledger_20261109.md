# V2 验收证据台账（2026-11-09）

本页把 G0–G4 拆到可核查子项。**子项 PASS 不等于关口 PASS**；关口仍需所有必需子项和冻结阈值。PARTIAL/BLOCKED/UNVERIFIED 不得合并成官方等价。

## G0 基准、坐标、PWM 契约

| ID | 子项 | 状态 | 证据 | 限制 |
|---|---|---|---|---|
| G0.1 | 模拟器版本与二进制哈希 | PASS | `config/baseline_manifest.yaml`、`config/official_manifest.yaml`：12.0.0.5，SHA-256 `5231addf…b97800` | 只证明本地保存的二进制 |
| G0.2 | 设置文件语义 | PARTIAL | `reports/settings_semantic_comparison.json`：两份 JSON 解析后相同 | 未回读运行时实际加载值 |
| G0.3 | AirSim 源码对应版本 | UNVERIFIED | `baseline_manifest.yaml` 仍为 `unknown` | 未从二进制/发布记录确认 commit |
| G0.4 | PWM ROS 字段与 MD5 | PASS | `evidence/RotorPWM.msg`、`pwm_contract.md` | 消息格式不等于物理契约 |
| G0.5 | PWM 范围/裁剪 | UNVERIFIED | `pwm_contract.md`；回调反编译仅做 double→float 和 4 ms 节流 | 未确认下游裁剪 |
| G0.6 | 命令保持/超时 | UNVERIFIED | `pwm_contract.md`；服务端行为未定位 | 需要运行时实验或源码 |
| G0.7 | 电机次序与旋向 | PARTIAL | `v2_motor_direction_audit.md`：三次进程 x/y 符号 12/12 一致 | yaw/z 低于阈值；运行时名称未解码 |
| G0.8 | UE↔NED、机体系变换 | PARTIAL | `frame_spec.md`、`tests/test_frames.py` | 代数测试通过；官方原点/轴对齐未验证 |
| G0.9 | 惯量参考系与单位 | PARTIAL | `config/quad_params.yaml`：source/unit/status 已标 | 单位与参考系仍为 published/unverified |
| G0.10 | 未知量显式清单 | PASS | `config/unknowns.yaml` | 清单存在不等于未知量已解决 |

## G1 无风无碰撞 PWM 物理

| ID | 子项 | 状态 | 证据 | 限制 |
|---|---|---|---|---|
| G1.P01 | 共模稳定段重复 | PARTIAL | `v2_cadence_repeat_collection_protocol.md`：20/50/100 Hz 各三次 | 初态、接触、执行时刻未完全匹配 |
| G1.P02 | 共模阶跃重复 | PARTIAL | `v2_p02_repeatability_protocol.md`：三次，位置最大差 1.26/0.96/0.30 mm | 窗口短、初速/接触未排除 |
| G1.P03/P04 | 横滚/俯仰重复 | PARTIAL | `v2_axis_repeat_collection_protocol.md`：三组最大姿态差 1.094°/0.350°/0.768° | 初态、风、接触、时钟未分离 |
| G1.P05 | 偏航重复 | PARTIAL | 同上：三组最大姿态差 0.0046°/0.0002°/0.0047° | 窗口小，未覆盖完整包线 |
| G1.P06 | 不同命令频率 | PARTIAL | `v2_cadence_repeatability.json`：20/50/100 Hz 位置差 2.70/0.85/3.82 mm | 频率执行时刻和采样质量未独立确认 |
| G1.电机方向 | 四电机 x/y 方向 | PARTIAL | `v2_motor_direction_audit.md`：三次进程 12/12 x/y 一致 | yaw/z 未解析；运行时通道未独立解码 |
| G1.时钟 | 源时间物理一致性 | PARTIAL | `v2_ballistic_clock_audit.md`：四组干净尾段，源时间加速度 9.783–9.791 m/s² | 只有同波形；不等于指令生效时刻 |
| G1.初速度 | 独立初速度 | **BLOCKED** | `v2_initial_velocity_audit.md`：两种估计差 0.018–0.022 m/s | 两者都不是独立测量 |
| G1.校准 | 电机时间常数/延迟 | **BLOCKED** | `v2_calibration_round1/2`：最优在边界，第二轮仅改善 8.05% | 两轮已用完，不得再扩网格 |
| G1.保留集 | 封存 `v2_holdout_axis` | **UNVERIFIED** | `v2_blind_holdout_protocol.md`、`v2_blind_holdout_inventory.json` | 未读取、未评价，阈值未冻结 |
| G1.阈值 | acceptance 数值 | **PROVISIONAL** | `config/acceptance.yaml` 全为 null | 官方重复性和任务误差预算未齐备 |

## G2 地图与碰撞

| ID | 子项 | 状态 | 证据 | 限制 |
|---|---|---|---|---|
| G2.OL1 资产 | 道路/建筑/货架/门候选 | PARTIAL | `maps/OL1/collision_manifest.json`：默认 508，全可选 12,609 | 候选数量不是覆盖 |
| G2.OL2–12 资产 | 每图独立候选 | PARTIAL | `maps/OTHER_MAPS.md`：默认 508，全可选 12,597 | 无逐图官方接触 |
| G2.缺失类别 | 12 建筑 + 480 货架 | PASS（数据完整性） | `v2_map_cooked_mesh_audit.json`：每图 492 个新增实例，5,904/5,904 查询命中 | 运行时碰撞通道未验证 |
| G2.剩余类别 | 39 个静态网格等 | PARTIAL | `OTHER_MAPS.md`：12 门、24 箭头、3 环境视觉体；5 Niagara、3 电缆、2 HISM | 仅归类，不证明碰撞 |
| G2.道路接触 | 官方侧栏接触 | PARTIAL | `g2_contact_road_seed123_20261109.json`：939 回调，残差 ≤3.6e-4 cm | 单点/单路段，GDB all-stop |
| G2.港口墙接触 | 官方内墙接触 | PARTIAL | `g2_contact_portwall_seed123_20261109.json`：115 回调，残差 ≤2.1e-4 cm | 单点，不能外推 |
| G2.枢纽接触 | 路线运行接触 | PARTIAL | `g2_contact_gate1_seed123_20261109_v2.json`：234 回调，近/次近重复 | 组件归属有歧义 |
| G2.路线门语义 | 12 个 `*_Gate` | PASS（静态） | `g2_gate_collision_re20261109.md`：激活 `BlockAll`，未激活 `NoCollision` | 是实体碰撞开关，不是门洞/trigger |
| G2.检测门候选 | 33+30 运行时评分组件 | PARTIAL | `maps/OL1/detection_gate_candidates.json`、`v2_detection_gates.md` | root 变换不等于 trigger plane |
| G2.road1 score | 33 个评分组件 | **PASS（受控）** | `g2_score_route_20261109.md`：score 0→33，索引 0–32 全出现 | 单路线/单 seed；player 超时；commands 缺失 |
| G2.road3 score | 30 个评分组件 | **UNVERIFIED** | 无 stage-2 候选路线 | 未跑 |
| G2.早门净空 | 32→31→30→29 | PARTIAL | `g2_detection_gate_sequence_20261109.md`：无命令后接触，最近 0.176/0.018/0.014/0.020 m | 中心平面穿越≠官方 aperture |
| G2.单门 32 | 中心净空 | PARTIAL | `g2_detection_gate32_20261109.md`：3.07 cm，无命令后接触 | 同上 |
| G2.几何探针 | 门/道路/墙/复杂体 | PARTIAL | `v2_map_geometry_probe.json`：12/12、64/64、64/64、25/25 | 同一几何自命中 |
| G2.工厂入口 | 官方入口/事件 | **UNVERIFIED** | `g2_gate_collision_re20261109.md` 的 `meter_report` 探针 | 服务 success 无入口日志 |
| G2.原点/尺度 | 全图坐标 | **UNVERIFIED** | `frame_spec.md`、`OTHER_MAPS.md` | 未逐图官方对齐 |

## G3 风与力度衰减

| ID | 子项 | 状态 | 证据 | 限制 |
|---|---|---|---|---|
| G3.生成 | 注入阶段关键帧 | PASS（受控） | `wind_generator_validation.json`：18/18 字节精确 | 自然随机流未验证 |
| G3.自然生成 | 自然赛程风区 | UNVERIFIED | 无自然路线连续数据 | 随机数消费顺序未知 |
| G3.施力 | 六面阻力候选 | PASS（受控） | `aero_paired_*`、`aero_net_tilt_*`：误差约 1e-7 | 注入/强制条件，不是自然风 |
| G3.W01 | 有风/无风轨迹 | PARTIAL | `wind_stage4_replay_*`：短窗口 RMSE 0.094–0.320 mm 对 0.913–1.747 mm | 同记录探索，初态/时钟未独立 |
| G3.W02 | 空间触发 | PARTIAL | `v2_g3_controlled_benchmark.json`：80 子步，25 非零/55 零 | 合成区域 |
| G3.F01 | 力场边界/作用层 | PARTIAL | forced flag 0.25/1.0；`v2_g3_controlled_benchmark.json` | 强制条件，自然边界未验证 |
| G3.F02 | 2 秒累计/恢复 | PARTIAL | `v2_g3_controlled_benchmark.json`：0.36 s 超阈值，最终约 0.20 s | 合成 flag 日程 |
| G3.参数 | 0.25/2 cm/6 cm/3.8 cm | UNVERIFIED | `force_field_evidence.md`、`field_direct_pose_protocol.md` | 不是官方参数 |

## G4 障碍与批量

| ID | 子项 | 状态 | 证据 | 限制 |
|---|---|---|---|---|
| G4.线障 | 胶囊扫掠 | PARTIAL | `tests/test_obstacles.py`、`v2_collision_sweep_benchmark.json` | 合成几何；官方通道未知 |
| G4.车辆 | 可配置/分段/停车 | PARTIAL | `foundation/obstacles.py`、`v2_scenario_factory.md` | 官方尺寸/速度分布未验证 |
| G4.随机生成 | seed 可复现 | PARTIAL | `v2_scenario_factory.md`、`tests/test_scenarios.py` | 通行空间/互避未官方验证 |
| G4.高速扫掠 | 90 合成 + 768 实际地图 | PARTIAL | `v2_collision_sweep_benchmark.json`；`v2_map_sweep_miss_benchmark.json`：0/768 | 阈值未冻结；无官方漏检率 |
| G4.单例/批量 | 同 seed 末态 | PASS（自建） | `v2_batch_scenario_benchmark.json`、`test_batch_*`：最大差 0 | 不是官方对齐 |
| G4.本机吞吐 | 64/256 自由飞行 | PASS（本机） | `v2_batch_vectorized_benchmark.json`：约 4.80/5.91 万 env-cmd/s | 无同机官方对照 |
| G4.密集地图 | 12,609 候选、64/256 表面 | PASS（本机） | `v2_batch_dense_map_benchmark.json`：777.2/710.9 env-cmd/s | 顺序参考路径，非密集赛道分布 |
| G4.官方吞吐 | 同操作官方 batch | **UNVERIFIED/不可得** | `v2_official_realtime_factor.json`：官方约 1.0000–1.0004 实时因子 | 不同度量，不能换算 |
| G4.贴地误拒 | 子盒球覆盖 | PARTIAL | `v2_vehicle_map_clearance_audit.json`：N=10 在 0.1 m 间隙接受 29/64 | 探测尺寸非官方；仍保守 |

## 复现命令

```powershell
.\.venv\Scripts\python.exe -m pytest -q foundation\tests
.\.venv\Scripts\python.exe run_smoke_test.py
.\.venv\Scripts\python.exe -m foundation.audit_map_cooked_meshes
.\.venv\Scripts\python.exe -m foundation.audit_map_geometry
.\.venv\Scripts\python.exe -m foundation.benchmark_batch_dense_map
.\.venv\Scripts\python.exe -m foundation.audit_vehicle_map_clearance
.\.venv\Scripts\python.exe -m foundation.analyze_v2_identifiability
.\.venv\Scripts\python.exe -m foundation.audit_ballistic_clock
.\.venv\Scripts\python.exe -m foundation.audit_official_realtime
```

## 结论

- G0 PARTIAL；G1 PARTIAL（参数辨识 BLOCKED）；G2 PARTIAL（road1 检测门 score 33/33，但 road3/工厂入口未完成）；G3 PARTIAL/UNVERIFIED；G4 PARTIAL；总体 PARTIAL。
- 子项 PASS 仅在其证据范围内成立，不能把整关升级为 PASS，也不能宣称官方等价。
