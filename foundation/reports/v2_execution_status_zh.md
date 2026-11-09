# PWM 基础模拟器 V2：执行与验收状态

> 最新状态：见文末 2026-11-09 22:30 G2 stage-1 road1 完整 score 序列及 [总体验收](foundation_acceptance.md)。
>
> 2026-11-09 目标更新：完成完整 V2；优先推进未完成关口；每个阻塞点允许少量尝试，若没有可收敛结果或策略则停止并明确报告，不用近似证据填充。

执行依据：[V2.0 方案全文](implementation_plan_v2_zh.md)。本页是迁移时的初始状态，不将以往候选数据自动升级为通过证据；每项状态应随证据更新。逐子项状态见 [V2 证据台账](v2_evidence_ledger_20261109.md)。

**目标：**交付以四路 PWM 为唯一动作、无需 UE/ROS 步进、可校准并可批量运行的基础模拟器；保留可追溯静态地图、风和力度衰减模型、随机车辆和细线，以及最小训练事件。各模块按证据分别标记 PASS、PARTIAL 或 BLOCKED；PARTIAL/BLOCKED 不可冒称官方一致。正式策略不可默认读取仿真真值。

**线程目标：**已按 V2 重新建立并开始执行。

| 关口 | 迁移时状态 | 下一步及必要证据 |
| --- | --- | --- |
| G0 基准、坐标和 PWM 契约 | PARTIAL | 核查二进制、配置和源码版本；整理参数来源、单位、状态及未知量；确认消息范围、保持/超时、旋向和坐标测试。 |
| G1 无风无碰撞 PWM 物理 | PARTIAL | 无运行时断点采集 P01–P06；每种至少三次独立重复，区分拟合集和保留集；核实初态、时间与无碰撞条件；再按冻结阈值评价。 |
| G2 OL1 地图与碰撞 | PARTIAL | 整理资产及坐标来源、碰撞清单和未覆盖区域；验证门洞、道路净空、代表性墙体及工厂入口。 |
| G3 风与力度衰减 | PARTIAL | 在 G1 通过后按 W01/W02/F01/F02 分别核查风生成、空间触发、施力及力场边界、作用层级、恢复与 2 秒规则；未知机制明确标注。 |
| G4 障碍和批量环境 | PARTIAL | 验证随机生成、通行空间、高速连续碰撞；单例与批量同 seed 对照，再测 64/256 环境吞吐并保存硬件及官方比较数据。 |
| 总体验收 | PARTIAL | 提供 smoke test 和各关口的证据、适用包线与禁止宣称范围；不能以现有单测通过替代物理验收。 |

优先顺序：SIM-01 → SIM-02 → SIM-03，G2 可独立推进；G3 以通过的 G1 为前提，G4 的物理相关联调和批量验收随后进行。已有数据继续保留，但需逐条核对采集方式、运行状态、重复性和初始状态才能复用。原始 GDB 采集只作为定位线索，不作为运行时动力学时序验收数据。

阈值在 `acceptance.yaml` 中以指标、数值、测试集、窗口和来源冻结，先依据官方重复性和任务误差预算制定；预算未确定时标为 PROVISIONAL，不宣布训练级精度完成。最多进行两轮有明确假设、预期和结果的模型校准；未改善时停止扩展该模块实验，保留阻塞报告与可辨识的下一实验。此规则不妨碍独立模块推进。

2026-11-08 迁移时已有 [88 项单测通过的历史记录](current_progress_zh.md)，但 G0–G4 的 V2 交付和验收尚未逐项审计完成。旧的字段命名、目录结构和报告不因存在而自动满足 V2 指定交付。

G0 初步审计已建立 [基线清单](../config/baseline_manifest.yaml)、[PWM 契约](pwm_contract.md)和[未知量清单](../config/unknowns.yaml)。更正：历史运行配置快照与另存的 Game2024 文件原始哈希虽不同，但 JSON 解析后的内容完全相同；前者为 UTF-8，后者为 UTF-16LE，见[比较结果](settings_semantic_comparison.json)。这不证明模拟器运行时实际读取并应用全部设置，仍需核实。坐标与模拟器相关的 9 项现有测试通过；G0 仍为 PARTIAL。

进一步核查发现逆向仓库的实际 Git checkout 位于 `D:/RMUA2026-re/repo`，而非其父目录；2026-11-09 检查时 HEAD 为 `d38ce6c9ac9a89a57ce3a6b2ea18bd30454cd81c`，工作区干净、origin 为 `voicepeak/RMUA2026-re`。已修正基线中“当前版本未知”的早期记录；该提交不证明已发布模拟器使用的 AirSim 版本。

G1 已按预先方案完成三次新的无断点横滚/俯仰和偏航采集，并和历史数据进行固定窗口重复性比较；结果、初态和局限见 [P03–P05 记录](v2_axis_repeat_collection_protocol.md) 和 [P02 记录](v2_p02_repeatability_protocol.md)。横滚/俯仰的两两相对姿态最大差最高 1.094°，尚未区分初态、时序、风和碰撞影响；不能据此冻结精度阈值。自动测试通过仍不代表 G1 通过。

已新增 [PWM 单环境运行接口](v2_runtime_api.md)：区分命令保持周期与固定物理步长、默认隐藏研究真值，并提供候选事件和超时。底层仍为未校准模型，G1 等关口状态不变。

G2 已建立 [OL1 碰撞清单](../maps/OL1/collision_manifest.json)：默认场景 508 个候选实例，显式启用全部可选分组后为 12,117 个；这只是资产加载数量，不是官方碰撞覆盖率。清单列明源哈希、缺失分组和查询限制；集成测试验证了加载的道路候选三角面能够在 PWM 运行接口中触发碰撞事件，但不证明与官方接触结果一致。G2 仍为 PARTIAL。

G1 的[回显审计](v2_pwm_echo_audit.md)分别保留发送记录、调试消息到达和 Header 时间；这些时间差不是已测的物理延迟。已建立 [PROVISIONAL 验收配置](../config/acceptance.yaml)，未确定的阈值和保留集保持空值。按 [P01/P06 重复采集方案](v2_cadence_repeat_collection_protocol.md)补齐频率实验。

此前产生但尚未汇总的[无断点短窗口回放](non_debug_replay_protocol.md)已补充结果：两组位置 RMSE 6.29/7.17 mm；初速度取自含未来位姿的拟合，实际指令生效时间、接触和风未确认，不能据此宣布 G1 通过。

新增可直接运行的 [run_smoke_test.py](../../run_smoke_test.py)，在合成参数下依次检查 PWM 驱动起飞/运动、候选风力、记录型力场衰减、线障接触和终止，已本地运行成功。该检查使用合成场景，不能证明官方对齐、真实地图覆盖或批量性能；对应 G1–G4 状态不因此改变。

[P01/P06 频率重复采集](v2_cadence_repeat_collection_protocol.md)的六次新进程均完成；在固定窗口中，20/50/100 Hz 同频率三次的两两相对位置最大差分别为 2.70/0.85/3.82 mm。无碰撞条件、输入执行时刻、独立保留集和验收预算仍未确认，G1 保持 PARTIAL。

V2 [第一轮有限校准](v2_calibration_round1_protocol.md)仅调整候选电机时间常数和统一延迟，拟合集平均角速度 RMSE 下降 25.28%，相同波形的另外两个进程复查下降 19.12%。最优值在搜索边界且物理时钟、初态、风和接触未消歧，因此未将参数升级为官方值，也未修改默认模型；这不等于 G1 通过。

V2 [第二轮有限校准](v2_calibration_round2_protocol.md)将小幅输入纳入拟合集；最优目标改善 8.05%，低于预定的 10% 筛选要求且仍在搜索边界。按两轮限制停止进一步参数网格扩展：电机响应/延迟辨识子问题暂时 BLOCKED，G1 整体保持 PARTIAL，独立地图和障碍工作继续。封存保留集未用于两轮校准。

G0 的[坐标契约](../frame_spec.md)现包含显式候选姿态变换及其逆变换；单测覆盖旋转矩阵双重反射、方向符号、批量往返与无效输入。该代数验证不证明官方运行时采用相同机体系、全局原点或角速度符号，G0 仍为 PARTIAL。

已按[保留集方案](v2_blind_holdout_protocol.md)采集并封存三个新进程的不同混合波形数据，仅核对成功状态、文件哈希及时间覆盖；尚未用它们计算模型误差或调整参数。拟合集模型、时间假设、任务误差预算和数值阈值冻结之前不得将其当作最终验收数据。

官方采集器新增 GPS、`/clock` 与 ROS 时间设置记录。一次[无断点时钟核查](v2_clock_gps_probe_protocol.md)收到 GPS 数据，启动时 `/use_sim_time=false` 且未发现或收到 `/clock`；ROS Header 推进与主机到达时间在该窗口接近，但尚不能证明物理时间和命令执行时间同步。

随机障碍新增种子可复现的预留航线走廊和车辆全时间范围检查；[90 个合成高速扫掠样例](v2_collision_sweep_benchmark.json)均符合各自解析预期。覆盖、保守近似和遗漏见[障碍报告](obstacles.md)；真实地图可通行性、车辆互避和官方碰撞仍未验证，G4 保持 PARTIAL。

新增[顺序参考批量接口](v2_runtime_api.md)：同 seed 输入下，64/256 个无扰动实例与单例最终状态一致；该次本机测量约 2606/2543 环境命令每秒，完整配置和限制见[基准](v2_batch_reference_benchmark.json)。这是单次、非向量化测量，未测官方同机吞吐比，不能视为高速或 G4 完成。

参考接口随后新增[自由飞行向量化快路径](v2_batch_vectorized_benchmark.json)：最近一次本机三次短试验的 64/256 环境吞吐中位数约 4.80 万/5.91 万环境命令每秒，最终状态与单环境最大差约 1.1e-16。地图、风、力场、障碍、门或超时场景仍回退到顺序参考实现；没有同机官方吞吐对照，G4 保持 PARTIAL。

批量回退路径新增合成复杂场景逐步对照，覆盖墙面、门、风、衰减、线障、移动车辆与超时。[64/256 配置场景的本机基准](v2_batch_scenario_benchmark.json)中位吞吐约 884/794 环境命令每秒，单例/批量末态一致；但位置远离地图和障碍物，未测实际碰撞密集负载或同机官方吞吐，G4 仍为 PARTIAL。

G2 查询新增显式可选的封闭网格内部检测，修补假定实体内部起步时不会穿过表面而漏报的情况；开放网格或射线结果分歧通过 `map_containment_unknown` 报告。默认 OL1 的 5 类网格中 4 类索引拓扑封闭、`Building_Port` 开放；拓扑封闭并不证明官方碰撞体为实心，也不代表地图覆盖完整，G2 仍为 PARTIAL。

修正候选门事件与碰撞的同子步顺序：由于当前碰撞检测没有精确接触时刻，同一子步的门穿越不再计入普通候选穿越，而单独标为 `ambiguous_gate_crossing`；已有较早子步的候选穿越继续保留。该机制不推断官方判罚或真实穿门语义。

力场候选不再隐式写死 0.25：若配置可能发生衰减的力场，调用方必须显式提供电机目标比例，事件返回所选数值。0.25 在特定受控条件下有证据，但自然场景边界和作用仍未完成验收，G3 保持 PARTIAL；仅含正常力场标志的风回放不需指定该参数。

显式系数变更后，用历史受控力场恢复记录重跑三个固定模型；[新结果](field_recovery_integrated_explicit_scale_seed123.json)与历史报告的三组误差指标完全一致。这是实现回归检查，不是新的官方自然力场验收。

候选风生成器现可通过 [CandidateWindField](../wind_field.py) 与气动力模型分层接入 PWM 环境：区域、源坐标到 NED 的变换和时间偏移均由调用方显式提供。模型覆盖不足、缺少气动力模型或重复指定风源时会拒绝执行；这些工程规则不验证官方自然风区或 W01，G3 仍为 PARTIAL。

新增 [ScenarioFactory](v2_scenario_factory.md)，按调用方明确指定的固定、随机、困难配置和 seed 生成受保护航线障碍并创建 PWM 环境；清单保留实际几何和哈希，环境不会在已检查的车辆运动时限之后继续运行。官方场景分布、真实地图可通行性及障碍间互避仍未验证，G4 保持 PARTIAL。

场景工厂现支持按显式段长、转角、速度和停车区间生成可复现的车辆分段路线，并对整条路线保守检查保护走廊、起飞区和场景边界。路线转向为瞬时简化，地图可行性、车辆间互避和官方车辆行为仍未证实。

场景工厂现要求调用方显式设置车辆的 NED 中心高度，直线和分段路线都会保持该高度并检查边界，避免之前在三维空间任意高度生成车辆。该值仍需依据道路与资产实测配置，尚不支持自动贴合坡道，不能宣称官方道路行为一致。

场景工厂新增线线、线车、车车的保守路径包络间隔检查，默认不允许包络相交，可指定额外间隔；无可行位置时失败。这可能拒绝时间上不会相撞的路线，且不替代官方地图净空或车辆行为验证。

风场种子核查确认本地关键帧生成使用独立纯 Python 随机数流；已修正 float32 时间四舍五入到末关键帧时可能沿用旧采样索引的问题，并拒绝 float32 下重合的关键帧。自然赛程随机数消费顺序仍未验证。

补充障碍场景与独立风 seed 的联合复现测试：相同障碍和风 seed 的两个合成环境逐状态一致；仅更换风 seed 时响应改变。最新本地复核 126 项自动测试及合成冒烟测试通过，G0–G4 状态不因此升级。

新增 [G1 原始采集清点](v2_g1_capture_audit.md)：主波形 P02、P03/P04、P05 和各 P06 频率均有三份通过元数据核对的普通采集，小幅横滚/俯仰与偏航各一份。此清点不证明无碰撞、同步或验收通过；两轮校准停止规则仍生效。

G0 复核历史源码快照和已保存反编译片段，仍不足以确认官方 PWM 裁剪与超时，限制已补入 [PWM 契约](pwm_contract.md)。同时修复本地运行接口将极短超时误接受为零个物理步的问题；现在至少需要一个物理步。新增边界测试后 127 项测试和合成冒烟测试通过；这不改变 G0–G4 验收状态。

补齐方案要求的最小越界代理事件：调用方可显式提供 NED 轴对齐边界，环境在无人机中心首次处于界外的物理步终止；同一步门事件标为顺序不明，批量环境回退参考路径。该边界不自动取自障碍生成范围，也未验证官方边界，检测限制造成的适用范围见 [运行接口](v2_runtime_api.md)；G2 和总体验收仍为 PARTIAL。

从 OL2–OL12 各自完整序列化地图提取了独立道路及建筑候选，并增加来源校验与查询加载器；每图 483 条道路、8,682 个建筑组件，默认实际加载 508 个道路和建筑实例。OL1 同流程重建与现有候选逐项相同；其他地图来源、生成方式与缺口见 [跨地图说明](../maps/OTHER_MAPS.md)。这不是官方碰撞或自由空间验收，G2 仍为 PARTIAL。

OL2–OL12 各自新增 12 个门网格候选及来源校验的穿越代理加载器。OL2 的 [六次历史门根对照](../maps/OL2/gate_runtime_evidence.md)均成功匹配 12 个变换；数据来自修改配置的 all-stop GDB 启动，不能据此证明门洞、碰撞或飞行验收。其他地图无对应运行时验证，G2 仍为 PARTIAL。

各地图又从本图资产提取道路附属立方体和货架附件链；跨地图查询支持显式选用建筑简单形状、道路立方体和货架候选，并核对来源。11 图均可加载 11,997 个实例，但仍遗漏 12 个建筑组件、480 个货架纸张/指针及其他类别；碰撞通道和官方接触尚未证实。详见 [跨地图说明](../maps/OTHER_MAPS.md)，G2 状态不变。

对第二轮四条拟合记录的固定窗口进行了[风消息审计](v2_calibration_wind_audit.md)：四场各 25 条调试风消息均为零，但消息间及窗口边缘未连续观测，物理风作用、接触和时钟仍未排除。没有重新拟合或读取封存集；两轮校准停止规则仍然生效。

补充 [64/256 环境候选地图表面接触测量](v2_batch_map_contact_benchmark.json)：默认和全部可选场景均实际发生候选地图碰撞，批量和独立单例事件及末态一致。测量使用同一高速起点，空间筛选只涉及两个实例，没有扰动或同机官方对照；不能代表密集赛道性能，G4 仍为 PARTIAL。

随机场景现可显式用运行时同一候选地图过滤障碍的完整路径包络；封闭网格内部和包含关系不明均拒绝。车辆包围球会保守拒绝一些贴地路线，部分地图和官方净空仍未验证，详细用法和限制见 [场景工厂](v2_scenario_factory.md)；G2/G4 状态不变。

OL2–OL12 各自新增 108 个 sprite 碰撞候选及来源核对；全部可选组启用时每图加载 12,105 个实例。OL1 为 120 个 sprite 候选，差异印证不能直接套用 OL1 的组件清单。实例级 sprite 资源覆盖若出现将标为未解析；运行时通道和官方接触仍未知，见 [跨地图说明](../maps/OTHER_MAPS.md)，G2 保持 PARTIAL。

对第二轮拟合记录的起点做了[初始速度估计敏感性审计](v2_initial_velocity_audit.md)：只用过去样本与前后样本的二次拟合相差约 0.018–0.022 m/s；两者都不是独立速度测量。没有回放或再次拟合模型、没有读取封存集；G1 参数辨识阻塞与停止规则不变。

补充同版本 ROS 库和[历史无断点传输证据](v2_ros_transport_evidence.md)：PWM 回显值与 Header 来自分别请求的 PWM 和 IMU 数据，回显 Header 不能代表电机采样时间。六场历史探针只界定 ROS 传输和数值匹配观察，未测物理执行；已修正回显报告的时钟解释，G0/G1 状态不变。

## 2026-11-09 02:00 会话中断交接

> 本段用于续接，不升级任何关口状态。中断时会话记录 145 项自动测试通过；最后两次审计（PWM 回显、ROS 传输证据）完成后未重跑全量测试。

- **仓库状态**：`main` @ `49fb7c0`，工作区干净；V2 实现与证据位于 `foundation/`。
- **本轮新增证据**：风调试消息审计（四场各 25 条零值，非连续观测）；初始速度估计敏感性（0.018–0.022 m/s，非独立测量）；64/256 候选地图实际接触基准（默认 645/637，全启用 570/576 env-cmd/s）；场景工厂地图包络过滤；OL2–OL12 各 108 个 sprite 候选（全启用每图 12,105 实例）；ROS 传输证据修正 PWM 回显 Header 来自 `GET_IMU_DATA`。
- **当前阻塞（按优先级）**：
  1. **G1/G0**：物理时钟与指令实际生效时刻不可辨识；回显 Header 非电机采样时间；初始速度无独立测量；接触与风未排除。两轮校准已用完，最优仍在搜索边界且第二轮仅改善 8.05%（低于 10% 门槛），按规则停止参数网格扩展。
  2. **G2**：各图仍缺 12 个建筑组件、480 个货架纸张/指针及部分实例级 sprite 资源；门洞、道路净空、代表性墙体和工厂入口未对官方接触结果验证。
  3. **G3**：自然风区生成/触发与 W01、F01–F02 边界未验证；以 G1 通过为前提。
  4. **G4**：密集地图负载、同机官方吞吐对照、车辆包围球贴地误拒未解决。
- **下一步**：只在无运行时断点条件下寻找独立的时钟、电机状态或接触证据；若不可得，保存参数区间/模型族与不可辨识范围并标 `BLOCKED`，转 G2 独立推进。G1 阈值须在官方重复性与任务误差预算齐备后冻结；封存保留集在此之前不得读取。G3/G4 按依赖顺序推进，不得用 `PARTIAL/BLOCKED` 宣称官方等价。
- **续推入口**：[方案](implementation_plan_v2_zh.md)、[验收阈值](../config/acceptance.yaml)、[ROS 传输证据](v2_ros_transport_evidence.md)、[初始速度审计](v2_initial_velocity_audit.md)、[G1 采集清点](v2_g1_capture_audit.md)、[第二轮校准](v2_calibration_round2_protocol.md)、[跨地图说明](../maps/OTHER_MAPS.md)、[OL1 碰撞清单](../maps/OL1/collision_manifest.json)。测试：`.\.venv\Scripts\python.exe -m pytest -q foundation\tests`；冒烟：`run_smoke_test.py`。

## 2026-11-09 03:38 G2 缺口：12 个建筑与 480 个货架 cooked 网格

- **改动**：`CandidateMap.load_serialized_candidates` 新增显式 `building_mesh_fallback`、`rack_mesh_fallback` 和 `rack_mesh_directory`；`CandidateMap.load` 新增 `include_unverified_building_meshes`、`include_unverified_rack_meshes`。两者都要求先启用对应 simple/rack 组，默认场景不变。
- **纳入内容**：每图 12 个 `AirSimROS_SM_plane` 建筑组件和 480 个 `Rack_Paper`/`Rack_Pointer` 货架组件，使用已解码的 PhysX cooked 三角网格前缀；`.npz` 与提取 `.json` 的 SHA-256 都进入 `source_sha256`。
- **计数**：OL1 默认 508；六个可选组全开 12,609。OL2–OL12 默认 508；六个可选组全开 12,597。每图新增 492 个候选；`excluded_building_components=0`、`excluded_rack_components=0`（仅指这两类缺口，其他 actor 类别仍未纳入）。
- **审计**：[v2_map_cooked_mesh_audit.json](v2_map_cooked_mesh_audit.json)（SHA-256 `7293e1019b6d4ad93d76a23cae41f983e280ab127936b13f5e724fe024708018`）记录 12 图共 5,904 个新增实例的变换查询，全部命中预期 actor/component；这是加载与查询自洽检查，不是官方接触。
- **清单**：[OL1 碰撞清单](../maps/OL1/collision_manifest.json) 已更新为 12,609 全可选实例，SHA-256 `982e2b14f6dbfd2819c71026f2747190811a741c7b25391c9c416a26b4e480fc`；`known_gaps` 明确 cooked 网格的运行时碰撞通道、箭头/环境视觉体/电缆/特效等未纳入项仍未验证。
- **复查**：`python -m pytest -q foundation\tests` 共 **148 项通过**；`python run_smoke_test.py` 通过（合成集成，非官方对齐）。
- **状态**：G2 仍为 **PARTIAL**。门洞、道路净空、代表性墙体和工厂入口的官方接触结果仍未取得；不能把候选实例数或本审计当作官方碰撞覆盖率。

## 2026-11-09 03:45 G4 密集候选地图负载

- **新增**：`foundation/benchmark_batch_dense_map.py` 加载 OL1 六个可选组共 12,609 个候选实例，64/256 个环境分别从不同候选表面采样初态，各执行一个 0.01 秒高速接近指令。
- **结果**：[v2_batch_dense_map_benchmark.json](v2_batch_dense_map_benchmark.json)（SHA-256 `b27188f9114b7b4405931701cb4cb1745ed3c736616615939a9451d16d37f259`）两次短测中位批处理吞吐约 777.2/710.9 环境命令每秒，独立单例约 777.0/716.5；64/256 个环境全部报告候选地图接触，批量与单例事件及末态一致，状态最大差为 0。
- **限制**：顺序参考路径，不是向量化加速结论；单次局部查询中位候选筛选数约 2，不是官方密集赛道分布；没有同机官方吞吐对照；候选变换、碰撞通道和接触响应仍未验证。
- **状态**：G4 仍为 **PARTIAL**。这补齐了“密集地图负载”的本地基准，但官方同机对照和车辆贴地保守误拒仍未解决。

## 2026-11-09 03:55 G4 车辆贴地保守误拒

- **改动**：`CandidateMapObstacleFilter` 从单个整车外接球改为 N×N×N 子盒球并集，默认 N=2；`ScenarioFactory(vehicle_subdivisions=N)` 可显式增大，清单记录该值。瞬时转向处仍用整车外接球覆盖中间朝向，保持保守。
- **审计**：[v2_vehicle_map_clearance_audit.json](v2_vehicle_map_clearance_audit.json)（SHA-256 `b119ab5a04ebad8937c6e1d58c4daf5d6ebc82da1ba3ac916dd5218a6a11c022`）在 64 个 OL1 顶面道路三角、显式探针车辆半尺寸 `(0.8, 0.4, 0.3) m` 上测试。间隙 0/0.1/0.3/0.5 m 时，单球接受 0/0/0/0；N=2 接受 0/0/1/45；N=4 接受 0/0/45/46；N=10 接受 0/29/45/46。
- **限制**：探针尺寸不是官方车辆资产；顶面采样可能与其他候选几何重叠；球并集仍比盒体大，精确贴地接触仍会拒绝；转向处仍用外接球；接受的路线不是官方可通行证明。
- **状态**：G4 仍为 **PARTIAL**。贴地误拒已量化并显著缓解，但官方同机吞吐和官方车辆/道路接触仍未获得。

## 2026-11-09 04:55 G4 实际候选地图高速扫掠漏检

- **新增**：`foundation/benchmark_map_sweep_miss.py` 和 [v2_map_sweep_miss_benchmark.json](v2_map_sweep_miss_benchmark.json)（SHA-256 `8c4b8ac6e67475cf68ac82ee5dba9d1eb81ac18164b6e78ba2f399b573cc86fd`）。
- **方法**：从 OL1 候选地图取 16 个道路顶面和 16 个非道路表面，各构造沿法向穿越的 2 m 路径；速度 10/50/100/150 m/s，最大子步 0.001–0.1 s；用直接连续球扫作为参考，再让 `PhysicsSim` 实际步进，统计候选碰撞是否发生。
- **结果**：32 个案例 × 4 速度 × 6 子步 = 768 次仿真，**0 漏检**；每个速度/子步组合的漏检率均为 0。该结果是候选地图在同一条扫掠实现下的自洽检查，不是官方漏检概率，也没有冻结的验收阈值。
- **限制**：只覆盖直线法向穿越，不含擦边、转向或运动障碍；候选变换、通道和接触响应未验证；不能把 0/768 当作官方碰撞漏检率。
- **状态**：G4 仍为 **PARTIAL**。

## 2026-11-09 05:05 G1 四电机差动脉冲方向

- **新增**：[v2_motor_direction_audit.md](v2_motor_direction_audit.md) 与 [v2_motor_direction_audit.json](v2_motor_direction_audit.json)（SHA-256 `bcf5a515f6f76a845d591ab3c442ea810987ca3d14762f828b01df4687fffb4f`）。
- **结果**：`d03_differential_retry_20261107`、`d03_differential_seed456_20261107` 和新官方进程 `v2_official_differential_seed260007_20261109` 三次独立进程，四个单电机 +0.004 脉冲的 x/y 角速度符号与候选 FRD 单电机符号 **12/12 一致**，三次采集完全相同。
- **限制**：四个 yaw/z 斜率都低于 0.1 rad/s²，z 符号未解析；公开电机顺序和候选机体系仍是假设；不独立解码运行时通道、不证明真实旋向或完整包线。
- **状态**：G1 的方向子证据增强，但 G1 仍为 **PARTIAL**，参数辨识仍 **BLOCKED**；PWM 范围/裁剪/保持/超时和保留集误差仍未完成。

## 2026-11-09 05:15 G3 受控 W02/F01/F02 接口

- **新增**：[v2_g3_controlled_benchmark.md](v2_g3_controlled_benchmark.md) 与 [v2_g3_controlled_benchmark.json](v2_g3_controlled_benchmark.json)（SHA-256 `66ca647c124328f8b88a86f5e8eb25f05f829e788b9e8601b78c4d005aff713a`）。
- **W02**：3 m/s 常值风、区域 `x∈[-1,1] m`，80 个物理子步中 25 个非零、55 个零；进入/离开区域采样正确，离开后归零。
- **F01**：常值 flag 0/1 下 PWM 0.20 的滤波信号为 0.0500000003/0.2，验证 0.25 倍目标缩放链。
- **F02**：flag 日程 `[1,0,1,0,1,1]`、阈值 0.15 s 时，仅在 flag=0 区间累计；0.36 s 时约 0.16 s 超阈值，最终约 0.20 s；flag=1 不清零。
- **限制**：全部为合成风帧/区域/flag，不是自然风区、W01/F01/F02 验收；G3 仍为 **PARTIAL/UNVERIFIED**。

## 2026-11-09 05:20 官方隔离容器运行

- **运行**：passive smoke（seed 123）exit 0；`heldout_mix` seed260004/260006 因 `motion guard exceeded` 中止但保留记录；`heldout_mix_small` seed260005 完成，8/8 命令边缘有回显匹配；`differential` seed260007 完成，四电机 x/y 方向与前两次一致。
- **官方哈希**：有 PWM 运行的 `pool.json` 二进制为 `5231addf…b97800`，设置为 `0d72886f…6c6c17`；容器均停止。
- **新时钟点**：seed260004/260006 的自由落体尾段源时间加速度 9.7897/9.7908 m/s²（-0.173%/-0.162% vs g），二次拟合 RMSE 0.0143/0.0141 mm；到达时间 9.7972/9.8790 m/s²、RMSE 1.322/1.083 mm。
- **回显**：seed260005 八个边缘的主机到达延迟约 16.2–66.0 ms，Header 差 -5.4 到 +44.4 ms；Header 仍来自 IMU 请求，不是电机采样时间。
- **实时因子**：passive 源/到达跨度比 1.0000036，heldout_mix_small 为 1.0004301；官方接近实时推进，不能换算成官方 64/256 环境吞吐。机器可读结果 [v2_official_realtime_factor.json](v2_official_realtime_factor.json)（SHA-256 `1b8151f6…4de2`）。
- **记录**：[v2_official_run_20261109.md](v2_official_run_20261109.md)；回声审计 [v2_official_heldout_mix_small_seed260005_echo.json](v2_official_heldout_mix_small_seed260005_echo.json)（SHA-256 `a138d034…e85c`）。
- **状态**：G1 参数辨识仍 **BLOCKED**，G2/G3/G4 状态不变；这些运行确认了隔离官方采集链路，但没有解除官方接触/吞吐缺口。


## 2026-11-09 04:05 G1 两轮校准的可辨识性归档

- **新增**：[v2_identifiability.md](v2_identifiability.md) 与 [v2_identifiability.json](v2_identifiability.json)（SHA-256 `ea8adb0108371bf686b24b782895b6923f9ef256f94918b04414c865032ef65d`），只汇总既有两轮网格，不重新拟合、不读取封存保留集。
- **结果**：第一轮最优 `tau=2 ms, delay=0`，在边界；第二轮最优 `tau=0.5 ms, delay=0`，仍在边界；5% 近优集合只有 `tau=0.5/1.0 ms, delay=0`。`delay=0` 是下边界，正延迟在第二轮使目标变差约 12%/33%/71%（2/5/10 ms）；这不能证明物理延迟为零。
- **指标**：第二轮最优的四条角速度 RMSE 比值为 0.814/1.007/0.885/0.971，其中一条基本不变；位置 RMSE 6.50–7.15 mm、姿态 RMSE 0.048–0.499°，变化很小。
- **结论**：只可给出“`tau` 不高于约 1 ms、Header 基准下延迟接近零”的候选区间；参数辨识子问题保持 **BLOCKED**，恢复条件是新的独立时钟/电机状态/初速度/接触测量，而不是扩大网格。G1 整体仍为 **PARTIAL**。

## 2026-11-09 04:12 G3 分层证据矩阵

- **新增**：[v2_g3_evidence_matrix.md](v2_g3_evidence_matrix.md)，把风生成、空间/时间触发、风施力、力度衰减边界与 2 秒规则分层列出，并标明受控事实与 UNVERIFIED 范围。
- **风生成**：注入阶段 4–6、seed 123/456/2026 的 18/18 关键帧字节精确匹配；自然赛程随机流、自然风区和官方边界未验证。
- **风施力**：注入/强制样本中相对速度、二次项、旋转项和六面合力与原生计算吻合到约 1e-7 量级；自然风到达与 W01 轨迹未验证。强制 stage-4 短窗口位置 RMSE 0.094–0.320 mm（有风）对 0.913–1.747 mm（零风）只是同记录探索。
- **力度衰减**：受控 flag 支持 0.25/1.0 目标比例、电机目标/滤波前作用点和 2 秒累计分支；自然区域成员、边界延迟、恢复与官方状态机未验证；2 cm 球/6 cm 偏移/3.8 cm 内缩只是候选假设。
- **状态**：G3 仍为 **PARTIAL/UNVERIFIED**，以 G1 通过为前提；不把 0.25、2 cm、6 cm、3.8 cm 写成官方参数。

## 2026-11-09 04:20 G2 代表性候选几何探针

- **新增**：`foundation/audit_map_geometry.py` 和 [v2_map_geometry_probe.json](v2_map_geometry_probe.json)（SHA-256 `47c0025ec105b51ae0e7b37afe774e3a458fb6a28d3e73c46ccfc8e8054524a7`）。
- **结果**：OL1 12/12 个门候选中心平面穿越匹配；64/64 个顶面道路三角自命中；64/64 个非道路候选表面自命中；25/25 个 complex-as-simple 建筑组件（12 工厂、12 港口、1 枢纽）自命中。官方接触证据仍只有两次启动过程的 `1_1_Port/R_logo_Sprite1`（各 4 次、候选表面最大距离 1.49e-8 cm）。
- **限制**：门探针是候选中心平面和显式局部轴，不是门洞/官方触发；道路/墙体是同一几何的自命中，不是官方净空或接触；工厂入口和代表性墙体没有官方接触对照。
- **剩余类别**：每图剩余 39 个静态网格组件已归类为 12 门、24 指示箭头和 3 个环境/天空视觉体；另有 5 Niagara、3 电缆 SplineMesh 和 2 个实例化网格未作为碰撞候选。该归类不证明它们的运行时碰撞通道。
- **状态**：G2 仍为 **PARTIAL**。这补齐了代表性几何的可复现自检，但不能替代官方接触验证。

## 2026-11-09 04:30 总体验收收口（PARTIAL）

- **总体验收**：[foundation_acceptance.md](foundation_acceptance.md) 已更新为 2026-11-09 证据矩阵：G0 PARTIAL；G1 PARTIAL，参数辨识 BLOCKED；G2 PARTIAL；G3 PARTIAL/UNVERIFIED；G4 PARTIAL；总体 PARTIAL。无模块 PASS。
- **交付**：PWM 单/批量接口、OL1 与 OL2–OL12 候选地图及缺口清单、风/力场分层证据、随机障碍、批量基准、`run_smoke_test.py`。
- **可复核**：`python -m pytest -q foundation\tests` 159 项通过；`python run_smoke_test.py` 通过（合成集成）。
- **未完成/禁止宣称**：G1 的物理时钟、电机状态、独立初速度、接触/风仍未辨识；G2 已取得道路侧栏、港口内墙和枢纽墙的官方接触，但检测门洞和工厂入口仍未验证；G3 自然风区和 F01/F02 未验证；G4 已有官方实时因子，但没有同操作官方 batch 吞吐。不得宣称官方等价、完整碰撞覆盖或官方吞吐比。
- **剩余独立工作**：继续在隔离官方容器中寻找检测门洞/工厂入口接触、独立电机状态/指令生效时刻和自然风/力场数据；本地参数网格、保留集读取和更多自洽审计不能替代这些证据。

## 2026-11-09 04:40 自由落体时钟一致性（探索性）

- **新增**：[v2_ballistic_clock_audit.md](v2_ballistic_clock_audit.md) 与 [v2_ballistic_clock_audit.json](v2_ballistic_clock_audit.json)（SHA-256 `1c3348c08962fe37b014a000f76153a6f12e2f3d41b88cb25edb32f563e0ebf7`）。
- **方法**：在最后一条非零 PWM 后寻找低 IMU 比力、平滑竖直下落的自由落体尾段；只使用已有无断点采集，不用 GDB、不跑参数网格、不读封存保留集。窗口准则是查看数据后确定的，属探索性。
- **结果**：四组满足干净自由落体准则：`heldout_mix` seed 27182/31415、新官方进程 seed260004/260006。源时间加速度 9.7832/9.7898/9.7897/9.7908 m/s²（相对 g 偏差约 -0.24% 到 -0.16%），二次拟合 RMSE 0.012–0.021 mm；到达时间加速度 9.7059/9.7011/9.7972/9.8790 m/s²，RMSE 约 1.1–1.3 mm。seed260004/260006 的 player 均因 motion guard 中止，原始失败记录保留。
- **解释**：这些短窗口支持 ROS Header 源时间作为均匀物理时间基；但拟合略低于 g 可能含残余推力/阻力，且均为同波形/相近初态，不能作为完整时钟校准。
- **状态**：`clock_mapping` 的部分证据已补入 `unknowns.yaml`；G1 参数辨识仍 **BLOCKED**，G1 整体仍 **PARTIAL**。

## 2026-11-09 19:30 G2 官方接触：道路侧栏与港口内墙

- **新增工具**：`foundation/run_contact_waypoints_local.py`（本地隔离容器 + 只读接触断点）、`foundation/probes/contact_waypoints.py`（速度航点诊断，使用 `debug/pose_gt`）、`foundation/analyze_contact_waypoints.py`（按源分组并对候选几何比较）。
- **道路**：[g2_contact_road_seed123_20261109.json](g2_contact_road_seed123_20261109.json)。seed 123、二进制/设置哈希同前；939 次持续接触回调匹配 `BP_Segment_Rot_C_242`（`Segment`），源变换误差 2.2e-4 cm，候选命中点距离 1.7e-7–3.6e-4 cm，次近候选 1040.6 cm。航点保持接触直到总时长保护触发，数据有效但 player 以 1 退出。
- **港口内墙**：[g2_contact_portwall_seed123_20261109.json](g2_contact_portwall_seed123_20261109.json)。115 次回调匹配 `1_1_Port/Building_Port`，源变换误差 1.5e-5 cm，命中点距离 3.5e-6–2.1e-4 cm，次近候选 102.7 cm；player 正常完成。
- **路线/枢纽**：[g2_contact_gate1_seed123_20261109_v2.json](g2_contact_gate1_seed123_20261109_v2.json)。航点沿候选 stage-1 路线到达约 `(589.5,-98.4,134.7) m`，随后在 `(617.0,-81.7,138.7) m` 被 `BP_Building_Junction_7` 挡住；距 `1_1_Gate` 中心约 9.6 m，未取得门洞接触/净空证据。最近/次近候选变换重复，组件归属仍有歧义。
- **失败记录**：z 轴符号、港口屋顶、断点脚本、外部爬升/外墙坠落和首次路线出口等失败运行均保留在 `foundation/datasets/`，不作为验收证据。
- **限制**：GDB all-stop 改变时序；速度航点为诊断控制并使用真值；持续接触回调重复计数；命中点偏移 `0x18` 未独立定型；门洞和工厂入口仍未验证。
- **状态**：G2 仍为 **PARTIAL**；新增道路和建筑墙官方接触，但不得宣称完整碰撞覆盖或自由空间。

## 2026-11-09 20:25 G2 直接函数：路线门碰撞配置与 meter_report

- **静态直接函数**：上游 RE 的 `followup_state.c` 已反编译 `0x2fc13d0`。未激活分支使用 `.rodata` `0x820f7c` 的 `NoCollision`，激活分支使用 `0x74213e` 的 `BlockAll`，随后调用 `0x52bb600(component, active, 1)` 修改组件碰撞使能标志（偏移 `0x14c`）。二进制字符串表同时存在 `SetActorEnableCollision`、`SetCollisionEnabled`、`SetCollisionProfileName`。
- **结论**：12 个命名 `*_Gate` actor 在激活时是 **BlockAll 实体碰撞**，不是 trigger/overlap 门洞；这解释了为什么路线在 `1_1_Gate` 上方通过而没有门接触，但**不能**证明检测门洞或自由空间。
- **meter_report 直接服务探测**：新 seed 123 进程调用 `meter_report` 的 index 0/1、value 0/25 全部返回 `success=true`，但模拟器日志没有出现 `OnMeterReport`、`FactoryA/B`、`EnterPort/LeftPort`；只能说明服务接受请求，不能作为工厂入口或仪表值验收。原始 pool 清单和日志保留在 [g2_meter_report_probe_seed123_20261109](../datasets/g2_meter_report_probe_seed123_20261109/)。
- **报告**：[g2_gate_collision_re20261109.md](g2_gate_collision_re20261109.md)。
- **状态**：G2 仍为 **PARTIAL**；门碰撞语义已澄清，检测门洞和工厂入口仍未验证。

## 2026-11-09 21:10 G2 检测门候选层

- **新增**：`data/audits/ol1_seed123` 的 33+30 个评分组件 root 变换整理为 [detection_gate_candidates.json](../maps/OL1/detection_gate_candidates.json)（SHA-256 `fca37435…b30d`）和查询类 `foundation.detection_gates.DetectionGateCandidates`。它们不是 12 个 `BlockAll` 实体路线门，也不是已验证 trigger volume。
- **路线偏差**：候选 `static_spline1` 与 road1 门中心的最近距离中位 2.443 m、最大 7.055 m，仅 7/33 在 1 m 内、13/33 在 2 m 内；road3 为 5/30、10/30，最大 6.080 m。详见 [v2_detection_gates.md](v2_detection_gates.md) 和 [v2_detection_gate_alignment.json](v2_detection_gate_alignment.json)（SHA-256 `c7bb9dc0…0fd2`）。
- **限制**：最近点距离不是门穿越或自由空间；检测门 trigger plane、aperture、官方计分语义和工厂入口仍未验证。
- **状态**：G2 仍为 **PARTIAL**；新增的是可追溯候选层和路线偏差量化，不是 PASS。

## 2026-11-09 21:35 G2 候选检测门 32 净空探测

- **运行**：隔离官方容器 seed 123，二进制/设置哈希同前；容器停止。诊断航点从 `x=1.35 m` 经中心 `x=2.504 m` 到 `x=3.65 m`，NED 高度 `z=-2.1557 m`。
- **结果**：player completed；轨迹在候选门中心平面的穿越点距门中心约 **0.0307 m（3.07 cm）**；命令开始后 `MAP_CONTACT` 为 **0**。全日志仅 2 次启动阶段 `1_1_Port/R_logo_Sprite1` 接触，发生在命令开始前。
- **记录**：[g2_detection_gate32_20261109.md](g2_detection_gate32_20261109.md)、[g2_detection_gate32_seed123_20261109.json](g2_detection_gate32_seed123_20261109.json)（SHA-256 `2a0fd51a…0da5`）；原始数据在 `foundation/datasets/g2_detection_gate32_seed123_20261109/`。
- **限制**：诊断航点使用真值；GDB all-stop；2 m 半尺寸是探测假设；无接触穿越不证明官方门触发、计分或完整门洞。
- **状态**：G2 仍为 **PARTIAL**；新增一个候选检测门中心的无接触穿越，其余 62 门和工厂入口仍未验证。

## 2026-11-09 21:50 G2 早期候选检测门序列

- **运行**：隔离官方容器 seed 123，诊断航点依次穿过 road1 index 32→31→30→29；player completed，容器停止。
- **结果**：四个门中心平面均被穿越；最近距离 0.1762/0.0179/0.0137/0.0195 m；命令开始后 `MAP_CONTACT` 为 **0**。全日志仅 2 次启动阶段 `1_1_Port/R_logo_Sprite1` 接触。
- **记录**：[g2_detection_gate_sequence_20261109.md](g2_detection_gate_sequence_20261109.md)、[g2_detection_gate_sequence_seed123_20261109.json](g2_detection_gate_sequence_seed123_20261109.json)（SHA-256 `0652d51b…af06`）；原始数据在 `foundation/datasets/g2_detection_gate_sequence_seed123_20261109/`。
- **限制**：诊断航点使用真值；GDB all-stop；2 m 半尺寸是假设；中心平面穿越不证明官方 trigger/计分；只覆盖 road1 的 4/33 门。
- **状态**：G2 仍为 **PARTIAL**；其余 59 个候选门、road3、工厂入口和官方 aperture 语义仍未验证。

## 2026-11-09 22:05 G2 候选检测门 score 触发

- **运行**：同一早期门序列航点，使用 `score_readonly2.gdb` 在隔离官方容器中记录评分回调；player exit 0，容器停止。本地 `commands.jsonl` 因 wrapper 超时缺失，`capture.jsonl`、GDB log 和 `run.json` 完整。
- **结果**：SEG 事件出现候选索引 29/30/31/32；score 从 0→3，增量对应 `idx31 enter=1`、`idx30 enter=1`、`idx31 enter=0`；`idx32` 和 `idx29` 未增加 score。不能把每个候选门当作一次 score。
- **意义**：33+30 运行时组件与官方 score 事件存在直接关联，候选检测门从纯静态层推进到可触发评分的证据层；但队列顺序、起点/终点门和重复事件语义仍未解码。
- **记录**：[g2_detection_gate_score_20261109.md](g2_detection_gate_score_20261109.md)、[g2_detection_gate_score_seed123_20261109.json](g2_detection_gate_score_seed123_20261109.json)（SHA-256 `012ec557…e39b`）。
- **状态**：G2 仍为 **PARTIAL**；完整 trigger/aperture、其余门、road3 和工厂入口仍未完成。

## 2026-11-09 22:30 G2 stage-1 road1 完整 score 序列

- **运行**：候选 stage-1 航点 + score 探针，隔离官方容器 seed 123；player 因 `maximum flight duration exceeded` 退出（exit 1），容器停止。
- **结果**：score **0→33**，33 次增量；`SEG` 观察到 road1 候选索引 **0–32 全部 33 个**；状态回调出现 `old=1 new=12`（score=32），随后 score 到 33。
- **意义**：候选 stage-1 路线实际触发全部 33 个 road1 评分组件，score 达到 33；这是目前最强的 road1 检测门触发证据。
- **限制**：GDB all-stop；player 超时；本地 `commands.jsonl` 缺失、`capture.jsonl` 可能截断，score 结论只依赖完整 GDB log；score 增量与门索引不是简单一一对应；road3 的 30 个、工厂入口和完整门洞语义仍未验证。
- **记录**：[g2_score_route_20261109.md](g2_score_route_20261109.md)、[g2_score_route_seed123_20261109.json](g2_score_route_seed123_20261109.json)（SHA-256 `7f493930…7a59`）。
- **状态**：G2 仍为 **PARTIAL**；road1 评分组件已覆盖，但 G2 PASS 仍未达到。

## 2026-11-10 V2 快速覆盖轮

- **覆盖矩阵**：新增 `foundation/build_v2_coverage_matrix.py`，生成 [v2_coverage_matrix_20261110.md](v2_coverage_matrix_20261110.md) / `.json`。OL1 stage1 road1 保持 33/33；road3 仍 0/30；OL1 stage2–6 与 OL2–OL12 的运行时组件/score/接触/工厂均显式标为 UNVERIFIED。静态候选不计入运行时覆盖。
- **OL1 路线清单**：新增 `foundation/build_stage_manifests.py`，由 `data/tables/routes.csv` 重建六个 stage 的路线和 waypoint，生成 `foundation/maps/OL1/stageN_route_manifest.json` 与 `stageN_waypoints.json`。stage1 重建长度 1478.85 m，与官方 `route_1_3` 的 1479.47 m 接近；其余 stage 为诊断重建，不是官方轨迹。
- **road3 尝试**：三次受限尝试均未取得 score，记录在 [g2_road3_attempt_20261110.md](g2_road3_attempt_20261110.md)：官方 route 高度导致高度包线中止；stage1 候选路线在枢纽墙被挡（距 `1_1_Gate` 9.6 m）；GDB 诊断传送到 road3 后观察到 idx 9–13 的 SEG 事件但 score 为 0，因为进程仍在 stage1，活动评分队列不是 road3。road3 仍为 **0/30**。
- **G1**：按限次完成两次非停止式尝试，记录在 [v2_g1_native_attempt_20261110.md](v2_g1_native_attempt_20261110.md)。probe 镜像没有 `perf`，无法启动 uprobe；`process_vm_readv` 父子原型可读 8 字节，但未取得官方 rotor 状态。G1 参数辨识保持 **BLOCKED**，未重开网格、未读封存集。
- **工厂入口**：新增 `foundation/maps/OL1/factory_entrance_candidates.json`，记录 12 个 `BP_Building_Factory_1_C` 的 Extent 体积和来源哈希；入口 trigger/事件仍未验证，`meter_report` success 仍不算证据。
- **训练环境**：新增 `foundation/training_env.py` 的 `RmuaPhysicsVecEnv`：四路 PWM、6DOF、批量、独立 seed、显式 truth 研究观测和 `observable_state` 占位；标注为近似沙盒。新增 `foundation/tests/test_training_env.py`。
- **可复核**：`python -m pytest -q foundation/tests` 通过；`python run_smoke_test.py` 通过（合成集成）。这些测试不升级 G0–G4。

## 2026-11-10 第二阶段：stage2 注入与 road3 score

- **road3 组件触发**：stage1 诊断传送运行 `g2_score_road3_teleport_seed123_20261110` 观测到 road3 **30/30** 组件索引 0–29 的 SEG enter/exit 事件；因活动队列仍是 stage1，score 为 0。解析：`foundation/reports/g2_road3_teleport_seed123_20261110.json`。
- **stage2 注入**：`foundation/probes/stage2_teleport.gdb` 在原生 stage-init 注入 `(3,5,2)` 并诊断传送到 road3 末端，短探针确认 score 0→1→2；road3 反向队列有效。
- **road3 score 30/30**：v7 routexy 基线 + gate11→7 走廊（+1 m）+ gate6→0 走廊（+3 m），注入 stage2 后 **score 30/30**，索引 0–29 全出现，player exit 0。这是受控 stage injection，不是官方 stage 进程。证据：`foundation/reports/g2_stage2_routexy_gate11_0_corridor_v12_seed123_20261110.json`。
- **stage3 失败记录**：注入 `(5,8,3)`、传送到 route5 末端后，`DUMP_BEGIN count=31`，但组件变换为 `nan`，SEG 索引 16–30、score 0。记录：`foundation/reports/g2_stage3_return_20261110.md`。
- **工具修正**：`run_contact_waypoints_local.py` 在停止容器前保存 `simulator.log`；`run_contact_waypoints_remote.py` 增加 SSH 重试和 tar 归档下载，避免 GDB 日志丢失。
- **覆盖状态**：OL1 评分组件已验证 **63/63**：road1 33/33，road3 30/30（注入 stage2）；stage1 直飞 road3 仍 0/30。OL1 stage3–6 与 OL2–OL12 仍未取得运行时 score。
- **测试**：`pytest -q foundation/tests` **166 项通过**；smoke 通过（合成集成）。

## 2026-11-10 OL2 尝试

- `foundation/probes/map_ol2_teleport.gdb` 成功将默认 Lobby→OL1 的 URL 重定向到 `RMUA2024OL2`，并在首次接触回调把无人机组件写到 OL2 route1 起点；日志见 `foundation/datasets/g2_ol2_route1_seed123_20261110/run/simulator.log`。
- 但无人机报告位姿保持 `(0,0,0)`、四元数 `(0.5,0,0,0)`，速度命令未产生运动；没有 `DUMP_BEGIN` 或 `SEG` 事件，player wrapper 超时（124）。
- OL2 运行时检测组件/score/接触仍为 **UNVERIFIED**；记录见 [g2_ol2_attempt_20261110.md](g2_ol2_attempt_20261110.md)。OL3–OL12 未运行。

## 2026-11-10 晚：road3 30/30 与 stage4/OL3 缺口

- **road3 30/30（受控注入）**：v12 使用 v7 routexy 基线 + gate11→7 走廊（+1 m）+ gate6→0 走廊（+3 m），注入 stage2 后 score **30/30**，索引 0–29 全出现，player exit 0。证据：`foundation/reports/g2_stage2_routexy_gate11_0_corridor_v12_seed123_20261110.json`。这是受控 stage injection/GDB 传送，不是官方 stage 进程。
- **OL1 有效评分覆盖**：road1 33/33 + road3 30/30 = **63/63**（road3 需注入 stage2；stage1 直飞仍 0/30）。
- **stage4 尝试**：注入 `(8,12,4)`、传送到 route8 末端前 12 m，无 SEG/DUMP；无人机停在 `(1358.72,-102.74,-8.87) m`。记录：`g2_stage4_return_20261110.md`。
- **OL3 尝试**：map redirect 成功、teleport 已写入，但无 SEG/DUMP，位姿停在 `(0,0,0)`。记录：`g2_ol3_attempt_20261110.md`。
- 测试：`pytest -q foundation/tests` **166 项通过**；smoke 通过。

## 2026-11-10 stage4 ground-lock 诊断

- stage4 传送后组件位姿写入目标，但速度指令完全无效：climb 探针目标上方 21 m、期望垂直速度 -6 m/s，位置仍保持 `(1358.72,-102.74,-8.87) m`。
- `/airsim_node/drone_1/takeoff` 服务调用失败：probe 容器中没有 `airsim_ros/Takeoff` 消息类型（`Unable to load type [airsim_ros/Takeoff]`），`/workspace/rmua_ws` 为空。
- 结论：当前注入路径下传送载具被冻结/保持，stage4 不能靠再加航点解决；需要不同的注入/reset 路径。stage4 仍 UNVERIFIED。记录：`g2_stage4_return_20261110.md`。

## 2026-11-10 stage4 冻结诊断与 OL4

- 已把远端 ROS workspace 只读挂载到 probe 容器 `/workspace/rmua_ws`；`/airsim_node/drone_1/takeoff` 的 `airsim_ros/Takeoff` 类型可加载。
- `waitOnLastTask: true/false` 两次调用均 15 s 超时，传送后的载具位置仍不变；说明不是消息类型问题，而是 component 写入后物理体被冻结。需要 physics-body state 写入或不同 reset/injection 路径。
- OL4 map redirect 与 teleport 已观测，但无 SEG/DUMP；记录：`g2_ol4_attempt_20261110.md`。
