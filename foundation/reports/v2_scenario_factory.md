# V2 候选障碍场景接口

`foundation.scenarios.ScenarioFactory` 将受保护航线的障碍生成与 `RmuaPhysics` 连接。调用方必须提供 `fixed`、`random`、`difficult` 三套 `ObstacleProfile`，自行决定线障数量、半径、长度、朝向、车辆尺寸和速度区间；接口不假定任何一套配置符合官方分布。

```python
from foundation.quad import Params
from foundation.scenarios import ObstacleProfile, ScenarioFactory

# 演示数字，不是官方赛道或车辆参数。
regular = ObstacleProfile(2, 1, .03, 3., (1., .5, .5), (0., 2.), vehicle_center_z_m=-1.)
difficult = ObstacleProfile(4, 2, .05, 4., (1., .5, .5), (1., 3.), vehicle_center_z_m=-1.)
factory = ScenarioFactory(bounds_min=[-20]*3, bounds_max=[20]*3,
    launch=[0, 0, 0], exclusion_m=2,
    protected_paths=[[[-15, 0, 0], [15, 0, 0]]], corridor_radius_m=1,
    horizon_s=2, fixed_seed=42,
    profiles={'fixed': regular, 'random': regular, 'difficult': difficult})
params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
sim, manifest = factory.build(params, 'random', seed=123, physics_dt=.01)
observation = sim.reset()
observation, events = sim.step([.18]*4, .02)
```

`fixed` 使用构造时的固定 seed，不受请求 seed 影响；`random` 和 `difficult` 使用请求 seed。返回清单记录有效 seed、完整候选障碍几何、参数、走廊、有限运动范围和几何哈希；相同 seed 和确定性配置/回调可复现障碍，风与其他自定义模型的随机数须单独管理。几何哈希不覆盖飞行器物理参数、风或地图。

`ObstacleProfile` 默认生成匀速直线车辆。需要转向和停车时，可显式设置 `vehicle_route_legs=2` 或更多，并给出 `route_leg_length_m`、`turn_range_rad`、`stop_range_s` 和正的速度区间；工厂按独立衍生 seed 生成水平分段路线、速度和中间停车。整条路线按车辆包围球保守检查边界、起飞区与保护走廊；转向在路点瞬时发生，并非已验证的车辆动力学。若提供 `feasible`，回调需能分别处理线障以及完整的 `VehiclePath`。

车辆数量大于零时，`vehicle_center_z_m` 必须由调用方明确设置为 NED 坐标系中的**车辆中心高度**；直线路径和转弯路径会保持该高度，超出边界的高度会被拒绝。它不是自动读取的道路表面，不能证明车辆贴地或适应道路坡度。旧配置需要补上该字段；低层通用生成器仍可按显式或未指定高度运行。

自动测试还分别创建了三份独立候选风模型：障碍 seed 相同、风 seed 相同的两次运行逐状态一致；更换风 seed 后候选飞行响应不同。该测试使用合成风参数，不证明官方自然赛程的随机数消费顺序一致。

走廊半径必须包含所配置的无人机碰撞半径。环境超时默认等于车辆轨迹检查时长，也可指定更短值，不能超过此时长。匀速车辆的整个有限时段、分段车辆的整条路线均按包围球保守排除进入走廊和起飞区，并检查车辆保持在给定边界内。

工厂还按 `obstacle_separation_m`（默认 0，可指定额外距离）保守分离线障和车辆的整个路径包络，覆盖线线、线车和车车；空间不足时拒绝生成。该检查不区分经过同一位置的时间，可能拒绝不会同时相撞的路径；对分段车辆检查整条路线，对匀速车辆检查配置时长。低层通用生成函数只有显式传入间隔时才启用这一检查。修改生成规则会改变部分 seed 对应的场景，实际生成结果应以清单中的几何哈希为准。

这些约束不证明用户给出的走廊在官方地图上真实可通行，也不证明车辆贴合道路或符合官方分布。调用方可另传可信的 `feasible` 回调，但回调的正确性和确定性由调用方负责；不能将部分地图的无碰撞查询视为官方净空证明。

也可在 `build` 时显式提供 `map_clearance_m`、`candidate_map` 和 `map_origin_ue_cm`，使用运行时同一张候选地图过滤障碍。细线以整个胶囊检查；直线车辆检查完整配置时限，分段车辆检查全部路段，均使用车辆包围球并额外增加指定间隔。调用方必须先用 `assume_closed_meshes_solid=True` 加载地图；查询发现接触、假定封闭实体内部或包含关系不明时拒绝候选。自定义 `feasible` 回调同时存在时两项均须通过，清单记录地图源哈希、原点与间隔。

车辆包围球比车辆本体大，可能与道路交叠而拒绝本可贴地行驶的车辆；此模式主要排除与候选表面的明显交叠，不提供道路贴合、可信车辆路线或官方可通行性证明。地图本身存在未加载组件、未验证通道和实体假设，路线走廊也未自动通过地图验证。空间不足时生成会失败；调用方应选择可信地图和场景，而不是将失败绕过后宣称净空。
