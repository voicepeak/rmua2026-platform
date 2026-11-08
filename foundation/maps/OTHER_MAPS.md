# OL2–OL12 静态道路和建筑候选

每张地图目录包含 `road_candidates.json` 和 `building_candidates.json`。它们分别由 `D:/RMUA2026-re/artifacts/maps_json/RMUA2024OL{N}.json` 的完整序列化导出独立生成，并记录该导出、蓝图、网格清单及生成器哈希；不是把 OL1 按门变换复制到其他地图。OL2 与 OL3 的道路实例内容并不完全相同。

各地图均提取到 483 个道路实例、485 个建筑 actor、8,682 个建筑组件，建筑附件链未解析数和缺失蓝图数均为 0。每图另有 12 个门网格候选、1,884 个道路附属立方体、1,440 个货架组件和 108 个 sprite 组件，附件链未解析数均为 0。数字是静态资产清点，不是碰撞覆盖率。默认碰撞查询加载 483 条道路及 25 个 complex-as-simple 建筑组件；显式启用以下候选组时，每图查询实例数为 12,105：额外的 8,645 个建筑简单形状、1,884 个道路立方体、960 个货架简单形状和 108 个 sprite 形状。仍有 12 个建筑组件、480 个不含简单形状的货架纸张/指针及其他类别未纳入；门仅用作穿越代理，未作为碰撞体加载。OL1 有 120 个 sprite 候选，多出的 12 个独立标识未在这些地图导出中出现。

```python
from foundation.map_scene import CandidateMap
scene = CandidateMap.load_serialized_candidates(
    'foundation/maps/OL2', 'foundation/asset_analysis/mesh_collision')
# 或显式启用尚未确认碰撞通道的附加候选：
scene_with_candidates = CandidateMap.load_serialized_candidates(
    'foundation/maps/OL2', 'foundation/asset_analysis/mesh_collision',
    simple_shapes_file='foundation/asset_analysis/OL1_simple_convex_candidates.json',
    road_cube_shapes_file='foundation/asset_analysis/OL1_simple_convex_candidates.json',
    rack_shapes_file='foundation/asset_analysis/rack_simple_candidates.json',
    sprite_shapes_file='foundation/asset_analysis/sprite_collision_candidates.json')
from foundation.gate_geometry import GateMeshVolumes
gates = GateMeshVolumes.load_serialized_candidates(
    'foundation/maps/OL2', 'foundation/asset_analysis', crossing_axis=0)
result = scene.sweep_sphere_ned([0, 0, -1], [0, 0, 1], .2,
                                origin_ue_cm=[0, 0, 0])
```

示例原点不代表经过确认的地图坐标。加载器拒绝来自不同来源的道路和建筑文件，也拒绝存在缺失蓝图或未解析附件链的文件。执行如下命令可对某张地图重新生成候选：

```powershell
python -m foundation.build_other_map_candidates D:/RMUA2026-re/artifacts/maps_json/RMUA2024OL2.json foundation/maps/OL2
python -m foundation.rack_placements D:/RMUA2026-re/artifacts/maps_json/RMUA2024OL2.json foundation/asset_analysis/blueprints foundation/maps/OL2/gate_candidates.json --actor-class BP_Gate_C
python -m foundation.rack_placements D:/RMUA2026-re/artifacts/maps_json/RMUA2024OL2.json foundation/asset_analysis/blueprints foundation/maps/OL2/road_cube_candidates.json --actor-class BP_Segment_Rot_C --component-prefix Cube
python -m foundation.rack_placements D:/RMUA2026-re/artifacts/maps_json/RMUA2024OL2.json foundation/asset_analysis/blueprints foundation/maps/OL2/rack_candidates.json --actor-class BP_Rack_C
python -m foundation.sprite_inventory D:/RMUA2026-re/artifacts/maps_json/RMUA2024OL2.json foundation/asset_analysis/blueprints foundation/maps/OL2/sprite_candidates.json
```

门、道路立方体与货架使用的生成脚本 `rack_placements.py` 的 SHA-256 为 `41ed071062aa1e1147f58414a6fec3ab5310bcb5df8f8454d4dd466b111db01e`；加载器核对这些候选与道路的源地图和蓝图哈希，简单形状文件也记录其哈希。门调用方必须自行指定穿越轴，示例中的轴不是官方方向。穿越事件为网格包围盒的几何代理，不是已验证的门洞或评分。附加组的运行时组件变换、构造脚本和碰撞通道均未验证，不能把添加候选数量当作官方碰撞覆盖率。

sprite 候选使用 `sprite_inventory.py`（SHA-256 `28b8c2b9ad00e6854f96e31228f4f3705a3acab8eceee7167c727373c397878a`）从各自地图生成，源地图和蓝图哈希与道路候选核对；实例级 SourceSprite 覆盖尚未实现，因此存在覆盖时会列为未解析并拒绝加载。OL1 启动接触曾与局部 sprite 候选面相符，但这不验证其他地图上该类组件的实际碰撞通道。

流程对 OL1 的独立重建与现有 OL1 道路、建筑候选逐项相同，但两者来自相同序列化资产，属于实现一致性检查，不是运行时验证。OL2 的六次历史启动门根对照见 [OL2 证据](OL2/gate_runtime_evidence.md)，该路径并未证明飞行初始化成功；其他地图的物理加载、实际组件变换、碰撞通道及无人机接触均未验证。缺失类别和表面近似意味着没有命中不证明可以通行。G2 保持 PARTIAL。
