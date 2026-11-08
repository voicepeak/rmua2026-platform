# OL1 静态碰撞候选

此目录的 [collision_manifest.json](collision_manifest.json) 记录默认和可选场景的实例数、源资产哈希与已知缺口。重新生成：在项目根目录运行 `python -m foundation.build_ol1_collision_manifest`。查询所用的派生资产位于 `foundation/asset_analysis`，原始解包数据位于 `foundation/raw_assets`；请按清单中的哈希核对。

使用时由调用方明确指定原点与需要的候选分组：

```python
from foundation.map_scene import CandidateMap

scene = CandidateMap.load('foundation/asset_analysis')
result = scene.sweep_sphere_ned([0, 0, -1], [0, 0, 1], .2,
                                origin_ue_cm=[0, 0, 0])
```

上述原点仅用于展示调用方式，不是已验证赛道原点。`include_unverified_simple`、`include_unverified_sprites`、`include_unverified_racks`、`include_unverified_road_cubes` 可显式增加候选几何，但碰撞通道和覆盖均未证实。门根变换、起点附近接触的局部证据不等于门洞、道路、工厂、建筑的全部碰撞验收。

默认扫掠只测试候选三角面。可显式传入 `assume_closed_meshes_solid=True`，对索引拓扑封闭的网格增加候选内部检测；这是假设网格代表实体，不能证明官方碰撞语义。开放网格或多射线判断不一致时，查询和运行接口的 `containment_unknown`／`map_containment_unknown` 会列出未知项，不能将其当作无碰撞证明。所有模式均不返回精确接触时间；漏掉未纳入物体时不能视作空间可通行。
