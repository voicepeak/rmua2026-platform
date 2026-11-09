# OL1 检测门候选（运行时评分组件）

本页把 `data/audits/ol1_seed123` 中 33+30 个运行时评分组件 root 变换整理为**候选检测门层**。它们不是 12 个命名 `*_Gate` 实体路线门，也不是已验证的 trigger volume、门洞或官方计分。

## 资产与入口

- 归一化清单：[detection_gate_candidates.json](../maps/OL1/detection_gate_candidates.json)，SHA-256 `fca3743516708f26bb269a081a143ee2c2f0642bf683607f1b1361e34a20b30d`。
- 源审计文件：`data/audits/ol1_seed123/road1_components.json`（33）和 `road3_components.json`（30）；源哈希已写入清单。
- 生成：`python -m foundation.build_detection_gate_candidates`。
- 查询：`foundation.detection_gates.DetectionGateCandidates`。调用方必须显式给出局部穿越轴和三个半尺寸；实现只做候选中心平面/横向边界查询，不推断官方 trigger plane。

## 与 12 个实体路线门的关系

新静态反汇编证据表明，12 个命名 `*_Gate` actor 在激活时是 `BlockAll` 实体碰撞，未激活时 `NoCollision`；它们是碰撞体开关，不是检测门洞。`detection_gate_candidates` 是另一层：来自评分组件轨迹的 63 个运行时 root，不能把两者混为同一门表。

## 与候选 stage-1 路线的偏差

用 `foundation/datasets/route_candidate_lift6_20261108/route.json` 的 `static_spline1` 最近点比较：

| 组 | 门数 | 最近距离中位 | 最大 | ≤1 m | ≤2 m | ≤5 m |
|---|---:|---:|---:|---:|---:|---:|
| road1（33） | 33 | 2.443 m | 7.055 m | 7 | 13 | 31 |
| road3（30） | 30 | 2.681 m | 6.080 m | 5 | 10 | 27 |

机器可读结果：[v2_detection_gate_alignment.json](v2_detection_gate_alignment.json)，SHA-256 `c7bb9dc06efccd7db8e2649662a33bdf124e9c9ac0baf45fb65458f6a9ac0fd2`。

这说明候选路线与检测门中心并不逐门重合；最近点距离不是穿越事件，也不是自由空间证明。路线生成器、连接段和门中心仍需独立核对。

## 状态

G2 仍为 **PARTIAL**。新增的是可追溯的检测门候选层和路线偏差量化；检测门的官方 trigger plane、aperture、碰撞/计分语义、工厂入口和真实飞行穿越仍未验证。
