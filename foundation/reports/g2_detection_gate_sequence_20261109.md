# G2 早期候选检测门序列净空（2026-11-09）

## 方法

- 隔离官方容器，seed 123，二进制/设置哈希同前；容器停止。
- 诊断航点从起点依次穿过 road1 index 32、31、30、29 的候选检测门中心，使用 `debug/pose_gt` 反馈，最大速度 4 m/s，总时限 60 s。
- 只读接触断点记录 `MAP_CONTACT`；原始数据在 `foundation/datasets/g2_detection_gate_sequence_seed123_20261109/`。

## 结果

| 门 | 最近距离 | 中心平面穿越 |
|---|---:|---|
| road1 32 | 0.1762 m | 是 |
| road1 31 | 0.0179 m | 是 |
| road1 30 | 0.0137 m | 是 |
| road1 29 | 0.0195 m | 是 |

- player 状态：**completed**。
- 命令开始后 `MAP_CONTACT`：**0**。
- 全日志仅 2 次启动阶段的 `1_1_Port/R_logo_Sprite1` 接触，发生在命令开始前。
- 轨迹从起点约 `(0,0,-0.309)` NED 到 `(66.99,1.85,-5.21)` NED；四个候选门中心平面均被穿越，除 index 32 的首个航点外，最近距离约 1.4–2.0 cm。

机器可读结果：[g2_detection_gate_sequence_seed123_20261109.json](g2_detection_gate_sequence_seed123_20261109.json)，SHA-256 `0652d51b4c0d850efc88c72f4b5daeeae2587ede454ad148be4cb485e75baf06`。

## 限制

- 诊断速度控制使用真值位姿；不是策略或比赛飞行。
- GDB all-stop 接触断点会改变时序和可能的碰撞响应。
- 2 m 半尺寸是探测假设，不是官方 trigger volume；中心平面穿越不等于官方门触发、计分或门洞几何。
- 只覆盖 road1 的 4/33 个早期候选门；road3 和其他门未验证。
- 无接触只说明该路径在候选几何下没有接触，不证明完整自由空间。

## 状态

G2 仍为 **PARTIAL**。新增 4 个早期候选检测门的可复核无接触穿越；其余 59 个候选门、road3、工厂入口和官方 trigger/aperture 语义仍未验证。
