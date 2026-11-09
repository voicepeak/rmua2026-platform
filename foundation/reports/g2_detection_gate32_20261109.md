# G2 候选检测门 32 净空探测（2026-11-09）

## 方法

- 使用 `foundation/run_contact_waypoints_remote.py` 在远端隔离 headless 容器中运行 `foundation/run_contact_waypoints_local.py`；二进制 SHA-256 `5231addf…b97800`、设置 `0d72886f…6c6c17`，容器已停止。
- 诊断航点控制器使用 `debug/pose_gt`，从 `x=1.35 m` 经 `x=2.504 m` 到 `x=3.65 m`，高度 NED `z=-2.1557 m`；目标是 `detection_gate_candidates.json` 的 road1 index 32 中心。
- 只读接触断点 `0x2630780` 记录 `MAP_CONTACT`；航点文件、控制脚本、采集脚本和容器日志均保存在 `foundation/datasets/g2_detection_gate32_seed123_20261109/`。

## 结果

- player 状态：**completed**。
- 轨迹在候选门中心平面上的穿越点：UE cm `(250.564, -2.941, 212.510)`；门中心 `(250.410, -2.888, 215.571)`，**最近距离约 0.0307 m（3.07 cm）**。
- 命令开始后 `MAP_CONTACT` 计数：**0**。
- 全日志仅有 2 次启动阶段的 `MAP_CONTACT`，均为 `1_1_Port/R_logo_Sprite1` 的既有起点接触，发生在命令开始前，与门探测无关。
- 使用显式 2 m 横向/竖直半尺寸的候选中心平面查询，检测到 road1 index 32 的穿越事件，方向为 +1。

机器可读结果：[g2_detection_gate32_seed123_20261109.json](g2_detection_gate32_seed123_20261109.json)，SHA-256 `2a0fd51a5ea3da667fe8dd2bb6f82dc6432d65caae300d1c8a82576eddf10da5`。

## 限制

- 航点控制是诊断工具，使用真值位姿，不是策略或比赛飞行。
- GDB all-stop 接触断点会改变时序和可能的碰撞响应。
- 2 m 半尺寸是探测假设，不是官方 trigger volume；中心平面穿越不等于官方门触发、计分或门洞几何。
- 只探测了一个候选门（road1 index 32）；road3 和其余 road1 门未验证。
- 无接触只能说明该路径在候选几何下没有接触，不证明完整自由空间。

## 状态

G2 仍为 **PARTIAL**。已新增一个候选检测门中心的可复核无接触穿越；检测门 trigger/aperture 的官方语义、其余 62 个门和工厂入口仍未验证。
