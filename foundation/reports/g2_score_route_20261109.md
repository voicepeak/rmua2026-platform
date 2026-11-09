# G2 stage-1 road1 检测门 score 序列（2026-11-09）

## 方法

- 使用 `foundation/config/g2_contact_gate1_20261109.json` 的候选 stage-1 航点，隔离官方容器，seed 123，二进制/设置哈希同前。
- `foundation/probes/score_readonly2.gdb` 在评分组件回调 `0x2fc8d80` 读取 index、enter/event 和 score；在状态回调 `0x2fb9960` 记录状态。
- player timeout 300 s，collector 180 s；容器停止。player 最终以 `maximum flight duration exceeded` 退出（exit 1），不是完整比赛。

## 结果

- **score 0 → 33，33 次增量**。
- `SEG` 事件观察到 road1 候选索引 **0–32 全部 33 个**。
- 状态回调出现 `old=1 new=12`（score=32），之后 score 到 33；状态 12 的语义和 stage 切换仍未独立解码。
- 前几次增量示例：
  - 0→1：idx31 enter=1
  - 1→2：idx30 enter=1
  - 2→3：idx31 enter=0
  - 3→4：idx30 enter=0
  - 4→5：idx29 enter=0
- 最后两次增量：31→32（idx1 enter=1）、32→33（idx0 enter=1）。

这说明候选 stage-1 路线实际触发了全部 33 个 road1 评分组件，score 达到 33。它比此前四个早期门更强：**road1 的 33 个检测组件都产生了官方 score 事件**。

机器可读结果：[g2_score_route_seed123_20261109.json](g2_score_route_seed123_20261109.json)，SHA-256 `7f4939308df939ffe860d9632e79c6d9bda73828976bd1956a24065130667a59`。原始数据：`foundation/datasets/g2_score_route_seed123_20261109/`。

## 重要限制

- GDB all-stop；score 事件时序受调试器影响。
- player 超时退出；这不是完整比赛或 stage 2 验证。
- 本地 `commands.jsonl` 因 wrapper 超时未下载，`capture.jsonl` 可能截断；`run.json` 和完整 GDB score log 保留。score 结论只依赖 GDB log。
- score 增量与门索引不是简单一一对应；`idx31/30` 的 enter/exit 都可触发增量，index 32/0 的顺序也需进一步解码。
- 只覆盖 road1 的 33 个早期评分组件；road3 的 30 个、工厂入口和 OL2–OL12 官方接触仍未验证。
- 候选门中心平面穿越和 score 事件不等于官方门洞几何、aperture 或完整计分等价。

## 状态

G2 仍为 **PARTIAL**。新增的是 **road1 全部 33 个候选评分组件的官方 score 事件序列**；road3、工厂入口、其余地图和完整门洞语义仍未完成。
