# G2 候选检测门评分触发验证（2026-11-09）

## 方法

- 复用 `foundation/probes/score_readonly2.gdb`：在评分组件回调 `0x2fc8d80` 读取 `score`，在状态回调 `0x2fb9960` 记录状态；不使用启动时全局指针读取。
- 通过 `foundation/run_contact_waypoints_remote.py` 在隔离官方容器中运行早期门序列航点（road1 index 32→31→30→29）。二进制 SHA-256 `5231addf…b97800`，设置 `0d72886f…6c6c17`。
- GDB all-stop 改变时序；这是评分事件诊断，不是比赛或策略结果。

## 结果

- player exit status：**0**（`run.json`；本地 `commands.jsonl` 因 wrapper 超时未下载，原始 capture 和 GDB log 保留）。
- GDB `SEG` 事件出现候选索引：**29、30、31、32**。
- score 从 **0 → 3**，共 3 次增量：

| 增量 | 事件 |
|---:|---|
| 0→1 | `idx=31 enter=1 event=0` |
| 1→2 | `idx=30 enter=1 event=0` |
| 2→3 | `idx=31 enter=0 event=0` |

- `idx=32` 的 enter/exit 事件未使 score 增加；`idx=29` 的 enter/exit 也未使 score 增加。因此不能把“每个候选门 = 一次 score”当作已验证映射。
- 评分组件回调确实由这些候选门索引触发，说明 33+30 运行时组件与官方 score 事件存在直接关联；但队列顺序、起点门/终点门以及重复事件语义仍未解码。

机器可读结果：[g2_detection_gate_score_seed123_20261109.json](g2_detection_gate_score_seed123_20261109.json)，SHA-256 `012ec5578e890e3ed6b00cc442b0d38be1163301713e898768827bb36c85e39b`。原始数据：`foundation/datasets/g2_score_gate_sequence_seed123_20261109_v3/`。

## 限制

- GDB all-stop；score 事件时序受调试器影响。
- 仅一个 seed、一个诊断路径；`commands.jsonl` 因本地传输超时缺失，player 状态取自 `run.json`，capture/GDB log 完整。
- 评分事件与门索引不是一一对应；`idx=32`/`29` 未增加 score，不能据此推断门洞/计分顺序。
- 只覆盖 road1 的四个早期门；road3、其余 59 门、工厂入口和 stage 切换仍未验证。
- “门洞几何”仍未证明；当前证据是候选门中心穿越 + 官方 score 事件关联。

## 状态

G2 仍为 **PARTIAL**。检测门从“仅静态候选”推进到“候选中心穿越可触发官方 score 事件”，但完整 trigger/aperture 语义、工厂入口和其余门仍未完成。
