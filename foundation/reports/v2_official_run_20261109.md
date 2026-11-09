# 2026-11-09 官方隔离运行记录

本次使用 `foundation/run_passive_remote.py` 启动五个新的隔离官方容器；每次使用新的进程/容器，结束后由 harness 停止。没有使用 GDB，未修改逆向仓库。远端目录和容器均为临时资源。

## 运行列表

| 数据集 | 模式 | seed | 结果 | 容器 |
|---|---|---:|---|---|
| `v2_official_passive_smoke_20261109` | passive 5 s | 123 | exit 0 | `rmua-re-0e4c22605365-0` 已停止 |
| `v2_official_heldout_mix_seed260004_20261109` | heldout_mix | 260004 | player=2，`motion guard exceeded`；保留失败记录 | `rmua-re-09e3e485bbfd-0` 已停止 |
| `v2_official_heldout_mix_small_seed260005_20261109` | heldout_mix_small | 260005 | player=0，完成 | `rmua-re-04807943a0fc-0` 已停止 |
| `v2_official_heldout_mix_seed260006_20261109` | heldout_mix | 260006 | player=2，`motion guard exceeded`；保留失败记录 | `rmua-re-d075547669c3-0` 已停止 |
| `v2_official_differential_seed260007_20261109` | differential | 260007 | player=0，完成 | `rmua-re-684eb67423a9-0` 已停止 |

四次有 PWM 的运行在 `pool/pool.json` 中记录的二进制 SHA-256 均为 `5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800`，设置 SHA-256 均为 `0d72886f92b091cbc5ba2ae58d25b4b4bc589045d4e725f483603efdc26c6c17`。

## 采集与哈希

| 数据集 | capture 行数 | pose/IMU/PWM/wind | commands | capture SHA-256 | commands SHA-256 |
|---|---:|---|---:|---|---|
| passive smoke | 1,797 | 499/498/499/250 | 0 | `86accaa331754424a03b6c63e507792dbfe7e49dfd9e909b3aa26b1d73103470` | 空文件 |
| heldout_mix seed260004 | 879 | 244/244/244/122 | 49 command + start/end | `41afc0cb61b6ea00d25a3f660153e10915eefdd1d27d314efd0c87252e04ffc1` | `9d234626dcaf70d5a54aaf383d88ccf278aa9b6d77b60dbfa16680ed14569e16` |
| heldout_mix_small seed260005 | 1,049 | 291/291/291/146 | 68 command + start/end | `a77759c8193b813f2f1412da9e39f698ba06e65103028e153e761ec6f277b692` | `27ba8646d18b3172a0e62b21485f8437638142a7c41bcdb542dc9be23825d592` |
| heldout_mix seed260006 | 895 | 248/248/248/124 | 53 command + start/end | `cc4984ab636ff61096458ce6d4c0e3dfbb350a7b4071a35574fc07d41a624864` | `e5af83122a59844a8afb80333e4195c1633c5567c1f11cc794fb63a31a0cca14` |
| differential seed260007 | 1,072 | 270/270/270/135 | 64 command + start/end | `8ccf95fc63bcf16f95c9345751846634d47f39208a7a5d9da5890dddfa870e05` | `042d69df35cd11511d457a049b0c46a4ebad457b8ac79c61c867da26c48eb530` |

`run.json` 与清理日志的哈希保留在各数据集目录；`cleanup.log` 均记录容器已停止。运行结束后只读复查远端：仅剩既有的 `rmua_noetic` 容器，未发现 `RMUA-Linux-Shipping` 进程或 `rmua-re-*` 专用容器。

## 新证据

- 新进程 `heldout_mix` seed260004 的自由落体尾段满足干净准则；源时间拟合加速度 9.7897 m/s²（相对 g -0.173%），到达时间 9.7972 m/s²，二次拟合 RMSE 0.0143/1.322 mm。该进程随后因 `motion guard exceeded` 中止，失败记录保留，不当作完整飞行。
- 新进程 `heldout_mix` seed260006 同样因运动保护中止，但其自由落体尾段给出源时间 9.7908 m/s²（-0.162%）、到达时间 9.8790 m/s²（+0.738%），二次拟合 RMSE 0.0141/1.083 mm；失败记录保留。
- 新进程 `heldout_mix_small` seed260005 的 8 个命令边缘均有数值匹配的调试 PWM 回显；首个匹配的主机到达时间比命令记录晚约 16.2–66.0 ms，Header 差约 -5.4 到 +44.4 ms。Header 仍来自单独的 IMU 请求，不能解释为物理执行延迟；完整边缘见 [v2_official_heldout_mix_small_seed260005_echo.json](v2_official_heldout_mix_small_seed260005_echo.json)（SHA-256 `a138d034ec000119ac0c037441773aef13313463998f6c3e2b321a5c2280e85c`）。
- 新进程 `differential` seed260007 完成；四个单电机脉冲的 x/y 角速度符号与前两次相同，使 G1 方向审计达到三次进程 **12/12** 一致；yaw/z 仍低于报告阈值。该进程 7/7 命令边缘有回显匹配，主机到达延迟约 15.9–52.3 ms，Header 差 -3.7 到 +32.7 ms；审计见 [v2_official_differential_seed260007_echo.json](v2_official_differential_seed260007_echo.json)（SHA-256 `3e21f07cea43f69486bada0b5273847723227280cc297b5b2bca8333fc97d088`）。

## 官方实时因子（同机、非吞吐）

新 passive 采集的 pose 源时间跨度 4.980106 s、主机到达跨度 4.980088 s，比值 **1.0000036**；`heldout_mix_small` seed260005 为 2.901062/2.899815 s，比值 **1.0004301**。这说明当前官方进程在该窗口接近实时推进，而不是可以按环境命令数直接比较的批处理引擎。该测量是源 Header 与主机到达的比值，不是物理 tick 计数，也不能换算成官方 64/256 环境吞吐。机器可读结果见 [v2_official_realtime_factor.json](v2_official_realtime_factor.json)（SHA-256 `1b8151f6e8be141bd825c102d529800123b3bfff8b3ae466ae10325bebff4de2`）。

## 限制

- passive 运行只验证采集链路，不提供 PWM 物理证据。
- seed260004 的 player 被运动保护中止；自由落体尾段可用于时钟一致性，不代表完整飞行或 G1 通过。
- 回显 Header 不是电机采样时间，数值匹配不证明物理应用时刻。
- 这些运行没有解除 G1 的参数辨识阻塞，也没有完成 G2 官方接触或 G4 官方吞吐对照。
