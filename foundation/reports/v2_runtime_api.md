# V2 单环境 PWM 接口

`foundation.runtime.RmuaPhysics` 包装现有候选 `PhysicsSim`，每次 `step(pwm, dt)` 接受四个数值，持续保持到此次命令结束、碰撞或超时。`dt` 是命令保持时间，必须为构造时 `physics_dt` 的整数倍；每个固定步长调用一次底层物理。默认候选输入范围 [0, 1] 不是官方裁剪已验证的证据。

```python
from foundation.quad import Params
from foundation.runtime import RmuaPhysics

# 以下数值仅用于演示接口，非官方验收参数。
p = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
sim = RmuaPhysics(p, physics_dt=.0025, timeout_s=5.)
observation = sim.reset(initial_state={'rotor_signal': [.18] * 4})
observation, events = sim.step([.18] * 4, dt=.02)
research_truth = sim.truth_state()  # 显式访问仿真真值
```

默认返回的 `observable_state` 为 `None`，是未来状态估计器的占位，**不是**已经实现的传感器模型或比赛可用状态。`truth_state()` 是显式研究接口；策略使用者不得将其误当作合法比赛观测。事件包括候选碰撞、门通过、力场状态和候选计时，以及自建环境超时；候选状态并非官方裁判结论。

地图、风、力场和障碍通过底层 `PhysicsSim` 的可选构造参数显式配置；此接口本身不保证完整地图、官方物理、seed 场景生成、不同进程时钟同步或批量执行。力回放和记录效果的时间覆盖会在执行前检查；即使路径可能提前碰撞，未覆盖的整段命令仍会被保守拒绝。G1/G2/G3/G4 不因接口存在而通过。
