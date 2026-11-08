# OL2 门根变换对照

独立读取 OL2 序列化地图提取的 12 个门候选，与历史开发补丁直接加载 OL2 的门初始化日志比较：seed 123、456、2026 各两次，六次均完成 12 对一一匹配。六次各自最大的中心位置差约 0.000000673 cm、旋转差约 0.00000355°、缩放分量差约 0.000000435；数值受日志输出精度限制。匹配通过最小位置距离分配，运行时 FName 没有解码。

每次对照的源文件哈希、逐门误差和运行记录见 [123-1](gate_runtime_seed123_repeat1.json)、[123-2](gate_runtime_seed123_repeat2.json)、[456-1](gate_runtime_seed456_repeat1.json)、[456-2](gate_runtime_seed456_repeat2.json)、[2026-1](gate_runtime_seed2026_repeat1.json)、[2026-2](gate_runtime_seed2026_repeat2.json)。这些是带 all-stop GDB 和修改默认地图配置的历史启动观测；该启动路径未证明 IMU 就绪或正常飞行。对照只支持所观察进程的门根初始化变换，不能证明门洞形状、碰撞、触发方向或官方计分。
