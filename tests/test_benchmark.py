import time

import numpy as np

from rmua.envs.sim_fast import SimFastConfig, SimFastVecEnv


def test_state_only_throughput():
    n = 64
    env = SimFastVecEnv(
        SimFastConfig(num_envs=n, segment_id=1, dynamics="kinematic", action_mode="vel")
    )
    action = np.zeros((n, 4))
    action[:, 0] = 3.0
    env.reset()
    steps = 100
    started = time.perf_counter()
    for _ in range(steps):
        env.step(action)
    elapsed = time.perf_counter() - started
    env_steps_per_s = steps * n / max(elapsed, 1e-9)
    # Loose local floor; the server benchmark is the real acceptance gate.
    assert env_steps_per_s > 20_000
