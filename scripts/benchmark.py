"""Measure sim-fast throughput on the local machine.

This is the Python reference implementation. The server acceptance gate should
use the same workload after the C++/JAX backend lands.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rmua.envs.sim_fast import SimFastConfig, SimFastVecEnv  # noqa: E402


def benchmark(n: int, steps: int, warmup: int, dynamics: str, segment: int) -> dict:
    env = SimFastVecEnv(
        SimFastConfig(
            num_envs=n,
            segment_id=segment,
            dynamics=dynamics,
            action_mode="vel",
        )
    )
    action = np.zeros((n, 4))
    action[:, 0] = 3.0
    env.reset()
    for _ in range(warmup):
        env.step(action)
    started = time.perf_counter()
    for _ in range(steps):
        env.step(action)
    elapsed = time.perf_counter() - started
    env_steps_per_s = steps * n / max(elapsed, 1e-9)
    return {
        "num_envs": n,
        "steps": steps,
        "wall_s": elapsed,
        "env_steps_per_s": env_steps_per_s,
        "per_env_realtime_factor": env_steps_per_s / n / 100.0,
        "sim_seconds_per_wall_second": env_steps_per_s * 0.01,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--envs", default="64,256,1024", help="comma-separated env counts")
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--dynamics", choices=("kinematic", "quad"), default="kinematic")
    parser.add_argument("--segment", type=int, default=1)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    results = []
    for n in [int(value) for value in args.envs.split(",") if value.strip()]:
        result = benchmark(n, args.steps, args.warmup, args.dynamics, args.segment)
        results.append(result)
        if not args.json:
            print(
                f"n={n:5d}  {result['env_steps_per_s']:10.1f} env-steps/s  "
                f"{result['per_env_realtime_factor']:6.2f}x realtime/env  "
                f"{result['wall_s']:.3f}s"
            )
    if args.json:
        print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
