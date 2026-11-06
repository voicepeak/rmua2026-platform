"""Evaluate a PPO/BC checkpoint on sim-fast with deterministic actions.

    python scripts/eval_policy.py --checkpoint runs/curriculum/seg1/best.pt
    python scripts/eval_policy.py --checkpoint runs/curriculum/seg2/best.pt --seeds 101,202,303
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import torch  # noqa: E402

from rmua.envs.sim_fast import SimFastConfig, SimFastVecEnv  # noqa: E402
from rmua.training.ppo import ActorCritic, PPOConfig  # noqa: E402


def evaluate(checkpoint: Path, segment: int | None, seeds, envs: int, max_steps: int):
    ckpt = torch.load(checkpoint, map_location="cpu", weights_only=False)
    cfg = PPOConfig(**ckpt["config"])
    model = ActorCritic(33, 4, cfg.hidden)
    model.load_state_dict(ckpt["model"])
    model.eval()
    scale = torch.tensor(ckpt["obs_scale"], dtype=torch.float32)
    segment_id = segment or cfg.segment_id
    rates = []
    times = []
    lengths = []
    for seed in seeds:
        env = SimFastVecEnv(
            SimFastConfig(
                num_envs=envs,
                segment_id=segment_id,
                dynamics="kinematic",
                action_mode="vel",
                action_frame=cfg.action_frame,
                line_max=cfg.line_max,
                vehicle_max=cfg.vehicle_max,
                seed=seed,
            )
        )
        env.reset()
        done = np.zeros(env.n, dtype=bool)
        info = None
        for _ in range(max_steps):
            obs = torch.as_tensor(env._obs(), dtype=torch.float32) / scale
            with torch.no_grad():
                action = model.actor(obs).numpy()
            _, _, terminated, truncated, info = env.step(action)
            done |= terminated | truncated
            if np.all(done):
                break
        success = float(np.mean(info["success"])) if info is not None else 0.0
        rates.append(success)
        lengths.append(env.track.total_length_m)
        if info is not None:
            times.extend(info["segment_time"][info["success"]].tolist())
    mean_time = float(np.mean(times)) if times else 0.0
    mean_length = float(np.mean(lengths)) if lengths else 0.0
    return {
        "checkpoint": str(checkpoint),
        "segment": segment_id,
        "seeds": list(seeds),
        "envs": envs,
        "rates": rates,
        "mean": float(np.mean(rates)),
        "mean_time_s": mean_time,
        "mean_speed_mps": mean_length / mean_time if mean_time > 0 else 0.0,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--segment", type=int, default=None)
    parser.add_argument("--envs", type=int, default=64)
    parser.add_argument("--seeds", default="101,202,303,404,505")
    parser.add_argument("--max-steps", type=int, default=20000)
    args = parser.parse_args(argv)
    seeds = [int(value) for value in args.seeds.split(",") if value.strip()]
    result = evaluate(args.checkpoint, args.segment, seeds, args.envs, args.max_steps)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
