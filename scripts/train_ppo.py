"""Train the first node-2 task: 1-3 state-only with PPO.

CPU example:
    python scripts/train_ppo.py --envs 256 --steps 2000000 --save-dir runs/ppo_1_3

Smoke test:
    python scripts/train_ppo.py --smoke
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rmua.training.ppo import PPOConfig, PPOTrainer  # noqa: E402


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--envs", type=int, default=256)
    parser.add_argument("--steps", type=int, default=2_000_000)
    parser.add_argument("--num-steps", type=int, default=256)
    parser.add_argument("--segment", type=int, default=1)
    parser.add_argument("--dynamics", choices=("kinematic", "quad"), default="kinematic")
    parser.add_argument("--action-frame", choices=("body", "world"), default="world")
    parser.add_argument("--line-max", type=int, default=0)
    parser.add_argument("--vehicle-max", type=int, default=0)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--save-dir", default="runs/ppo_1_3")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--eval-every", type=int, default=20)
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args(argv)

    if args.smoke:
        args.envs = min(args.envs, 8)
        args.steps = args.num_steps * args.envs * 2
        args.eval_every = 1
        args.save_dir = "runs/smoke"

    config = PPOConfig(
        total_steps=args.steps,
        num_envs=args.envs,
        num_steps=args.num_steps,
        segment_id=args.segment,
        dynamics=args.dynamics,
        action_frame=args.action_frame,
        line_max=args.line_max,
        vehicle_max=args.vehicle_max,
        device=args.device,
        save_dir=args.save_dir,
        seed=args.seed,
        eval_every=args.eval_every,
    )
    trainer = PPOTrainer(config)
    print(
        f"device={config.device} envs={config.num_envs} steps={config.total_steps} "
        f"segment={config.segment_id} obs={trainer.env.observation_dim}",
        flush=True,
    )
    trainer.train()
    print(f"saved {trainer.save_dir}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
