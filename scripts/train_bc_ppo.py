"""Warm-start 1-3 with scripted demos + DAgger, then PPO fine-tune on CPU.

The scripted expert follows start gate -> hub -> end gate with a world-frame
velocity command. BC gives PPO a flying policy instead of spending hours
discovering forward flight; DAgger fixes the BC distribution shift.

Run:
    python scripts/train_bc_ppo.py --save-dir runs/bc_ppo_1_3
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

import torch  # noqa: E402

from rmua.envs.sim_fast import SimFastConfig, SimFastVecEnv  # noqa: E402
from rmua.training.ppo import (  # noqa: E402
    ActorCritic,
    PPOConfig,
    PPOTrainer,
    _observation_scale,
)


def scripted_action(env, wp=None):
    """Path-following expert with a curvature speed limit."""
    distance, _, _, _, _ = env.track.project(env.pos)
    ahead_distance = np.minimum(distance + 8.0, env.track.total_length_m)
    target = env.track.points_at_distances(ahead_distance)
    tangent_now = env.track.tangents_at_distances(distance)
    tangent_ahead = env.track.tangents_at_distances(
        np.minimum(distance + 25.0, env.track.total_length_m)
    )
    cosine = np.clip(np.sum(tangent_now * tangent_ahead, axis=1), -1.0, 1.0)
    turn_angle = np.arccos(cosine)
    turn_factor = np.clip(1.0 - turn_angle / 1.6, 0.55, 1.0)
    delta = target - env.pos
    target_distance = np.linalg.norm(delta, axis=1, keepdims=True)
    speed = np.clip(target_distance * 3.0, 0.0, 25.0) * turn_factor[:, None]
    velocity = delta / np.maximum(target_distance, 1e-6) * speed
    return np.concatenate((velocity, np.zeros((env.n, 1))), axis=1)


def update_waypoints(env, wp):
    start = env.track.start_gate.center
    hub = env.map_data.centroid
    d_start = np.linalg.norm(env.pos - start, axis=1)
    d_hub = np.linalg.norm(env.pos - hub, axis=1)
    wp = np.where((wp == 0) & ((env.gate_index >= 1) | (d_start < 1.5)), 1, wp)
    wp = np.where((wp == 1) & (d_hub < 5.0), 2, wp)
    return np.clip(wp, 0, 2)


def _flatten(obs_list, action_list):
    obs_dim = obs_list[0].shape[1]
    action_dim = action_list[0].shape[1]
    return (
        np.asarray(obs_list, dtype=np.float32).reshape(-1, obs_dim),
        np.asarray(action_list, dtype=np.float32).reshape(-1, action_dim),
    )


def collect_demos(env, steps, log_every=100000):
    obs_list = []
    action_list = []
    obs = env.reset()
    wp = np.zeros(env.n, dtype=np.int64)
    started = time.perf_counter()
    for step in range(steps):
        wp = update_waypoints(env, wp)
        action = scripted_action(env, wp)
        obs_list.append(obs)
        action_list.append(action.astype(np.float32))
        next_obs, _, terminated, truncated, _ = env.step(action)
        done = np.logical_or(terminated, truncated)
        if np.any(done):
            obs = env.reset_indices(np.nonzero(done)[0])
            wp[done] = 0
        else:
            obs = next_obs
        if (step + 1) % log_every == 0:
            print(
                json.dumps(
                    {"demo_steps": step + 1, "wall_s": time.perf_counter() - started}
                ),
                flush=True,
            )
    return _flatten(obs_list, action_list)


def collect_policy_demos(env, steps, model, obs_scale, log_every=50000):
    obs_list = []
    action_list = []
    obs = env.reset()
    wp = np.zeros(env.n, dtype=np.int64)
    started = time.perf_counter()
    for step in range(steps):
        wp = update_waypoints(env, wp)
        expert = scripted_action(env, wp)
        obs_list.append(obs)
        action_list.append(expert.astype(np.float32))
        with torch.no_grad():
            obs_t = torch.as_tensor(obs, dtype=torch.float32) / obs_scale
            action = model.actor(obs_t).cpu().numpy()
        next_obs, _, terminated, truncated, _ = env.step(action)
        done = np.logical_or(terminated, truncated)
        if np.any(done):
            obs = env.reset_indices(np.nonzero(done)[0])
            wp[done] = 0
        else:
            obs = next_obs
        if (step + 1) % log_every == 0:
            print(
                json.dumps(
                    {"dagger_steps": step + 1, "wall_s": time.perf_counter() - started}
                ),
                flush=True,
            )
    return _flatten(obs_list, action_list)


def train_bc(model, optimizer, obs, actions, obs_scale, epochs, batch, tag):
    obs_t = torch.as_tensor(obs, dtype=torch.float32) / obs_scale
    act_t = torch.as_tensor(actions, dtype=torch.float32)
    n = obs_t.shape[0]
    for epoch in range(epochs):
        perm = torch.randperm(n)
        total = 0.0
        count = 0
        for start in range(0, n, batch):
            index = perm[start : start + batch]
            pred = model.actor(obs_t[index])
            loss = torch.nn.functional.mse_loss(pred, act_t[index])
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            total += float(loss.detach()) * index.numel()
            count += index.numel()
        print(
            json.dumps({"tag": tag, "epoch": epoch + 1, "mse": total / max(count, 1)}),
            flush=True,
        )


def evaluate_actor(model, env, obs_scale, max_steps=3000):
    env.reset()
    done = np.zeros(env.n, dtype=bool)
    info = None
    for _ in range(max_steps):
        obs = torch.as_tensor(env._obs(), dtype=torch.float32) / obs_scale
        with torch.no_grad():
            action = model.actor(obs).cpu().numpy()
        _, _, terminated, truncated, info = env.step(action)
        done |= terminated | truncated
        if np.all(done):
            break
    return float(np.mean(info["success"])) if info is not None else 0.0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--segment", type=int, default=1)
    parser.add_argument("--demo-envs", type=int, default=64)
    parser.add_argument("--demo-steps", type=int, default=30000)
    parser.add_argument("--dagger-rounds", type=int, default=4)
    parser.add_argument("--dagger-steps", type=int, default=25000)
    parser.add_argument("--max-demos", type=int, default=4_000_000)
    parser.add_argument("--bc-epochs", type=int, default=12)
    parser.add_argument("--dagger-bc-epochs", type=int, default=6)
    parser.add_argument("--bc-batch", type=int, default=8192)
    parser.add_argument("--envs", type=int, default=256)
    parser.add_argument("--ppo-steps", type=int, default=100_000_000)
    parser.add_argument("--num-steps", type=int, default=256)
    parser.add_argument("--save-dir", default="runs/bc_ppo_1_3")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-wall-hours", type=float, default=3.0)
    parser.add_argument("--target-success", type=float, default=0.8)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args(argv)

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()

    demo_env = SimFastVecEnv(
        SimFastConfig(
            num_envs=args.demo_envs,
            segment_id=args.segment,
            dynamics="kinematic",
            action_mode="vel",
            action_frame="world",
            line_max=0,
            vehicle_max=0,
            seed=args.seed + 100,
        )
    )
    eval_env = SimFastVecEnv(
        SimFastConfig(
            num_envs=16,
            segment_id=args.segment,
            dynamics="kinematic",
            action_mode="vel",
            action_frame="world",
            line_max=0,
            vehicle_max=0,
            seed=args.seed + 200,
        )
    )

    print("collecting scripted demos", flush=True)
    demo_obs, demo_actions = collect_demos(demo_env, args.demo_steps)
    print(
        json.dumps({"demos": int(demo_obs.shape[0]), "obs_dim": int(demo_obs.shape[1])}),
        flush=True,
    )

    obs_scale = torch.tensor(_observation_scale(demo_obs.shape[1]), dtype=torch.float32)
    model = ActorCritic(demo_obs.shape[1], demo_actions.shape[1], hidden=128)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)

    print("initial behavior cloning", flush=True)
    train_bc(
        model,
        optimizer,
        demo_obs,
        demo_actions,
        obs_scale,
        args.bc_epochs,
        args.bc_batch,
        "bc",
    )
    print(json.dumps({"bc_success": evaluate_actor(model, eval_env, obs_scale)}), flush=True)

    for round_index in range(args.dagger_rounds):
        print(json.dumps({"dagger_round": round_index + 1}), flush=True)
        new_obs, new_actions = collect_policy_demos(
            demo_env, args.dagger_steps, model, obs_scale
        )
        demo_obs = np.concatenate((demo_obs, new_obs), axis=0)
        demo_actions = np.concatenate((demo_actions, new_actions), axis=0)
        if demo_obs.shape[0] > args.max_demos:
            keep = np.random.default_rng(args.seed + round_index).choice(
                demo_obs.shape[0], args.max_demos, replace=False
            )
            demo_obs = demo_obs[keep]
            demo_actions = demo_actions[keep]
        train_bc(
            model,
            optimizer,
            demo_obs,
            demo_actions,
            obs_scale,
            args.dagger_bc_epochs,
            args.bc_batch,
            f"dagger{round_index + 1}",
        )
        success = evaluate_actor(model, eval_env, obs_scale)
        print(json.dumps({"dagger_round": round_index + 1, "success": success}), flush=True)

    torch.save({"model": model.state_dict()}, save_dir / "bc_model.pt")
    print(json.dumps({"bc_final_success": evaluate_actor(model, eval_env, obs_scale)}), flush=True)

    remaining = max(0.0, args.max_wall_hours * 3600.0 - (time.perf_counter() - started))
    config = PPOConfig(
        total_steps=args.ppo_steps,
        num_envs=args.envs,
        num_steps=args.num_steps,
        segment_id=args.segment,
        dynamics="kinematic",
        action_frame="world",
        line_max=0,
        vehicle_max=0,
        device=args.device,
        save_dir=str(save_dir),
        seed=args.seed,
        eval_every=20,
        target_success=args.target_success,
        patience=2,
        max_wall_s=remaining,
    )
    trainer = PPOTrainer(config)
    trainer.model.load_state_dict(model.state_dict())
    trainer.optimizer = torch.optim.Adam(trainer.model.parameters(), lr=config.lr)
    print("starting ppo fine-tune", flush=True)
    trainer.train()
    print(json.dumps({"saved": str(save_dir), "wall_s": time.perf_counter() - started}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
