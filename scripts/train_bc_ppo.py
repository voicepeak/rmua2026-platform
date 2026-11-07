"""Warm-start OL1 stage 1 with scripted demos + DAgger, then optional PPO.

The scripted expert follows the audited stage-1 path with a world-frame
velocity command. Training, evaluation and checkpoint metadata all use the
track mode selected on the command line; the default is the audited stage 1
environment, and the legacy proxy must be selected explicitly.

Run:
    python scripts/train_bc_ppo.py --save-dir runs/stage1_audited_bc

Legacy proxy pipeline comparison:
    python scripts/train_bc_ppo.py --track-mode proxy --segment 1 --save-dir runs/proxy_check
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import torch  # noqa: E402

from rmua.envs.sim_fast import SimFastConfig, SimFastVecEnv  # noqa: E402
from rmua.eval.metrics import summarize_episodes  # noqa: E402
from rmua.training.checkpoint import track_metadata  # noqa: E402
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


class EpisodeRecorder:
    """Write one JSON line per finished training episode."""

    def __init__(self, path: Path, required_gates: int, track_length_m: float):
        self.path = Path(path)
        self.required_gates = int(required_gates)
        self.track_length_m = float(track_length_m)
        self.records: list[dict] = []
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text("", encoding="utf-8")

    def record(self, info, terminated, truncated):
        done = np.logical_or(terminated, truncated)
        for index in np.nonzero(done)[0]:
            success = bool(info["success"][index])
            reason = str(info["failure_reason"][index])
            if not reason:
                reason = (
                    "success"
                    if success
                    else ("truncated" if bool(truncated[index]) else "terminated")
                )
            record = {
                "episode": len(self.records),
                "env": int(index),
                "success": success,
                "gates_passed": int(info["gates_passed"][index]),
                "gates_required": self.required_gates,
                "segment_time_s": float(info["segment_time"][index]),
                "progress_m": float(info["progress_m"][index]),
                "failure_reason": reason,
            }
            self.records.append(record)
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record) + "\n")

    def summary(self) -> dict:
        info = {
            "success": np.array([r["success"] for r in self.records], dtype=bool),
            "gates_passed": np.array(
                [r["gates_passed"] for r in self.records], dtype=np.float64
            ),
            "segment_time": np.array(
                [r["segment_time_s"] for r in self.records], dtype=np.float64
            ),
            "progress_m": np.array(
                [r["progress_m"] for r in self.records], dtype=np.float64
            ),
            "failure_reason": [r["failure_reason"] for r in self.records],
        }
        terminated = np.array(
            [
                r["failure_reason"] not in ("success", "truncated")
                for r in self.records
            ],
            dtype=bool,
        )
        truncated = np.array(
            [r["failure_reason"] == "truncated" for r in self.records],
            dtype=bool,
        )
        return summarize_episodes(
            info,
            required_gates=self.required_gates,
            terminated=terminated,
            truncated=truncated,
            track_length_m=self.track_length_m,
        )


def make_env_config(args, num_envs: int, seed: int) -> SimFastConfig:
    return SimFastConfig(
        num_envs=num_envs,
        segment_id=args.segment,
        map_name=args.map,
        track_mode=args.track_mode,
        detection_gate_width_m=args.detection_gate_width_m,
        detection_gate_height_m=args.detection_gate_height_m,
        stage1_enable_obstacles=args.stage1_enable_obstacles,
        dynamics="kinematic",
        action_mode="vel",
        action_frame="world",
        line_max=0,
        vehicle_max=0,
        seed=seed,
    )


def collect_demos(env, steps, recorder, noise_std=0.0, log_every=100000):
    obs_list = []
    action_list = []
    obs = env.reset()
    wp = np.zeros(env.n, dtype=np.int64)
    rng = np.random.default_rng(env.config.seed + 317)
    started = time.perf_counter()
    for step in range(steps):
        wp = update_waypoints(env, wp)
        expert = scripted_action(env, wp)
        action = expert
        if noise_std > 0.0:
            action = expert + rng.normal(0.0, noise_std, size=expert.shape)
        obs_list.append(obs)
        action_list.append(expert.astype(np.float32))
        next_obs, _, terminated, truncated, info = env.step(action)
        recorder.record(info, terminated, truncated)
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


def collect_policy_demos(
    env, steps, model, obs_scale, recorder, expert_prob=0.35, log_every=50000
):
    obs_list = []
    action_list = []
    obs = env.reset()
    wp = np.zeros(env.n, dtype=np.int64)
    rng = np.random.default_rng(env.config.seed + 911)
    started = time.perf_counter()
    for step in range(steps):
        wp = update_waypoints(env, wp)
        expert = scripted_action(env, wp)
        obs_list.append(obs)
        action_list.append(expert.astype(np.float32))
        with torch.no_grad():
            obs_t = torch.as_tensor(obs, dtype=torch.float32) / obs_scale
            action = model.actor(obs_t).cpu().numpy()
        if expert_prob > 0.0:
            takeover = rng.random(env.n) < expert_prob
            action = np.where(takeover[:, None], expert, action)
        next_obs, _, terminated, truncated, info = env.step(action)
        recorder.record(info, terminated, truncated)
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


def evaluate_actor(model, env, obs_scale):
    env.reset()
    done = np.zeros(env.n, dtype=bool)
    terminal = np.zeros(env.n, dtype=bool)
    truncated_mask = np.zeros(env.n, dtype=bool)
    info = None
    for _ in range(env.max_steps):
        obs = torch.as_tensor(env._obs(), dtype=torch.float32) / obs_scale
        with torch.no_grad():
            action = model.actor(obs).cpu().numpy()
        _, _, terminated, truncated, info = env.step(action)
        terminal |= terminated
        truncated_mask |= truncated
        done |= terminated | truncated
        if np.all(done):
            break
    if info is None:
        info = env._info()
    truncated_mask |= ~done
    required = len(env.track.scoring_gates) if env.track.audited_stage1 else 2
    return summarize_episodes(
        info if info is not None else {},
        required_gates=required,
        terminated=terminal,
        truncated=truncated_mask,
        track_length_m=env.track.total_length_m,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--segment", type=int, default=1)
    parser.add_argument("--map", default="RMUA2024OL1")
    parser.add_argument(
        "--track-mode", choices=("stage1_audited", "proxy"), default="stage1_audited"
    )
    parser.add_argument("--detection-gate-width-m", type=float, default=10.0)
    parser.add_argument("--detection-gate-height-m", type=float, default=5.0)
    parser.add_argument("--stage1-enable-obstacles", action="store_true")
    parser.add_argument("--skip-ppo", action="store_true")
    parser.add_argument("--demo-envs", type=int, default=64)
    parser.add_argument("--demo-steps", type=int, default=30000)
    parser.add_argument("--demo-noise-std", type=float, default=0.5)
    parser.add_argument("--dagger-rounds", type=int, default=4)
    parser.add_argument("--dagger-steps", type=int, default=15000)
    parser.add_argument("--dagger-expert-prob", type=float, default=0.35)
    parser.add_argument("--max-demos", type=int, default=4_000_000)
    parser.add_argument("--bc-epochs", type=int, default=10)
    parser.add_argument("--dagger-bc-epochs", type=int, default=5)
    parser.add_argument("--bc-batch", type=int, default=8192)
    parser.add_argument("--hidden", type=int, default=128)
    parser.add_argument("--envs", type=int, default=256)
    parser.add_argument("--ppo-steps", type=int, default=100_000_000)
    parser.add_argument("--num-steps", type=int, default=256)
    parser.add_argument("--save-dir", default="runs/stage1_audited_bc")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-wall-hours", type=float, default=3.0)
    parser.add_argument("--target-success", type=float, default=0.8)
    parser.add_argument("--device", default="cpu")
    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.track_mode == "stage1_audited" and args.segment != 1:
        parser.error("stage1_audited is available only for segment 1")

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()

    demo_env = SimFastVecEnv(make_env_config(args, args.demo_envs, args.seed + 100))
    eval_env = SimFastVecEnv(make_env_config(args, 16, args.seed + 200))
    track = track_metadata(eval_env)
    required_gates = len(eval_env.track.scoring_gates) if eval_env.track.audited_stage1 else 2
    print(
        json.dumps(
            {
                "map": args.map,
                "track_mode": args.track_mode,
                "segment": args.segment,
                "track_length_m": round(float(eval_env.track.total_length_m), 3),
                "scoring_gates": len(eval_env.track.scoring_gates),
                "observation_source": track["observation_source"],
            }
        ),
        flush=True,
    )

    recorder = EpisodeRecorder(
        save_dir / "train_episodes.jsonl",
        required_gates,
        eval_env.track.total_length_m,
    )

    print("collecting scripted demos", flush=True)
    demo_obs, demo_actions = collect_demos(
        demo_env, args.demo_steps, recorder, noise_std=args.demo_noise_std
    )
    print(
        json.dumps(
            {
                "stage": "demo_collect",
                "demos": int(demo_obs.shape[0]),
                "obs_dim": int(demo_obs.shape[1]),
                **recorder.summary(),
            }
        ),
        flush=True,
    )

    obs_scale = torch.tensor(_observation_scale(demo_obs.shape[1]), dtype=torch.float32)
    model = ActorCritic(demo_obs.shape[1], demo_actions.shape[1], hidden=args.hidden)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)

    ppo_config = PPOConfig(
        total_steps=args.ppo_steps,
        num_envs=args.envs,
        num_steps=args.num_steps,
        segment_id=args.segment,
        map_name=args.map,
        track_mode=args.track_mode,
        detection_gate_width_m=args.detection_gate_width_m,
        detection_gate_height_m=args.detection_gate_height_m,
        stage1_enable_obstacles=args.stage1_enable_obstacles,
        dynamics="kinematic",
        action_frame="world",
        line_max=0,
        vehicle_max=0,
        hidden=args.hidden,
        device=args.device,
        save_dir=str(save_dir),
        seed=args.seed,
        eval_every=20,
        target_success=args.target_success,
        patience=2,
    )

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
    bc_stats = evaluate_actor(model, eval_env, obs_scale)
    print(json.dumps({"stage": "bc_eval", **bc_stats}), flush=True)

    for round_index in range(args.dagger_rounds):
        print(json.dumps({"dagger_round": round_index + 1}), flush=True)
        new_obs, new_actions = collect_policy_demos(
            demo_env,
            args.dagger_steps,
            model,
            obs_scale,
            recorder,
            expert_prob=args.dagger_expert_prob,
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
        dagger_stats = evaluate_actor(model, eval_env, obs_scale)
        print(
            json.dumps(
                {
                    "stage": f"dagger_round_{round_index + 1}",
                    "demos": int(demo_obs.shape[0]),
                    **dagger_stats,
                }
            ),
            flush=True,
        )

    bc_payload = {
        "model": model.state_dict(),
        "config": asdict(ppo_config),
        "obs_scale": obs_scale.tolist(),
        "total_steps": 0,
        "track": track,
    }
    torch.save(bc_payload, save_dir / "bc_model.pt")
    final_stats = evaluate_actor(model, eval_env, obs_scale)
    print(json.dumps({"stage": "bc_final", **final_stats}), flush=True)
    torch.save(bc_payload, save_dir / "best.pt")
    (save_dir / "config.json").write_text(
        json.dumps(asdict(ppo_config), indent=2) + "\n", encoding="utf-8"
    )
    (save_dir / "track.json").write_text(
        json.dumps(track, indent=2) + "\n", encoding="utf-8"
    )
    train_summary = {
        "map": args.map,
        "track_mode": args.track_mode,
        "segment": args.segment,
        "seed": args.seed,
        "args": vars(args),
        "track": track,
        "episode_summary": recorder.summary(),
        "final_eval": final_stats,
    }
    (save_dir / "train_summary.json").write_text(
        json.dumps(train_summary, indent=2) + "\n", encoding="utf-8"
    )

    if args.skip_ppo:
        print(json.dumps({"skip_ppo": True, "saved": str(save_dir)}), flush=True)
        return 0

    remaining = max(0.0, args.max_wall_hours * 3600.0 - (time.perf_counter() - started))
    ppo_config.max_wall_s = remaining
    trainer = PPOTrainer(ppo_config)
    trainer.model.load_state_dict(model.state_dict())
    trainer.optimizer = torch.optim.Adam(trainer.model.parameters(), lr=ppo_config.lr)
    print("starting ppo fine-tune", flush=True)
    trainer.train()
    print(json.dumps({"saved": str(save_dir), "wall_s": time.perf_counter() - started}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
