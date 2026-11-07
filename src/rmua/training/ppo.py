"""Minimal PPO for the state-only sim-fast environment.

CPU-friendly: small MLP, vectorized env, no rendering. This is the first node-2
training loop, not a final algorithm. Run `scripts/train_ppo.py` to use it.
"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

try:
    import torch
    import torch.nn as nn
    from torch.distributions import Normal
except ImportError as exc:  # pragma: no cover - optional dependency
    raise ImportError(
        "PPO training needs PyTorch. Install the CPU wheel: "
        "python -m pip install torch --index-url https://download.pytorch.org/whl/cpu"
    ) from exc

from ..envs.sim_fast import SimFastConfig, SimFastVecEnv
from ..eval.metrics import summarize_episodes
from .checkpoint import track_metadata


@dataclass
class PPOConfig:
    total_steps: int = 2_000_000
    num_envs: int = 256
    num_steps: int = 256
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip: float = 0.2
    lr: float = 1e-3
    epochs: int = 10
    minibatch: int = 4096
    entropy_coef: float = 0.0001
    value_coef: float = 0.5
    reward_scale: float = 0.1
    max_grad_norm: float = 0.5
    hidden: int = 128
    seed: int = 1
    segment_id: int = 1
    map_name: str = "RMUA2024OL1"
    track_mode: str = "proxy"
    detection_gate_width_m: float = 10.0
    detection_gate_height_m: float = 5.0
    stage1_enable_obstacles: bool = False
    dynamics: str = "kinematic"
    action_frame: str = "world"
    line_max: int = 0
    vehicle_max: int = 0
    eval_envs: int = 16
    eval_every: int = 20
    target_success: float = 0.85
    patience: int = 2
    max_wall_s: float = 0.0
    save_dir: str = "runs/ppo"
    device: str = "cpu"
    log_every: int = 1
    action_bias: tuple[float, float, float, float] = (4.0, 0.0, 0.0, 0.0)
    init_log_std: tuple[float, float, float, float] = (-0.5, -0.5, -0.5, -2.5)


class ActorCritic(nn.Module):
    def __init__(self, obs_dim: int, act_dim: int, hidden: int = 128):
        super().__init__()
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
            nn.Linear(hidden, act_dim),
        )
        self.critic = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
            nn.Linear(hidden, 1),
        )
        self.log_std = nn.Parameter(torch.zeros(act_dim))

    def distribution(self, obs):
        mean = self.actor(obs)
        std = torch.exp(self.log_std).expand_as(mean)
        return Normal(mean, std)

    def value(self, obs):
        return self.critic(obs).squeeze(-1)


class PPOTrainer:
    def __init__(self, config: PPOConfig):
        self.config = config
        self.device = torch.device(config.device)
        torch.manual_seed(config.seed)
        np.random.seed(config.seed)
        self.env = self._make_env(config.num_envs, eval_env=False)
        self.eval_env = self._make_env(config.eval_envs, eval_env=True)
        self.model = ActorCritic(
            self.env.observation_dim, self.env.action_dim, config.hidden
        ).to(self.device)
        self.obs_scale = torch.tensor(
            _observation_scale(self.env.observation_dim),
            dtype=torch.float32,
            device=self.device,
        )
        with torch.no_grad():
            # Start as a forward-flying controller, then let PPO refine it.
            if config.action_frame == "world":
                normal = self.env.track.start_gate.normal
                bias = [float(normal[0]) * 4.0, float(normal[1]) * 4.0, 0.0, 0.0]
            else:
                bias = list(config.action_bias)
            self.model.actor[-1].bias.copy_(
                torch.tensor(bias, dtype=torch.float32, device=self.device)
            )
            self.model.log_std.copy_(
                torch.tensor(config.init_log_std, dtype=torch.float32, device=self.device)
            )
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=config.lr)
        self.save_dir = Path(config.save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.metrics_path = self.save_dir / "metrics.jsonl"
        self.total_steps = 0
        self.obs = torch.as_tensor(
            self.env.reset(), dtype=torch.float32, device=self.device
        )

    def _make_env(self, num_envs: int, eval_env: bool) -> SimFastVecEnv:
        cfg = SimFastConfig(
            num_envs=num_envs,
            segment_id=self.config.segment_id,
            map_name=self.config.map_name,
            track_mode=self.config.track_mode,
            detection_gate_width_m=self.config.detection_gate_width_m,
            detection_gate_height_m=self.config.detection_gate_height_m,
            stage1_enable_obstacles=self.config.stage1_enable_obstacles,
            dynamics=self.config.dynamics,
            action_mode="vel",
            action_frame=self.config.action_frame,
            seed=self.config.seed + (1000 if eval_env else 0),
            line_max=self.config.line_max,
            vehicle_max=self.config.vehicle_max,
        )
        return SimFastVecEnv(cfg)

    # ------------------------------------------------------------------
    def collect(self):
        cfg = self.config
        n = cfg.num_envs
        obs_buf = torch.zeros((cfg.num_steps, n, self.env.observation_dim), device=self.device)
        action_buf = torch.zeros((cfg.num_steps, n, self.env.action_dim), device=self.device)
        logp_buf = torch.zeros((cfg.num_steps, n), device=self.device)
        value_buf = torch.zeros((cfg.num_steps, n), device=self.device)
        reward_buf = torch.zeros((cfg.num_steps, n), device=self.device)
        done_buf = torch.zeros((cfg.num_steps, n), device=self.device)
        next_value_buf = torch.zeros((cfg.num_steps, n), device=self.device)

        for step in range(cfg.num_steps):
            obs_norm = self._norm(self.obs)
            obs_buf[step] = obs_norm
            with torch.no_grad():
                dist = self.model.distribution(obs_norm)
                action = dist.sample()
                logp = dist.log_prob(action).sum(-1)
                value = self.model.value(obs_norm)
            action_np = action.cpu().numpy()
            next_obs_np, reward, terminated, truncated, _ = self.env.step(action_np)
            done = np.logical_or(terminated, truncated)
            next_obs = torch.as_tensor(next_obs_np, dtype=torch.float32, device=self.device)
            with torch.no_grad():
                next_value = self.model.value(self._norm(next_obs))
            done_t = torch.as_tensor(done, dtype=torch.float32, device=self.device)
            next_value = next_value * (1.0 - done_t)

            action_buf[step] = action
            logp_buf[step] = logp
            value_buf[step] = value
            reward_buf[step] = (
                torch.as_tensor(reward, dtype=torch.float32, device=self.device)
                * cfg.reward_scale
            )
            done_buf[step] = done_t
            next_value_buf[step] = next_value

            if np.any(done):
                reset_obs = self.env.reset_indices(np.nonzero(done)[0])
                self.obs = torch.as_tensor(reset_obs, dtype=torch.float32, device=self.device)
            else:
                self.obs = next_obs

        with torch.no_grad():
            last_value = self.model.value(self._norm(self.obs))
        return {
            "obs": obs_buf,
            "actions": action_buf,
            "logp": logp_buf,
            "values": value_buf,
            "rewards": reward_buf,
            "dones": done_buf,
            "next_values": next_value_buf,
            "last_value": last_value,
        }

    def _norm(self, obs):
        return obs / self.obs_scale

    def compute_gae(self, rollout):
        cfg = self.config
        rewards = rollout["rewards"]
        values = rollout["values"]
        dones = rollout["dones"]
        next_values = rollout["next_values"]
        advantages = torch.zeros_like(rewards)
        last = torch.zeros_like(values[0])
        for step in reversed(range(cfg.num_steps)):
            delta = rewards[step] + cfg.gamma * next_values[step] * (1.0 - dones[step]) - values[step]
            last = delta + cfg.gamma * cfg.gae_lambda * (1.0 - dones[step]) * last
            advantages[step] = last
        returns = advantages + values
        return advantages, returns

    def update(self, rollout, advantages, returns):
        cfg = self.config
        obs = rollout["obs"].reshape(-1, self.env.observation_dim)
        actions = rollout["actions"].reshape(-1, self.env.action_dim)
        old_logp = rollout["logp"].reshape(-1)
        old_values = rollout["values"].reshape(-1)
        returns = returns.reshape(-1)
        advantages = advantages.reshape(-1)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        batch_size = obs.shape[0]
        indices = np.arange(batch_size)
        stats = {"policy_loss": 0.0, "value_loss": 0.0, "entropy": 0.0, "updates": 0}
        for _ in range(cfg.epochs):
            np.random.shuffle(indices)
            for start in range(0, batch_size, cfg.minibatch):
                batch = indices[start : start + cfg.minibatch]
                batch_obs = obs[batch]
                dist = self.model.distribution(batch_obs)
                new_logp = dist.log_prob(actions[batch]).sum(-1)
                entropy = dist.entropy().sum(-1).mean()
                ratio = torch.exp(new_logp - old_logp[batch])
                surrogate = torch.min(
                    ratio * advantages[batch],
                    torch.clamp(ratio, 1.0 - cfg.clip, 1.0 + cfg.clip) * advantages[batch],
                )
                policy_loss = -surrogate.mean()
                value = self.model.value(batch_obs)
                value_loss = 0.5 * (value - returns[batch]).pow(2).mean()
                loss = policy_loss + cfg.value_coef * value_loss - cfg.entropy_coef * entropy
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.model.parameters(), cfg.max_grad_norm)
                self.optimizer.step()
                stats["policy_loss"] += float(policy_loss.detach())
                stats["value_loss"] += float(value_loss.detach())
                stats["entropy"] += float(entropy.detach())
                stats["updates"] += 1
        for key in ("policy_loss", "value_loss", "entropy"):
            stats[key] /= max(stats["updates"], 1)
        return stats

    def evaluate(self):
        env = self.eval_env
        env.reset()
        done = np.zeros(env.n, dtype=bool)
        terminal = np.zeros(env.n, dtype=bool)
        truncated_mask = np.zeros(env.n, dtype=bool)
        info = None
        for _ in range(env.max_steps):
            with torch.no_grad():
                obs = torch.as_tensor(env._obs(), dtype=torch.float32, device=self.device)
                action = self.model.actor(self._norm(obs)).cpu().numpy()
            _, _, terminated, truncated, info = env.step(action)
            terminal |= terminated
            truncated_mask |= truncated
            done |= terminated | truncated
            if np.all(done):
                break
        if info is None:
            info = env._info()
        truncated_mask |= ~done
        required = (
            len(env.track.scoring_gates)
            if env.track.audited_stage1
            else 2
        )
        return summarize_episodes(
            info,
            required_gates=required,
            terminated=terminal,
            truncated=truncated_mask,
            track_length_m=env.track.total_length_m,
        )

    def train(self):
        cfg = self.config
        started = time.perf_counter()
        best_success = -1.0
        good_evals = 0
        update_index = 0
        while self.total_steps < cfg.total_steps:
            rollout = self.collect()
            advantages, returns = self.compute_gae(rollout)
            stats = self.update(rollout, advantages, returns)
            self.total_steps += cfg.num_steps * cfg.num_envs
            update_index += 1
            if update_index % cfg.log_every == 0:
                record = {
                    "steps": self.total_steps,
                    "wall_s": time.perf_counter() - started,
                    **stats,
                }
                with self.metrics_path.open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps(record) + "\n")
                print(json.dumps(record), flush=True)
            if cfg.eval_every and update_index % cfg.eval_every == 0:
                eval_stats = self.evaluate()
                success = float(eval_stats["success_rate"])
                record = {"steps": self.total_steps, "eval_success": success, **eval_stats}
                with self.metrics_path.open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps(record) + "\n")
                print(json.dumps(record), flush=True)
                if success > best_success:
                    best_success = success
                    self.save("best.pt")
                if success >= cfg.target_success:
                    good_evals += 1
                    if good_evals >= cfg.patience:
                        print(json.dumps({"early_stop": True, "success": success}), flush=True)
                        break
                else:
                    good_evals = 0
                self.save()
            if cfg.max_wall_s and time.perf_counter() - started > cfg.max_wall_s:
                print(json.dumps({"wall_limit": True, "steps": self.total_steps}), flush=True)
                break
        self.save()
        return self.total_steps

    def save(self, filename: str = "model.pt"):
        path = self.save_dir / filename
        track = track_metadata(self.env)
        torch.save(
            {
                "model": self.model.state_dict(),
                "config": asdict(self.config),
                "obs_scale": self.obs_scale.cpu().tolist(),
                "total_steps": self.total_steps,
                "track": track,
            },
            path,
        )
        (self.save_dir / "config.json").write_text(
            json.dumps(asdict(self.config), indent=2) + "\n", encoding="utf-8"
        )
        (self.save_dir / "track.json").write_text(
            json.dumps(track, indent=2) + "\n", encoding="utf-8"
        )
        return path

def _observation_scale(obs_dim: int):
    scale = [1.0] * obs_dim
    scale[0:3] = [100.0, 100.0, 100.0]
    scale[3:6] = [10.0, 10.0, 10.0]
    scale[10:13] = [10.0, 10.0, 10.0]
    scale[13:16] = [10.0, 10.0, 10.0]
    scale[19:22] = [10.0, 10.0, 10.0]
    scale[23] = 180.0
    scale[30] = 2.0
    return scale
