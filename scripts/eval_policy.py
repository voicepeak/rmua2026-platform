"""Evaluate a PPO/BC checkpoint on sim-fast with deterministic actions.

The checkpoint's track metadata selects the map and track mode. A checkpoint
without metadata is a legacy proxy checkpoint and is rejected unless
``--allow-legacy-proxy`` is given. CLI overrides must match the checkpoint.

    python scripts/eval_policy.py --checkpoint runs/stage1_audited_bc/best.pt
    python scripts/eval_policy.py --checkpoint runs/stage1_audited_bc/best.pt \
        --seeds 611,722,833,944,1055 --output eval_heldout.json
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
from rmua.eval.metrics import summarize_episodes  # noqa: E402
from rmua.training.checkpoint import (  # noqa: E402
    missing_track_metadata_error,
    track_metadata,
    validate_track_metadata,
)
from rmua.training.ppo import ActorCritic, PPOConfig  # noqa: E402


def _resolve_track(args, cfg: PPOConfig, saved: dict | None):
    if saved is None:
        if not args.allow_legacy_proxy:
            raise missing_track_metadata_error(str(args.checkpoint))
        mode = args.track_mode or cfg.track_mode
        map_name = args.map or cfg.map_name
        if mode != "proxy":
            raise ValueError(
                "legacy checkpoints have no track metadata and cannot be evaluated "
                "on stage1_audited"
            )
        segment = args.segment if args.segment is not None else cfg.segment_id
        width = (
            args.detection_gate_width_m
            if args.detection_gate_width_m is not None
            else cfg.detection_gate_width_m
        )
        height = (
            args.detection_gate_height_m
            if args.detection_gate_height_m is not None
            else cfg.detection_gate_height_m
        )
        return mode, map_name, segment, width, height

    saved_mode = saved.get("track_mode")
    saved_map = saved.get("map_name")
    if cfg.track_mode != saved_mode:
        raise ValueError(
            f"checkpoint config track_mode={cfg.track_mode!r} disagrees with "
            f"checkpoint track metadata track_mode={saved_mode!r}"
        )
    if args.track_mode is not None and args.track_mode != saved_mode:
        raise ValueError(
            f"--track-mode={args.track_mode!r} conflicts with checkpoint "
            f"track_mode={saved_mode!r}"
        )
    if args.map is not None and args.map != saved_map:
        raise ValueError(
            f"--map={args.map!r} conflicts with checkpoint map={saved_map!r}"
        )
    segment = args.segment if args.segment is not None else cfg.segment_id
    if segment != saved.get("segment_id"):
        raise ValueError(
            f"--segment={segment!r} conflicts with checkpoint segment="
            f"{saved.get('segment_id')!r}"
        )
    width = (
        args.detection_gate_width_m
        if args.detection_gate_width_m is not None
        else saved["detection_gate_width_m"]
    )
    height = (
        args.detection_gate_height_m
        if args.detection_gate_height_m is not None
        else saved["detection_gate_height_m"]
    )
    return saved_mode, saved_map, segment, width, height


def evaluate(checkpoint: Path, args) -> dict:
    ckpt = torch.load(checkpoint, map_location="cpu", weights_only=False)
    if "config" not in ckpt:
        raise ValueError(
            f"{checkpoint} has no training config; use best.pt/model.pt from a "
            "training run"
        )
    cfg = PPOConfig(**ckpt["config"])
    saved_track = ckpt.get("track")
    track_mode, map_name, segment_id, width, height = _resolve_track(
        args, cfg, saved_track
    )
    obs_dim = len(ckpt["obs_scale"])
    model = ActorCritic(obs_dim, 4, cfg.hidden)
    model.load_state_dict(ckpt["model"])
    model.eval()
    scale = torch.tensor(ckpt["obs_scale"], dtype=torch.float32)

    seeds = list(args.seeds)
    seed_records = []
    rows = []
    infos = []
    terminals = []
    truncateds = []
    required_total = None
    track_length = None
    for seed in seeds:
        env = SimFastVecEnv(
            SimFastConfig(
                num_envs=args.envs,
                segment_id=segment_id,
                map_name=map_name,
                track_mode=track_mode,
                detection_gate_width_m=width,
                detection_gate_height_m=height,
                stage1_enable_obstacles=cfg.stage1_enable_obstacles,
                dynamics=cfg.dynamics,
                action_mode="vel",
                action_frame=cfg.action_frame,
                line_max=cfg.line_max,
                vehicle_max=cfg.vehicle_max,
                seed=seed,
            )
        )
        current = track_metadata(env)
        if saved_track is not None:
            validate_track_metadata(
                saved_track, current, checkpoint=str(checkpoint)
            )
        required = len(env.track.scoring_gates) if env.track.audited_stage1 else 2
        required_total = required
        track_length = env.track.total_length_m
        env.reset(seed=seed)
        done = np.zeros(env.n, dtype=bool)
        terminal = np.zeros(env.n, dtype=bool)
        truncated_mask = np.zeros(env.n, dtype=bool)
        info = None
        limit = args.max_steps if args.max_steps is not None else env.max_steps
        for _ in range(limit):
            obs = torch.as_tensor(env._obs(), dtype=torch.float32) / scale
            with torch.no_grad():
                action = model.actor(obs).numpy()
            _, _, terminated, truncated, info = env.step(action)
            terminal |= terminated
            truncated_mask |= truncated
            done |= terminated | truncated
            if np.all(done):
                break
        if info is None:
            info = env._info()
        truncated_mask |= ~done
        infos.append(info)
        terminals.append(terminal)
        truncateds.append(truncated_mask)
        stats = summarize_episodes(
            info,
            required_gates=required,
            terminated=terminal,
            truncated=truncated_mask,
            track_length_m=env.track.total_length_m,
        )
        stats["seed"] = seed
        seed_records.append(stats)
        for index in range(env.n):
            success = bool(info["success"][index])
            reason = str(info["failure_reason"][index])
            if not reason:
                reason = (
                    "success"
                    if success
                    else (
                        "truncated"
                        if bool(truncated_mask[index])
                        else ("terminated" if bool(terminal[index]) else "running")
                    )
                )
            rows.append(
                {
                    "seed": seed,
                    "env": index,
                    "success": success,
                    "gates_passed": int(info["gates_passed"][index]),
                    "gates_required": required,
                    "segment_time_s": float(info["segment_time"][index]),
                    "progress_m": float(info["progress_m"][index]),
                    "failure_reason": reason,
                }
            )

    merged_info = {
        "success": np.concatenate([np.asarray(i["success"], dtype=bool) for i in infos]),
        "gates_passed": np.concatenate(
            [np.asarray(i["gates_passed"], dtype=np.float64) for i in infos]
        ),
        "segment_time": np.concatenate(
            [np.asarray(i["segment_time"], dtype=np.float64) for i in infos]
        ),
        "progress_m": np.concatenate(
            [np.asarray(i["progress_m"], dtype=np.float64) for i in infos]
        ),
        "failure_reason": [
            reason for info in infos for reason in info["failure_reason"].tolist()
        ],
    }
    overall = summarize_episodes(
        merged_info,
        required_gates=required_total,
        terminated=np.concatenate(terminals),
        truncated=np.concatenate(truncateds),
        track_length_m=track_length,
    )
    return {
        "checkpoint": str(checkpoint),
        "map": map_name,
        "track_mode": track_mode,
        "segment": segment_id,
        "track_length_m": round(float(track_length), 3),
        "required_gates": required_total,
        "observation_source": (
            saved_track.get("observation_source") if saved_track else "unknown"
        ),
        "seeds": seeds,
        "envs": args.envs,
        "per_seed": seed_records,
        "overall": overall,
        "success_rate": overall["success_rate"],
        "full_gate_rate": overall["full_gate_rate"],
        "mean_time_s": overall["mean_time_s"],
        "mean_speed_mps": overall["mean_speed_mps"],
        "mean_time_all_s": overall["mean_time_all_s"],
        "mean_progress_speed_mps": overall["mean_progress_speed_mps"],
        "failure_counts": overall["failure_counts"],
        "episodes": rows,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--segment", type=int, default=None)
    parser.add_argument("--map", default=None)
    parser.add_argument(
        "--track-mode", choices=("stage1_audited", "proxy"), default=None
    )
    parser.add_argument("--detection-gate-width-m", type=float, default=None)
    parser.add_argument("--detection-gate-height-m", type=float, default=None)
    parser.add_argument("--allow-legacy-proxy", action="store_true")
    parser.add_argument("--envs", type=int, default=64)
    parser.add_argument("--seeds", default="611,722,833,944,1055")
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--episodes-jsonl", type=Path, default=None)
    args = parser.parse_args(argv)
    args.seeds = [int(value) for value in args.seeds.split(",") if value.strip()]
    if not args.seeds:
        parser.error("--seeds must not be empty")
    result = evaluate(args.checkpoint, args)
    episodes = result.pop("episodes")
    if args.episodes_jsonl is not None:
        args.episodes_jsonl.parent.mkdir(parents=True, exist_ok=True)
        with args.episodes_jsonl.open("w", encoding="utf-8") as handle:
            for row in episodes:
                handle.write(json.dumps(row) + "\n")
    text = json.dumps(result, indent=2)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    if result["observation_source"] == "sim_truth_state_only":
        print(
            "WARNING: state-only policy trained on simulated truth observations; "
            "deployment requires sensor/EKF estimates, never debug/pose_gt.",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
