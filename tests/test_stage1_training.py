import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pytest
import torch

from rmua.envs.sim_fast import SimFastConfig, SimFastVecEnv
from rmua.eval.metrics import summarize_episodes
from rmua.training.checkpoint import track_metadata, validate_track_metadata
from rmua.training.ppo import PPOConfig, PPOTrainer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from eval_policy import _resolve_track, evaluate  # noqa: E402
from train_bc_ppo import (  # noqa: E402
    EpisodeRecorder,
    build_parser,
    collect_demos,
    make_env_config,
)


def test_scripted_expert_records_full_audited_episodes(tmp_path):
    env = SimFastVecEnv(
        SimFastConfig(
            num_envs=2,
            track_mode="stage1_audited",
            action_frame="world",
            line_max=0,
            vehicle_max=0,
            seed=71,
        )
    )
    assert len(env.track.scoring_gates) == 63
    recorder = EpisodeRecorder(
        tmp_path / "episodes.jsonl",
        len(env.track.scoring_gates),
        env.track.total_length_m,
    )
    collect_demos(env, 7000, recorder, noise_std=0.0, log_every=10**9)
    summary = recorder.summary()
    assert summary["episodes"] >= 2
    assert summary["success_rate"] == 1.0
    assert summary["full_gate_rate"] == 1.0
    assert 60.0 <= summary["mean_time_s"] <= 80.0
    assert 15.0 <= summary["mean_speed_mps"] <= 25.0
    assert summary["failure_counts"] == {"success": summary["episodes"]}
    rows = [
        json.loads(line)
        for line in (tmp_path / "episodes.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert len(rows) == summary["episodes"]
    assert all(row["gates_passed"] == 63 for row in rows)
    assert all(row["success"] for row in rows)


def test_training_cli_defaults_to_audited_environment():
    args = build_parser().parse_args([])
    assert args.track_mode == "stage1_audited"
    env = SimFastVecEnv(make_env_config(args, 1, 3))
    assert env.track.audited_stage1
    assert len(env.track.scoring_gates) == 63
    assert env.config.track_mode == "stage1_audited"


def test_ppo_checkpoint_records_and_validates_track(tmp_path):
    cfg = PPOConfig(
        total_steps=8,
        num_envs=1,
        num_steps=4,
        segment_id=1,
        map_name="RMUA2024OL1",
        track_mode="stage1_audited",
        seed=5,
        eval_envs=1,
        eval_every=0,
        save_dir=str(tmp_path),
    )
    trainer = PPOTrainer(cfg)
    current = track_metadata(trainer.env)
    assert current["track_mode"] == "stage1_audited"
    assert current["scoring_gate_count"] == 63
    assert current["observation_source"] == "sim_truth_state_only"
    path = trainer.save("best.pt")
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    assert checkpoint["track"] == current
    validate_track_metadata(checkpoint["track"], current)

    wrong_mode = dict(checkpoint["track"], track_mode="proxy")
    with pytest.raises(ValueError):
        validate_track_metadata(wrong_mode, current)
    with pytest.raises(ValueError):
        validate_track_metadata(None, current)
    wrong_gates = dict(checkpoint["track"], scoring_gate_count=2)
    with pytest.raises(ValueError):
        validate_track_metadata(wrong_gates, current)


def test_eval_resolution_refuses_legacy_or_conflicting_track(tmp_path):
    cfg = PPOConfig(track_mode="stage1_audited")
    saved = {
        "track_mode": "stage1_audited",
        "map_name": "RMUA2024OL1",
        "segment_id": 1,
        "detection_gate_width_m": 10.0,
        "detection_gate_height_m": 5.0,
    }
    args = argparse.Namespace(
        checkpoint=tmp_path / "best.pt",
        track_mode=None,
        map=None,
        segment=None,
        detection_gate_width_m=None,
        detection_gate_height_m=None,
        allow_legacy_proxy=False,
    )
    mode, map_name, segment, width, height = _resolve_track(args, cfg, saved)
    assert (mode, map_name, segment) == ("stage1_audited", "RMUA2024OL1", 1)
    assert (width, height) == (10.0, 5.0)

    args.track_mode = "proxy"
    with pytest.raises(ValueError):
        _resolve_track(args, cfg, saved)

    args.track_mode = None
    with pytest.raises(ValueError):
        _resolve_track(args, cfg, None)

    args.allow_legacy_proxy = True
    with pytest.raises(ValueError):
        _resolve_track(args, cfg, None)


def test_eval_allow_legacy_proxy_runs_without_track_metadata(tmp_path):
    cfg = PPOConfig(
        total_steps=1,
        num_envs=1,
        num_steps=1,
        segment_id=1,
        track_mode="proxy",
        seed=5,
        eval_envs=1,
        eval_every=0,
        save_dir=str(tmp_path),
    )
    trainer = PPOTrainer(cfg)
    checkpoint = torch.load(
        trainer.save("legacy.pt"), map_location="cpu", weights_only=False
    )
    checkpoint.pop("track")
    legacy = tmp_path / "legacy.pt"
    torch.save(checkpoint, legacy)
    args = argparse.Namespace(
        checkpoint=legacy,
        track_mode=None,
        map=None,
        segment=None,
        detection_gate_width_m=None,
        detection_gate_height_m=None,
        allow_legacy_proxy=True,
        envs=1,
        seeds=[1],
        max_steps=1,
    )
    result = evaluate(legacy, args)
    assert result["track_mode"] == "proxy"
    assert result["observation_source"] == "unknown"
    assert len(result["episodes"]) == 1
    assert result["episodes"][0]["failure_reason"] == "truncated"

    args.allow_legacy_proxy = False
    with pytest.raises(ValueError):
        evaluate(legacy, args)


def test_summarize_episodes_reports_failures_and_all_speeds():
    info = {
        "success": np.array([True, False, False]),
        "gates_passed": np.array([63, 40, 7]),
        "segment_time": np.array([70.0, 100.0, 20.0]),
        "progress_m": np.array([1474.0, 900.0, 100.0]),
        "failure_reason": np.array(["success", "collision", ""]),
    }
    stats = summarize_episodes(
        info,
        required_gates=63,
        terminated=np.array([True, True, False]),
        truncated=np.array([False, False, True]),
        track_length_m=1474.0,
    )
    assert stats["success_rate"] == pytest.approx(1.0 / 3.0)
    assert stats["full_gate_rate"] == pytest.approx(1.0 / 3.0)
    assert stats["mean_time_s"] == pytest.approx(70.0)
    assert stats["mean_time_all_s"] == pytest.approx(190.0 / 3.0)
    assert stats["mean_speed_mps"] == pytest.approx(1474.0 / 70.0)
    assert stats["mean_progress_speed_mps"] == pytest.approx(
        (1474.0 / 70.0 + 9.0 + 5.0) / 3.0
    )
    assert stats["failure_counts"] == {
        "success": 1,
        "collision": 1,
        "truncated": 1,
    }
