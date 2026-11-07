# OL1 stage 1 development environment

Select `SimFastConfig(track_mode="stage1_audited", segment_id=1)` explicitly.
The default `proxy` mode is retained only for old pipeline comparisons and is
invalid for official stage 1. The audited mode rejects other maps and stages.

The path evaluates the OL1 map's Hermite splines 1 and 3, reverses spline 3,
and connects them with a tangent-following cubic through the hub. That
connector is a **development approximation**, not a verified official road.
The path begins at the NED origin (internal z up) and ends at the measured
`end_goal` converted to internal z up. It is resampled approximately every
metre. World/UE-to-NED alignment beyond the recorded OL1 checks remains to be
validated against runtime poses, particularly at the hub.

The 63 captured component centers are used in the observed order: road 1
indices 32 to 0 followed by road 3 indices 0 to 29. Component root transforms
come from `data/audits/ol1_seed123/`; source and runtime limits are described
in `official_stage_logic.md`. Start and end zones are separate from scoring
gates. Crossing a later detection gate before the expected gate does not
score. Scoring is permitted before departure from the start zone; completion
requires all 63 gates and entry into the end zone.

Detection uses a swept plane with configurable approximate width and height
(`detection_gate_width_m`, `detection_gate_height_m`, defaults 10 m and 5 m).
This does not reproduce the as-yet undecoded UE trigger volume. Start/end
zones use approximate 10 × 5 × 7 m boxes. Neither gate passage nor successful
completion establishes official equivalence. Dynamics, force field, collision
and sensor models remain local approximations. Current observations expose
simulated state; a deployed actor must use sensor/EKF estimates, not pose_gt.
Proxy line and vehicle obstacles are disabled by default in this mode; they
can be enabled explicitly with `stage1_enable_obstacles=True` for later work.

Example (environment smoke test; no training):

```python
import numpy as np
from rmua.envs.sim_fast import SimFastConfig, SimFastVecEnv

env = SimFastVecEnv(SimFastConfig(
    segment_id=1, map_name="RMUA2024OL1", track_mode="stage1_audited",
    num_envs=64, line_max=0, vehicle_max=0,
))
obs = env.reset(seed=123)
obs, reward, terminated, truncated, info = env.step(np.zeros((64, 4)))
```

## Training and evaluation integration (2026-11-06)

`scripts/train_bc_ppo.py`, `PPOTrainer`/`PPOConfig` and
`scripts/eval_policy.py` now carry `map_name`, `track_mode`, gate dimensions and
stage-1 obstacle flags through every environment. The training CLI defaults to
`stage1_audited`; the legacy proxy must be selected explicitly. Checkpoints
store a `track` identity block (map, mode, segment, path length, scoring-gate
count, aperture, observation source). Evaluation selects the mode from the
checkpoint and rejects missing legacy metadata or any mismatch, so proxy and
audited tracks cannot be mixed silently. The PPO training-time evaluation
horizon now covers the full episode instead of truncating after 20 s.

Training episodes are recorded one JSON line per episode with success, gates
passed/required, segment time, path progress and failure reason. Evaluation
reports success rate, 63-gate completion rate, mean completion time and speed,
mean progress speed over all episodes, and failure counts. This avoids hiding
failures by reporting only successful samples.

Commands for the local development policy:

    .\.venv\Scripts\python.exe scripts\train_bc_ppo.py --track-mode stage1_audited --map RMUA2024OL1 --segment 1 --demo-envs 64 --demo-steps 30000 --bc-epochs 12 --dagger-rounds 4 --dagger-steps 15000 --dagger-expert-prob 0.0 --dagger-bc-epochs 8 --hidden 128 --seed 7 --skip-ppo --save-dir runs\stage1_audited_seed7_20261106

    .\.venv\Scripts\python.exe scripts\eval_policy.py --checkpoint runs\stage1_audited_seed7_20261106\best.pt --seeds 1201,1302,1403,1504,1605 --envs 64 --output runs\stage1_audited_seed7_20261106\eval_heldout.json --episodes-jsonl runs\stage1_audited_seed7_20261106\eval_heldout_episodes.jsonl

The resulting BC+DAgger policy completed the local audited environment on all
320 held-out episodes (5 seeds x 64 envs): 100% success, 100% at 63/63 gates,
67.061 s mean time, 21.986 m/s mean speed. These are local development results
only. The policy uses simulated state truth; deployment requires sensor/EKF
estimates, never `debug/pose_gt`. Official stage-2 transition and a valid
complete race remain unverified. See `training_results.md` and the HF bundle
`policies/stage1_audited_seed7_20261106`.

Initial smoke test: a simple 8 m/s scripted path follower reached about 1438 m
and scored 61 gates by the 18001-step truncation; it did not finish. The two
remaining component centers project around 1447 m and 1469 m. This verifies
continuous stepping only, not task success or a trained-policy result. A
zero-action, 100-step run measured about 45.6k and 122.9k env-steps/s at 64
and 256 environments respectively on this host; these figures are specific to
this configuration and should not be compared to official simulator speed.
