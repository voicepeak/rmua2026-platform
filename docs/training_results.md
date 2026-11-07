# Training results

> **2026-11-06 correction:** the legacy `1_1 + 2_1` / two-gates-per-segment
> checkpoints under `runs/route_seg{1,2,3}/` and HF `route_spline_20261106`
> remain **invalid for official stage 1**; keep them only as pipeline smoke
> tests. The new local policy below uses the audited 63-component development
> environment, but it is still a **local development result**, not an official
> competition score.

## OL1 stage 1 audited development policy (2026-11-06)

Official simulator diagnostic on random seed 283441 did not leave the start
zone with the experimental truth-based adapter (internal Score 0). The adapter
and plant differ materially from training, so this is not an isolated policy
evaluation or a valid competition score. See `official_policy_smoke_283441.md`.

Date: 2026-11-06. Machine: local CPU-only (20 logical cores).

Track: `SimFastConfig(track_mode="stage1_audited", map_name="RMUA2024OL1",
segment_id=1)`. Path length 1474.431 m: spline 1 -> approximate hub connector
-> reversed spline 3 -> measured `end_goal`. Scoring components are the audited
63 centers in the observed order (road 1 indices 32->0, road 3 indices 0->29).
Detection aperture is the documented approximate 10 m x 5 m swept plane; start
and end trigger boxes are separate. Proxy line/vehicle obstacles were disabled.

Policy: ActorCritic MLP 33 -> 128 -> 128 -> 4, world-frame velocity action,
kinematic plant, 100 Hz. Training used **simulated state truth** observations
(`observation_source=sim_truth_state_only`); a deployed actor must use
sensor/EKF estimates, never `debug/pose_gt`.

Training: BC + DAgger from scratch, no PPO fine-tune. Seed 7.

- Demonstrations: 64 envs x 30000 steps = 1,920,000 samples; 256 recorded
  episodes, 100% success, 21.90 m/s mean completion speed.
- BC: 12 epochs, batch 8192.
- DAgger: 4 rounds x 15000 steps, policy-driven states
  (`dagger_expert_prob=0`), 8 BC epochs per round, capped at 4,000,000 samples.
- Training episode log: 768 episodes, 640 success, 128 collisions. The 128
  collisions are the initial BC policy's failed rollouts during the first
  DAgger collection; the training eval was 100% from DAgger round 1 onward.

Command:

    .\.venv\Scripts\python.exe scripts\train_bc_ppo.py --track-mode stage1_audited --map RMUA2024OL1 --segment 1 --demo-envs 64 --demo-steps 30000 --bc-epochs 12 --dagger-rounds 4 --dagger-steps 15000 --dagger-expert-prob 0.0 --dagger-bc-epochs 8 --hidden 128 --seed 7 --skip-ppo --save-dir runs\stage1_audited_seed7_20261106

Training-eval trajectory (16 envs, seed 207): initial BC 0% success, 62/63 mean
gates, all late-route collisions; DAgger round 1 onward 100% success, 63/63,
about 22.0 m/s.

### Held-out acceptance

Seeds were not used in training: 1201, 1302, 1403, 1504, 1605; 64 envs per seed
(320 episodes), deterministic policy, 180 s limit.

| Metric | Value |
|---|---|
| Success rate | 100.0% (320/320) |
| 63-gate completion rate | 100.0% (320/320) |
| Mean completion time | 67.061 s |
| Mean completion speed | 21.986 m/s |
| Mean progress speed, all episodes | 21.938 m/s |
| Failure counts | success: 320 |

The ≥15 m/s local target was met (21.99 m/s), but this is a local sim-fast
result only. It is not an official simulator or competition result.

Command:

    .\.venv\Scripts\python.exe scripts\eval_policy.py --checkpoint runs\stage1_audited_seed7_20261106\best.pt --seeds 1201,1302,1403,1504,1605 --envs 64 --output runs\stage1_audited_seed7_20261106\eval_heldout.json --episodes-jsonl runs\stage1_audited_seed7_20261106\eval_heldout_episodes.jsonl

Artefacts: `runs/stage1_audited_seed7_20261106/` (`best.pt`, `bc_model.pt`,
`config.json`, `track.json`, `train_summary.json`, `train_episodes.jsonl`,
`eval_heldout.json`, `eval_heldout_episodes.jsonl`). Weights and metadata are
backed up at:

    https://huggingface.co/Vociepeak/rmua2026-platform-policies/tree/main/policies/stage1_audited_seed7_20261106

### Scope and limitations

- State-only: no camera/LiDAR/perception, wind, factory, or dynamic obstacles.
- Kinematic plant; deployment needs EKF/body-frame conversion and the official
  first-signal lock.
- Gate order is from an OL1 seed-123 runtime audit; hub connector, gate
  aperture, coordinate alignment and collision remain approximations.
- Official stage-2 transition and a complete valid race have **not** been
  verified. Do not claim official equivalence.
- The checkpoint is valid only for `stage1_audited` / `RMUA2024OL1` / segment 1;
  loading rejects track metadata mismatches. It is not valid for the legacy
  proxy track or for stage 2/3.
- The training pipeline passed 41 tests, including audited-mode wiring,
  checkpoint identity checks, terminal-state bookkeeping and expert/episode-log
  checks.

## Legacy proxy results (invalid for official stage 1)

The results below use a **proxy** route pair (`1_1 + 2_1`, etc.) and only two
gates per segment. The official stage logic is a shuttle
(`stage 1 = route 1 + reverse(route 3)`, then route 3 return -> route 5, etc.).
OL1 seed 123 runtime inspection found 63 ordered scoring components; valid
stage completion remains unverified. **These checkpoints are invalid for
official stage 1**; keep them only as pipeline smoke tests. See
`docs/official_stage_logic.md`.

Date: 2026-11-06. Machine: local CPU-only (20 logical cores, 32 GB RAM).

Policy: ActorCritic MLP 33 -> 128 -> 128 -> 4, world-frame velocity.
Training: scripted path-following demos + 6 DAgger rounds + behavior cloning.
Track: Catmull-Rom route-pair proxy (outbound RouteConfig + hub connector +
reversed return RouteConfig), 1 m resampling, 10 m wide / 5 m high corridor.

### Results

Held-out evaluation, deterministic policy mean, 5 seeds x 64 envs:

| Segment | Path | Routes | Success | Mean time | Mean speed |
|---|---|---|---|---|---|
| 1-3 | 1469.8 m | 1_1 + 2_1 | 100% | 71.38 s | 20.59 m/s |
| 3-5 | 1411.9 m | 3_1 + 4_1 | 100% | 67.57 s | 20.90 m/s |
| 5-8 | 1295.0 m | 5_1 + 6_1 | 100% | 63.02 s | 20.55 m/s |

Seeds: 101, 202, 303, 404, 505. Time limit: 180 s per segment.

The previous 142 m proxy checkpoints are superseded and should not be used for
speed or route claims.

### Reproduce (legacy proxy only)

    python scripts/eval_policy.py --checkpoint runs/route_seg1/best.pt --allow-legacy-proxy

Retrain route segments 1-3 (legacy proxy):

    python scripts/train_curriculum_1_3.py

(BC/DAgger-only route run: `scripts/train_route_2_3.py`, or add `--skip-ppo`.)

### Weights backup (legacy proxy)

    https://huggingface.co/Vociepeak/rmua2026-platform-policies/tree/main/policies/route_spline_20261106

The bundle contains `best.pt`, `config.json`, `eval_5seeds.json`, `train.log`
and a manifest with route mapping and limitations.

### Scope and limitations (legacy proxy)

- State-only: no line/vehicle obstacles, wind, factory, perception or camera/LiDAR.
- Kinematic plant, world-frame VEL action; deployment needs EKF/body-frame
  conversion and the official first-signal lock.
- Route/gate pairing (1_1+2_1, 3_1+4_1, 5_1+6_1) is an **approx hypothesis**
  from the OL1 route order; runtime calibration is required.
- Gate trigger is a 10x5x7 m box proxy; official trigger volume/aperture not decoded.
- No official simulator equivalence claim.
