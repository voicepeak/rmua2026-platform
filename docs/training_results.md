# Training results: state-only 1-3, 3-5, 5-8

Date: 2026-11-06. Machine: local CPU-only (20 logical cores, 32 GB RAM).
Policy: ActorCritic MLP 33 -> 128 -> 128 -> 4, world-frame velocity.
Training: scripted demos + 3 DAgger rounds (BC) + PPO fine-tune.

## Results

Held-out evaluation, deterministic policy mean, 5 seeds x 64 envs:

| Segment | Checkpoint | Best step | Success |
|---|---|---|---|
| 1-3 | `runs/curriculum/seg1/best.pt` | 1,310,720 | 100% |
| 3-5 | `runs/curriculum/seg2/best.pt` | 1,310,720 | 100% |
| 5-8 | `runs/curriculum/seg3/best.pt` | 2,621,440 | 100% |

Seeds: 101, 202, 303, 404, 505.

PPO later oscillated on 1-3 (final eval 0%, best 100% at 1.31M steps); always use
`best.pt`, not the final `model.pt`.

## Reproduce

    python scripts/eval_policy.py --checkpoint runs/curriculum/seg1/best.pt
    python scripts/eval_policy.py --checkpoint runs/curriculum/seg2/best.pt
    python scripts/eval_policy.py --checkpoint runs/curriculum/seg3/best.pt

Retrain all three:

    python scripts/train_curriculum_1_3.py

## Weights backup

    https://huggingface.co/Vociepeak/rmua2026-platform-policies/tree/main/policies/curriculum_20261106

The bundle contains `best.pt`, `config.json`, `metrics.jsonl`, `train.log` and a
manifest with seeds/limitations.

## Scope and limitations

- State-only: no line/vehicle obstacles, wind, factory, perception or camera/LiDAR.
- Kinematic plant, world-frame VEL action; deployment still needs EKF/body-frame
  conversion and the official first-signal lock.
- Gate trigger is a node-1 10x5x3 m box proxy; the official trigger volume and
  gate aperture are not decoded.
- No official simulator equivalence claim. This validates the training pipeline,
  not the competition result.
