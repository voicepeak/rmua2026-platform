# Training results: route-spline 1-3, 3-5, 5-8

> **2026-11-06 correction:** These results use a **proxy** route pair
> (`1_1 + 2_1`, etc.) and only two gates per segment. The official stage logic is
> a shuttle (`stage 1 = route 1 + reverse(route 3)`, then route 3 return ->
> route 5, etc.). The official 1→3 route is `route_1_3.yaml`
> (`spline0 + reverse(spline2)`), and the official gate sequence is still under
> audit. **The checkpoints below are invalid for official stage 1**; keep them
> only as pipeline smoke tests. See `docs/official_stage_logic.md`.

Date: 2026-11-06. Machine: local CPU-only (20 logical cores, 32 GB RAM).

Policy: ActorCritic MLP 33 -> 128 -> 128 -> 4, world-frame velocity.
Training: scripted path-following demos + 6 DAgger rounds + behavior cloning.
Track: Catmull-Rom route-pair proxy (outbound RouteConfig + hub connector +
reversed return RouteConfig), 1 m resampling, 10 m wide / 5 m high corridor.

## Results

Held-out evaluation, deterministic policy mean, 5 seeds x 64 envs:

| Segment | Path | Routes | Success | Mean time | Mean speed |
|---|---|---|---|---|---|
| 1-3 | 1469.8 m | 1_1 + 2_1 | 100% | 71.38 s | 20.59 m/s |
| 3-5 | 1411.9 m | 3_1 + 4_1 | 100% | 67.57 s | 20.90 m/s |
| 5-8 | 1295.0 m | 5_1 + 6_1 | 100% | 63.02 s | 20.55 m/s |

Seeds: 101, 202, 303, 404, 505. Time limit: 180 s per segment.

The previous 142 m proxy checkpoints are superseded and should not be used for
speed or route claims.

## Reproduce

    python scripts/eval_policy.py --checkpoint runs/route_seg1/best.pt
    python scripts/eval_policy.py --checkpoint runs/route_seg2/best.pt
    python scripts/eval_policy.py --checkpoint runs/route_seg3/best.pt

Retrain route segments 1-3:

    python scripts/train_curriculum_1_3.py

(BC/DAgger-only route run: `scripts/train_route_2_3.py`, or add `--skip-ppo`.)

## Weights backup

    https://huggingface.co/Vociepeak/rmua2026-platform-policies/tree/main/policies/route_spline_20261106

The bundle contains `best.pt`, `config.json`, `eval_5seeds.json`, `train.log`
and a manifest with route mapping and limitations.

## Scope and limitations

- State-only: no line/vehicle obstacles, wind, factory, perception or camera/LiDAR.
- Kinematic plant, world-frame VEL action; deployment needs EKF/body-frame
  conversion and the official first-signal lock.
- Route/gate pairing (1_1+2_1, 3_1+4_1, 5_1+6_1) is an **approx hypothesis**
  from the OL1 route order; runtime calibration is required.
- Gate trigger is a 10x5x7 m box proxy; official trigger volume/aperture not decoded.
- No official simulator equivalence claim.
