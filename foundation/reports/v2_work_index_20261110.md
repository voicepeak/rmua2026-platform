# V2.0 2026-11-10 work index

## Round result

| item | result | evidence |
|---|---|---|
| OL1 stage1 road1 | 33/33 score, indices 0-32 | `g2_score_route_seed123_20261109.json` |
| OL1 road3 components | 30/30 SEG triggers observed (stage1 queue inactive, score 0) | `g2_score_road3_teleport_seed123_20261110.json` |
| OL1 stage2 road3 score | 30/30, indices 0-29, controlled stage2 injection | `g2_stage2_routexy_gate11_0_corridor_v12_seed123_20261110.json` |
| OL1 stage3 | 31 declared components, `nan` roots, score 0 | `g2_stage3_return_seed123_20261110.json` |
| OL1 stage4 | frozen after component teleport (even 50 m); takeoff type loads but calls hang | `g2_stage4_return_20261110.md` |
| OL2 | map redirect + teleport observed; no SEG/DUMP, no flyable state | `g2_ol2_attempt_20261110.md` |
| OL3 | map redirect + teleport observed; no SEG/DUMP, stuck at origin | `g2_ol3_attempt_20261110.md` |
| OL4 | map redirect + teleport observed; no SEG/DUMP | `g2_ol4_attempt_20261110.md` |
| OL3-OL12 | prepared map probes; not run | `v2_map_audit_plan_20261110.md` |
| factory | 12 static Extent volumes; entry trigger unverified | `maps/OL1/factory_entrance_candidates.json` |
| G1 | two bounded non-stopping attempts; BLOCKED | `v2_g1_native_attempt_20261110.md` |
| G3/G4 | natural wind / official throughput explicitly unavailable | `v2_g3_g4_20261110.md` |
| training env | `RmuaPhysicsVecEnv` approximate sandbox | `training_env.py`, `v2_training_env_20261110.md` |

Tests: `python -m pytest -q foundation/tests` -> 166 passed.  Smoke: PASS (synthetic integration only).
No commit or push was made.


| file | bytes | sha256 |
|---|---:|---|
| `foundation/reports/v2_coverage_matrix_20261110.md` | 3148 | `3c88bb7eee915819…` |
| `foundation/reports/v2_coverage_matrix_20261110.json` | 14705 | `f00927725b39de99…` |
| `foundation/reports/g2_road3_attempt_20261110.md` | 3335 | `0769f06e09768b1f…` |
| `foundation/reports/g2_road3_teleport_seed123_20261110.json` | 7429 | `39b7f80ba5ba72ac…` |
| `foundation/reports/g2_stage2_inject_probe_20261110.md` | 3043 | `8d2abe6c7cb9f186…` |
| `foundation/reports/g2_stage2_routexy_seed123_20261110.json` | 9297 | `d7167e9eabbd03ef…` |
| `foundation/reports/g2_stage2_routexy_long_seed123_20261110.json` | 9302 | `bfb20d6eb28a3ecf…` |
| `foundation/reports/g2_stage3_return_20261110.md` | 1452 | `885e966c559e88d6…` |
| `foundation/reports/g2_stage3_return_seed123_20261110.json` | 4077 | `52d33d69cb6e7aa9…` |
| `foundation/reports/g2_ol2_attempt_20261110.md` | 1234 | `43252881b2ecdb33…` |
| `foundation/reports/v2_g1_native_attempt_20261110.md` | 1991 | `33dc7d68e592c5ba…` |
| `foundation/reports/v2_g3_g4_20261110.md` | 1637 | `de3dfd6586cadc68…` |
| `foundation/reports/v2_stage_route_manifests_20261110.md` | 2069 | `d491bf925187ce22…` |
| `foundation/reports/v2_training_env_20261110.md` | 1926 | `346dd94ebde63d70…` |
| `foundation/reports/v2_map_audit_plan_20261110.md` | 1913 | `2363fe6afbe74ce5…` |
| `foundation/maps/OL1/factory_entrance_candidates.json` | 9213 | `183ab31d1404b3cb…` |
| `foundation/training_env.py` | 9812 | `0f5b6b909a4c3303…` |
| `foundation/parse_score_dump.py` | 4470 | `505e0d34ae42ecc0…` |
| `foundation/build_v2_coverage_matrix.py` | 13853 | `cb77c3262250465e…` |
| `foundation/build_stage_manifests.py` | 8328 | `addf60152f6f32af…` |
| `foundation/build_stage_probes.py` | 3882 | `901c12592f7f4d7d…` |

## Raw runs

- `foundation/datasets/g2_score_route13_full_seed123_20261110/` — route-height sink failure.
- `foundation/datasets/g2_score_road3_seed123_20261110/` — hub-wall block failure.
- `foundation/datasets/g2_score_road3_teleport_seed123_20261110/` — stage1 teleport, 30/30 component triggers, score 0.
- `foundation/datasets/g2_stage2_inject_probe_seed123_20261110/` — short stage2 score 0→2 probe.
- `foundation/datasets/g2_stage2_full_road3_seed123_20261110/` — gate-centre full run, score 15, then blocked.
- `foundation/datasets/g2_stage2_routexy_seed123_20261110/` — routexy run, score 18/30.
- `foundation/datasets/g2_stage2_routexy_long_seed123_20261110/` — repeat, score 18/30; partial tar recovery.
- `foundation/datasets/g2_stage2_routexy_v4_seed123_20261110/` — score 20/30.
- `foundation/datasets/g2_stage2_routexy_v5_seed123_20261110/` — repeat, score 20/30.
- `foundation/datasets/g2_stage2_routexy_v7_seed123_20261110/` — score 22/30.
- `foundation/datasets/g2_stage2_routexy_gate11_corridor_seed123_20261110/` — score 23/30.
- `foundation/datasets/g2_stage2_routexy_gate11_0_corridor_v12_seed123_20261110/` — best, score 30/30.
- `foundation/datasets/g2_stage2_routexy_v7_repeat_seed123_20261110/` — repeat, score 21/30.
- `foundation/datasets/g2_gate7_normal_lift1_probe_seed123_20261110/` — gate-normal probe, SEG idx7 observed.
- `foundation/datasets/g2_stage2_normal_corridor_seed123_20261110/` — full normal corridor, stalled at gate29, score 0.
- `foundation/datasets/g2_stage2_normal_corridor_v2_seed123_20261110/` — pre→center only, score 7/30.
- `foundation/datasets/g2_stage3_return_seed123_20261110/` — stage3 injection, 31 declared, nan roots, score 0.
- `foundation/datasets/g2_ol2_route1_seed123_20261110/` — OL2 redirect/teleport, no SEG/DUMP.

## 2026-11-10 map-clone correction

- OL2–OL12 are not 12 different tracks: 483 roads, 8,682 building placements, 12 gates,
  12 route factories and 12 route ports are congruent under yaw + translation
  (max residual < 0.06 m). See `v2_map_clone_20261110.md`.
- OL1's GameConfig has the full gate/factory/port/side-track/arrow arrays; OL2–OL12
  only have `RouteConfigs`. Runtime impact remains unverified.
- The old "12 independent map audit" task is superseded by "OL1 full audit + rigid-transform
  validation + runtime sampling".

## 2026-11-10 stage4 reset_v2

- `foundation/datasets/g2_stage4_reset_v2_seed123_20261110/` shows that reset/takeoff can
  unfreeze the vehicle (position moves from the origin to about `(35, -2.7, 2.2)`), but
  the ROS reset clears the stage-4 injection. The DUMP is 33 road1 components, not stage 4;
  the run aborts on the altitude envelope.
- This is evidence that the reset/takeoff path can move the physics body, not evidence that
  stage 4 is reachable. Next attempt must inject stage 4 after reset, or use AirSim
  `simSetVehiclePose` to set the physics-body pose directly.

See the coverage matrix for status; these are diagnostic records, not official equivalence.
