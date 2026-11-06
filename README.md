# RMUA2026 Training Platform

Self-built training platform for the RoboMaster 2026 autonomous UAV challenge.
This repository is the **workspace**, not a copy of the reverse-engineering
dossier. The dossier stays at `voicepeak/RMUA2026-re`; this repo turns its
static data into a trainable, testable system.

## Status

| Node | Scope | Status |
|---|---|---|
| 0 | Rules, segments, sensors, coordinate/data contracts, G0 tests | Done |
| 1 | State-only vectorized `sim-fast` core | Done (Python reference) |
| 2 | 1-3/3-5/5-8 state-only BC+DAgger+PPO; obstacles/wind/system ID pending | Partial (100% in sim-fast) |
| 3 | Factory + perception + `sim-render` | Not started |
| 4 | Six-segment curriculum + scale + official calibration | Not started |
| 5 | Docker/ROS/EKF/ONNX deployment | Not started |

Node 2 first results: 1-3, 3-5, 5-8 each 100% on 5 held-out seeds x 64 envs in
the state-only sim-fast. See `docs/training_results.md`. Weights are backed up on
Hugging Face. This is not official-sim equivalence.

Node 1 is deliberately **state-only**: no cameras, no LiDAR, no ROS, no render.
That is what makes local iteration and later high-concurrency training cheap.
The actor/critic split (`critic` may use truth, `actor` must use deployable
sensors) is already part of the contract.

## Quick start

    python -m pip install -e ".[dev]"
    python -m pytest -q
    python scripts/validate_data.py
    python scripts/benchmark.py --envs 64,256,1024 --steps 200

Windows PowerShell:

    $env:PYTHONPATH="D:\RMUA2026-Platform\src"
    python -m pytest -q

## Node 0: rule contract

`configs/rules_2026.yaml` encodes the official rules as data:

- Six segments: `1-3, 3-5, 5-8, 8-12, 12-10, 10-7`
- Time limits: `180/180/180/200/220/200 s`
- 30 s start trigger, 2 s force-field limit
- Wind only after marker 8, indoors no wind
- Factory schedule: `8-12` one; `12-10`, `10-7` two each
- 20 devices, three red lamps, meter 0-50, wrong report is a hard failure
- PWM/VEL first-signal lock, 100 Hz max, 4 cameras, 20000 LiDAR points/frame
- Docker <15 G, no GUI, no active exit, 10 submissions/day

`tests/test_rules.py` is the G0 regression gate. Any rule change must update the
YAML and the test together.

## Node 1: sim-fast

`src/rmua/envs/sim_fast.py` provides a vectorized, one-segment environment:

- 6-DOF quad plant (`dynamics="quad"`) or kinematic velocity plant (`kinematic`)
- Gate order, 30 s start, segment timer, force field, wind, collision,
  factory prerequisite, reward and termination
- Deterministic per-env RNG; vectorized reset and step
- 30-float observation contract; 4-value VEL or 4-value PWM action

Run a smoke test:

    python -c "from rmua.envs.sim_fast import SimFastConfig, SimFastVecEnv; import numpy as np; e=SimFastVecEnv(SimFastConfig(num_envs=4)); o=e.reset(); print(o.shape); print(e.step(np.zeros((4,4)))[0].shape)"

The Python reference is intentionally simple. The server version should move
the inner loop to C++/JAX/MJX; `scripts/benchmark.py` records the local baseline.

## Server handoff

The local node 0/1 build is complete. On a fresh server, run:

    bash scripts/server_smoke.sh

It validates data, runs all tests, records the throughput baseline and prints the
hardware lines. See `docs/server_handoff.md` for the checklist and stop lines.

## Data and weights

- Small data (`data/`) is in Git.
- Large data and policy weights go to Hugging Face:
  - `Vociepeak/rmua2026-platform-data` (dataset, private)
  - `Vociepeak/rmua2026-platform-policies` (model, private)
- Backup/restore scripts: `scripts/hf_backup.py`, `scripts/hf_restore.py`.
- Set `HF_TOKEN` in the environment; never commit it.
- Use the Clash HTTP proxy on the local machine if Hugging Face is unreachable:

      $env:HTTPS_PROXY="http://127.0.0.1:7890"
      $env:HTTP_PROXY="http://127.0.0.1:7890"

## Known limitations

- Gate aperture, factory interior, building/road collision and the exact
  segment-to-spline mapping are approximations. See `configs/rules_2026.yaml`
  and `data/README.md`.
- Node 1 has no camera/LiDAR; `sim-render` and perception are node 3.
- The official simulator remains the calibration source. Do not claim
  sim-to-sim equivalence.
- `pose_gt`/`debug` truth must never be an actor input in deployment.

## Repository layout

    configs/        rules and segment contracts
    data/           small static map/table bundle + manifest
    docs/           architecture, rules, data, local build, roadmap
    scripts/        data validation, benchmark, HF backup/restore
    src/rmua/       core, rules, scene, envs, policies, eval
    tests/          G0 rules, map, physics, gate, env and policy tests
    vendor/         reference reverse-engineering models (read-only)

