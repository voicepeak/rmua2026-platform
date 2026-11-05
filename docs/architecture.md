# Architecture

## Main chain

    rules + coordinate/data contract
              |
              v
    sim-fast (state-only) --> PPO/APPO --> policy
              |                              |
              v                              v
    sim-render (offline perception)     deployment adapter
              |                       (EKF + safety + ONNX + ROS)
              v
    sim-official (OL1, <=4 instances, calibration only)

The critical path is `rules -> sim-fast -> low-level -> single segment ->
obstacles/wind/forcefield -> factory/perception -> six segments -> scale ->
deployment`.

## Three simulators

| Simulator | Rendering | Sensors | Scale | Purpose |
|---|---|---|---|---|
| `sim-fast` | none | state-only (node 1) | 256 -> 4k -> 10k | RL throughput |
| `sim-render` | yes | cameras + LiDAR | tens | perception data |
| `sim-official` | yes/no | official | <=4 | calibration/acceptance |

## Observation/action contract

- `src/rmua/policies/obs.py`: 30-float state-only observation.
- `src/rmua/policies/action.py`: 4-value VEL or 4-value PWM.
- `src/rmua/policies/safety.py`: first-signal lock and veto layer.
- The actor must consume deployable observations. The critic may consume truth
  during training; the critic is removed at deployment.

## Data flow

1. `scene/map_data.py` loads `data/maps/maps_summary_all.json`.
2. `scene/track.py` builds a segment from start gate -> hub -> end gate.
3. `scene/obstacles.py` and `scene/wind.py` generate seed-driven instances.
4. `envs/sim_fast.py` steps the plant, checks rules and returns reward.
5. `eval/metrics.py` summarizes success, gates, time and failure reasons.

## What is deliberately not here yet

- Camera/LiDAR rendering.
- Factory interior device layout and meter perception.
- Native official reset; use new process/container per episode.
- Multi-map official flight; OL1 is the only calibrated map.
- Official scoring equivalence.
