# Data and interface contract

## Frames and units

- Internal sim-fast: `x` forward, `y` right, `z` up, meters.
- Official rules: NED (north, east, down), origin at marker 1.
- Map export: UE4, centimeters, yaw degrees.
- Convert at the boundary only: `ue_cm_to_m`, `ue_to_ned`, `ned_to_ue`,
  `yaw_deg_to_forward`.
- Never mix cm and m in `src/rmua/core`.

## Map schema

`data/maps/maps_summary_all.json` is keyed by `RMUA2024OL1..OL12`. Each map has:

    config.RouteConfigs[]       RouteIndex, RouteSpline.name, SegmentInterval, spline
    config.RouteGateConfigs[]   RouteGateIndex, RouteGate.name
    config.FactoryConfigs[]
    config.CheckPortConfigs[]
    config.SideTrackConfigs[]
    config.ArrowConfigs[]
    gates{}                     name -> location/rotation/scale
    factories{}                 name -> location/rotation/scale
    ports{}                     name -> location/rotation/scale

`scene/map_data.py` converts locations to meters and skips entries whose
transform is missing. `scene/track.py` builds a segment path from the start gate
to the hub to the end gate.

## Gate crossing

- Gate normal is the UE yaw forward vector, oriented by the track.
- Crossing is from negative to positive along the normal.
- Aperture is 10 m wide x 5 m high (`confidence: approx`).
- The segment start and end each have one gate in node 1; the official 12
  detection gates are represented by two per segment.

## Observation (30 floats)

    pos(3), vel(3), quat(4), omega(3),
    next_gate_rel_body(3), gate_normal_body(3),
    progress(1), time_left(1), segment_onehot(6),
    gates_passed(1), force_field(1), start_phase(1)

`src/rmua/policies/obs.py` validates the shape. The actor must eventually use
EKF/sensor-derived values, not `pose_gt`. The critic may use truth during
training.

## Action

- `vel`: `[vx, vy, vz, yaw_rate]`; official message adds `acceleration` and
  `emergency_stop`.
- `pwm`: four values in `[0, 1]`; order right-front, left-rear, left-front,
  right-rear.
- The deployment adapter must lock the first received signal type.

## Reward and info

Reward: route progress, gate pass, segment complete, minus time/force/collision/
timeout. Official ranking is gates first, time second; keep gate bonus dominant.

`info`: `gates_passed`, `total_gates`, `segment_time`, `phase`, `success`,
`failure_reason`, `factory_reports`, `episode_return`.

## Seed streams

Each env gets `SeedSequence([base_seed, env_index])`. The official simulator
uses separate streams for random trees, vehicles, lines and wind; node 1 uses
one stream for the simplified obstacles/wind. Do not assume cross-stream replay.
