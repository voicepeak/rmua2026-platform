# Rule contract

Source of truth: `configs/rules_2026.yaml`. Tests: `tests/test_rules.py`.

## Course

- Six fixed continuous paths: `1-3`, `3-5`, `5-8`, `8-12`, `12-10`, `10-7`.
- Each path starts at the previous endpoint, passes the central hub and ends at
  the next marker.
- Time limits: 180 / 180 / 180 / 200 / 220 / 200 s.
- Each start trigger zone is 10 m wide, 5 m high and 7 m long.
- The drone must leave the trigger zone within 30 s of deployment/segment start.
- Ranking: detection gates first, total time second.

## Gates and factories

- Detection gates trigger in path order; a later gate cannot score before the
  previous one.
- `8-12`: one factory before the hub.
- `12-10` and `10-7`: one factory before and one after the hub.
- Skipping a factory inspection prevents the next detection gate from triggering.
- Factory: 20 devices, one target with three red lamps, meter value 0-50.
- A wrong meter report is a hard failure.

## Wind, force field, collision

- Wind exists only after marker 8 (`8-12`, `12-10`, `10-7`); indoors has no wind.
- Outside the road/hub corridor, a static force field reduces thrust; cumulative
  stay >2 s is a hard failure.
- Collision with obstacles, buildings or decorations is a hard failure.

## Interface and deployment

- Control: PWM or body velocity, maximum 100 Hz.
- The first received control signal type locks the interface.
- VEL fields: `vx, vy, vz, yaw_rate, acceleration, emergency_stop`.
- PWM motor order: right-front, left-rear, left-front, right-rear.
- Sensors: IMU 100 Hz; GPS 10 Hz with 0.1 m / 0.2 rad error; LiDAR 10 Hz,
  20000 points/frame, 30 m; four cameras 960x720 @20 Hz, FOV 60.
- Initial pose and current endpoint are published at 10 Hz.
- Docker image <15 G, no GUI, must not exit actively, 10 submissions/day.
- The official seed changes with the submission count.

## Confidence

Start/end markers, time limits, wind/factory schedule and sensor rates are
official. The segment-to-gate/spline mapping and gate aperture are
`confidence: approx` until runtime calibration.
