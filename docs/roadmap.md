# Roadmap

## Completed locally (node 0/1)

- Rule YAML and G0 tests.
- Static map loader, segment track, gate crossing, force field, wind/line/vehicle
  generators, 6-DOF plant, kinematic plant, vectorized env, benchmark.
- GitHub repo + Hugging Face backup scripts.

## Next: node 2 (after server is rented)

1. Replace the Python inner loop with a C++/JAX/MJX backend.
2. System-identify PWM step, hover and braking against the official simulator.
3. Use the official wind/line/vehicle distributions from `vendor/rmua_re`.
4. Add domain randomization: sensor noise, latency, GPS dropout, seed streams.
5. Freeze throughput and task acceptance on the server.

## Node 3

- `sim-render`: four cameras, LiDAR, factory interior.
- Perception: gate/line/vehicle, red lamps, meter.
- GPS-denied localization and report confidence.
- Zero wrong reports as a hard gate.

## Node 4

- Six-segment curriculum and hierarchical policy.
- 1k -> 4k envs; async PPO/APPO.
- Official OL1, <=4 instances, new process/container per episode.
- Held-out seeds and sim-to-sim gap report.

## Node 5

- ONNX + EKF + safety layer + ROS node.
- First-signal lock, 100 Hz, 15 G image, no GUI, 30 s start.
- Official container closed loop on `1-3`.

## Non-goals

- Official score equivalence.
- 12 independent map layouts.
- 8-instance official stability.
- Native official reset.
- End-to-end pixels -> PWM for the first version.
