# RmuaPhysicsVecEnv — approximate PWM training sandbox (2026-11-10)

`foundation/training_env.py` adds a batched PWM-only environment over the
existing `RmuaPhysics`.  It is an **approximate sandbox**, not an
official-equivalent environment.

## Interface

```python
from foundation.quad import Params
from foundation.training_env import RmuaPhysicsVecEnv

params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
env = RmuaPhysicsVecEnv(params, 64, gates=gate_xyz, seed=7,
                        physics_dt=0.0025, command_dt=0.02)
obs = env.reset()
obs, reward, done, truncated, gate_reached, infos = env.step(pwm)  # pwm (64, 4)
```

- Action: four PWM channels in `[0, 1]`, held for `command_dt`.
- Dynamics: 6DOF candidate quad from `foundation.quad`; no official calibration.
- Observation: 22 values — position, velocity, quaternion `wxyz`, body rates,
  rotor signal, previous action, time.  This is the explicit
  `truth_research` mode.
- `observable_state()` returns `None`; no IMU/GPS/EKF/noise/dropout model exists.
- Reward: progress toward the next gate centre, gate bonus, contact/bounds penalty.
- Truncation: `done` for terminal physics/bounds, `truncated` for timeout.
- Seeds: each environment owns `numpy.random.default_rng(seed + index)` for
  reset jitter; no global RNG.
- Batch: Python loop over independent `RmuaPhysics` instances.  It is correct
  but not a high-throughput vectorised backend.

## Tests

`foundation/tests/test_training_env.py` covers observation shape, seed isolation,
PWM validation, terminal masking, gate reward/index advance and per-index reset.

## Limitations

- No sensor model, estimator, camera, LiDAR, noise, latency or packet loss.
- No official map, detection-gate aperture, wind, factory, obstacle or scoring
  equivalence.
- No claim of policy transfer to the official simulator.
- The reward is a training convenience, not an official score.
