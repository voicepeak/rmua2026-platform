# Physics foundation (research candidate)

`PhysicsSim` is a single-vehicle PWM stepping interface joining the uncalibrated
6DOF candidate, optional measured external-force replay, and line/vehicle swept
training proxies. The caller must explicitly supply physical parameters and
initial state. World positions/velocities are NED metres; body angular velocity
is FRD rad/s; quaternions are wxyz. A collision terminates at the end of the
substep containing the detected hit, not at the exact contact instant.

```python
from foundation.quad import Params
from foundation.sim import PhysicsSim

# Illustrative only: coefficients and lag are not official calibration.
params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
sim = PhysicsSim(params)
sim.reset({'rotor_signal': [.18] * 4})
state, info = sim.step([.18] * 4, .01)
```

The `info` result explicitly identifies missing static-map collision, wind force
and degradation. Map geometry artifacts remain in UE centimetres and must not
be passed as NED without validated transforms. The simulator is **No-Go for
PWM policy training** until the official experiments and held-out acceptance
gates in `reports/foundation_acceptance.md` pass. Seed generation and official
scoring are intentionally not implied by this interface.

Run development checks with `python -m pytest -q foundation/tests`.
