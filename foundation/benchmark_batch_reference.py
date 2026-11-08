"""Measure reference batch parity and local throughput, with no official claim."""
import hashlib
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np

from foundation.batch import BatchRmuaPhysics
from foundation.quad import Params
from foundation.runtime import RmuaPhysics


def exercise(size, *, seed=20261109, commands=12, backend='reference'):
    rng = np.random.default_rng(seed)
    initial = [{'position': [float(i), 0., 0.], 'rotor_signal': [.18]*4} for i in range(size)]
    actions = rng.uniform(.16, .20, (commands, size, 4))
    params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    batch = BatchRmuaPhysics([RmuaPhysics(params, physics_dt=.0025) for _ in range(size)], backend=backend)
    singles = [RmuaPhysics(params, physics_dt=.0025) for _ in range(size)]
    batch.reset(initial)
    for env, state in zip(singles, initial):
        env.reset(state)
    t0 = time.perf_counter()
    for action in actions:
        batch.batch_step(action, .01)
    batch_s = time.perf_counter() - t0
    t0 = time.perf_counter()
    for action in actions:
        for env, pwm in zip(singles, action):
            env.step(pwm, .01)
    singles_s = time.perf_counter() - t0
    max_errors = {}
    for key in ('position', 'velocity', 'quaternion_wxyz', 'omega', 'rotor_signal'):
        max_errors[key] = max(float(np.max(np.abs(batch.truth_state(i)[key] - env.truth_state()[key])))
                              for i, env in enumerate(singles))
    max_errors['time_s'] = max(abs(batch.truth_state(i)['t'] - env.truth_state()['t'])
                               for i, env in enumerate(singles))
    return {'environments': size, 'backend_requested': backend, 'backend_used': batch.last_backend,
            'commands_per_environment': commands, 'physics_dt_s': .0025,
            'command_dt_s': .01, 'seed': seed, 'action_bytes_sha256': hashlib.sha256(actions.tobytes()).hexdigest(),
            'batch_wall_s': batch_s, 'singles_wall_s': singles_s,
            'batch_environment_commands_per_wall_s': size*commands / batch_s,
            'single_environment_commands_per_wall_s': size*commands / singles_s,
            'final_max_absolute_errors': max_errors}


def main():
    runs = [exercise(n) for n in (64, 256)]
    result = {'status': 'sequential reference baseline; not optimized and not official comparison',
              'platform': platform.platform(), 'processor': platform.processor(),
              'python_version': sys.version.split()[0], 'numpy_version': np.__version__,
              'runs': runs, 'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'code_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                              (Path('foundation/batch.py'), Path('foundation/runtime.py'),
                               Path('foundation/sim.py'), Path('foundation/quad.py'))},
              'limitations': ['no maps, wind, field, obstacles or terminal events in this benchmark',
                              'final states only; step-by-step parity is tested separately at small batch size',
                              'single local timing per size; warmup, variance and CPU contention not controlled',
                              'no official simulator on the same hardware was timed; no official speed ratio',
                              'this batch is sequential and does not establish V2 throughput or physics alignment']}
    Path('foundation/reports/v2_batch_reference_benchmark.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for row in runs:
        print(row['environments'], 'batch commands/s', round(row['batch_environment_commands_per_wall_s'], 1),
              'max state error', max(row['final_max_absolute_errors'].values()))
    if any(max(r['final_max_absolute_errors'].values()) > 1e-12 for r in runs):
        raise SystemExit('batch/single parity failed')


if __name__ == '__main__':
    main()
