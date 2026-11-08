"""Benchmark full-feature reference fallback on a synthetic far-from-map scene."""
import hashlib
import json
import platform
import statistics
import sys
import time
from pathlib import Path

import numpy as np

from foundation.aero import DragFaces
from foundation.batch import BatchRmuaPhysics
from foundation.effects import RecordedEffects
from foundation.map_scene import CandidateMap
from foundation.obstacles import Line, Vehicle
from foundation.out_time import OutTimeCounter
from foundation.quad import Params
from foundation.runtime import RmuaPhysics


def exercise(scene, size, seed):
    rng = np.random.default_rng(seed)
    actions = rng.uniform(.16, .20, (6, size, 4))
    params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    effects = RecordedEffects([0., .02, .05, .2], [[0, 1, 0]]*4, [1, 0, 1, 1])
    normals = np.vstack([np.eye(3), -np.eye(3)])

    def make(i):
        # Candidate map is loaded but initial states are intentionally far from
        # its geometry: this is not a realistic map-contact throughput claim.
        position = np.array([10000. + 2*i, 10000., -10000.])
        line = Line(tuple(position + [100, 0, -3]), tuple(position + [100, 0, 3]), .03)
        vehicle = Vehicle(tuple(position + [200, 0, 0]), (2, 1, 1), (1, 0, 0), 0)
        aero = DragFaces(np.zeros((6, 3)), normals, [.08]*6, 1.225, 0.)
        env = RmuaPhysics(params, physics_dt=.0025, candidate_map=scene, map_origin_ue_cm=[0, 0, 0],
                          lines=[line], vehicles=[vehicle], aero_model=aero,
                          recorded_effects=effects, field_rotor_target_scale=.25, out_time_counter=OutTimeCounter(.5))
        state = {'position': position, 'rotor_signal': [.18]*4}
        return env, state

    batch_pairs = [make(i) for i in range(size)]
    single_pairs = [make(i) for i in range(size)]
    batch = BatchRmuaPhysics([e for e, _ in batch_pairs])
    batch.reset([s for _, s in batch_pairs])
    for env, state in single_pairs:
        env.reset(state)
    t0 = time.perf_counter()
    for pwm in actions:
        _, batch_events = batch.batch_step(pwm, .01)
    batch_time = time.perf_counter() - t0
    t0 = time.perf_counter()
    for pwm in actions:
        single_events = [env.step(command, .01)[1] for (env, _), command in zip(single_pairs, pwm)]
    single_time = time.perf_counter() - t0
    if batch.last_backend != 'reference' or batch_events != single_events:
        raise ValueError('fallback or event parity failed')
    errors = {key: max(float(np.max(np.abs(batch.truth_state(i)[key] - env.truth_state()[key])))
                       for i, (env, _) in enumerate(single_pairs))
              for key in ('position', 'velocity', 'quaternion_wxyz', 'omega', 'rotor_signal')}
    field_errors = [abs(batch_events[i]['force_zone_elapsed_s'] - single_events[i]['force_zone_elapsed_s'])
                    for i in range(size)]
    return {'environments': size, 'seed': seed, 'commands_per_environment': len(actions),
            'action_bytes_sha256': hashlib.sha256(actions.tobytes()).hexdigest(),
            'batch_wall_s': batch_time, 'singles_wall_s': single_time,
            'batch_environment_commands_per_wall_s': size*len(actions)/batch_time,
            'single_environment_commands_per_wall_s': size*len(actions)/single_time,
            'final_max_absolute_errors': errors, 'max_field_counter_error_s': max(field_errors),
            'backend': batch.last_backend, 'terminal_count': sum(x['terminal'] for x in batch_events)}


def main():
    scene = CandidateMap.load('foundation/asset_analysis')
    groups = {}
    for size in (64, 256):
        runs = [exercise(scene, size, 20261109+i) for i in range(2)]
        if any(max(x['final_max_absolute_errors'].values()) > 1e-12 or x['max_field_counter_error_s'] > 1e-12 for x in runs):
            raise ValueError('batch/single final parity failed')
        groups[str(size)] = {'runs': runs, 'median_batch_commands_per_s': statistics.median(x['batch_environment_commands_per_wall_s'] for x in runs)}
    result = {'status': 'synthetic configured-scenario reference benchmark; not official parity',
              'platform': platform.platform(), 'processor': platform.processor(), 'python_version': sys.version.split()[0],
              'numpy_version': np.__version__, 'candidate_map_instances': len(scene.placements),
              'candidate_map_source_sha256': scene.source_sha256, 'groups': groups,
              'code_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                              (Path(__file__), Path('foundation/batch.py'), Path('foundation/runtime.py'),
                               Path('foundation/sim.py'), Path('foundation/quad.py'), Path('foundation/obstacles.py'),
                               Path('foundation/map_scene.py'), Path('foundation/aero.py'),
                               Path('foundation/effects.py'), Path('foundation/out_time.py'))},
              'limitations': ['synthetic starting positions far from candidate map and obstacles: no collision workload',
                              'wind and field parameters are illustrative, not validated against official',
                              'two short trials per size, no CPU isolation, no same-machine official baseline',
                              'events and final states are compared; complex event sequences tested separately at small batch size']}
    Path('foundation/reports/v2_batch_scenario_benchmark.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for size, group in groups.items():
        print(size, 'reference commands/s', round(group['median_batch_commands_per_s'], 1),
              'max error', max(max(x['final_max_absolute_errors'].values()) for x in group['runs']))


if __name__ == '__main__':
    main()
