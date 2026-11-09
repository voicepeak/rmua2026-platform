"""Dense candidate-map contact workload across many distinct OL1 surfaces.

This benchmark exercises the sequential reference batch path with all optional
OL1 candidates, one distinct surface point per environment, and one short
command. It is a local workload measurement, not official throughput or an
official collision comparison.
"""
import hashlib
import json
import platform
import statistics
import sys
import time
from pathlib import Path

import numpy as np

from foundation.batch import BatchRmuaPhysics
from foundation.frames import ue_cm_to_ned_m
from foundation.map_scene import CandidateMap
from foundation.quad import Params
from foundation.runtime import RmuaPhysics


def _surface_states(scene, count, seed, command_dt=.01, standoff_m=.5):
    rng = np.random.default_rng(seed)
    order = rng.permutation(len(scene.placements))
    states, probes, actors = [], [], []
    for index in order:
        obj = scene.placements[int(index)]
        rotation, translation, scale = scene.transforms[int(index)]
        vertices, indices = scene.meshes[obj['mesh']]
        for tri_index in indices[:min(len(indices), 64)]:
            tri = rotation.apply(vertices[tri_index] * scale) + translation
            normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            norm = np.linalg.norm(normal)
            if norm < 1e-9:
                continue
            normal /= norm
            center = tri.mean(axis=0)
            start_ue = center + normal * (standoff_m * 100)
            end_ue = center - normal * (standoff_m * 100)
            if scene.sweep_sphere_ue(start_ue, start_ue, 10)['hits']:
                continue
            probe = scene.sweep_sphere_ue(start_ue, end_ue, 10)
            if not probe['hits']:
                continue
            start = ue_cm_to_ned_m(start_ue, [0, 0, 0])
            end = ue_cm_to_ned_m(end_ue, [0, 0, 0])
            states.append({'position': start.tolist(),
                           'velocity': ((end - start) / command_dt).tolist(),
                           'rotor_signal': [.18] * 4})
            probes.append({'source_actor': obj['actor'], 'source_component': obj.get('component'),
                           'start_ue_cm': start_ue.tolist(), 'end_ue_cm': end_ue.tolist(),
                           'candidate_count': probe['candidate_count'],
                           'first_hits': probe['hits'][:4]})
            actors.append(obj['actor'])
            break
        if len(states) == count:
            break
    if len(states) != count:
        raise ValueError(f'only found {len(states)} of {count} distinct surface states')
    return states, probes, actors


def exercise(scene, size, seed, command_dt=.01):
    params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    states, probes, actors = _surface_states(scene, size, seed, command_dt=command_dt)
    def make():
        return RmuaPhysics(params, physics_dt=command_dt, candidate_map=scene,
                           map_origin_ue_cm=[0, 0, 0], drone_radius_m=.1)
    batch = BatchRmuaPhysics([make() for _ in range(size)])
    singles = [make() for _ in range(size)]
    batch.reset(states)
    for env, state in zip(singles, states):
        env.reset(state)
    actions = np.random.default_rng(seed + 1).uniform(.16, .20, (size, 4))
    t0 = time.perf_counter()
    obs, events = batch.batch_step(actions, command_dt)
    batch_seconds = time.perf_counter() - t0
    t0 = time.perf_counter()
    reference = [env.step(action, command_dt) for env, action in zip(singles, actions)]
    single_seconds = time.perf_counter() - t0
    if batch.last_backend != 'reference':
        raise ValueError('dense map workload unexpectedly vectorized')
    if any((obs[i], events[i]) != reference[i] for i in range(size)):
        raise ValueError('batch/single event mismatch')
    if any(e['contact'] is None or e['contact']['kind'] != 'static_candidate' for e in events):
        raise ValueError('expected static candidate contact for every environment')
    errors = {key: max(float(np.max(np.abs(batch.truth_state(i)[key] - env.truth_state()[key])))
                       for i, env in enumerate(singles))
              for key in ('position', 'velocity', 'quaternion_wxyz', 'omega', 'rotor_signal')}
    return {
        'environments': size,
        'seed': seed,
        'pwm_sha256': hashlib.sha256(actions.tobytes()).hexdigest(),
        'distinct_surface_actors': len(set(actors)),
        'distinct_surface_components': len({(p['source_actor'], p['source_component']) for p in probes}),
        'median_probe_candidate_count': statistics.median(p['candidate_count'] for p in probes),
        'min_probe_candidate_count': min(p['candidate_count'] for p in probes),
        'max_probe_candidate_count': max(p['candidate_count'] for p in probes),
        'batch_wall_s': batch_seconds,
        'single_wall_s': single_seconds,
        'batch_environment_commands_per_wall_s': size / batch_seconds,
        'single_environment_commands_per_wall_s': size / single_seconds,
        'terminal_count': sum(e['terminal'] for e in events),
        'max_state_errors': errors,
        'probes': probes,
    }


def main():
    scene = CandidateMap.load(
        'foundation/asset_analysis', include_unverified_simple=True,
        include_unverified_sprites=True, include_unverified_racks=True,
        include_unverified_road_cubes=True, include_unverified_building_meshes=True,
        include_unverified_rack_meshes=True)
    groups = {}
    for size in (64, 256):
        runs = [exercise(scene, size, 20261109 + i) for i in range(2)]
        groups[str(size)] = {
            'runs': runs,
            'median_batch_commands_per_s': statistics.median(
                x['batch_environment_commands_per_wall_s'] for x in runs),
            'median_single_commands_per_s': statistics.median(
                x['single_environment_commands_per_wall_s'] for x in runs),
        }
    paths = [Path(__file__), *(Path('foundation') / name for name in
             ('batch.py', 'runtime.py', 'sim.py', 'quad.py', 'map_scene.py',
              'swept_mesh.py', 'frames.py'))]
    result = {
        'status': 'dense candidate-map contact workload; not official throughput or geometry validation',
        'platform': platform.platform(), 'processor': platform.processor(),
        'python_version': sys.version.split()[0], 'numpy_version': np.__version__,
        'candidate_instance_count': len(scene.placements),
        'candidate_map_source_sha256': scene.source_sha256,
        'groups': groups,
        'code_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
        'limitations': [
            'each environment starts at a different sampled candidate surface, but all use the same one-step high-speed approach',
            'candidate transforms, collision channels and solidity remain unverified',
            'sequential reference path only; this is not a vectorized-speed claim',
            'single process, two short trials per size, no CPU isolation, no official same-machine baseline',
            'a candidate contact is not an official collision response or time of impact',
        ],
    }
    output = Path('foundation/reports/v2_batch_dense_map_benchmark.json')
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for size, group in groups.items():
        print(size, 'instances', len(scene.placements),
              'batch commands/s', round(group['median_batch_commands_per_s'], 1),
              'single commands/s', round(group['median_single_commands_per_s'], 1))


if __name__ == '__main__':
    main()
