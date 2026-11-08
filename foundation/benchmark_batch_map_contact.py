"""Time candidate-map contact sweeps and compare batch to separate instances."""
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


def initial_state(scene):
    obj = scene.placements[0]
    rotation, translation, scale = scene.transforms[0]
    vertices, indices = scene.meshes[obj['mesh']]
    tri = rotation.apply(vertices[indices[0]] * scale) + translation
    center = tri.mean(axis=0)
    normal = np.cross(tri[1]-tri[0], tri[2]-tri[0])
    normal /= np.linalg.norm(normal)
    start, end = center + normal*50, center - normal*50
    # Candidate queries establish this benchmark's own intended geometry only.
    if scene.sweep_sphere_ue(start, start, 10)['hits']:
        raise ValueError('benchmark begins in candidate surface contact')
    probe = scene.sweep_sphere_ue(start, end, 10)
    if not probe['hits']:
        raise ValueError('benchmark sweep missed candidate surfaces')
    start_ned = ue_cm_to_ned_m(start, [0, 0, 0])
    end_ned = ue_cm_to_ned_m(end, [0, 0, 0])
    return {'position': start_ned.tolist(), 'velocity': ((end_ned-start_ned)/.01).tolist()}, probe


def exercise(scene, size, seed):
    initial, probe = initial_state(scene)
    params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    def make():
        return RmuaPhysics(params, physics_dt=.01, candidate_map=scene,
                           map_origin_ue_cm=[0, 0, 0], drone_radius_m=.1)
    batch = BatchRmuaPhysics([make() for _ in range(size)])
    singles = [make() for _ in range(size)]
    batch.reset([initial]*size)
    for env in singles:
        env.reset(initial)
    actions = np.random.default_rng(seed).uniform(.16, .20, (size, 4))
    t0 = time.perf_counter()
    obs, events = batch.batch_step(actions, .01)
    batch_seconds = time.perf_counter() - t0
    t0 = time.perf_counter()
    reference = [env.step(action, .01) for env, action in zip(singles, actions)]
    single_seconds = time.perf_counter() - t0
    if batch.last_backend != 'reference' or any((obs[i], events[i]) != reference[i] for i in range(size)):
        raise ValueError('batch/single event mismatch')
    if any(e['contact'] is None or e['contact']['kind'] != 'static_candidate' for e in events):
        raise ValueError('expected map contact was not observed')
    errors = {key: max(float(np.max(np.abs(batch.truth_state(i)[key]-env.truth_state()[key])))
                       for i, env in enumerate(singles))
              for key in ('position', 'velocity', 'quaternion_wxyz', 'omega', 'rotor_signal')}
    return {'environments': size, 'seed': seed, 'pwm_sha256': hashlib.sha256(actions.tobytes()).hexdigest(),
            'initial_state': initial, 'preflight_candidate_count': probe['candidate_count'],
            'preflight_hits': probe['hits'], 'batch_wall_s': batch_seconds, 'single_wall_s': single_seconds,
            'batch_environment_commands_per_wall_s': size/batch_seconds,
            'single_environment_commands_per_wall_s': size/single_seconds,
            'terminal_count': sum(e['terminal'] for e in events), 'max_state_errors': errors}


def main():
    groups = {}
    for mode in ('default', 'all_optional'):
        enabled = mode == 'all_optional'
        scene = CandidateMap.load('foundation/asset_analysis', include_unverified_simple=enabled,
                                  include_unverified_sprites=enabled, include_unverified_racks=enabled,
                                  include_unverified_road_cubes=enabled)
        by_size = {}
        for size in (64, 256):
            runs = [exercise(scene, size, 20261109+i) for i in range(2)]
            by_size[str(size)] = {'runs': runs,
                                  'median_batch_commands_per_s': statistics.median(x['batch_environment_commands_per_wall_s'] for x in runs)}
        groups[mode] = {'candidate_instance_count': len(scene.placements), 'source_sha256': scene.source_sha256,
                        'sizes': by_size}
    paths = [Path(__file__), *(Path('foundation')/name for name in
             ('batch.py', 'runtime.py', 'sim.py', 'quad.py', 'map_scene.py', 'swept_mesh.py', 'frames.py'))]
    result = {'status': 'candidate-map one-step contact workload; not official throughput or geometry validation',
              'platform': platform.platform(), 'processor': platform.processor(), 'python_version': sys.version.split()[0],
              'numpy_version': np.__version__, 'code_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
              'groups': groups, 'limitations': ['same candidate surface and initial state for all environments',
                    'one .01 s command at 100 m/s aimed at a mesh, no full route or realistic dynamics',
                    'single process, two short trials per size, no CPU isolation, no official same-machine baseline',
                    'no time of impact, no contact physics comparison, loaded geometry and channels unverified']}
    Path('foundation/reports/v2_batch_map_contact_benchmark.json').write_text(
        json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    for mode, group in groups.items():
        for size, data in group['sizes'].items():
            print(mode, size, 'commands/s', round(data['median_batch_commands_per_s'], 1))


if __name__ == '__main__':
    main()
