"""Compare candidate contact bytes to the nearest runtime-matched map surface.

The hit vector at byte 0x18 and source transform offsets are hypotheses.
Nearest component is attributed geometrically, not through decoded UE names.
"""
import argparse
import hashlib
import json
import re
import struct
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from foundation.analyze_contact_transform import EVENT
from foundation.map_scene import CandidateMap
from foundation.swept_mesh import point_triangle_distance_squared


def analyze(scene, raw_log):
    placements = np.array([t for _, t, _ in scene.transforms])
    events = []
    for line in raw_log.splitlines():
        m = EVENT.search(line)
        if not m:
            continue
        hit = re.search(r'\bhit_bytes=([0-9a-f]{272})\b', line)
        if not hit:
            raise ValueError('contact has no readable hit bytes')
        source = np.asarray([float(x) for x in m[7].split(',')])
        quaternion = Rotation.from_quat([float(x) for x in m[8].split(',')])
        point = np.asarray(struct.unpack_from('<3f', bytes.fromhex(hit.group(1)), 0x18))
        if source.shape != (3,) or not np.isfinite(np.r_[source, point]).all():
            raise ValueError('invalid contact source or point')
        distances = np.linalg.norm(placements-source, axis=1)
        nearest = np.argsort(distances)[:2]
        i = int(nearest[0])
        obj = scene.placements[i]
        rot, translation, scale = scene.transforms[i]
        angle = float(np.rad2deg((quaternion.inv()*rot).magnitude()))
        vertices, triangles = scene.meshes[obj['mesh']]
        world = rot.apply(vertices*scale)+translation
        surface_dist = min(point_triangle_distance_squared(point, world[tri]) for tri in triangles)**.5
        events.append({'monotonic_s': float(m[1]), 'candidate_actor': obj['actor'],
                       'candidate_component': obj.get('component'), 'candidate_mesh': obj['mesh'],
                       'runtime_component_ptr': m[6], 'source_transform_error_cm': float(distances[i]),
                       'second_nearest_transform_distance_cm': float(distances[nearest[1]]),
                       'source_rotation_error_deg': angle, 'candidate_hit_point_cm': point.tolist(),
                       'candidate_surface_distance_cm': float(surface_dist),
                       'hit_bytes_sha256': hashlib.sha256(bytes.fromhex(hit.group(1))).hexdigest()})
    if not events:
        raise ValueError('no contact events')
    return events


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--assets', type=Path, default=Path('foundation/asset_analysis'))
    a = p.parse_args()
    meta_raw = (a.run/'run.json').read_bytes()
    meta = json.loads(meta_raw)
    binary = (a.run/'binary_check.log').read_bytes()
    if meta['exit_status'] != 0 or not (b'OK' in binary or '成功'.encode('utf-8') in binary):
        raise ValueError('run did not complete with verified binary')
    logs = list((a.run/'pool').glob('*.log'))
    if len(logs) != 1:
        raise ValueError('expected one simulator log')
    log_raw = logs[0].read_bytes()
    scene = CandidateMap.load(a.assets, include_unverified_simple=True, include_unverified_sprites=True)
    events = analyze(scene, log_raw.decode('utf-8', 'replace'))
    result = {'status': 'runtime contact source and candidate surface comparison; not complete G01',
              'seed': meta['seed_requested'], 'event_count': len(events), 'events': events,
              'max_candidate_surface_distance_cm': max(x['candidate_surface_distance_cm'] for x in events),
              'source_sha256': {'run': hashlib.sha256(meta_raw).hexdigest(),
                                'binary_check': hashlib.sha256(binary).hexdigest(),
                                'log': hashlib.sha256(log_raw).hexdigest(), 'map': scene.source_sha256},
              'limitations': ['candidate hit point offset 0x18 not independently typed',
                              'source component identity inferred from nearest transform, names not decoded',
                              'only captured contact points and one local area are covered',
                              'GDB all-stop changes timing, collision channels and response not fully established']}
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('seed', result['seed'], 'events', len(events), 'source', events[0]['candidate_actor'],
          events[0]['candidate_component'], 'max point distance cm', result['max_candidate_surface_distance_cm'])


if __name__ == '__main__':
    main()
