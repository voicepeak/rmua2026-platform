"""Compare runtime contact-source transforms to saved candidate components.

Nearest-transform correspondence does not establish object identity. Missing
actors, non-mesh components, construction scripts and debugger timing matter.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from foundation.map_scene import CandidateMap

EVENT = re.compile(r'MAP_CONTACT mono=([\d.]+) drone=(0x[0-9a-f]+) drone_component=(0x[0-9a-f]+) '
                   r'drone_xyz=([^\s]+) source_actor=(0x[0-9a-f]+) source_component=(0x[0-9a-f]+) '
                   r'source_xyz=([^\s]+) source_quat=([^\s]+)')


def analyze(scene, log):
    raw = log.read_bytes()
    placements = np.array([t for _, t, _ in scene.transforms])
    events = []
    for m in EVENT.finditer(raw.decode('utf-8', errors='replace')):
        try:
            source = np.array([float(x) for x in m[7].split(',')])
            quat = np.array([float(x) for x in m[8].split(',')])
            drone = np.array([float(x) for x in m[4].split(',')])
            if source.shape != (3,) or quat.shape != (4,) or drone.shape != (3,) or not np.isfinite(np.r_[source, quat, drone]).all():
                raise ValueError('invalid component data')
            rotation = Rotation.from_quat(quat)
        except ValueError:
            continue
        distances = np.linalg.norm(placements-source, axis=1)
        nearest = []
        for i in np.argsort(distances)[:5]:
            obj = scene.placements[i]
            delta = (rotation.inv()*scene.transforms[i][0]).magnitude()*180/np.pi
            nearest.append({'actor': obj['actor'], 'component': obj.get('component'), 'mesh': obj['mesh'],
                            'distance_cm': float(distances[i]), 'rotation_difference_deg': float(delta)})
        events.append({'monotonic_s': float(m[1]), 'drone_world_cm': drone.tolist(),
                       'source_actor_ptr': m[5], 'source_component_ptr': m[6],
                       'source_world_cm': source.tolist(), 'source_quaternion_xyzw': quat.tolist(),
                       'nearest_component_transforms': nearest})
    if not events:
        raise ValueError('no valid contact samples')
    return {'log_sha256': hashlib.sha256(raw).hexdigest(), 'event_count': len(events), 'events': events,
            'map_sha256': scene.source_sha256,
            'limitations': ['nearest transform is not an identity match; source may be an excluded component or actor',
                            'runtime component position/quaternion offsets need independent type confirmation',
                            'source component transform is not hit point or collision shape',
                            'GDB all-stop affects timing and callback behavior',
                            'the candidate map contains unverified simple modes and transforms']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('log', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--assets', type=Path, default=Path('foundation/asset_analysis'))
    p.add_argument('--include-unverified-sprites', action='store_true')
    a = p.parse_args()
    result = analyze(CandidateMap.load(a.assets, include_unverified_simple=True,
                                       include_unverified_sprites=a.include_unverified_sprites), a.log)
    result['includes_unverified_sprites'] = a.include_unverified_sprites
    result['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('events', result['event_count'], 'nearest:', result['events'][0]['nearest_component_transforms'][0])


if __name__ == '__main__':
    main()
