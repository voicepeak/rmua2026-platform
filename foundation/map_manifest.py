"""Generate a reproducible coverage manifest for the candidate map query."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from foundation.map_scene import CandidateMap


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    scene = CandidateMap.load(a.directory)
    first = scene.placements[0]
    vertices, triangles = scene.meshes[first['mesh']]
    triangle = vertices[triangles[0]]
    center = triangle.mean(axis=0)
    normal = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
    norm = np.linalg.norm(normal)
    if norm < 1e-8:
        raise ValueError('degenerate test triangle')
    normal /= norm
    r = Rotation.from_quat(first['quaternion_xyzw'])
    t, scale = np.asarray(first['translation_ue_cm']), np.asarray(first['scale'])
    start = r.apply((center+20*normal)*scale)+t
    end = r.apply((center-20*normal)*scale)+t
    query = scene.raycast_ue(start, end)
    if not any(x['actor'] == first['actor'] for x in query['hits']):
        raise ValueError('own mesh ray self-test failed')
    data = {'status': 'U, partial candidate; not official collision map', 'frame': 'UE world centimetres',
            'included_instances': len(scene.placements), 'excluded_unknown_building_components': scene.excluded_count,
            'excluded_other': ['road auxiliary cube components', 'unknown map objects', 'collision channels'],
            'self_test': {'actor': first['actor'], 'query': query, 'scope': 'synthetic ray through its own triangle'},
            'source_sha256': scene.source_sha256,
            'limitations': ['all placement transforms remain unverified', 'zero-radius ray is not swept drone collision',
                            'no-hit result does not prove free space', 'no official contact comparison']}
    a.output.write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
    print('included', data['included_instances'], 'excluded unknown buildings', data['excluded_unknown_building_components'])


if __name__ == '__main__':
    main()
