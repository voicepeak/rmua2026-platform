"""Compose provisional building-component placements; retain unresolved cases."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from foundation.road_scene import compose, relative

EXTERNAL_MESH_ALIASES = {'/Engine/BasicShapes/Cube': 'Engine_BasicShapes_Cube',
                         '/Engine/EngineMeshes/Cube': 'Engine_EngineMeshes_Cube',
                         '/AirSimROS/Calibration/Resources/SM_plane': 'AirSimROS_SM_plane'}


def build(inventory, blueprints, mesh_inventory):
    meshes = {x['asset']: x for x in mesh_inventory['assets']}
    placements, unresolved = [], []
    for actor in inventory['actors']:
        bp = blueprints.get(actor['class'][:-2])
        if bp is None:
            unresolved.append({'actor': actor['actor'], 'reason': 'missing blueprint'})
            continue
        templates = {e.get('ObjectName'): {p['Name']: p.get('Value') for p in e.get('Data', [])}
                     for e in bp.get('Exports', [])}
        for component in actor['components']:
            try:
                if component['unresolved_parent'] is not None or not component['mesh']:
                    raise ValueError('unresolved parent or mesh')
                transform = (np.zeros(3), Rotation.identity(), np.ones(3))
                for node in reversed(component['chain']):
                    template = templates.get(node['name'] + '_GEN_VARIABLE')
                    if template is None:
                        raise ValueError('missing parent template: '+str(node['name']))
                    transform = compose(transform, relative({**template, **node['fields']}))
                t, r, s = transform
                mesh_name = EXTERNAL_MESH_ALIASES.get(component['mesh']['package'], component['mesh']['name'])
                mesh = meshes.get(mesh_name)
                placements.append({'actor': actor['actor'], 'component': component['name'],
                                   'export_index': component['export_index'], 'mesh': mesh_name,
                                   'mesh_package': component['mesh']['package'],
                                   'mesh_status': mesh['triangle_status'] if mesh else 'not extracted',
                                   'collision_trace_flag': mesh['collision_trace_flag'] if mesh else None,
                                   'translation_ue_cm': t.tolist(), 'quaternion_xyzw': r.as_quat().tolist(),
                                   'scale': s.tolist()})
            except (KeyError, TypeError, ValueError) as exc:
                unresolved.append({'actor': actor['actor'], 'component': component['name'], 'reason': str(exc)})
    return {'status': 'U: candidate placements, not runtime verified', 'frame': 'UE centimetres',
            'assumptions': ['serialized fields override blueprint defaults', 'missing transform fields use identity',
                            'positive-scale UE FTransform composition', 'construction scripts and collision channels unverified'],
            'placements': placements, 'unresolved': unresolved}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    inventory_path = a.directory/'OL1_building_components.json'
    mesh_path = a.directory/'mesh_collision/inventory.json'
    blueprints = {f.stem: json.loads(f.read_bytes()) for f in (a.directory/'blueprints').glob('*.json')}
    result = build(json.loads(inventory_path.read_bytes()), blueprints, json.loads(mesh_path.read_bytes()))
    result['source_sha256'] = {'building_inventory': hashlib.sha256(inventory_path.read_bytes()).hexdigest(),
                               'mesh_inventory': hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
                               'blueprints': {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                                              for f in (a.directory/'blueprints').glob('*.json')}}
    a.output.write_text(json.dumps(result, separators=(',', ':'))+'\n', encoding='utf-8')
    print('placements', len(result['placements']), 'unresolved', len(result['unresolved']))
    print('unresolved reasons', collections.Counter(x['reason'] for x in result['unresolved']).most_common(8))


if __name__ == '__main__':
    main()
