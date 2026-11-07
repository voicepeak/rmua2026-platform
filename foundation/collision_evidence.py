"""Extract explicit component collision fields without inventing UE defaults.

Template, instance and mesh-default fields are retained separately. Effective
responses require Unreal config merging, component initialization, runtime
overrides, query type and drone channel; this inventory does not resolve them.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path
from foundation.extract_physx_mesh import body_setups

WANTED = {'CollisionProfileName', 'CollisionEnabled', 'ObjectType', 'ResponseToChannels',
          'CollisionResponses', 'bUseDefaultCollision', 'CollisionTraceFlag'}


def collect(value):
    result = []
    def visit(node):
        if isinstance(node, dict):
            if node.get('Name') in WANTED:
                item = {'name': node['Name']}
                for key in ('Value', 'EnumValue', 'EnumType'):
                    if key in node:
                        item[key] = node[key]
                result.append(item)
                # Nested responses are still preserved within Value.
                return
            for x in node.values():
                visit(x)
        elif isinstance(node, list):
            for x in node:
                visit(x)
    visit(value)
    return result


def build(map_asset, inventory, blueprints, meshes):
    exports = map_asset['Exports']
    mesh_fields = {}
    for name, asset in meshes.items():
        bodies = body_setups(asset)
        mesh_fields[name] = {'body_count': len(bodies),
                             'fields': collect(bodies[0].get('Data', [])) if len(bodies) == 1 else []}
    rows = []
    aliases = {'/Engine/BasicShapes/Cube': 'Engine_BasicShapes_Cube',
               '/Engine/EngineMeshes/Cube': 'Engine_EngineMeshes_Cube',
               '/AirSimROS/Calibration/Resources/SM_plane': 'AirSimROS_SM_plane'}
    for actor in inventory['actors']:
        bp = blueprints[actor['class'][:-2]]
        templates = {x['ObjectName']: x for x in bp['Exports']}
        for c in actor['components']:
            name = c['name']+'_GEN_VARIABLE'
            template = templates.get(name)
            if template is None:
                raise ValueError('missing component template '+name)
            instance = exports[c['export_index']-1]
            mesh = aliases.get(c['mesh']['package'], c['mesh']['name'])
            rows.append({'actor': actor['actor'], 'component': c['name'], 'export_index': c['export_index'],
                         'mesh': mesh, 'template_fields': collect(template.get('Data', [])),
                         'instance_fields': collect(instance.get('Data', []))})
    return {'components': rows, 'mesh_defaults': mesh_fields,
            'limitations': ['explicit serialized fields only; absent fields are unknown',
                            'template and instance fields are not merged',
                            'project and engine defaults, object channel and runtime overrides unresolved']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('map', type=Path)
    p.add_argument('analysis', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    raw = a.map.read_bytes()
    bp_paths = sorted((a.analysis/'blueprints').glob('*.json'))
    mesh_paths = sorted((a.analysis/'meshes').glob('*.json'))
    inventory_path = a.analysis/'OL1_building_components.json'
    data = build(json.loads(raw), json.loads(inventory_path.read_bytes()),
                 {f.stem: json.loads(f.read_bytes()) for f in bp_paths},
                 {f.stem: json.loads(f.read_bytes()) for f in mesh_paths})
    data['source_sha256'] = {'map': hashlib.sha256(raw).hexdigest(),
                             'inventory': hashlib.sha256(inventory_path.read_bytes()).hexdigest(),
                             'blueprints': {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in bp_paths},
                             'meshes': {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in mesh_paths}}
    a.output.write_text(json.dumps(data, separators=(',', ':'))+'\n', encoding='utf-8')
    for layer in ('template_fields', 'instance_fields'):
        counts = collections.Counter((v['name'], str(v.get('Value', v.get('EnumValue'))))
                                     for row in data['components'] for v in row[layer] if v['name'] != 'ResponseToChannels')
        print(layer, counts.most_common(12))


if __name__ == '__main__':
    main()
