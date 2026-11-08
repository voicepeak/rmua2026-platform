"""Resolve candidate rack placements through their owning building attachments."""
import argparse
import collections
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from foundation.blueprint_mesh_refs import package_for_import
from foundation.road_instances import props
from foundation.road_scene import compose, relative


def build(asset, blueprints, actor_class='BP_Rack_C', component_prefix=None,
          component_class='StaticMeshComponent'):
    exports, imports = asset['Exports'], asset['Imports']
    templates = {name: {e.get('ObjectName'): props(e) for e in bp['Exports']}
                 for name, bp in blueprints.items()}

    def class_name(entry):
        idx = entry.get('ClassIndex', 0)
        return imports[-idx-1].get('ObjectName') if isinstance(idx, int) and -len(imports) <= idx < 0 else None

    def template_for(node):
        owner = node.get('OuterIndex')
        if not isinstance(owner, int) or not 0 < owner <= len(exports):
            raise ValueError('missing component owner')
        owner_class = class_name(exports[owner-1])
        if not owner_class or not owner_class.endswith('_C'):
            raise ValueError('unknown component owner class: '+str(owner_class))
        name = owner_class[:-2]
        fields = templates.get(name, {}).get(node.get('ObjectName', '')+'_GEN_VARIABLE')
        if fields is None:
            raise ValueError('missing template: '+name+'/'+str(node.get('ObjectName')))
        return fields, name

    children = collections.defaultdict(list)
    for i, node in enumerate(exports, 1):
        children[node.get('OuterIndex')].append((i, node))
    placements, unresolved = [], []
    actor_count = 0
    for actor_index, actor in enumerate(exports, 1):
        if class_name(actor) != actor_class:
            continue
        actor_count += 1
        for index, component in children[actor_index]:
            if class_name(component) != component_class:
                continue
            if component_prefix is not None and not component.get('ObjectName', '').startswith(component_prefix):
                continue
            try:
                fields, bp_name = template_for(component)
                if component_class == 'StaticMeshComponent':
                    if 'StaticMesh' in props(component):
                        raise ValueError('instance mesh override requires map import resolution')
                    mesh_index = fields.get('StaticMesh')
                    if not isinstance(mesh_index, int) or mesh_index >= 0:
                        raise ValueError('missing mesh import')
                    bp_imports = blueprints[bp_name]['Imports']
                    extra = {'mesh': bp_imports[-mesh_index-1]['ObjectName'],
                             'mesh_package': package_for_import(bp_imports, mesh_index)}
                elif component_class == 'BoxComponent':
                    from foundation.road_scene import value
                    template = next(e for e in blueprints[bp_name]['Exports']
                                    if e.get('ObjectName') == component['ObjectName']+'_GEN_VARIABLE')
                    all_fields = {p['Name']: p.get('Value') for p in template.get('Data', [])}
                    all_fields.update({p['Name']: p.get('Value') for p in component.get('Data', [])})
                    if 'BoxExtent' not in all_fields:
                        raise ValueError('missing box extent')
                    extent = value(all_fields, 'BoxExtent', 'XYZ', [0, 0, 0])
                    if np.any(extent <= 0):
                        raise ValueError('invalid box extent')
                    extra = {'box_half_extents_cm': extent.tolist()}
                else:
                    raise ValueError('unsupported component class')
                chain, seen, node_index, owners = [], set(), index, []
                while node_index not in (None, 0):
                    if not isinstance(node_index, int) or not 0 < node_index <= len(exports) or node_index in seen:
                        raise ValueError('invalid or cyclic attachment')
                    seen.add(node_index)
                    node = exports[node_index-1]
                    owner_index = node.get('OuterIndex')
                    if not owners or owners[-1]['export_index'] != owner_index:
                        owner = exports[owner_index-1]
                        owners.append({'export_index': owner_index, 'actor': owner.get('ObjectName'),
                                       'class': class_name(owner)})
                    default, _ = template_for(node)
                    fields = {**default, **props(node)}
                    chain.append(fields)
                    node_index = fields.get('AttachParent')
                transform = (np.zeros(3), Rotation.identity(), np.ones(3))
                for fields in reversed(chain):
                    transform = compose(transform, relative(fields))
                t, r, s = transform
                placements.append({'actor': actor.get('ObjectName'), 'component': component.get('ObjectName'),
                                   'export_index': index, **extra,
                                   'attachment_owners': owners,
                                   'translation_ue_cm': t.tolist(), 'quaternion_xyzw': r.as_quat().tolist(),
                                   'scale': s.tolist()})
            except (KeyError, IndexError, TypeError, ValueError) as exc:
                unresolved.append({'actor': actor.get('ObjectName'), 'component': component.get('ObjectName'),
                                   'reason': str(exc)})
    return {'status': 'U: candidate placements, not runtime verified', 'frame': 'UE centimetres',
            'actor_count': actor_count, 'placements': placements, 'unresolved': unresolved,
            'assumptions': ['serialized properties override owner Blueprint defaults',
                            'child actor attachment follows serialized component chain',
                            'construction scripts and runtime collision channels unverified']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('map', type=Path)
    p.add_argument('blueprints', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--actor-class', default='BP_Rack_C')
    p.add_argument('--component-prefix')
    p.add_argument('--component-class', choices=('StaticMeshComponent', 'BoxComponent'), default='StaticMeshComponent')
    a = p.parse_args()
    raw = a.map.read_bytes()
    files = list(a.blueprints.glob('*.json'))
    result = build(json.loads(raw), {f.stem: json.loads(f.read_bytes()) for f in files},
                   actor_class=a.actor_class, component_prefix=a.component_prefix, component_class=a.component_class)
    result['actor_class'] = a.actor_class
    result['component_prefix'] = a.component_prefix
    result['component_class'] = a.component_class
    result['source_sha256'] = {'map': hashlib.sha256(raw).hexdigest(),
                               'blueprints': {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in files}}
    a.output.write_text(json.dumps(result, separators=(',', ':'))+'\n', encoding='utf-8')
    print('actors', result['actor_count'], 'placements', len(result['placements']),
          'unresolved', len(result['unresolved']))
    print('reasons', collections.Counter(x['reason'] for x in result['unresolved']).most_common(8))


if __name__ == '__main__':
    main()
