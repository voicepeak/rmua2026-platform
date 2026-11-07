"""Inventory serialized OL1 building components and blueprint mesh bindings.

This is not world placement: templates, parent chains, construction scripts,
effective collision modes and instance overrides require further resolution.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path
from foundation.blueprint_mesh_refs import package_for_import
from foundation.road_instances import props


def build(asset, blueprints, source_sha256):
    exports, imports = asset['Exports'], asset['Imports']
    children = collections.defaultdict(list)
    for i, entry in enumerate(exports, 1):
        children[entry.get('OuterIndex')].append((i, entry))
    actor_rows = []
    for actor_index, actor in enumerate(exports, 1):
        cls_index = actor.get('ClassIndex', 0)
        if not isinstance(cls_index, int) or cls_index >= 0 or -cls_index > len(imports):
            continue
        class_name = imports[-cls_index-1].get('ObjectName', '')
        if not class_name.startswith('BP_Building_') or not class_name.endswith('_C'):
            continue
        bp_name = class_name[:-2]
        blueprint = blueprints.get(bp_name)
        if blueprint is None:
            actor_rows.append({'actor': actor.get('ObjectName'), 'class': class_name, 'status': 'missing blueprint', 'components': []})
            continue
        templates = {e.get('ObjectName'): e for e in blueprint.get('Exports', [])}
        actor_data = {p['Name']: p.get('Value') for p in actor.get('Data', [])}
        components = []
        for index, child in children[actor_index]:
            class_id = child.get('ClassIndex', 0)
            if not isinstance(class_id, int) or class_id >= 0 or -class_id > len(imports):
                continue
            if imports[-class_id-1].get('ObjectName') != 'StaticMeshComponent':
                continue
            component_name = child.get('ObjectName')
            template = templates.get(component_name + '_GEN_VARIABLE')
            template_props = props(template) if template else {}
            mesh_ref = template_props.get('StaticMesh')
            mesh = None
            if isinstance(mesh_ref, int) and mesh_ref < 0:
                mesh_imports = blueprint['Imports']
                if -mesh_ref <= len(mesh_imports):
                    mesh = {'name': mesh_imports[-mesh_ref-1].get('ObjectName'),
                            'package': package_for_import(mesh_imports, mesh_ref)}
            node, chain, seen = index, [], set()
            while isinstance(node, int) and 0 < node <= len(exports) and node not in seen:
                seen.add(node)
                obj = exports[node-1]
                fields = props(obj)
                chain.append({'export_index': node, 'name': obj.get('ObjectName'), 'fields': fields})
                node = fields.get('AttachParent')
            components.append({'export_index': index, 'name': component_name,
                               'template_found': template is not None, 'template_fields': template_props,
                               'mesh': mesh, 'chain': chain,
                               'unresolved_parent': node if node not in (None, 0) else None})
        actor_rows.append({'actor': actor.get('ObjectName'), 'export_index': actor_index, 'class': class_name,
                           'root_component': actor_data.get('RootComponent'), 'status': 'inventoried',
                           'components': components})
    return {'source_sha256': source_sha256, 'actors': actor_rows,
            'limitations': ['mesh bindings are template imports; instance override not resolved',
                            'component default inheritance and parent transforms uncomposed',
                            'runtime construction and collision channel not verified']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('map', type=Path)
    p.add_argument('blueprints', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    raw = a.map.read_bytes()
    blueprints = {f.stem: json.loads(f.read_bytes()) for f in a.blueprints.glob('*.json')}
    result = build(json.loads(raw), blueprints, hashlib.sha256(raw).hexdigest())
    a.output.write_text(json.dumps(result, separators=(',', ':'))+'\n', encoding='utf-8')
    print('actors', len(result['actors']), 'components', sum(len(x['components']) for x in result['actors']))


if __name__ == '__main__':
    main()
