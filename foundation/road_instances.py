"""Inventory OL1 road actors and their unresolved component attachment chains.

No world matrices are asserted: defaults, inherited Blueprint values, parent
attachments, and UE rotator conventions must be validated before placement.
"""
import argparse
import hashlib
import json
from pathlib import Path

FIELDS = ('RelativeLocation', 'RelativeRotation', 'RelativeScale3D', 'AttachParent',
          'bAbsoluteLocation', 'bAbsoluteRotation', 'bAbsoluteScale', 'StaticMesh')


def props(export):
    return {p['Name']: p.get('Value') for p in (export.get('Data') or []) if p.get('Name') in FIELDS}


def build(asset, digest):
    exports, imports = asset['Exports'], asset['Imports']
    actors = []
    for i, obj in enumerate(exports, 1):
        ci = obj.get('ClassIndex', 0)
        if not isinstance(ci, int) or ci >= 0 or -ci > len(imports):
            continue
        class_name = imports[-ci-1].get('ObjectName')
        if class_name not in ('BP_Segment_Rot_C', 'BP_Segment_Half_C'):
            continue
        items = {p['Name']: p.get('Value') for p in (obj.get('Data') or [])}
        component_name = 'Segment' if class_name == 'BP_Segment_Rot_C' else 'Segment_Half'
        node = items.get(component_name)
        chain, seen = [], set()
        while isinstance(node, int) and node > 0 and node <= len(exports) and node not in seen:
            seen.add(node)
            entry = exports[node-1]
            fields = props(entry)
            chain.append({'export_index': node, 'name': entry.get('ObjectName'), 'fields': fields})
            node = fields.get('AttachParent')
        actors.append({'export_index': i, 'actor': obj.get('ObjectName'), 'class': class_name,
                       'root_component': items.get('RootComponent'), 'chain': chain,
                       'unresolved_parent': node if node not in (None, 0) else None})
    return {'source_sha256': digest, 'coordinate_scope': 'uncomposed UE attachment chains',
            'limitations': ['inherited Blueprint component defaults are not resolved here',
                            'absolute flags, component/world transforms and collision channels need runtime validation',
                            'other map assets are not included'], 'actors': actors}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    raw = a.source.read_bytes()
    result = build(json.loads(raw), hashlib.sha256(raw).hexdigest())
    a.output.write_text(json.dumps(result, separators=(',', ':'), ensure_ascii=False) + '\n', encoding='utf-8')
    print('road actors:', len(result['actors']))


if __name__ == '__main__':
    main()
