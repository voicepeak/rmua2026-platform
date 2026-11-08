"""Inventory candidate PaperSprite components on OL1 buildings and sprite actors.

Transforms use the same provisional blueprint/attachment composition as the
building inventory. Sprite geometry and collision channels remain unknown.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from foundation.blueprint_mesh_refs import package_for_import
from foundation.road_instances import props
from foundation.road_scene import compose, relative


def build(asset, blueprints):
    exports, imports = asset['Exports'], asset['Imports']
    placements, unresolved = [], []
    for index, child in enumerate(exports, 1):
        cls = child.get('ClassIndex')
        if not isinstance(cls, int) or cls >= 0 or -cls > len(imports) or imports[-cls-1].get('ObjectName') != 'PaperSpriteComponent':
            continue
        owner = child.get('OuterIndex')
        if not isinstance(owner, int) or not 0 < owner <= len(exports):
            continue
        actor = exports[owner-1]
        actor_cls = actor.get('ClassIndex')
        if not isinstance(actor_cls, int) or actor_cls >= 0 or -actor_cls > len(imports):
            continue
        name = imports[-actor_cls-1].get('ObjectName', '')
        if name == 'PaperSpriteActor':
            try:
                fields = props(child)
                if fields.get('AttachParent') not in (None, 0):
                    raise ValueError('standalone sprite has a parent')
                t, r, s = relative(fields)
                ref = next((p.get('Value') for p in child.get('Data', []) if p.get('Name') == 'SourceSprite'), None)
                sprite = package_for_import(imports, ref) if isinstance(ref, int) and ref < 0 else None
                if sprite is None:
                    raise ValueError('missing SourceSprite')
                placements.append({'actor': actor['ObjectName'], 'component': child['ObjectName'],
                                   'export_index': index, 'sprite_package': sprite,
                                   'translation_ue_cm': t.tolist(), 'quaternion_xyzw': r.as_quat().tolist(),
                                   'scale': s.tolist(), 'transform_source': 'serialized standalone component'})
            except (KeyError, TypeError, ValueError) as exc:
                unresolved.append({'actor': actor.get('ObjectName'), 'component': child.get('ObjectName'), 'reason': str(exc)})
            continue
        bp = blueprints.get(name[:-2]) if name.endswith('_C') else None
        if bp is None:
            unresolved.append({'actor': actor.get('ObjectName'), 'component': child.get('ObjectName'), 'reason': 'missing blueprint'})
            continue
        template_exports = {e.get('ObjectName'): e for e in bp['Exports']}
        templates = {key: props(e) for key, e in template_exports.items()}
        try:
            if any(p.get('Name') == 'SourceSprite' for p in child.get('Data', [])):
                raise ValueError('instance SourceSprite override requires map import resolution')
            chain, seen, node = [], set(), index
            while isinstance(node, int) and 0 < node <= len(exports) and node not in seen:
                seen.add(node)
                current = exports[node-1]
                fields = props(current)
                template = templates.get(current['ObjectName']+'_GEN_VARIABLE')
                if template is None:
                    raise ValueError('missing template '+current['ObjectName'])
                chain.append({**template, **fields})
                node = fields.get('AttachParent')
            if node not in (None, 0):
                raise ValueError('unresolved or cyclic parent')
            transform = (np.zeros(3), Rotation.identity(), np.ones(3))
            for fields in reversed(chain):
                transform = compose(transform, relative(fields))
            template = template_exports[child['ObjectName']+'_GEN_VARIABLE']
            ref = next((p.get('Value') for p in template.get('Data', []) if p.get('Name') == 'SourceSprite'), None)
            sprite = package_for_import(bp['Imports'], ref) if isinstance(ref, int) and ref < 0 else None
            t, r, s = transform
            placements.append({'actor': actor['ObjectName'], 'component': child['ObjectName'],
                               'export_index': index, 'sprite_package': sprite,
                               'translation_ue_cm': t.tolist(), 'quaternion_xyzw': r.as_quat().tolist(),
                               'scale': s.tolist()})
        except (KeyError, TypeError, ValueError) as exc:
            unresolved.append({'actor': actor.get('ObjectName'), 'component': child.get('ObjectName'), 'reason': str(exc)})
    return {'placements': placements, 'unresolved': unresolved,
            'limitations': ['sprite geometry and runtime collision mode are not decoded',
                            'FTransform and template composition are candidate interpretations',
                            'construction overrides and runtime object identity are not confirmed']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('map', type=Path)
    p.add_argument('blueprints', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    raw = args.map.read_bytes()
    files = sorted(args.blueprints.glob('*.json'))
    result = build(json.loads(raw), {f.stem: json.loads(f.read_bytes()) for f in files})
    result['source_sha256'] = {'map': hashlib.sha256(raw).hexdigest(),
                               'blueprints': {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in files}}
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('placements', len(result['placements']), 'unresolved', len(result['unresolved']))


if __name__ == '__main__':
    main()
