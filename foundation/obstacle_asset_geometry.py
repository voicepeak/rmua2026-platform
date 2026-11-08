"""Derive conservative local obstacle bounds from serialized convex assets.

Bounds are component-parent-local, not verified world instances or channels.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from foundation.blueprint_mesh_refs import package_for_import
from foundation.road_instances import props
from foundation.road_scene import relative
from foundation.simple_collision_mesh import simple_mesh


def build(meshes, blueprints):
    result = {}
    for asset, blueprint_name, component_name in (
            ('thinline', 'BP_ThinLine', 'StaticMesh'),
            ('thickline', 'BP_ThickLine', 'StaticMesh'),
            ('HoverTruck', 'BP_Hover_1', 'Hover')):
        bp = blueprints[blueprint_name]
        templates = [e for e in bp['Exports'] if e.get('ObjectName') == component_name+'_GEN_VARIABLE']
        if len(templates) != 1:
            raise ValueError('missing or ambiguous component template')
        fields = props(templates[0])
        index = fields.get('StaticMesh')
        if not isinstance(index, int) or index >= 0 or bp['Imports'][-index-1]['ObjectName'].lower() != asset.lower():
            raise ValueError('unexpected Blueprint mesh binding')
        source = meshes[asset]['convex_elements']
        simple = {'convex': [{'vertices_local_ue_cm': x['vertices_local'],
                              'triangle_indices': x['triangle_indices'],
                              'element_transform_raw': x['element_transform_raw']} for x in source]}
        vertices, triangles = simple_mesh(simple)
        translation, rotation, scale = relative(fields)
        transformed = rotation.apply(vertices*scale)+translation
        lo, hi = transformed.min(axis=0), transformed.max(axis=0)
        row = {'mesh_package': package_for_import(bp['Imports'], index),
               'blueprint': blueprint_name, 'component': component_name,
               'frame': 'component parent, UE cm', 'vertex_count': len(vertices),
               'triangle_count': len(triangles), 'bounds_min_cm': lo.tolist(),
               'bounds_max_cm': hi.tolist(), 'bounds_center_cm': ((lo+hi)/2).tolist(),
               'bounds_half_extents_m': ((hi-lo)/200).tolist(),
               'transformed_vertices_cm': transformed.tolist(), 'triangles': triangles.tolist()}
        if asset != 'HoverTruck':
            radial = np.linalg.norm(transformed[:, :2], axis=1)
            row['conservative_capsule'] = {'axis': 'parent local Z',
                                          'start_cm': [0., 0., float(lo[2])],
                                          'end_cm': [0., 0., float(hi[2])],
                                          'radius_m': float(radial.max()/100)}
        result[asset] = row
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('meshes', type=Path)
    p.add_argument('blueprints', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    files = [a.meshes] + [a.blueprints/(name+'.json') for name in ('BP_ThinLine', 'BP_ThickLine', 'BP_Hover_1')]
    raw = [f.read_bytes() for f in files]
    blueprints = {f.stem: json.loads(data) for f, data in zip(files[1:], raw[1:])}
    result = {'status': 'serialized parent-local candidate bounds; runtime collision unverified',
              'source_sha256': {f.name: hashlib.sha256(data).hexdigest() for f, data in zip(files, raw)},
              'assets': build(json.loads(raw[0]), blueprints),
              'limitations': ['parent component and actor instance transforms are not composed',
                              'collision profile, channel, effective trace mode and runtime behavior unknown',
                              'capsule and box conservatively bound convex vertices and may overestimate collision']}
    a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    for name, row in result['assets'].items():
        print(name, 'half extents m', row['bounds_half_extents_m'],
              'capsule radius m', row.get('conservative_capsule', {}).get('radius_m'))


if __name__ == '__main__':
    main()
