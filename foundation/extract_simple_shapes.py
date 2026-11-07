"""Extract serialized convex shapes without assuming their runtime use."""
import argparse
import hashlib
import json
from pathlib import Path
from foundation.extract_physx_mesh import body_setups


def extract(source):
    raw = source.read_bytes()
    asset = json.loads(raw)
    bodies = body_setups(asset)
    if len(bodies) != 1:
        return {'source_sha256': hashlib.sha256(raw).hexdigest(), 'status': 'missing or ambiguous BodySetup', 'convex': []}
    body = bodies[0]
    flag = next((p.get('EnumValue') for p in body.get('Data', []) if p.get('Name') == 'CollisionTraceFlag'), None)
    arrays = [p for p in body.get('Data', []) if p.get('Name') == 'AggGeom']
    shapes, boxes, unsupported = [], [], []
    for geom in arrays:
        for group in geom.get('Value', []):
            if group.get('Name') == 'BoxElems':
                for entry in group.get('Value', []):
                    fields = {p['Name']: p for p in entry.get('Value', [])}
                    center = fields['Center']['Value'][0]['Value']
                    rotation = fields['Rotation']['Value'][0]['Value']
                    size = [float(fields[k]['Value']) for k in 'XYZ']
                    if any(v <= 0 for v in size):
                        raise ValueError(f'invalid box geometry in {source}')
                    boxes.append({'center_local_ue_cm': [float(center[k]) for k in 'XYZ'],
                                  'rotation_ue_deg': [float(rotation[k]) for k in ('Pitch', 'Yaw', 'Roll')],
                                  'size_ue_cm': size})
                continue
            if group.get('Name') != 'ConvexElems':
                if group.get('Value'):
                    unsupported.append(group.get('Name'))
                continue
            for entry in group.get('Value', []):
                fields = {p['Name']: p for p in entry.get('Value', [])}
                vertices = [[float(v['Value'][0]['Value'][axis]) for axis in 'XYZ']
                            for v in fields['VertexData']['Value']]
                indices = [int(v['Value']) for v in fields['IndexData']['Value']]
                if not vertices or len(indices) % 3 or any(i < 0 or i >= len(vertices) for i in indices):
                    raise ValueError(f'invalid convex geometry in {source}')
                shapes.append({'vertices_local_ue_cm': vertices, 'triangle_indices': indices,
                               'element_transform_raw': fields.get('Transform')})
    return {'source_sha256': hashlib.sha256(raw).hexdigest(), 'status': 'extracted',
            'collision_trace_flag': flag, 'convex': shapes, 'boxes': boxes, 'unsupported_shapes': unsupported}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = {'assets': {f.stem: extract(f) for f in sorted(a.directory.glob('*.json'))},
              'limitations': ['local element transforms not applied', 'missing collision flag means unknown',
                              'not proof of collision channels or runtime instances']}
    a.output.write_text(json.dumps(result, separators=(',', ':'))+'\n', encoding='utf-8')
    print('assets:', len(result['assets']), 'convex elements:',
          sum(len(v['convex']) for v in result['assets'].values()))


if __name__ == '__main__':
    main()
