"""Provisional UE-world road mesh placement and finite-segment ray queries.

Attachment chains and Blueprint template defaults are composed as UE FTransforms
(positive scale only). This is partial static geometry; the placement, collision
channels and runtime construction remain unverified. Input/output coordinates
are UE centimetres, not AirSim NED. No free-space guarantees are implied.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def value(fields, key, keys, default):
    if key not in fields:
        return np.array(default, dtype=float)
    raw = fields[key]
    if not isinstance(raw, list) or len(raw) != 1 or not isinstance(raw[0].get('Value'), dict):
        raise ValueError(f'unsupported {key} serialization')
    x = np.array([float(raw[0]['Value'][k]) for k in keys], dtype=float)
    if not np.isfinite(x).all():
        raise ValueError(f'nonfinite {key}')
    return x


def relative(fields):
    if any(fields.get(k) not in (None, False, 0) for k in
           ('bAbsoluteLocation', 'bAbsoluteRotation', 'bAbsoluteScale')):
        raise ValueError('absolute component transforms not supported')
    t = value(fields, 'RelativeLocation', 'XYZ', [0, 0, 0])
    pitch, yaw, roll = value(fields, 'RelativeRotation', ('Pitch', 'Yaw', 'Roll'), [0, 0, 0])
    # UE Rotator: yaw about +Z, pitch about -Y, roll about -X.
    r = Rotation.from_euler('ZYX', [yaw, -pitch, -roll], degrees=True)
    s = value(fields, 'RelativeScale3D', 'XYZ', [1, 1, 1])
    if np.any(s <= 0):
        raise ValueError('negative/zero scale needs UE matrix decomposition')
    return t, r, s


def compose(parent, child):
    pt, pr, ps = parent
    ct, cr, cs = child
    return pt + pr.apply(ps * ct), pr * cr, ps * cs


def template_fields(path, name):
    data = json.loads(path.read_text(encoding='utf-8'))
    matches = [e for e in data['Exports'] if e.get('ObjectName') == name]
    if len(matches) != 1:
        raise ValueError(f'expected one {name} template')
    return {p['Name']: p.get('Value') for p in matches[0].get('Data', [])}


def instances(chains, templates):
    out = []
    for actor in chains['actors']:
        if actor['unresolved_parent'] is not None or not actor['chain']:
            raise ValueError('unresolved attachment chain')
        transform = (np.zeros(3), Rotation.identity(), np.ones(3))
        for index in reversed(range(len(actor['chain']))):
            fields = dict(actor['chain'][index]['fields'])
            if index == 0:
                fields = {**templates[actor['class']], **fields}
            transform = compose(transform, relative(fields))
        t, r, s = transform
        out.append({'actor': actor['actor'], 'export_index': actor['export_index'],
                    'mesh': 'Segment' if actor['class'] == 'BP_Segment_Rot_C' else 'Segment_Half',
                    'translation_ue_cm': t.tolist(), 'quaternion_xyzw': r.as_quat().tolist(),
                    'scale': s.tolist()})
    return out


def segment_intersections(start, end, vertices, triangles):
    """Return segment fractions against triangles; both faces count as hits."""
    start, end = np.asarray(start, float), np.asarray(end, float)
    if start.shape != (3,) or end.shape != (3,) or not np.isfinite([start, end]).all():
        raise ValueError('invalid segment')
    direction = end - start
    if np.linalg.norm(direction) < 1e-12:
        raise ValueError('zero-length segment')
    tri = vertices[triangles]
    e1, e2 = tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]
    h = np.cross(np.broadcast_to(direction, e2.shape), e2)
    det = np.einsum('ij,ij->i', e1, h)
    valid = np.abs(det) > 1e-10
    inv = np.divide(1., det, out=np.zeros_like(det), where=valid)
    v = start - tri[:, 0]
    u = inv * np.einsum('ij,ij->i', v, h)
    q = np.cross(v, e1)
    w = inv * np.einsum('j,ij->i', direction, q)
    fraction = inv * np.einsum('ij,ij->i', e2, q)
    mask = valid & (u >= -1e-9) & (w >= -1e-9) & (u+w <= 1+1e-9) & (fraction >= 0) & (fraction <= 1)
    return fraction[mask]


class RoadScene:
    def __init__(self, placements, meshes):
        self.placements, self.meshes = placements, meshes

    def raycast(self, start, end):
        start, end = np.asarray(start, float), np.asarray(end, float)
        if (start.shape != (3,) or end.shape != (3,) or not np.isfinite([start, end]).all()
                or np.linalg.norm(end-start) < 1e-12):
            raise ValueError('invalid or zero-length segment')
        hits = []
        for obj in self.placements:
            r = Rotation.from_quat(obj['quaternion_xyzw'])
            t, scale = np.asarray(obj['translation_ue_cm']), np.asarray(obj['scale'])
            a, b = r.inv().apply(start-t)/scale, r.inv().apply(end-t)/scale
            vertices, triangles = self.meshes[obj['mesh']]
            bounds_min, bounds_max = vertices.min(axis=0), vertices.max(axis=0)
            if np.any(np.maximum(a, b) < bounds_min) or np.any(np.minimum(a, b) > bounds_max):
                continue
            fractions = segment_intersections(a, b, vertices, triangles)
            if len(fractions):
                f = float(fractions.min())
                hits.append({'actor': obj['actor'], 'mesh': obj['mesh'], 'fraction': f,
                             'point_ue_cm': (start + f*(end-start)).tolist()})
        return sorted(hits, key=lambda h: h['fraction'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path, help='asset_analysis directory')
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    paths = {name: args.directory / name for name in
             ('OL1_road_chains.json', 'BP_Segment_Rot.json', 'BP_Segment_Half.json')}
    chains = json.loads(paths['OL1_road_chains.json'].read_text(encoding='utf-8'))
    templates = {'BP_Segment_Rot_C': template_fields(paths['BP_Segment_Rot.json'], 'Segment_GEN_VARIABLE'),
                 'BP_Segment_Half_C': template_fields(paths['BP_Segment_Half.json'], 'Segment_Half_GEN_VARIABLE')}
    result = {'status': 'U: static candidate, not runtime verified', 'frame': 'UE world, centimetres',
              'source_sha256': {name: hashlib.sha256(p.read_bytes()).hexdigest() for name, p in paths.items()},
              'assumptions': ['missing instance fields inherit blueprint template or identity',
                              'attachment chains use UE positive-scale FTransform composition',
                              'unserialized absolute flags are false',
                              'no dynamic construction override or collision channel validation'],
              'instances': instances(chains, templates)}
    args.output.write_text(json.dumps(result, separators=(',', ':'))+'\n', encoding='utf-8')
    print('candidate road placements:', len(result['instances']))


if __name__ == '__main__':
    main()
