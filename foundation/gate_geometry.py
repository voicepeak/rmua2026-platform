"""Candidate gate-mesh center-plane crossings; not official gate scoring.

The crossing axis must be supplied explicitly: neither a mesh's orientation nor
its box bounds establish the official course direction or trigger semantics.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from foundation.frames import ned_m_to_ue_cm
from foundation.simple_collision_mesh import simple_mesh


class GateMeshVolumes:
    def __init__(self, placements, bounds, *, crossing_axis):
        if crossing_axis not in (0, 1, 2):
            raise ValueError('crossing_axis must be 0, 1 or 2')
        self.axis = crossing_axis
        self.low, self.high = (np.asarray(x, float) for x in bounds)
        if (self.low.shape != (3,) or self.high.shape != (3,)
                or not np.isfinite([self.low, self.high]).all() or np.any(self.low >= self.high)):
            raise ValueError('invalid local mesh bounds')
        self.gates = []
        for obj in placements:
            if obj['mesh_package'] != '/Engine/EngineMeshes/Cube':
                raise ValueError('unexpected gate mesh')
            t = np.asarray(obj['translation_ue_cm'], float)
            s = np.asarray(obj['scale'], float)
            if t.shape != (3,) or s.shape != (3,) or not np.isfinite([t, s]).all() or np.any(s <= 0):
                raise ValueError('invalid gate transform')
            rotation = Rotation.from_quat(obj['quaternion_xyzw'])
            self.gates.append((obj['actor'], t, rotation, s))

    @classmethod
    def load(cls, directory, *, crossing_axis):
        directory = Path(directory)
        paths = [directory/'OL1_gate_mesh_placements_candidate.json',
                 directory/'OL1_simple_convex_candidates.json']
        placements = json.loads(paths[0].read_text(encoding='utf-8'))
        if placements['unresolved']:
            raise ValueError('unresolved gate transforms')
        asset = json.loads(paths[1].read_text(encoding='utf-8'))['assets']['Engine_EngineMeshes_Cube']
        vertices, _ = simple_mesh(asset)
        result = cls(placements['placements'], (vertices.min(axis=0), vertices.max(axis=0)),
                     crossing_axis=crossing_axis)
        result.source_sha256 = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
        return result

    @classmethod
    def load_serialized_candidates(cls, map_directory, assets, *, crossing_axis):
        """Load a map's own gate candidates and require matching road provenance."""
        map_directory, assets = Path(map_directory), Path(assets)
        gate_path = map_directory/'gate_candidates.json'
        road_path = map_directory/'road_candidates.json'
        shape_path = assets/'OL1_simple_convex_candidates.json'
        gate, road, shapes = (json.loads(p.read_bytes()) for p in (gate_path, road_path, shape_path))
        if (gate['unresolved'] or not gate['placements'] or gate.get('actor_class') != 'BP_Gate_C'
                or gate['source_sha256']['map'] != road['source_sha256']['map_json']
                or gate['source_sha256']['blueprints'].get('BP_Gate.json')
                   != road['source_sha256']['building_blueprints'].get('BP_Gate.json')):
            raise ValueError('mismatched or unresolved gate candidates')
        vertices, _ = simple_mesh(shapes['assets']['Engine_EngineMeshes_Cube'])
        result = cls(gate['placements'], (vertices.min(axis=0), vertices.max(axis=0)),
                     crossing_axis=crossing_axis)
        result.source_sha256 = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in (gate_path, road_path, shape_path)}
        return result

    def crossings_ue(self, start, end):
        start, end = np.asarray(start, float), np.asarray(end, float)
        if start.shape != (3,) or end.shape != (3,) or not np.isfinite([start, end]).all():
            raise ValueError('invalid crossing segment')
        hits = []
        axis = self.axis
        mid = (self.low[axis] + self.high[axis])/2
        transverse = [i for i in range(3) if i != axis]
        for name, t, r, s in self.gates:
            a, b = r.inv().apply(start-t)/s, r.inv().apply(end-t)/s
            d0, d1 = a[axis]-mid, b[axis]-mid
            # A half-open interval avoids duplicate events at substep boundaries.
            if not ((d0 < 0 <= d1) or (d0 > 0 >= d1)):
                continue
            alpha = -d0/(d1-d0)
            local = a + alpha*(b-a)
            if all(self.low[i] <= local[i] <= self.high[i] for i in transverse):
                hits.append({'gate': name, 'fraction': float(alpha),
                             'direction': 1 if d1 > d0 else -1,
                             'point_ue_cm': (start+alpha*(end-start)).tolist(),
                             'evidence': 'candidate mesh center plane; official trigger unverified'})
        return sorted(hits, key=lambda h: h['fraction'])

    def crossings_ned(self, start, end, *, origin_ue_cm):
        return self.crossings_ue(ned_m_to_ue_cm(start, origin_ue_cm),
                                 ned_m_to_ue_cm(end, origin_ue_cm))
