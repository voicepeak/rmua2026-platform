"""Candidate detection-gate components from the runtime OL1 score audit.

These are runtime root transforms of 33+30 components associated with the two
stage-1 scoring splines. They are not the 12 named solid route gates, not
verified trigger volumes, and not official scoring evidence.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from foundation.frames import ned_m_to_ue_cm


class DetectionGateCandidates:
    def __init__(self, groups, *, source_sha256, axis, half_extents_cm):
        if axis not in (0, 1, 2):
            raise ValueError('axis must be 0, 1 or 2')
        extents = np.asarray(half_extents_cm, float)
        if extents.shape != (3,) or not np.isfinite(extents).all() or np.any(extents <= 0):
            raise ValueError('half_extents_cm must be three positive values')
        if not isinstance(source_sha256, dict) or not source_sha256:
            raise ValueError('source hashes are required')
        self.axis = axis
        self.half_extents = extents.copy()
        self.source_sha256 = dict(source_sha256)
        self.groups = []
        for name, gates in groups.items():
            records = []
            for gate in gates:
                position = np.asarray(gate['position_cm'], float)
                quaternion = np.asarray(gate['quaternion_xyzw'], float)
                scale = np.asarray(gate['scale'], float)
                if (position.shape != (3,) or quaternion.shape != (4,) or scale.shape != (3,)
                        or not np.isfinite(np.r_[position, quaternion, scale]).all()
                        or np.linalg.norm(quaternion) < 1e-12 or np.any(scale <= 0)):
                    raise ValueError('invalid detection gate transform')
                records.append({
                    'index': int(gate['index']),
                    'actor': gate.get('actor'),
                    'root': gate.get('root'),
                    'fname': gate.get('fname'),
                    'position_ue_cm': position,
                    'rotation': Rotation.from_quat(quaternion),
                    'scale': scale,
                })
            self.groups.append((name, records))

    @classmethod
    def load(cls, path, *, axis, half_extents_cm):
        path = Path(path)
        raw = path.read_bytes()
        data = json.loads(raw)
        groups = {name: entry['gates'] for name, entry in data['groups'].items()}
        result = cls(groups, source_sha256=data['source_sha256'], axis=axis,
                     half_extents_cm=half_extents_cm)
        result.manifest_sha256 = hashlib.sha256(raw).hexdigest()
        result.manifest_path = str(path)
        return result

    def crossings_ue(self, start, end):
        start, end = np.asarray(start, float), np.asarray(end, float)
        if start.shape != (3,) or end.shape != (3,) or not np.isfinite([start, end]).all():
            raise ValueError('invalid crossing segment')
        axis = self.axis
        transverse = [i for i in range(3) if i != axis]
        hits = []
        for group, gates in self.groups:
            for gate in gates:
                rotation, translation, scale = gate['rotation'], gate['position_ue_cm'], gate['scale']
                a, b = rotation.inv().apply(start-translation)/scale, rotation.inv().apply(end-translation)/scale
                d0, d1 = a[axis], b[axis]
                if not ((d0 < 0 <= d1) or (d0 > 0 >= d1)):
                    continue
                alpha = -d0/(d1-d0)
                local = a + alpha*(b-a)
                if not all(abs(local[i]) <= self.half_extents[i] for i in transverse):
                    continue
                hits.append({'group': group, 'index': gate['index'], 'actor': gate['actor'],
                             'root': gate['root'], 'fraction': float(alpha),
                             'direction': 1 if d1 > d0 else -1,
                             'point_ue_cm': (start+alpha*(end-start)).tolist(),
                             'evidence': 'candidate detection-gate center plane; trigger volume and official scoring unverified'})
        return sorted(hits, key=lambda hit: hit['fraction'])

    def crossings_ned(self, start, end, *, origin_ue_cm):
        return self.crossings_ue(ned_m_to_ue_cm(start, origin_ue_cm),
                                 ned_m_to_ue_cm(end, origin_ue_cm))
