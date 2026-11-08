"""Queryable serialized box-component candidates, not official zone membership."""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from foundation.frames import ned_m_to_ue_cm


class CandidateBoxVolumes:
    def __init__(self, placements):
        self.placements = list(placements)
        if not self.placements:
            raise ValueError('empty volume set')
        self.centers = np.asarray([x['translation_ue_cm'] for x in placements], float)
        self.scales = np.asarray([x['scale'] for x in placements], float)
        self.extents = np.asarray([x['box_half_extents_cm'] for x in placements], float)
        self.rotations = Rotation.from_quat([x['quaternion_xyzw'] for x in placements])
        if (self.centers.shape != (len(placements), 3) or self.scales.shape != self.centers.shape
                or self.extents.shape != self.centers.shape or not np.isfinite([self.centers, self.scales, self.extents]).all()
                or np.any(self.scales <= 0) or np.any(self.extents <= 0)):
            raise ValueError('invalid volume transforms')

    @classmethod
    def load(cls, source):
        source = Path(source)
        raw = source.read_bytes()
        data = json.loads(raw)
        if data['unresolved']:
            raise ValueError('unresolved box volumes')
        result = cls(data['placements'])
        result.source_sha256 = hashlib.sha256(raw).hexdigest()
        return result

    def _signed_ue(self, point):
        point = np.asarray(point, float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError('invalid query point')
        # Rotation gives axes; scaling the half extents makes distance physical
        # in UE centimetres even for nonuniform component scale.
        local = self.rotations.inv().apply(point-self.centers)
        delta = np.abs(local) - self.extents*self.scales
        outside = np.linalg.norm(np.maximum(delta, 0), axis=1)
        return outside + np.minimum(np.max(delta, axis=1), 0)

    def query_ue(self, point):
        signed = self._signed_ue(point)
        return [{'actor': x['actor'], 'component': x['component'],
                 'signed_distance_cm': float(signed[i]), 'contains': bool(signed[i] <= 0)}
                for i, x in enumerate(self.placements)]

    def query_ned(self, point, *, origin_ue_cm):
        return self.query_ue(ned_m_to_ue_cm(point, origin_ue_cm))

    def containing_ned(self, point, *, origin_ue_cm):
        signed = self._signed_ue(ned_m_to_ue_cm(point, origin_ue_cm))
        return [{'actor': self.placements[i]['actor'], 'component': self.placements[i]['component']}
                for i in np.flatnonzero(signed <= 0)]


class CandidateRegionUnion:
    """Opt-in point-based safe-area approximation; not an official predicate."""
    def __init__(self, groups, *, origin_ue_cm):
        self.groups = dict(groups)
        self.origin = np.asarray(origin_ue_cm, float)
        if not self.groups or self.origin.shape != (3,) or not np.isfinite(self.origin).all():
            raise ValueError('invalid region union')
        if not all(isinstance(group, CandidateBoxVolumes) for group in self.groups.values()):
            raise ValueError('invalid volume group')

    @classmethod
    def from_candidates(cls, road, port, factory, *, origin_ue_cm):
        roads = CandidateBoxVolumes.load(road)
        charge = [x for x in roads.placements if x['component'] == 'ChargeArea']
        return cls({'road': CandidateBoxVolumes(charge), 'port': CandidateBoxVolumes.load(port),
                    'factory': CandidateBoxVolumes.load(factory)}, origin_ue_cm=origin_ue_cm)

    def evaluate(self, position_ned_m, *, rotation=None):
        matches = {}
        for name, group in self.groups.items():
            found = group.containing_ned(position_ned_m, origin_ue_cm=self.origin)
            if found:
                matches[name] = found
        return {'flag': int(bool(matches)), 'matches': matches,
                'status': 'candidate point-based union; official membership unverified'}


class CandidateSphereRegionUnion(CandidateRegionUnion):
    """Opt-in candidate sphere overlap using explicit UE-local offset and radius.

    Uses an axis-aligned UE/NED reflection and assumes unit component scale;
    the Blueprint component's runtime role and the coordinate mapping are not
    established by this implementation.
    """
    def __init__(self, groups, *, origin_ue_cm, offset_ue_cm, radius_cm):
        super().__init__(groups, origin_ue_cm=origin_ue_cm)
        self.offset = np.asarray(offset_ue_cm, float)
        self.radius = float(radius_cm)
        if self.offset.shape != (3,) or not np.isfinite(self.offset).all() or not np.isfinite(self.radius) or self.radius <= 0:
            raise ValueError('invalid sphere geometry')

    @classmethod
    def from_candidates(cls, road, port, factory, *, origin_ue_cm, offset_ue_cm, radius_cm):
        roads = CandidateBoxVolumes.load(road)
        charge = [x for x in roads.placements if x['component'] == 'ChargeArea']
        return cls({'road': CandidateBoxVolumes(charge), 'port': CandidateBoxVolumes.load(port),
                    'factory': CandidateBoxVolumes.load(factory)}, origin_ue_cm=origin_ue_cm,
                   offset_ue_cm=offset_ue_cm, radius_cm=radius_cm)

    def evaluate(self, position_ned_m, *, rotation=None):
        if not isinstance(rotation, Rotation) or not rotation.single:
            raise ValueError('single NED/FRD rotation required for sphere offset')
        reflection = np.diag([1., 1., -1.])
        root = ned_m_to_ue_cm(position_ned_m, self.origin)
        center = root + reflection@rotation.as_matrix()@reflection@self.offset
        matches = {}
        for name, group in self.groups.items():
            signed = group._signed_ue(center)
            indices = np.flatnonzero(signed <= self.radius)
            if len(indices):
                matches[name] = [{'actor':group.placements[i]['actor'],
                                  'component':group.placements[i]['component']} for i in indices]
        return {'flag': int(bool(matches)), 'matches': matches, 'sphere_center_ue_cm':center.tolist(),
                'status': 'candidate sphere-based union; official membership unverified'}
