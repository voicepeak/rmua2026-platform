"""Explicit drag-face candidate based on upstream AirSim, not calibrated.

The caller must supply face positions, outward normals, drag factors, density
and velocity cutoff. This module does not infer those parameters or wind zones.
"""
import numpy as np
from scipy.spatial.transform import Rotation
import hashlib
import json
from pathlib import Path


class DragFaces:
    def __init__(self, positions_body_m, normals_body, drag_factors, air_density, min_velocity):
        self.positions = np.asarray(positions_body_m, dtype=float)
        self.normals = np.asarray(normals_body, dtype=float)
        self.factors = np.asarray(drag_factors, dtype=float)
        if (self.positions.ndim != 2 or self.positions.shape[1] != 3 or len(self.positions) == 0
                or self.normals.shape != self.positions.shape or self.factors.shape != (len(self.positions),)
                or not np.isfinite(self.positions).all() or not np.isfinite(self.normals).all()
                or not np.isfinite(self.factors).all() or np.any(self.factors < 0)
                or not np.allclose(np.linalg.norm(self.normals, axis=1), 1., atol=1e-7)
                or not np.isfinite(air_density) or air_density <= 0
                or not np.isfinite(min_velocity) or min_velocity < 0):
            raise ValueError('invalid drag faces or parameters')
        self.positions = self.positions.copy()
        self.normals = self.normals.copy()
        self.factors = self.factors.copy()
        self.air_density = float(air_density)
        self.min_velocity = float(min_velocity)

    @classmethod
    def from_observed_snapshot(cls, path, *, air_density, min_velocity):
        """Load initialized candidate faces; density/cutoff remain caller inputs."""
        raw = Path(path).read_bytes()
        snapshot = json.loads(raw)
        expected = '5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800'
        faces = snapshot['faces']
        if (snapshot.get('binary_sha256') != expected or len(faces) != 6
                or [face.get('index') for face in faces] != list(range(6))
                or not all(face.get('current_matches_initial') is True for face in faces)):
            raise ValueError('snapshot is not an initialized six-face candidate for the verified build')
        model = cls([f['initial_position_m'] for f in faces], [f['initial_normal'] for f in faces],
                    [f['candidate_drag_factor'] for f in faces], air_density, min_velocity)
        model.snapshot_sha256 = hashlib.sha256(raw).hexdigest()
        return model

    def wrench(self, rotation, velocity_world, omega_body, wind_world):
        if not isinstance(rotation, Rotation) or len(rotation.as_quat().shape) != 1:
            raise ValueError('expected one body-to-world rotation')
        v, w, wind = (np.asarray(x, dtype=float) for x in (velocity_world, omega_body, wind_world))
        if any(x.shape != (3,) or not np.isfinite(x).all() for x in (v, w, wind)):
            raise ValueError('expected finite velocity vectors')
        relative_body = rotation.inv().apply(v-wind)
        face_velocity = relative_body + np.cross(np.broadcast_to(w, self.positions.shape), self.positions)
        component = np.einsum('ij,ij->i', self.normals, face_velocity)
        magnitude = np.where(component > self.min_velocity,
                             -self.factors * self.air_density * component**2, 0.)
        forces_body = self.normals * magnitude[:, None]
        force_world = rotation.apply(forces_body.sum(axis=0))
        torque_body = np.cross(self.positions, forces_body).sum(axis=0)
        return force_world, torque_body
