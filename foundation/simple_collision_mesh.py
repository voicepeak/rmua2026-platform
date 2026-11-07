"""Decode serialized UE simple shapes into candidate local triangles.

Presence does not establish that a component uses simple collision at runtime.
"""
import itertools
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation


def simple_mesh(asset):
    if asset.get('unsupported_shapes'):
        raise ValueError('unsupported simple collision element')
    vertices, triangles = [], []

    def add(points, faces):
        points, faces = np.asarray(points, float), np.asarray(faces, int)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('invalid vertices')
        if faces.ndim != 2 or faces.shape[1] != 3 or not len(faces) or faces.min() < 0 or faces.max() >= len(points):
            raise ValueError('invalid triangles')
        triangles.extend((faces + len(vertices)).tolist())
        vertices.extend(points.tolist())

    for shape in asset.get('convex', []):
        transform = shape.get('element_transform_raw')
        if not transform:
            raise ValueError('missing convex transform')
        fields = {item['Name']: item['Value'][0]['Value'] for item in transform['Value']}
        q = np.array([float(fields['Rotation'][k]) for k in 'XYZW'])
        t = np.array([float(fields['Translation'][k]) for k in 'XYZ'])
        s = np.array([float(fields['Scale3D'][k]) for k in 'XYZ'])
        if not np.isfinite(np.r_[q, t, s]).all() or np.linalg.norm(q) < 1e-12 or np.any(s <= 0):
            raise ValueError('invalid convex transform')
        points = Rotation.from_quat(q).apply(np.asarray(shape['vertices_local_ue_cm'], float) * s) + t
        indices = shape['triangle_indices']
        # Engine cube has vertices but no serialized index buffer. A convex
        # hull is a candidate reconstruction, not an observed cooked mesh.
        faces = np.asarray(indices, int).reshape(-1, 3) if indices else ConvexHull(points).simplices
        add(points, faces)

    for box in asset.get('boxes', []):
        size = np.asarray(box['size_ue_cm'], float)
        center = np.asarray(box['center_local_ue_cm'], float)
        pitch, yaw, roll = box['rotation_ue_deg']
        if size.shape != (3,) or center.shape != (3,) or not np.isfinite(np.r_[size, center]).all() or np.any(size <= 0):
            raise ValueError('invalid box')
        corners = np.array(list(itertools.product(*[(-x/2, x/2) for x in size])))
        points = Rotation.from_euler('ZYX', [yaw, -pitch, -roll], degrees=True).apply(corners) + center
        add(points, ConvexHull(points).simplices)
    if not vertices:
        raise ValueError('no simple shapes')
    return np.asarray(vertices), np.asarray(triangles, dtype=int)
