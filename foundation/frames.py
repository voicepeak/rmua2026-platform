"""Explicit axis-aligned candidate UE/NED position and attitude conversions.

These are axis-aligned transforms only; determine the actual origin and any
map rotation from official data before using them for simulator comparisons.
"""
import numpy as np


def ue_cm_to_ned_m(position, origin_ue_cm):
    position = np.asarray(position, dtype=float)
    origin = np.asarray(origin_ue_cm, dtype=float)
    if (position.shape[-1:] != (3,) or origin.shape != (3,)
            or not np.isfinite(position).all() or not np.isfinite(origin).all()):
        raise ValueError("position must end in 3 coordinates; origin must have 3")
    return (position - origin) * np.array([0.01, 0.01, -0.01])


def ned_m_to_ue_cm(position, origin_ue_cm):
    position = np.asarray(position, dtype=float)
    origin = np.asarray(origin_ue_cm, dtype=float)
    if (position.shape[-1:] != (3,) or origin.shape != (3,)
            or not np.isfinite(position).all() or not np.isfinite(origin).all()):
        raise ValueError("position must end in 3 coordinates; origin must have 3")
    return position * np.array([100.0, 100.0, -100.0]) + origin


def ue_body_to_ned_frd_quaternion_wxyz(quaternion):
    """Apply S R S to a body-to-world rotation, with S=diag(1,1,-1).

    Both UE world and UE body coordinates are reflected; callers must verify
    those assumed frames and any map yaw before applying this to official data.
    Input and output use scalar-first wxyz and may have leading batch axes.
    """
    q = np.asarray(quaternion, dtype=float)
    if q.shape[-1:] != (4,) or not np.isfinite(q).all():
        raise ValueError('quaternion must contain finite wxyz values')
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    if np.any(norm < 1e-12):
        raise ValueError('zero quaternion')
    return q / norm * np.array([1., -1., -1., 1.])


def ned_frd_to_ue_body_quaternion_wxyz(quaternion):
    """Inverse candidate transform; the same double reflection is involutive."""
    return ue_body_to_ned_frd_quaternion_wxyz(quaternion)
