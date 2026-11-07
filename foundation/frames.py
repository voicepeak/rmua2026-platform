"""Explicit origin-aware UE centimetre / NED metre position conversions.

These are axis-aligned transforms only; determine the actual origin and any
map rotation from official data before using them for simulator comparisons.
"""
import numpy as np


def ue_cm_to_ned_m(position, origin_ue_cm):
    position = np.asarray(position, dtype=float)
    origin = np.asarray(origin_ue_cm, dtype=float)
    if position.shape[-1:] != (3,) or origin.shape != (3,):
        raise ValueError("position must end in 3 coordinates; origin must have 3")
    return (position - origin) * np.array([0.01, 0.01, -0.01])


def ned_m_to_ue_cm(position, origin_ue_cm):
    position = np.asarray(position, dtype=float)
    origin = np.asarray(origin_ue_cm, dtype=float)
    if position.shape[-1:] != (3,) or origin.shape != (3,):
        raise ValueError("position must end in 3 coordinates; origin must have 3")
    return position * np.array([100.0, 100.0, -100.0]) + origin
