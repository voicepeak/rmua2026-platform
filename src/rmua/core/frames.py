"""Frame and unit conventions.

Internal convention (sim-fast):
    x = forward/north, y = right/east, z = up
    unit = meter
    yaw = radians, counter-clockwise around +z when viewed from above

Official rules:
    NED, origin at marker 1, meters.

The map export is UE4: centimeters, x/y/z, z up, yaw in degrees.
Use the helpers below at every boundary; never mix cm and m in the core.
"""

from __future__ import annotations

import math

import numpy as np

UE_CM_PER_M = 100.0
DEG = math.pi / 180.0
RAD = 180.0 / math.pi


def ue_cm_to_m(value):
    """Convert UE centimeters to meters (scalar or array)."""
    return np.asarray(value, dtype=np.float64) / UE_CM_PER_M


def m_to_ue_cm(value):
    return np.asarray(value, dtype=np.float64) * UE_CM_PER_M


def ue_to_ned(vec):
    """UE (x forward, y right, z up) to NED (north, east, down)."""
    vec = np.asarray(vec, dtype=np.float64)
    if vec.shape[-1] != 3:
        raise ValueError("last dimension must be 3")
    return np.stack((vec[..., 0], vec[..., 1], -vec[..., 2]), axis=-1)


def ned_to_ue(vec):
    """NED (north, east, down) to UE (x forward, y right, z up)."""
    vec = np.asarray(vec, dtype=np.float64)
    if vec.shape[-1] != 3:
        raise ValueError("last dimension must be 3")
    return np.stack((vec[..., 0], vec[..., 1], -vec[..., 2]), axis=-1)


def wrap_pi(angle):
    """Wrap radians to [-pi, pi)."""
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


def wrap_deg(angle):
    return (np.asarray(angle) + 180.0) % 360.0 - 180.0


def yaw_deg_to_forward(yaw_deg):
    """UE yaw (degrees) to unit forward vector in the x/y plane."""
    yaw = np.asarray(yaw_deg, dtype=np.float64) * DEG
    return np.stack((np.cos(yaw), np.sin(yaw), np.zeros_like(yaw)), axis=-1)


def yaw_deg_to_right(yaw_deg):
    yaw = np.asarray(yaw_deg, dtype=np.float64) * DEG
    return np.stack((-np.sin(yaw), np.cos(yaw), np.zeros_like(yaw)), axis=-1)


def yaw_deg_to_up(yaw_deg):
    yaw = np.asarray(yaw_deg, dtype=np.float64) * 0.0
    return np.stack((np.zeros_like(yaw), np.zeros_like(yaw), np.ones_like(yaw)), axis=-1)


def rotation_z(yaw_rad):
    """Rotation matrix about +z. Works for scalars or (...,) arrays."""
    yaw = np.asarray(yaw_rad, dtype=np.float64)
    c = np.cos(yaw)
    s = np.sin(yaw)
    shape = yaw.shape + (3, 3)
    out = np.zeros(shape, dtype=np.float64)
    out[..., 0, 0] = c
    out[..., 0, 1] = -s
    out[..., 1, 0] = s
    out[..., 1, 1] = c
    out[..., 2, 2] = 1.0
    return out


def horizontal_distance(a, b):
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    return np.linalg.norm(a[..., :2] - b[..., :2], axis=-1)


def safe_norm(vec, eps=1e-9):
    return np.sqrt(np.sum(np.asarray(vec) ** 2, axis=-1) + eps)
