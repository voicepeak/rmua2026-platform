"""Quaternion helpers. Convention: [w, x, y, z], scalar first."""

from __future__ import annotations

import numpy as np

from .frames import DEG


def normalize(q, eps=1e-12):
    q = np.asarray(q, dtype=np.float64)
    norm = np.sqrt(np.sum(q * q, axis=-1, keepdims=True) + eps)
    return q / norm


def identity(batch_shape=()):
    out = np.zeros(batch_shape + (4,), dtype=np.float64)
    out[..., 0] = 1.0
    return out


def multiply(a, b):
    """Hamilton product a ⊗ b, last axis [w,x,y,z]."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ),
        axis=-1,
    )


def conjugate(q):
    q = np.asarray(q, dtype=np.float64)
    return np.stack((q[..., 0], -q[..., 1], -q[..., 2], -q[..., 3]), axis=-1)


def rotate(q, vec):
    """Rotate world vector(s) by q: R(q) @ vec."""
    q = normalize(q)
    vec = np.asarray(vec, dtype=np.float64)
    qv = q[..., 1:4]
    uv = np.cross(qv, vec)
    uuv = np.cross(qv, uv)
    return vec + 2.0 * (q[..., 0:1] * uv + uuv)


def rotate_inverse(q, vec):
    return rotate(conjugate(q), vec)


def from_euler(yaw_rad, pitch_rad=0.0, roll_rad=0.0):
    """UE-style intrinsic yaw(Z), pitch(Y), roll(X), returned as [w,x,y,z]."""
    yaw = np.asarray(yaw_rad, dtype=np.float64)
    pitch = np.asarray(pitch_rad, dtype=np.float64)
    roll = np.asarray(roll_rad, dtype=np.float64)

    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return normalize(np.stack((w, x, y, z), axis=-1))


def to_euler(q):
    """Return (yaw, pitch, roll) in radians."""
    q = normalize(q)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return yaw, pitch, roll


def from_euler_deg(yaw_deg, pitch_deg=0.0, roll_deg=0.0):
    return from_euler(
        np.asarray(yaw_deg, dtype=np.float64) * DEG,
        np.asarray(pitch_deg, dtype=np.float64) * DEG,
        np.asarray(roll_deg, dtype=np.float64) * DEG,
    )


def integrate(q, omega_body, dt):
    """Integrate quaternion with body angular velocity (rad/s)."""
    q = normalize(q)
    omega = np.asarray(omega_body, dtype=np.float64)
    zero = np.zeros_like(omega[..., 0])
    omega_q = np.stack((zero, omega[..., 0], omega[..., 1], omega[..., 2]), axis=-1)
    q_dot = 0.5 * multiply(q, omega_q)
    return normalize(q + q_dot * np.asarray(dt, dtype=np.float64)[..., None])


def body_z(q):
    """World-space body +z axis."""
    z = np.zeros(q.shape[:-1] + (3,), dtype=np.float64)
    z[..., 2] = 1.0
    return rotate(q, z)


def from_matrix(matrix):
    """Rotation matrix (..., 3, 3) to quaternion [w, x, y, z]."""
    m = np.asarray(matrix, dtype=np.float64)
    if m.shape[-2:] != (3, 3):
        raise ValueError("matrix must have shape (..., 3, 3)")
    with np.errstate(all="ignore"):
        return _from_matrix_unchecked(m)


def _from_matrix_unchecked(m):
    m00, m01, m02 = m[..., 0, 0], m[..., 0, 1], m[..., 0, 2]
    m10, m11, m12 = m[..., 1, 0], m[..., 1, 1], m[..., 1, 2]
    m20, m21, m22 = m[..., 2, 0], m[..., 2, 1], m[..., 2, 2]

    trace = m00 + m11 + m22
    q = np.zeros(m.shape[:-2] + (4,), dtype=np.float64)

    cond0 = trace > 0.0
    s0 = np.sqrt(trace + 1.0) * 2.0
    q0 = np.stack(
        (0.25 * s0, (m21 - m12) / s0, (m02 - m20) / s0, (m10 - m01) / s0), axis=-1
    )

    cond1 = (~cond0) & (m00 > m11) & (m00 > m22)
    s1 = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
    q1 = np.stack(
        ((m21 - m12) / s1, 0.25 * s1, (m01 + m10) / s1, (m02 + m20) / s1), axis=-1
    )

    cond2 = (~cond0) & (~cond1) & (m11 > m22)
    s2 = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
    q2 = np.stack(
        ((m02 - m20) / s2, (m01 + m10) / s2, 0.25 * s2, (m12 + m21) / s2), axis=-1
    )

    cond3 = (~cond0) & (~cond1) & (~cond2)
    s3 = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
    q3 = np.stack(
        ((m10 - m01) / s3, (m02 + m20) / s3, (m12 + m21) / s3, 0.25 * s3), axis=-1
    )

    q = np.where(cond0[..., None], q0, q)
    q = np.where(cond1[..., None], q1, q)
    q = np.where(cond2[..., None], q2, q)
    q = np.where(cond3[..., None], q3, q)
    return normalize(q)


def to_matrix(q):
    """Quaternion to rotation matrix (..., 3, 3)."""
    q = normalize(q)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    shape = q.shape[:-1] + (3, 3)
    m = np.zeros(shape, dtype=np.float64)
    m[..., 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    m[..., 0, 1] = 2.0 * (x * y - w * z)
    m[..., 0, 2] = 2.0 * (x * z + w * y)
    m[..., 1, 0] = 2.0 * (x * y + w * z)
    m[..., 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    m[..., 1, 2] = 2.0 * (y * z - w * x)
    m[..., 2, 0] = 2.0 * (x * z - w * y)
    m[..., 2, 1] = 2.0 * (y * z + w * x)
    m[..., 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return m


def rotation_between_vectors(a, b):
    """Shortest-arc quaternion rotating unit vector a to unit vector b."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    a = a / np.linalg.norm(a, axis=-1, keepdims=True)
    b = b / np.linalg.norm(b, axis=-1, keepdims=True)
    dot = np.sum(a * b, axis=-1, keepdims=True)
    cross = np.cross(a, b)
    w = 1.0 + dot
    q = np.concatenate((w, cross), axis=-1)
    # Opposite vectors: pick an arbitrary perpendicular axis.
    opposite = dot[..., 0] < -0.999999
    if np.any(opposite):
        axis = np.zeros_like(a)
        axis[..., 0] = 1.0
        alt = np.cross(a, axis)
        bad = np.linalg.norm(alt, axis=-1) < 1e-6
        axis = np.where(bad[..., None], np.array([0.0, 1.0, 0.0]), axis)
        alt = np.cross(a, axis)
        alt = alt / np.linalg.norm(alt, axis=-1, keepdims=True)
        q_opp = np.concatenate((np.zeros_like(dot), alt), axis=-1)
        q = np.where(opposite[..., None], q_opp, q)
    return normalize(q)
