"""Configurable training obstacles, with swept tests; not official collision parity.

Line collision is a swept sphere against a capsule. Vehicle collision uses an
oriented box expanded by drone radius: conservative near edges and corners.
Vehicle yaw is constant during each step; subdivide steps when turning.
"""
from dataclasses import dataclass
import math
import numpy as np


def vec(value):
    x = np.asarray(value, dtype=float)
    if x.shape != (3,) or not np.isfinite(x).all():
        raise ValueError('expected finite 3-vector')
    return x


def segment_distance_squared(a, b, c, d):
    """Minimum squared distance between finite segments, including degenerate ones."""
    a, b, c, d = map(vec, (a, b, c, d))
    u, v, w = b-a, d-c, a-c
    aa, bb, cc, dd, ee = u@u, u@v, v@v, u@w, v@w
    candidates = []
    def add(s, t):
        delta = w + s*u - t*v
        candidates.append(float(delta@delta))
    if cc > 0:
        add(0, np.clip(ee/cc, 0, 1))
        add(1, np.clip((ee+bb)/cc, 0, 1))
    if aa > 0:
        add(np.clip(-dd/aa, 0, 1), 0)
        add(np.clip((bb-dd)/aa, 0, 1), 1)
    determinant = aa*cc-bb*bb
    if determinant > 1e-14*aa*cc:
        s, t = (bb*ee-cc*dd)/determinant, (aa*ee-bb*dd)/determinant
        if 0 <= s <= 1 and 0 <= t <= 1:
            add(s, t)
    if not candidates:
        add(0, 0)
    return min(candidates)


@dataclass(frozen=True)
class Line:
    start: tuple
    end: tuple
    radius_m: float

    def hit_swept(self, start, end, drone_radius_m):
        if not math.isfinite(self.radius_m) or not math.isfinite(drone_radius_m) or min(self.radius_m, drone_radius_m) < 0:
            raise ValueError('invalid radius')
        return segment_distance_squared(start, end, self.start, self.end) <= (self.radius_m+drone_radius_m)**2


@dataclass(frozen=True)
class Vehicle:
    center: tuple
    half_extents_m: tuple
    velocity_m_s: tuple
    yaw_rad: float

    def hit_swept(self, drone_start, drone_end, dt, drone_radius_m):
        center, ext, velocity = map(vec, (self.center, self.half_extents_m, self.velocity_m_s))
        if (np.any(ext <= 0) or not math.isfinite(dt) or dt < 0 or
                not math.isfinite(drone_radius_m) or drone_radius_m < 0 or not math.isfinite(self.yaw_rad)):
            raise ValueError('invalid vehicle or step')
        a, b = vec(drone_start)-center, vec(drone_end)-(center+velocity*dt)
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        rotation = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        a, b = rotation@a, rotation@b
        lo, hi = 0., 1.
        for x, delta, limit in zip(a, b-a, ext+drone_radius_m):
            if abs(delta) < 1e-12:
                if abs(x) > limit:
                    return False
            else:
                t0, t1 = sorted(((-limit-x)/delta, (limit-x)/delta))
                lo, hi = max(lo, t0), min(hi, t1)
                if lo > hi:
                    return False
        return True


def generate(seed, bounds_min, bounds_max, launch, exclusion_m, n_lines, n_vehicles,
             line_radius_m, line_length_m, vehicle_half_extents_m, speed_range_m_s):
    """Sample explicit scene bounds; caller must check road/geometry feasibility."""
    low, high, launch, ext = map(vec, (bounds_min, bounds_max, launch, vehicle_half_extents_m))
    params = (exclusion_m, line_radius_m, line_length_m, *speed_range_m_s)
    if (np.any(low >= high) or np.any(ext <= 0) or n_lines < 0 or n_vehicles < 0 or
            any(not math.isfinite(x) for x in params) or exclusion_m < 0 or line_radius_m < 0 or
            line_length_m <= 0 or speed_range_m_s[0] < 0 or speed_range_m_s[0] > speed_range_m_s[1]):
        raise ValueError('invalid generation parameters')
    rng = np.random.default_rng(seed)
    def center(clearance):
        for _ in range(10000):
            p = rng.uniform(low, high)
            if np.linalg.norm(p-launch) >= exclusion_m + clearance:
                return p
        raise ValueError('cannot place obstacle outside launch exclusion')
    lines = []
    for _ in range(n_lines):
        p = center(line_length_m/2+line_radius_m)
        delta = np.array([0, 0, line_length_m/2])
        lines.append(Line(tuple(p-delta), tuple(p+delta), line_radius_m))
    vehicles = []
    for _ in range(n_vehicles):
        p, yaw = center(float(np.linalg.norm(ext))), float(rng.uniform(-np.pi, np.pi))
        speed = float(rng.uniform(*speed_range_m_s))
        vehicles.append(Vehicle(tuple(p), tuple(ext), (speed*math.cos(yaw), speed*math.sin(yaw), 0.), yaw))
    return lines, vehicles
