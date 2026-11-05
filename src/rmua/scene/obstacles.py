"""Simple seed-driven line and vehicle obstacles for sim-fast.

The official assets are HISM line segments and hover vehicles with extracted
convex collision meshes. Node 1 uses capsules/boxes as a fast proxy; swap in the
mesh-based collision backend after calibration. Every generated instance is
recorded so a failed episode can be replayed from its seed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .track import SegmentTrack

DRONE_RADIUS_M = 0.25


@dataclass
class LineObstacles:
    start: np.ndarray  # (N, M, 3)
    end: np.ndarray    # (N, M, 3)
    radius: np.ndarray  # (N, M)
    active: np.ndarray  # (N, M) bool

    @property
    def max_count(self) -> int:
        return self.start.shape[1]

    def collides(self, positions: np.ndarray) -> np.ndarray:
        """Return (N,) bool for capsule/point collision."""
        n, m = self.active.shape
        if m == 0:
            return np.zeros(n, dtype=bool)
        p = positions[:, None, :]
        a = self.start
        b = self.end
        ab = b - a
        denom = np.sum(ab * ab, axis=2)
        t = np.sum((p - a) * ab, axis=2) / np.maximum(denom, 1e-12)
        t = np.clip(t, 0.0, 1.0)
        closest = a + t[..., None] * ab
        distance = np.linalg.norm(p - closest, axis=2)
        hit = distance <= (self.radius + DRONE_RADIUS_M)
        hit = hit & self.active
        return np.any(hit, axis=1)


@dataclass
class VehicleObstacles:
    center: np.ndarray  # (N, V, 3)
    half_extents: np.ndarray  # (V, 3)
    yaw: np.ndarray  # (N, V)
    velocity: np.ndarray  # (N, V, 3)
    active: np.ndarray  # (N, V) bool

    @property
    def max_count(self) -> int:
        return self.center.shape[1]

    def step(self, dt: float):
        self.center = self.center + self.velocity * dt

    def collides(self, positions: np.ndarray) -> np.ndarray:
        n, v = self.active.shape
        if v == 0:
            return np.zeros(n, dtype=bool)
        delta = positions[:, None, :] - self.center
        c = np.cos(-self.yaw)
        s = np.sin(-self.yaw)
        x = delta[..., 0] * c - delta[..., 1] * s
        y = delta[..., 0] * s + delta[..., 1] * c
        z = delta[..., 2]
        limit = self.half_extents[None, :, :] + DRONE_RADIUS_M
        hit = (
            (np.abs(x) <= limit[..., 0])
            & (np.abs(y) <= limit[..., 1])
            & (np.abs(z) <= limit[..., 2])
        )
        hit = hit & self.active
        return np.any(hit, axis=1)


def generate_lines(
    track: SegmentTrack,
    rngs: list[np.random.Generator],
    max_count: int = 8,
    density_probability: float = 0.65,
    lateral_limit: float = 4.0,
    vertical_limit: float = 2.0,
) -> LineObstacles:
    n = len(rngs)
    starts = np.zeros((n, max_count, 3), dtype=np.float64)
    ends = np.zeros((n, max_count, 3), dtype=np.float64)
    radii = np.zeros((n, max_count), dtype=np.float64)
    active = np.zeros((n, max_count), dtype=bool)

    for env_index, rng in enumerate(rngs):
        count = int(rng.integers(0, max_count + 1))
        for index in range(count):
            if rng.random() > density_probability:
                continue
            distance = float(rng.uniform(0.1, 0.9)) * track.total_length_m
            base = track.point_at_distance(distance)
            tangent = track.tangent_at_distance(distance)
            right = np.cross(tangent, np.array([0.0, 0.0, 1.0]))
            right = right / max(np.linalg.norm(right), 1e-9)
            lateral = float(rng.uniform(-lateral_limit, lateral_limit))
            vertical = float(rng.uniform(-vertical_limit, vertical_limit))
            center = base + right * lateral + np.array([0.0, 0.0, vertical])
            angle = float(rng.uniform(0.0, np.pi))
            direction = right * np.cos(angle) + tangent * np.sin(angle)
            direction = direction / max(np.linalg.norm(direction), 1e-9)
            half_length = float(rng.uniform(0.5, 4.0))
            starts[env_index, index] = center - direction * half_length
            ends[env_index, index] = center + direction * half_length
            radii[env_index, index] = float(rng.uniform(0.08, 0.25))
            active[env_index, index] = True
    return LineObstacles(starts, ends, radii, active)


def generate_vehicles(
    track: SegmentTrack,
    rngs: list[np.random.Generator],
    max_count: int = 3,
    speed_range: tuple[float, float] = (1.0, 6.0),
) -> VehicleObstacles:
    n = len(rngs)
    centers = np.zeros((n, max_count, 3), dtype=np.float64)
    yaws = np.zeros((n, max_count), dtype=np.float64)
    velocities = np.zeros((n, max_count, 3), dtype=np.float64)
    active = np.zeros((n, max_count), dtype=bool)
    half_extents = np.tile(np.array([2.0, 1.0, 1.0]), (max_count, 1))

    for env_index, rng in enumerate(rngs):
        count = int(rng.integers(0, max_count + 1))
        for index in range(count):
            distance = float(rng.uniform(0.1, 0.9)) * track.total_length_m
            tangent = track.tangent_at_distance(distance)
            right = np.cross(tangent, np.array([0.0, 0.0, 1.0]))
            right = right / max(np.linalg.norm(right), 1e-9)
            lateral = float(rng.uniform(-3.0, 3.0))
            center = track.point_at_distance(distance) + right * lateral
            direction = 1.0 if rng.random() < 0.5 else -1.0
            speed = float(rng.uniform(*speed_range))
            centers[env_index, index] = center
            yaws[env_index, index] = float(np.arctan2(tangent[1], tangent[0]))
            velocities[env_index, index] = tangent * speed * direction
            active[env_index, index] = True
    return VehicleObstacles(centers, half_extents, yaws, velocities, active)
