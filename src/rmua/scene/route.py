"""Route-spline proxy built from the exported route control points.

The dossier gives route control points per map. Node 1 interpolates them with a
Catmull-Rom spline and resamples at 1 m. This is a structural proxy, not the UE
spline reparam table; runtime calibration remains required.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
ROUTES_CSV = ROOT / "data" / "tables" / "routes.csv"


@lru_cache(maxsize=1)
def _rows() -> list[dict]:
    with ROUTES_CSV.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def route_points(map_name: str, route_name: str) -> np.ndarray:
    rows = [
        row
        for row in _rows()
        if row["map"] == map_name and row["spline"] == route_name
    ]
    if not rows:
        raise KeyError(f"route {route_name!r} not found for {map_name}")
    rows.sort(key=lambda row: int(row["point"]))
    points = np.array(
        [
            [
                float(row["world_x"]) / 100.0,
                float(row["world_y"]) / 100.0,
                float(row["world_z"]) / 100.0,
            ]
            for row in rows
        ],
        dtype=np.float64,
    )
    return points


def catmull_rom(points: np.ndarray, samples_per_segment: int = 24) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    if len(points) < 2:
        return points.copy()
    padded = np.concatenate((points[:1], points, points[-1:]), axis=0)
    out = []
    for index in range(1, len(padded) - 2):
        p0, p1, p2, p3 = padded[index - 1 : index + 3]
        for step in range(samples_per_segment):
            t = step / samples_per_segment
            t2 = t * t
            t3 = t2 * t
            point = 0.5 * (
                2.0 * p1
                + (-p0 + p2) * t
                + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
            )
            out.append(point)
    out.append(points[-1])
    return np.asarray(out, dtype=np.float64)


@dataclass(frozen=True)
class RoutePath:
    points: np.ndarray
    cumulative: np.ndarray
    length_m: float

    @classmethod
    def from_points(cls, points: np.ndarray, spacing_m: float = 1.0):
        dense = catmull_rom(points)
        lengths = np.linalg.norm(np.diff(dense, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
        total = float(cumulative[-1])
        if total <= spacing_m:
            return cls(dense, cumulative, total)
        sample_distances = np.arange(0.0, total, spacing_m)
        sample_distances = np.append(sample_distances, total)
        indices = np.searchsorted(cumulative, sample_distances, side="right") - 1
        indices = np.clip(indices, 0, len(dense) - 2)
        segment_length = cumulative[indices + 1] - cumulative[indices]
        alpha = (sample_distances - cumulative[indices]) / np.maximum(segment_length, 1e-12)
        resampled = dense[indices] + (dense[indices + 1] - dense[indices]) * alpha[:, None]
        new_lengths = np.linalg.norm(np.diff(resampled, axis=0), axis=1)
        new_cumulative = np.concatenate(([0.0], np.cumsum(new_lengths)))
        return cls(resampled, new_cumulative, float(new_cumulative[-1]))

    def point_at(self, distance):
        distance = np.clip(np.asarray(distance, dtype=np.float64), 0.0, self.length_m)
        index = np.searchsorted(self.cumulative, distance, side="right") - 1
        index = np.clip(index, 0, len(self.points) - 2)
        segment_length = self.cumulative[index + 1] - self.cumulative[index]
        alpha = (distance - self.cumulative[index]) / np.maximum(segment_length, 1e-12)
        return self.points[index] + (self.points[index + 1] - self.points[index]) * alpha[..., None]


def concat_paths(paths: list[np.ndarray], connector_step_m: float = 2.0) -> RoutePath:
    points = [paths[0]]
    for next_points in paths[1:]:
        a = points[-1][-1]
        b = next_points[0]
        distance = float(np.linalg.norm(b - a))
        if distance > connector_step_m:
            count = max(2, int(np.ceil(distance / connector_step_m)))
            connector = a + (b - a) * np.linspace(0.0, 1.0, count)[:, None]
            points.append(connector[1:])
        points.append(next_points)
    return RoutePath.from_points(np.concatenate(points, axis=0))
