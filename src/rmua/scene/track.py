"""Segment track built from official gate coordinates.

The static data gives gate transforms but not the full road mesh. Node 1 uses a
polyline through start gate -> hub -> end gate and a 10 m wide / 5 m high
corridor. This is a documented approximation; replace the polyline with the
runtime route spline after official calibration.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..rules.model import SegmentSpec
from .map_data import GateData, MapData
from .route import catmull_rom, concat_paths, route_points

DEFAULT_CORRIDOR_HALF_WIDTH_M = 5.0
DEFAULT_CORRIDOR_HEIGHT_M = 5.0
DEFAULT_TRIGGER_LENGTH_M = 7.0


@dataclass(frozen=True)
class GateCrossing:
    gate: GateData
    center: np.ndarray
    normal: np.ndarray
    right: np.ndarray
    up: np.ndarray
    width_m: float
    height_m: float
    trigger_depth_m: float = 7.0

    @classmethod
    def from_gate(cls, gate: GateData, normal: np.ndarray | None = None):
        normal = np.asarray(normal if normal is not None else gate.normal, dtype=np.float64)
        normal = normal / max(np.linalg.norm(normal), 1e-9)
        up = np.array([0.0, 0.0, 1.0])
        right = np.cross(normal, up)
        right = right / max(np.linalg.norm(right), 1e-9)
        return cls(
            gate=gate,
            center=gate.center.copy(),
            normal=normal,
            right=right,
            up=up,
            width_m=gate.width_m,
            height_m=gate.height_m,
        )

    def signed_distance(self, pos):
        return np.sum((np.asarray(pos) - self.center) * self.normal, axis=-1)

    def local_uv(self, pos):
        delta = np.asarray(pos) - self.center
        return np.sum(delta * self.right, axis=-1), np.sum(delta * self.up, axis=-1)

    def contains_crossing(self, previous, current, margin=0.0):
        previous = np.atleast_2d(np.asarray(previous, dtype=np.float64))
        current = np.atleast_2d(np.asarray(current, dtype=np.float64))
        s0 = self.signed_distance(previous)
        s1 = self.signed_distance(current)
        crossed = (s0 < 0.0) & (s1 >= 0.0)
        if not np.any(crossed):
            return crossed
        # Interpolate the crossing point and check the aperture.
        denom = np.where(np.abs(s1 - s0) < 1e-9, 1e-9, s1 - s0)
        alpha = np.clip(-s0 / denom, 0.0, 1.0)
        point = previous + (current - previous) * alpha[..., None]
        u, v = self.local_uv(point)
        inside = (np.abs(u) <= self.width_m * 0.5 + margin) & (
            np.abs(v) <= self.height_m * 0.5 + margin
        )
        return crossed & inside

    def contains_trigger(self, pos):
        """Box trigger volume around the gate center.

        The exact official detection volume is not decoded; a 10 x 5 x 3 m box
        is the node-1 proxy. It avoids the plane-orientation failure when the
        route approaches a gate from below/above.
        """
        pos = np.atleast_2d(np.asarray(pos, dtype=np.float64))
        delta = pos - self.center
        u = np.sum(delta * self.right, axis=-1)
        v = np.sum(delta * self.up, axis=-1)
        w = np.sum(delta * self.normal, axis=-1)
        return (
            (np.abs(u) <= self.width_m * 0.5)
            & (np.abs(v) <= self.height_m * 0.5)
            & (np.abs(w) <= self.trigger_depth_m * 0.5)
        )


@dataclass
class SegmentTrack:
    spec: SegmentSpec
    start_gate: GateCrossing
    end_gate: GateCrossing
    path: np.ndarray
    cumulative_s: np.ndarray
    total_length_m: float
    corridor_half_width_m: float = DEFAULT_CORRIDOR_HALF_WIDTH_M
    corridor_height_m: float = DEFAULT_CORRIDOR_HEIGHT_M
    start_trigger_length_m: float = DEFAULT_TRIGGER_LENGTH_M
    end_trigger_length_m: float = DEFAULT_TRIGGER_LENGTH_M
    route_mode: bool = False
    scoring_gates: tuple[GateCrossing, ...] = ()
    audited_stage1: bool = False
    _tree: object = field(default=None, init=False, repr=False)

    def __post_init__(self):
        try:
            from scipy.spatial import cKDTree

            self._tree = cKDTree(self.path)
        except Exception:
            self._tree = None

    @property
    def start_position(self) -> np.ndarray:
        return self.path[0].copy()

    @property
    def end_position(self) -> np.ndarray:
        return self.path[-1].copy()

    @property
    def goal(self) -> np.ndarray:
        return self.end_gate.center.copy()

    def project(self, pos):
        """Project a batch of positions onto the path.

        Returns progress_m, closest point, tangent, signed lateral error,
        vertical error. Nearest-sample search is vectorized over the path; the
        1 m resampling makes this accurate enough for reward/corridor use.
        """
        pos = np.asarray(pos, dtype=np.float64)
        if pos.ndim == 1:
            pos = pos[None, :]
        n = pos.shape[0]
        if self._tree is not None:
            _, index = self._tree.query(pos, k=1)
            index = np.asarray(index, dtype=np.int64)
        else:
            d2 = np.sum((pos[:, None, :] - self.path[None, :, :]) ** 2, axis=2)
            index = np.argmin(d2, axis=1)
        index = np.clip(index, 0, len(self.path) - 2)
        closest = self.path[index]
        tangent = self.path[index + 1] - self.path[index]
        tangent = tangent / np.maximum(
            np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9
        )
        best_s = self.cumulative_s[index]
        delta = pos - closest
        right = np.cross(tangent, np.array([0.0, 0.0, 1.0]))
        right = right / np.maximum(np.linalg.norm(right, axis=1, keepdims=True), 1e-9)
        lateral = np.sum(delta * right, axis=1)
        vertical = delta[:, 2]
        return best_s, closest, tangent, lateral, vertical

    def point_at_distance(self, distance_m: float) -> np.ndarray:
        distance_m = float(np.clip(distance_m, 0.0, self.total_length_m))
        index = int(np.searchsorted(self.cumulative_s, distance_m, side="right") - 1)
        index = min(max(index, 0), len(self.path) - 2)
        a = self.path[index]
        b = self.path[index + 1]
        segment_length = self.cumulative_s[index + 1] - self.cumulative_s[index]
        if segment_length <= 1e-12:
            return a.copy()
        alpha = (distance_m - self.cumulative_s[index]) / segment_length
        return a + (b - a) * alpha

    def points_at_distances(self, distances) -> np.ndarray:
        """Vectorized point lookup for a batch of arc-length values."""
        distances = np.clip(
            np.asarray(distances, dtype=np.float64), 0.0, self.total_length_m
        )
        index = np.searchsorted(self.cumulative_s, distances, side="right") - 1
        index = np.clip(index, 0, len(self.path) - 2)
        a = self.path[index]
        b = self.path[index + 1]
        segment_length = self.cumulative_s[index + 1] - self.cumulative_s[index]
        alpha = (distances - self.cumulative_s[index]) / np.maximum(segment_length, 1e-12)
        return a + (b - a) * alpha[..., None]

    def tangent_at_distance(self, distance_m: float) -> np.ndarray:
        index = int(np.searchsorted(self.cumulative_s, distance_m, side="right") - 1)
        index = min(max(index, 0), len(self.path) - 2)
        tangent = self.path[index + 1] - self.path[index]
        return tangent / max(np.linalg.norm(tangent), 1e-9)

    def tangents_at_distances(self, distances) -> np.ndarray:
        distances = np.clip(
            np.asarray(distances, dtype=np.float64), 0.0, self.total_length_m
        )
        index = np.searchsorted(self.cumulative_s, distances, side="right") - 1
        index = np.clip(index, 0, len(self.path) - 2)
        tangent = self.path[index + 1] - self.path[index]
        return tangent / np.maximum(
            np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9
        )

    def in_corridor(self, pos):
        _, _, _, lateral, vertical = self.project(pos)
        return (np.abs(lateral) <= self.corridor_half_width_m) & (
            np.abs(vertical) <= self.corridor_height_m
        )


def _orientation(gate: GateData, hub: np.ndarray, toward_hub: bool) -> np.ndarray:
    normal = gate.normal.copy()
    direction = (hub - gate.center) if toward_hub else (gate.center - hub)
    if np.dot(normal, direction) < 0.0:
        normal = -normal
    return normal


def build_track(map_data: MapData, spec: SegmentSpec, hub: np.ndarray | None = None) -> SegmentTrack:
    if spec.route_names:
        return _build_route_track(map_data, spec)
    hub = map_data.centroid if hub is None else np.asarray(hub, dtype=np.float64)
    start = map_data.gate(spec.gates[0])
    end = map_data.gate(spec.gates[-1])
    # Start gate normal points from the start marker toward the hub; the drone
    # starts outside the gate and flies inward. End gate normal points outward.
    start_normal = _orientation(start, hub, toward_hub=True)
    end_normal = _orientation(end, hub, toward_hub=False)

    start_crossing = GateCrossing.from_gate(start, start_normal)
    end_crossing = GateCrossing.from_gate(end, end_normal)

    approach = start.center - start_normal * 10.0
    exit_point = end.center + end_normal * 10.0
    points = [approach, start.center, hub, end.center, exit_point]
    path = np.stack(points, axis=0).astype(np.float64)
    # Drop duplicate consecutive points.
    keep = [0]
    for index in range(1, len(path)):
        if np.linalg.norm(path[index] - path[keep[-1]]) > 1e-6:
            keep.append(index)
    path = path[keep]
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    return SegmentTrack(
        spec=spec,
        start_gate=start_crossing,
        end_gate=end_crossing,
        path=path,
        cumulative_s=cumulative,
        total_length_m=float(cumulative[-1]),
    )

def _build_route_track(map_data: MapData, spec: SegmentSpec) -> SegmentTrack:
    """Two-leg route proxy: outbound route + hub connector + reversed return route."""
    route_a = catmull_rom(route_points(map_data.name, spec.route_names[0]))
    route_b = catmull_rom(route_points(map_data.name, spec.route_names[1]))[::-1]
    start = map_data.gate(spec.gates[0])
    end = map_data.gate(spec.gates[1])
    # Insert the actual gate centers so the trigger boxes lie on the path.
    hub_connector = np.stack((start.center, end.center), axis=0)
    route = concat_paths([route_a, hub_connector, route_b])
    start_crossing = GateCrossing.from_gate(start)
    end_crossing = GateCrossing.from_gate(end)
    return SegmentTrack(
        spec=spec,
        start_gate=start_crossing,
        end_gate=end_crossing,
        path=route.points,
        cumulative_s=route.cumulative,
        total_length_m=route.length_m,
        route_mode=True,
    )
