"""Offline spline model reconstructed from artifacts/maps/maps_summary_all.json.

The serialized spline points carry InVal, OutVal and arrive/leave tangents.
This module evaluates the same cubic Hermite segments and builds an
arc-length table.  It is a structural model of USplineComponent: route
selection and lengths are validated against captured stage 4/5/6 line bases,
but exact UE reparam-table sampling remains an engine detail.

Evidence for the stage-to-route mapping:
    stage 4 reverse base 65782.34375 cm = route 12 (2_2) length - 100 cm
    stage 5 reverse base 64831.64844 cm = route 10 (4_2) length - 100 cm
    stage 6 reverse base 69056.57812 cm = route  7 (1_2) length - 100 cm
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path

from .ue_rng import f32

DEFAULT_MAPS = Path(__file__).resolve().parents[1] / "artifacts" / "maps" / "maps_summary_all.json"
DEFAULT_MAP = "RMUA2024OL1"
STAGE_ROUTE_INDEX = {4: 12, 5: 10, 6: 7}
STAGE_SPLINE_NAME = {4: "2_2", 5: "4_2", 6: "1_2"}
DEFAULT_SAMPLES_PER_SEGMENT = 2000


def _vec3(value: dict | None) -> tuple[float, float, float]:
    if not value:
        return (0.0, 0.0, 0.0)
    return (f32(value["X"]), f32(value["Y"]), f32(value["Z"]))


def _add(a, b):
    return (f32(a[0] + b[0]), f32(a[1] + b[1]), f32(a[2] + b[2]))


def _sub(a, b):
    return (f32(a[0] - b[0]), f32(a[1] - b[1]), f32(a[2] - b[2]))


def _mul(a, s):
    s = f32(s)
    return (f32(a[0] * s), f32(a[1] * s), f32(a[2] * s))


def _dist(a, b) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _hermite(p0, p1, t0, t1, alpha: float):
    alpha = f32(alpha)
    a2 = f32(alpha * alpha)
    a3 = f32(a2 * alpha)
    h00 = f32(f32(f32(2.0 * a3) - f32(3.0 * a2)) + 1.0)
    h10 = f32(f32(f32(a3 - f32(2.0 * a2)) + alpha))
    h01 = f32(f32(-2.0 * a3) + f32(3.0 * a2))
    h11 = f32(a3 - a2)
    out = []
    for index in range(3):
        value = f32(
            f32(f32(h00 * p0[index]) + f32(h10 * t0[index]))
            + f32(f32(h01 * p1[index]) + f32(h11 * t1[index]))
        )
        out.append(value)
    return tuple(out)


def _hermite_derivative(p0, p1, t0, t1, alpha: float):
    alpha = f32(alpha)
    a2 = f32(alpha * alpha)
    dh00 = f32(f32(f32(6.0 * a2) - f32(6.0 * alpha)))
    dh10 = f32(f32(f32(3.0 * a2) - f32(4.0 * alpha)) + 1.0)
    dh01 = f32(f32(-6.0 * a2) + f32(6.0 * alpha))
    dh11 = f32(f32(3.0 * a2) - f32(2.0 * alpha))
    out = []
    for index in range(3):
        value = f32(
            f32(f32(dh00 * p0[index]) + f32(dh10 * t0[index]))
            + f32(f32(dh01 * p1[index]) + f32(dh11 * t1[index]))
        )
        out.append(value)
    return tuple(out)


@dataclass(frozen=True)
class SplinePoint:
    in_val: float
    pos: tuple[float, float, float]
    arrive: tuple[float, float, float]
    leave: tuple[float, float, float]


@dataclass
class Spline:
    actor: str
    component: str
    location: tuple[float, float, float]
    rotation: tuple[float, float, float]
    points: list[SplinePoint]
    samples_per_segment: int = DEFAULT_SAMPLES_PER_SEGMENT
    _table: list[tuple[float, float]] | None = None

    @property
    def in_range(self) -> tuple[float, float]:
        return (self.points[0].in_val, self.points[-1].in_val)

    def _segment(self, t: float) -> tuple[SplinePoint, SplinePoint, float]:
        if t <= self.points[0].in_val:
            return self.points[0], self.points[0], 0.0
        if t >= self.points[-1].in_val:
            return self.points[-1], self.points[-1], 0.0
        for first, second in zip(self.points, self.points[1:]):
            if first.in_val <= t <= second.in_val:
                span = second.in_val - first.in_val
                alpha = 0.0 if span == 0 else (t - first.in_val) / span
                return first, second, f32(alpha)
        return self.points[-1], self.points[-1], 0.0

    def local_at_key(self, key: float):
        first, second, alpha = self._segment(key)
        if first is second:
            return first.pos
        return _hermite(first.pos, second.pos, first.leave, second.arrive, alpha)

    def local_derivative_at_key(self, key: float):
        first, second, alpha = self._segment(key)
        if first is second:
            return (0.0, 0.0, 0.0)
        return _hermite_derivative(first.pos, second.pos, first.leave, second.arrive, alpha)

    def _rotate(self, point):
        yaw = math.radians(self.rotation[1])
        pitch = math.radians(self.rotation[0])
        roll = math.radians(self.rotation[2])
        x, y, z = point
        if roll:
            c, s = math.cos(roll), math.sin(roll)
            y, z = c * y - s * z, s * y + c * z
        if pitch:
            c, s = math.cos(pitch), math.sin(pitch)
            x, z = c * x + s * z, -s * x + c * z
        if yaw:
            c, s = math.cos(yaw), math.sin(yaw)
            x, y = c * x - s * y, s * x + c * y
        return (f32(x), f32(y), f32(z))

    def world_at_key(self, key: float):
        return _add(self._rotate(self.local_at_key(key)), self.location)

    def world_tangent_at_key(self, key: float):
        return self._rotate(self.local_derivative_at_key(key))

    def _build_table(self):
        if self._table is not None:
            return self._table
        table = [(self.points[0].in_val, 0.0)]
        total = 0.0
        previous = self.local_at_key(self.points[0].in_val)
        for first, second in zip(self.points, self.points[1:]):
            span = second.in_val - first.in_val
            if span == 0:
                continue
            for step in range(1, self.samples_per_segment + 1):
                key = f32(first.in_val + span * (step / self.samples_per_segment))
                current = self.local_at_key(key)
                total += _dist(current, previous)
                table.append((key, total))
                previous = current
        self._table = table
        return table

    @property
    def length_cm(self) -> float:
        return self._build_table()[-1][1]

    def key_at_distance(self, distance_cm: float) -> float:
        table = self._build_table()
        if distance_cm <= 0.0:
            return table[0][0]
        if distance_cm >= table[-1][1]:
            return table[-1][0]
        low, high = 0, len(table) - 1
        while low + 1 < high:
            middle = (low + high) // 2
            if table[middle][1] <= distance_cm:
                low = middle
            else:
                high = middle
        key0, dist0 = table[low]
        key1, dist1 = table[high]
        if dist1 == dist0:
            return key1
        ratio = (distance_cm - dist0) / (dist1 - dist0)
        return f32(key0 + (key1 - key0) * ratio)

    def world_at_distance(self, distance_cm: float):
        return self.world_at_key(self.key_at_distance(distance_cm))

    def world_tangent_at_distance(self, distance_cm: float):
        return self.world_tangent_at_key(self.key_at_distance(distance_cm))


def load_map(map_name: str = DEFAULT_MAP, path: Path | None = None) -> dict:
    path = Path(os.environ.get("RMUA_MAPS_SUMMARY", path or DEFAULT_MAPS))
    data = json.loads(path.read_text(encoding="utf-8"))
    return data[map_name]


def load_routes(map_name: str = DEFAULT_MAP, path: Path | None = None) -> dict[int, Spline]:
    summary = load_map(map_name, path)
    routes: dict[int, Spline] = {}
    for config in summary["config"]["RouteConfigs"]:
        spline = config["spline"]
        points = [
            SplinePoint(
                f32(point["in"]),
                _vec3(point["pos"]),
                _vec3(point["arrive"]),
                _vec3(point["leave"]),
            )
            for point in spline["points"]
        ]
        rotation = spline["rotation"]
        routes[int(config["RouteIndex"])] = Spline(
            actor=spline["actor"],
            component=spline["component"],
            location=_vec3(spline["location"]),
            rotation=(f32(rotation["Pitch"]), f32(rotation["Yaw"]), f32(rotation["Roll"])),
            points=points,
        )
    return routes


def stage_spline(stage: int, map_name: str = DEFAULT_MAP, path: Path | None = None) -> Spline:
    route_index = STAGE_ROUTE_INDEX[stage]
    return load_routes(map_name, path)[route_index]
