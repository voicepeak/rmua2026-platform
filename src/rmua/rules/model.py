"""Node 0: rule and segment contracts loaded from YAML."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RULES_PATH = ROOT / "configs" / "rules_2026.yaml"


@dataclass(frozen=True)
class SegmentSpec:
    id: int
    name: str
    start_marker: int
    end_marker: int
    time_limit_s: float
    wind: bool
    factory_before: int
    factory_after: int
    gates: tuple[str, ...]
    route_names: tuple[str, ...] = ()
    confidence: str = "approx"
    route_hints: tuple[str, ...] = ()
    notes: str = ""

    @property
    def factory_reports(self) -> int:
        return self.factory_before + self.factory_after


@dataclass(frozen=True)
class Rules:
    version: str
    world: dict[str, Any]
    control: dict[str, Any]
    sensors: dict[str, Any]
    course: dict[str, Any]
    deployment: dict[str, Any]
    drone: dict[str, Any]
    segments: tuple[SegmentSpec, ...]
    source_path: Path = field(default=DEFAULT_RULES_PATH, compare=False)

    @property
    def segment_by_id(self) -> dict[int, SegmentSpec]:
        return {segment.id: segment for segment in self.segments}

    @property
    def total_time_limit_s(self) -> float:
        return sum(segment.time_limit_s for segment in self.segments)

    @property
    def detection_gates_total(self) -> int:
        return int(self.course["detection_gates_total"])

    def check(self) -> list[str]:
        problems: list[str] = []
        ids = [segment.id for segment in self.segments]
        if ids != list(range(1, 7)):
            problems.append(f"segment ids must be 1..6, got {ids}")
        limits = [segment.time_limit_s for segment in self.segments]
        if limits != [180.0, 180.0, 180.0, 200.0, 220.0, 200.0]:
            problems.append(f"unexpected time limits: {limits}")
        wind = [segment.wind for segment in self.segments]
        if wind != [False, False, False, True, True, True]:
            problems.append(f"wind must start at segment 4 (8-12), got {wind}")
        factories = [(s.factory_before, s.factory_after) for s in self.segments]
        expected = [(0, 0), (0, 0), (0, 0), (1, 0), (1, 1), (1, 1)]
        if factories != expected:
            problems.append(f"factory schedule mismatch: {factories}")
        gate_count = sum(len(segment.gates) for segment in self.segments)
        if gate_count != self.detection_gates_total:
            problems.append(
                f"gate count {gate_count} != detection_gates_total {self.detection_gates_total}"
            )
        if self.course.get("start_timeout_s") != 30.0:
            problems.append("start timeout must be 30 s")
        if self.course.get("force_field_limit_s") != 2.0:
            problems.append("force-field limit must be 2 s")
        if self.control.get("max_hz") != 100:
            problems.append("control max_hz must be 100")
        if self.sensors.get("lidar_points_per_frame") != 20000:
            problems.append("LiDAR must be 20000 points/frame")
        if self.sensors.get("camera_hz") != 20:
            problems.append("camera must be 20 Hz")
        return problems


def _tuple(value) -> tuple:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    return tuple(value)


def load_rules(path: str | Path | None = None) -> Rules:
    path = Path(path) if path else DEFAULT_RULES_PATH
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    segments = tuple(
        SegmentSpec(
            id=int(item["id"]),
            name=str(item["name"]),
            start_marker=int(item["start_marker"]),
            end_marker=int(item["end_marker"]),
            time_limit_s=float(item["time_limit_s"]),
            wind=bool(item["wind"]),
            factory_before=int(item.get("factory_before", 0)),
            factory_after=int(item.get("factory_after", 0)),
            gates=_tuple(item.get("gates")),
            route_names=_tuple(item.get("route_names")),
            confidence=str(item.get("confidence", "approx")),
            route_hints=_tuple(item.get("route_hints")),
            notes=str(item.get("notes", "")),
        )
        for item in raw["segments"]
    )
    rules = Rules(
        version=str(raw["meta"]["version"]),
        world=dict(raw["world"]),
        control=dict(raw["control"]),
        sensors=dict(raw["sensors"]),
        course=dict(raw["course"]),
        deployment=dict(raw["deployment"]),
        drone=dict(raw["drone"]),
        segments=segments,
        source_path=path,
    )
    problems = rules.check()
    if problems:
        raise ValueError("invalid rules:\n- " + "\n- ".join(problems))
    return rules
