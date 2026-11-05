"""Load the static map export from the reverse-engineering dossier."""

from __future__ import annotations

import json
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

from ..core.frames import ue_cm_to_m, yaw_deg_to_forward, yaw_deg_to_right

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MAP_PATH = ROOT / "data" / "maps" / "maps_summary_all.json"

# The static export gives gate scale, not the physical aperture. The official
# trigger zone is 10 m x 5 m x 7 m; use 10 x 5 as a documented approximation
# until the gate mesh is measured.
DEFAULT_GATE_WIDTH_M = 10.0
DEFAULT_GATE_HEIGHT_M = 5.0


@dataclass(frozen=True)
class GateData:
    name: str
    center: np.ndarray
    yaw_deg: float
    scale: np.ndarray
    normal: np.ndarray
    right: np.ndarray
    up: np.ndarray
    width_m: float = DEFAULT_GATE_WIDTH_M
    height_m: float = DEFAULT_GATE_HEIGHT_M
    aperture_confidence: str = "approx"

    @property
    def center_m(self) -> np.ndarray:
        return self.center


@dataclass(frozen=True)
class FactoryData:
    name: str
    center: np.ndarray
    yaw_deg: float


@dataclass(frozen=True)
class PortData:
    name: str
    center: np.ndarray


@dataclass(frozen=True)
class MapData:
    name: str
    gates: dict[str, GateData]
    factories: dict[str, FactoryData]
    ports: dict[str, PortData]
    check_ports: dict[str, PortData]
    route_configs: tuple[dict, ...]
    raw: dict

    @property
    def gate_names(self) -> tuple[str, ...]:
        return tuple(self.gates.keys())

    @property
    def centroid(self) -> np.ndarray:
        centers = np.stack([gate.center for gate in self.gates.values()], axis=0)
        return centers.mean(axis=0)

    def gate(self, name: str) -> GateData:
        try:
            return self.gates[name]
        except KeyError as exc:
            raise KeyError(f"gate {name!r} not in map {self.name}") from exc


def _center(node: dict) -> np.ndarray:
    if not node or not node.get("location"):
        return np.full(3, np.nan)
    loc = node["location"]
    return ue_cm_to_m(np.array([loc["X"], loc["Y"], loc["Z"]], dtype=np.float64))


def _yaw(node: dict) -> float:
    if not node:
        return 0.0
    return float((node.get("rotation") or {}).get("Yaw", 0.0))


def _scale(node: dict) -> np.ndarray:
    scale = node.get("scale") or {}
    return np.array(
        [scale.get("X", 1.0), scale.get("Y", 1.0), scale.get("Z", 1.0)],
        dtype=np.float64,
    )


def _parse_gate(name: str, node: dict) -> GateData:
    yaw = _yaw(node)
    center = _center(node)
    return GateData(
        name=name,
        center=center,
        yaw_deg=yaw,
        scale=_scale(node),
        normal=yaw_deg_to_forward(yaw),
        right=yaw_deg_to_right(yaw),
        up=np.array([0.0, 0.0, 1.0]),
    )


def _parse_factories(nodes: dict) -> dict[str, FactoryData]:
    out = {}
    for name, node in nodes.items():
        center = _center(node)
        if np.any(~np.isfinite(center)):
            continue
        out[name] = FactoryData(name=name, center=center, yaw_deg=_yaw(node))
    return out


def _parse_ports(nodes: dict) -> dict[str, PortData]:
    out = {}
    for name, node in nodes.items():
        center = _center(node)
        if np.any(~np.isfinite(center)):
            continue
        out[name] = PortData(name=name, center=center)
    return out


@lru_cache(maxsize=4)
def _load_all(path: str) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def available_maps(path: str | Path | None = None) -> tuple[str, ...]:
    data = _load_all(str(path or DEFAULT_MAP_PATH))
    return tuple(data.keys())


def load_map_data(map_name: str = "RMUA2024OL1", path: str | Path | None = None) -> MapData:
    data = _load_all(str(path or DEFAULT_MAP_PATH))
    if map_name not in data:
        raise KeyError(f"map {map_name!r} not in {tuple(data.keys())}")
    raw = data[map_name]
    gates = {name: _parse_gate(name, node) for name, node in raw["gates"].items()}
    factories = _parse_factories(raw.get("factories", {}))
    ports = _parse_ports(raw.get("ports", {}))
    check_ports = _parse_ports(raw.get("check_ports", {}))
    return MapData(
        name=map_name,
        gates=gates,
        factories=factories,
        ports=ports,
        check_ports=check_ports,
        route_configs=tuple(raw.get("config", {}).get("RouteConfigs", ())),
        raw=raw,
    )
