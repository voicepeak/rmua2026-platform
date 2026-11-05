"""Explicit boundaries and small vector types shared by the offline models.

Anything that needs live UE/AirSim state is represented by one of these
interfaces and raises `EngineBoundaryError` by default.  The message names the
native function and the runtime input that would be required.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .ue_rng import f32


class EngineBoundaryError(NotImplementedError):
    def __init__(self, function: str, required_input: str):
        self.function = function
        self.required_input = required_input
        super().__init__(
            f"{function} is an engine/runtime boundary; required input: {required_input}"
        )


Vec3 = tuple[float, float, float]


def vec3(x: float, y: float, z: float) -> Vec3:
    return (f32(x), f32(y), f32(z))


def add(a: Vec3, b: Vec3) -> Vec3:
    return (f32(a[0] + b[0]), f32(a[1] + b[1]), f32(a[2] + b[2]))


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (f32(a[0] - b[0]), f32(a[1] - b[1]), f32(a[2] - b[2]))


def mul(a: Vec3, scale: float) -> Vec3:
    scale = f32(scale)
    return (f32(a[0] * scale), f32(a[1] * scale), f32(a[2] * scale))


def dot(a: Vec3, b: Vec3) -> float:
    return f32(f32(f32(a[0] * b[0]) + f32(a[1] * b[1])) + f32(a[2] * b[2]))


def distance(a: Vec3, b: Vec3) -> float:
    dx = f32(a[0] - b[0])
    dy = f32(a[1] - b[1])
    dz = f32(a[2] - b[2])
    return f32((dx * dx + dy * dy + dz * dz) ** 0.5)


@dataclass(frozen=True)
class Frame:
    """A component frame supplied by the engine.

    `forward`, `right`, `up` are the component basis vectors in world space.
    Their scale convention is the engine's GetForwardVector/GetRightVector/
    GetUpVector result; the offline model never invents them.
    """

    location: Vec3
    forward: Vec3
    right: Vec3
    up: Vec3

    def point(self, forward: float = 0.0, right: float = 0.0, up: float = 0.0) -> Vec3:
        return add(
            add(add(self.location, mul(self.forward, forward)), mul(self.right, right)),
            mul(self.up, up),
        )


@dataclass
class LineComponentState:
    frame: Frame
    base_points: list[Vec3] = field(default_factory=list)
    end_points: list[Vec3] = field(default_factory=list)

    @property
    def count(self) -> int:
        return len(self.base_points)

    def append(self, base: Vec3, end: Vec3) -> None:
        self.base_points.append(base)
        self.end_points.append(end)


class LineSpawnSink:
    def add_instance(self, selector: int, base: Vec3, end: Vec3) -> int:
        raise EngineBoundaryError(
            "0x2fc3db0/0x054775d0",
            "HISM_ThickLine/HISM_ThinLine component and AddInstance world transform",
        )


class VehicleSpawnSink:
    def attach_initial_component(self, route, point: Vec3) -> object:
        raise EngineBoundaryError(
            "0x2fc3270",
            "vehicle route actor, candidate component and GameState+0x220 element",
        )

    def spawn_vehicle(self, descriptor: dict) -> object:
        raise EngineBoundaryError(
            "0x2fc9220/0x2fc9f30",
            "UE actor allocation, class/owner/path spline and spawn transform",
        )

    def movement_target(self, location: Vec3, tangent: Vec3, dt: float) -> None:
        raise EngineBoundaryError(
            "0x052b53b0",
            "movement component, target location, tangent and delta time",
        )
