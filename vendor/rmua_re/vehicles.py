"""Outer control-flow skeleton for vehicle selection at 0x2fc8900.

Translated exactly: the initial +-9000 cm spline lookup distance, the 2100 cm
tolerance, the FRandomStream step, the float fraction times 11.0f bucket, the
`bucket < 6` branch, and the direct/indirect call argument construction.

Not translated: 0x2fc7620 (spline lookup), 0x2fc3270 (component attach),
0x2fc93c0 (direct spawn), 0x2fc9220 (indirect spawn / vector-field path) and
the later actor/event plumbing.  Those remain documented gaps and the default
backend raises NotImplementedError.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .engine import (
    EngineBoundaryError,
    Frame,
    Vec3,
    add,
    distance,
    mul,
)
from .ue_rng import FRandomStream, f32

LOOKUP_HALF_SPAN_CM = f32(9000.0)  # _DAT_006c1864 / _DAT_006c12fc
LOOKUP_TOLERANCE_CM = f32(2100.0)  # _DAT_006c0f30
ENTRY_SHIFT_CM = f32(500.0)  # _DAT_006c2d28
SPEED_SCALE = f32(100.0)  # _DAT_006c1b34
DIRECT_BUCKET_LIMIT = 6


@dataclass(frozen=True)
class VehicleBranch:
    state: int
    fraction: float
    bucket: int
    direct: bool


def decide(rng: FRandomStream) -> VehicleBranch:
    """0x2fc8900: feed the route decision from one FRandomStream draw."""
    state = rng.step()
    fraction = rng.fraction()
    bucket = int(f32(fraction * f32(11.0)))
    return VehicleBranch(state=state, fraction=fraction, bucket=bucket, direct=bucket < DIRECT_BUCKET_LIMIT)


def entry_arguments(distance_cm: float, forward: bool) -> tuple[float, float]:
    """Return (start, end) used by both spawn branches."""
    shift = f32(-ENTRY_SHIFT_CM) if forward else ENTRY_SHIFT_CM
    span = LOOKUP_HALF_SPAN_CM if forward else f32(-LOOKUP_HALF_SPAN_CM)
    return f32(distance_cm + shift), f32(distance_cm + span)


class VehicleBackend:
    """Boundary for the unresolved route and spawn implementation."""

    reverse = False
    speed_offset_26c = f32(0.0)
    speed_offset_270 = f32(0.0)

    def sample_point(self, route, distance_cm: float, tolerance_cm: float):
        raise NotImplementedError("0x2fc7620 spline point lookup is not translated")

    def attach_initial_component(self, route, point) -> None:
        raise NotImplementedError("0x2fc3270 component attach is not translated")

    def spawn_direct(
        self,
        route,
        start_cm: float,
        end_cm: float,
        tolerance_cm: float,
        speed_cm_s: float,
    ) -> None:
        raise NotImplementedError("0x2fc93c0 direct vehicle spawn is not translated")

    def spawn_indirect(
        self,
        route,
        start_cm: float,
        end_cm: float,
        tolerance_cm: float,
        speed_cm_s: float,
    ) -> None:
        raise NotImplementedError("0x2fc9220 indirect vehicle spawn is not translated")


def run(
    route,
    distance_cm: float,
    rng: FRandomStream,
    backend: VehicleBackend,
) -> VehicleBranch:
    """Translate the outer flow of 0x2fc8900 for one route position."""
    forward = not bool(backend.reverse)
    lookup = f32(
        distance_cm + (LOOKUP_HALF_SPAN_CM if forward else f32(-LOOKUP_HALF_SPAN_CM))
    )
    point = backend.sample_point(route, lookup, LOOKUP_TOLERANCE_CM)
    if point is not None:
        backend.attach_initial_component(route, point)

    branch = decide(rng)
    start, end = entry_arguments(distance_cm, forward)
    if branch.direct:
        speed = f32(backend.speed_offset_270 * SPEED_SCALE)
        backend.spawn_direct(route, start, end, LOOKUP_TOLERANCE_CM, speed)
    else:
        speed = f32(backend.speed_offset_26c * SPEED_SCALE)
        backend.spawn_indirect(route, start, end, LOOKUP_TOLERANCE_CM, speed)
    return branch


PATH_STEP_CM = f32(2100.0)  # _DAT_006c2238 / 0x6c223c
PATH_STATE_ENABLED_OFFSET = 0x230
PATH_PROGRESS_OFFSET = 0x234
PATH_SPEED_OFFSET = 0x238


@dataclass(frozen=True)
class RouteComponent:
    identity: object
    frame: Frame


class RouteComponentProvider:
    """Runtime route+0x240 array and 0x2fc96b0/0x2fc7620 selection."""

    def components(self, route) -> list[RouteComponent]:
        raise EngineBoundaryError(
            "route+0x240",
            "the route's BP_Segment_Rot_C/component array with world frames",
        )

    def initial_component_point(self, route) -> Vec3:
        raise EngineBoundaryError(
            "0x2fc96b0",
            "route+0x264 distance and the nearest component transform/quaternion",
        )

    def transform_component_point(self, component: RouteComponent, local: Vec3) -> Vec3:
        raise EngineBoundaryError(
            "0x2fc98c0",
            "component rotation quaternion, translation and scale at +0x1c0/+0x1d0",
        )


def nearest_component(point: Vec3, components: list[RouteComponent]) -> RouteComponent | None:
    """0x2fc7620/0x2fc98c0 nearest-component scan (strictly smaller wins)."""
    best = None
    best_distance = f32(100000000.0)  # _DAT_006c1588
    for component in components:
        candidate_distance = distance(component.frame.location, point)
        if candidate_distance < best_distance:
            best = component
            best_distance = candidate_distance
    return best


def path_sample_distances(end_distance_cm: float, start_distance_cm: float) -> list[float]:
    """0x2fc98c0 distance walk: end, end+-2100, ... while before start."""
    end = f32(end_distance_cm)
    start = f32(start_distance_cm)
    if end == start:
        return []
    step = f32(-PATH_STEP_CM) if start < end else PATH_STEP_CM
    distances = [end]
    value = end
    while True:
        value = f32(value + step)
        if start < end:
            if not (start < value):
                break
        elif not (value < start):
            break
        distances.append(value)
    return distances


def build_path(
    route,
    end_distance_cm: float,
    start_distance_cm: float,
    provider: RouteComponentProvider,
) -> list[Vec3]:
    """Translate the outer 0x2fc98c0 path assembly.

    It samples the route spline at the fixed distance walk, selects the nearest
    component, transforms the initial component vector, and appends only when
    the selected component changes.  The component array and transform remain
    provider inputs.
    """
    seed_vector = provider.initial_component_point(route)
    components = provider.components(route)
    path: list[Vec3] = []
    previous = None
    for distance_cm in path_sample_distances(end_distance_cm, start_distance_cm):
        sample = route.spline.world_at_distance(distance_cm)
        component = nearest_component(sample, components)
        if component is None:
            continue
        if previous is None or component.identity != previous:
            path.append(provider.transform_component_point(component, seed_vector))
            previous = component.identity
    return path


@dataclass
class PathState:
    speed_cm_s: float
    points: list[Vec3] = field(default_factory=list)
    progress_cm: float = 0.0
    enabled: bool = False
    spline_length_cm: float | None = None

    def begin(self, speed_cm_s: float, points: list[Vec3]) -> None:
        """0x2fbff60 state fields; the UE spline rebuild itself is an engine call."""
        self.enabled = True
        self.progress_cm = 0.0
        self.speed_cm_s = f32(speed_cm_s)
        self.points = list(points)

    def tick(self, dt: float) -> bool:
        """Progress update from 0x2fc0180; returns whether movement was issued."""
        if not self.enabled:
            return False
        limit = self.spline_length_cm
        if limit is None:
            raise EngineBoundaryError(
                "0x052effa0",
                "spline length of the spline component rebuilt by 0x2fbff60",
            )
        if not (self.progress_cm < limit):
            return False
        self.progress_cm = f32(f32(dt) * self.speed_cm_s + self.progress_cm)
        return True


@dataclass(frozen=True)
class VehicleDescriptor:
    vtable: int
    param_1: float
    param_2: float
    param_3: float
    param_4: float
    owner: object
    path_source: object


def indirect_descriptor(
    param_1: float,
    param_2: float,
    param_3: float,
    param_4: float,
    owner,
    path_source,
) -> VehicleDescriptor:
    """0x2fc9220 -> 0x2fc9f30 field layout (vtable 0xcd88c8 after init)."""
    return VehicleDescriptor(
        vtable=0xCD88C8,
        param_1=f32(param_1),
        param_2=f32(param_2),
        param_3=f32(param_3),
        param_4=f32(param_4),
        owner=owner,
        path_source=path_source,
    )


def direct_spawn_descriptor(
    end_distance_cm: float, start_distance_cm: float, tolerance_cm: float, speed_cm_s: float
) -> dict:
    """0x2fc93c0/0x2fdc2f0 argument tuple before the engine spawn call."""
    return {
        "end_distance_cm": f32(end_distance_cm),
        "start_distance_cm": f32(start_distance_cm),
        "tolerance_cm": f32(tolerance_cm),
        "speed_cm_s": f32(speed_cm_s),
    }


class FourScalarSource:
    """0x03363830/0x0339aa70 accessors and the +0x20 increment in 0x2fdc2f0."""

    def read_four(self) -> tuple[float, float, float, float]:
        raise EngineBoundaryError(
            "0x0339aa70",
            "the four float fields at the actor/component offsets read by 0x2fdc2f0",
        )

    def advance(self) -> None:
        raise EngineBoundaryError(
            "0x2fdc2f0",
            "the +0x20 counter and its zero-wrap rule",
        )


def four_scalar_driver(source: FourScalarSource, route) -> dict:
    """Translate 0x2fdc2f0: read four floats, advance +0x20, call 0x2fc93c0."""
    values = source.read_four()
    source.advance()
    return {
        "route": route,
        "call": direct_spawn_descriptor(*values),
        "function": "0x2fc93c0",
    }
