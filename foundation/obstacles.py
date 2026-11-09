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


@dataclass(frozen=True)
class VehiclePath:
    """Piecewise-linear NED path; yaw is constant within each path segment.

    Repeated points produce stops. Instantaneous yaw changes are conservatively
    covered by the vehicle's circumscribed sphere, not an exact turning model.
    The vehicle remains stationary at the final waypoint after the last time.
    """
    times_s: tuple
    positions_m: tuple
    half_extents_m: tuple
    yaw_rad: tuple

    def __post_init__(self):
        times = np.asarray(self.times_s, float)
        positions = np.asarray(self.positions_m, float)
        ext = vec(self.half_extents_m)
        yaw = np.asarray(self.yaw_rad, float)
        if (times.ndim != 1 or len(times) < 2 or times[0] != 0 or
                not np.isfinite(times).all() or np.any(np.diff(times) <= 0) or
                positions.shape != (len(times), 3) or not np.isfinite(positions).all() or
                yaw.shape != (len(times)-1,) or not np.isfinite(yaw).all() or np.any(ext <= 0)):
            raise ValueError('invalid vehicle path')

    def hit_swept_interval(self, drone_start, drone_end, start_s, dt, drone_radius_m):
        a, b = vec(drone_start), vec(drone_end)
        if not math.isfinite(start_s) or start_s < 0 or not math.isfinite(dt) or dt < 0:
            raise ValueError('invalid path interval')
        if not math.isfinite(drone_radius_m) or drone_radius_m < 0:
            raise ValueError('invalid drone radius')
        times = np.asarray(self.times_s, float)
        points = np.asarray(self.positions_m, float)
        end_s = start_s + dt
        boundaries = [start_s, *(t for t in times[1:] if start_s < t < end_s), end_s]
        if dt == 0:
            boundaries = [start_s, start_s]
        for begin, end in zip(boundaries, boundaries[1:]):
            index = min(np.searchsorted(times, begin, side='right')-1, len(times)-2)
            if begin >= times[-1]:
                center, velocity = points[-1], np.zeros(3)
            else:
                velocity = (points[index+1]-points[index])/(times[index+1]-times[index])
                center = points[index] + velocity*(begin-times[index])
            f0 = (begin-start_s)/dt if dt else 0
            f1 = (end-start_s)/dt if dt else 0
            v = Vehicle(tuple(center), self.half_extents_m, tuple(velocity), self.yaw_rad[index])
            if v.hit_swept(a+(b-a)*f0, a+(b-a)*f1, end-begin, drone_radius_m):
                return True
        # At a turn, the physical interpolation of yaw is unspecified. A
        # circumscribed sphere bounds every intermediate orientation.
        for i in range(1, len(times)-1):
            if start_s <= times[i] <= end_s and self.yaw_rad[i-1] != self.yaw_rad[i]:
                f = (times[i]-start_s)/dt if dt else 0
                if np.linalg.norm(a+(b-a)*f-points[i]) <= np.linalg.norm(self.half_extents_m)+drone_radius_m:
                    return True
        return False


def generate(seed, bounds_min, bounds_max, launch, exclusion_m, n_lines, n_vehicles,
             line_radius_m, line_length_m, vehicle_half_extents_m, speed_range_m_s,
             *, line_directions=((0, 0, 1),), feasible=None, vehicle_center_z_m=None):
    """Sample full obstacles inside bounds; optional feasibility must be supplied externally.

    ``feasible`` receives each candidate obstacle and must return true before
    acceptance. A partial static-map miss is not proof of road feasibility.
    """
    low, high, launch, ext = map(vec, (bounds_min, bounds_max, launch, vehicle_half_extents_m))
    params = (exclusion_m, line_radius_m, line_length_m, *speed_range_m_s)
    if (np.any(low >= high) or np.any(ext <= 0) or n_lines < 0 or n_vehicles < 0 or
            any(not math.isfinite(x) for x in params) or exclusion_m < 0 or line_radius_m < 0 or
            line_length_m <= 0 or speed_range_m_s[0] < 0 or speed_range_m_s[0] > speed_range_m_s[1]):
        raise ValueError('invalid generation parameters')
    directions = np.asarray(line_directions, float)
    if (directions.ndim != 2 or directions.shape[1] != 3 or not len(directions)
            or not np.isfinite(directions).all() or np.any(np.linalg.norm(directions, axis=1) < 1e-12)):
        raise ValueError('invalid line directions')
    directions = directions / np.linalg.norm(directions, axis=1)[:, None]
    if feasible is not None and not callable(feasible):
        raise ValueError('feasible must be callable')
    if vehicle_center_z_m is not None and (not math.isfinite(vehicle_center_z_m)
            or (n_vehicles and not low[2]+ext[2] <= vehicle_center_z_m <= high[2]-ext[2])):
        raise ValueError('vehicle center height is outside bounds')
    rng = np.random.default_rng(seed)
    lines = []
    for _ in range(n_lines):
        for _ in range(10000):
            direction = directions[int(rng.integers(len(directions)))]
            delta = direction*line_length_m/2
            margin = np.abs(delta)+line_radius_m
            if np.any(low+margin > high-margin):
                continue
            p = rng.uniform(low+margin, high-margin)
            candidate = Line(tuple(p-delta), tuple(p+delta), line_radius_m)
            if not candidate.hit_swept(launch, launch, exclusion_m) and (feasible is None or feasible(candidate)):
                lines.append(candidate)
                break
        else:
            raise ValueError('cannot place feasible line')
    vehicles = []
    for _ in range(n_vehicles):
        for _ in range(10000):
            yaw = float(rng.uniform(-np.pi, np.pi))
            c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
            margin = np.array([c*ext[0]+s*ext[1], s*ext[0]+c*ext[1], ext[2]])
            if np.any(low+margin > high-margin):
                continue
            p = rng.uniform(low+margin, high-margin)
            speed = float(rng.uniform(*speed_range_m_s))
            if vehicle_center_z_m is not None:
                p[2] = vehicle_center_z_m
            candidate = Vehicle(tuple(p), tuple(ext), (speed*math.cos(yaw), speed*math.sin(yaw), 0.), yaw)
            if not candidate.hit_swept(launch, launch, 0, exclusion_m) and (feasible is None or feasible(candidate)):
                vehicles.append(candidate)
                break
        else:
            raise ValueError('cannot place feasible vehicle')
    return lines, vehicles


def _obstacle_envelope(obstacle, horizon_s):
    if isinstance(obstacle, Line):
        return [(vec(obstacle.start), vec(obstacle.end))], float(obstacle.radius_m)
    if isinstance(obstacle, Vehicle):
        start = vec(obstacle.center)
        return [(start, start + vec(obstacle.velocity_m_s) * horizon_s)], float(np.linalg.norm(vec(obstacle.half_extents_m)))
    if isinstance(obstacle, VehiclePath):
        points = [vec(p) for p in obstacle.positions_m]
        return list(zip(points[:-1], points[1:])), float(np.linalg.norm(vec(obstacle.half_extents_m)))
    raise ValueError('unsupported obstacle type for clearance')


def _subbox_spheres(half_extents, subdivisions):
    """Cover a box by a union of conservative sub-box circumscribed spheres."""
    ext = vec(half_extents)
    if type(subdivisions) is not int or subdivisions < 1:
        raise ValueError('subdivisions must be a positive integer')
    step = 2 * ext / subdivisions
    axes = [[-ext[i] + (index + .5) * step[i] for index in range(subdivisions)]
            for i in range(3)]
    # Build the Cartesian product without importing itertools at module scope.
    offsets = []
    for x in axes[0]:
        for y in axes[1]:
            for z in axes[2]:
                offsets.append(np.array([x, y, z]))
    radius = float(np.linalg.norm(ext / subdivisions))
    return offsets, radius


def _obstacle_spheres(obstacle, horizon_s, *, vehicle_subdivisions=2):
    """Return conservative sphere sweeps for a partial-map clearance check."""
    if isinstance(obstacle, Line):
        return [(vec(obstacle.start), vec(obstacle.end), float(obstacle.radius_m))]
    if isinstance(obstacle, (Vehicle, VehiclePath)):
        ext = vec(obstacle.half_extents_m)
        offsets, radius = _subbox_spheres(ext, vehicle_subdivisions)
        if isinstance(obstacle, Vehicle):
            center, velocity = vec(obstacle.center), vec(obstacle.velocity_m_s)
            c, s = math.cos(obstacle.yaw_rad), math.sin(obstacle.yaw_rad)
            result = []
            for offset in offsets:
                world = np.array([c*offset[0] - s*offset[1], s*offset[0] + c*offset[1], offset[2]])
                start = center + world
                result.append((start, start + velocity * horizon_s, radius))
            return result
        points, yaw = [vec(p) for p in obstacle.positions_m], np.asarray(obstacle.yaw_rad, float)
        result = []
        for index, (start, end) in enumerate(zip(points[:-1], points[1:])):
            c, s = math.cos(yaw[index]), math.sin(yaw[index])
            for offset in offsets:
                world = np.array([c*offset[0] - s*offset[1], s*offset[0] + c*offset[1], offset[2]])
                result.append((start + world, end + world, radius))
        # Instantaneous yaw changes have unspecified intermediate orientations.
        # The circumscribed sphere at each changed waypoint remains conservative.
        full_radius = float(np.linalg.norm(ext))
        for index in range(1, len(points)-1):
            if yaw[index-1] != yaw[index]:
                result.append((points[index], points[index], full_radius))
        return result
    raise ValueError('unsupported obstacle type for clearance')


class CandidateMapObstacleFilter:
    """Conservative path-envelope check against an explicitly partial map.

    A miss is not official free-space evidence. Vehicles are covered by a union
    of conservative sub-box spheres; increasing ``vehicle_subdivisions`` reduces
    the false-rejection margin at the cost of more map queries. Exact wheel/road
    contact still remains outside this candidate check.
    """
    def __init__(self, scene, *, origin_ue_cm, horizon_s, clearance_m=0., vehicle_subdivisions=2):
        from foundation.map_scene import CandidateMap
        if not isinstance(scene, CandidateMap) or not scene.assume_closed_meshes_solid:
            raise ValueError('candidate map must explicitly enable closed-mesh solid assumption')
        self.scene = scene
        self.origin = vec(origin_ue_cm).copy()
        if not math.isfinite(horizon_s) or horizon_s <= 0 or not math.isfinite(clearance_m) or clearance_m < 0:
            raise ValueError('invalid map clearance or horizon')
        if type(vehicle_subdivisions) is not int or vehicle_subdivisions < 1:
            raise ValueError('vehicle_subdivisions must be a positive integer')
        self.horizon_s = float(horizon_s)
        self.clearance_m = float(clearance_m)
        self.vehicle_subdivisions = vehicle_subdivisions

    def __call__(self, obstacle):
        spheres = _obstacle_spheres(obstacle, self.horizon_s,
                                    vehicle_subdivisions=self.vehicle_subdivisions)
        for start, end, radius in spheres:
            result = self.scene.sweep_sphere_ned(start, end, radius + self.clearance_m,
                                                 origin_ue_cm=self.origin)
            if result['hits'] or result['containment_unknown']:
                return False
        return True


def _obstacle_conflict(a, b, horizon_s, separation_m):
    paths_a, radius_a = _obstacle_envelope(a, horizon_s)
    paths_b, radius_b = _obstacle_envelope(b, horizon_s)
    limit2 = (radius_a + radius_b + separation_m) ** 2
    return any(segment_distance_squared(x0, x1, y0, y1) <= limit2
               for x0, x1 in paths_a for y0, y1 in paths_b)


def generate_constrained(seed, bounds_min, bounds_max, launch, exclusion_m, n_lines, n_vehicles,
                         line_radius_m, line_length_m, vehicle_half_extents_m, speed_range_m_s,
                         *, protected_paths, corridor_radius_m, horizon_s,
                         line_directions=((0, 0, 1),), feasible=None, vehicle_center_z_m=None,
                         obstacle_separation_m=None):
    """Generate reproducible obstacles outside supplied protected route tubes.

    The complete constant-velocity vehicle trajectory over ``horizon_s`` is
    checked using a bounding sphere. This is conservative and can reject
    feasible scenarios. Caller-provided paths/clearance and optional trusted
    geometry predicate determine whether routes are actually flyable; partial
    map misses and this route check alone never prove official clearance. If
    obstacle_separation_m is set, candidate path envelopes must be disjoint.
    """
    low, high, launch_v, ext = map(vec, (bounds_min, bounds_max, launch, vehicle_half_extents_m))
    if (not math.isfinite(corridor_radius_m) or corridor_radius_m <= 0
            or not math.isfinite(horizon_s) or horizon_s < 0):
        raise ValueError('invalid corridor radius or horizon')
    if feasible is not None and not callable(feasible):
        raise ValueError('feasible must be callable')
    if obstacle_separation_m is not None and (not math.isfinite(obstacle_separation_m) or obstacle_separation_m < 0):
        raise ValueError('invalid obstacle separation')
    paths = []
    for path in protected_paths:
        points = np.asarray(path, float)
        if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 2
                or not np.isfinite(points).all() or np.any(points < low) or np.any(points > high)):
            raise ValueError('invalid protected path')
        paths.extend(zip(points[:-1], points[1:]))
    if not paths:
        raise ValueError('at least one protected path is required')
    accepted = []

    def accept(obj):
        if isinstance(obj, Line):
            a, b = vec(obj.start), vec(obj.end)
            clearance = corridor_radius_m + obj.radius_m
        else:
            a, b = vec(obj.center), vec(obj.center) + vec(obj.velocity_m_s) * horizon_s
            c, s = abs(math.cos(obj.yaw_rad)), abs(math.sin(obj.yaw_rad))
            margin = np.array([c*ext[0] + s*ext[1], s*ext[0] + c*ext[1], ext[2]])
            if np.any(np.minimum(a, b) - margin < low) or np.any(np.maximum(a, b) + margin > high):
                return False
            radius = float(np.linalg.norm(ext))
            if segment_distance_squared(a, b, launch_v, launch_v) <= (exclusion_m + radius)**2:
                return False
            clearance = corridor_radius_m + radius
        if any(segment_distance_squared(a, b, p, q) <= clearance**2 for p, q in paths):
            return False
        if obstacle_separation_m is not None and any(_obstacle_conflict(obj, prior, horizon_s, obstacle_separation_m) for prior in accepted):
            return False
        if feasible is not None and not feasible(obj):
            return False
        if obstacle_separation_m is not None:
            accepted.append(obj)
        return True

    return generate(seed, low, high, launch_v, exclusion_m, n_lines, n_vehicles,
                    line_radius_m, line_length_m, ext, speed_range_m_s,
                    line_directions=line_directions, feasible=accept, vehicle_center_z_m=vehicle_center_z_m)


def generate_vehicle_paths(seed, routes, half_extents_m, speed_range_m_s, stop_range_s,
                           bounds_min, bounds_max, launch, exclusion_m, *, feasible):
    """Time caller-supplied horizontal routes, with seeded speeds and stops.

    A mandatory callback must evaluate the whole path against the caller's
    trusted geometry/road constraints. Bounding spheres are conservative for
    scene and launch clearance, including instantaneous turns. The callback's
    quality determines feasibility; partial map misses do not prove clearance.
    """
    low, high, launch, ext = map(vec, (bounds_min, bounds_max, launch, half_extents_m))
    speed_range = np.asarray(speed_range_m_s, float)
    stop_range = np.asarray(stop_range_s, float)
    if (np.any(low >= high) or np.any(ext <= 0) or not math.isfinite(exclusion_m) or exclusion_m < 0
            or speed_range.shape != (2,) or stop_range.shape != (2,)
            or not np.isfinite(speed_range).all() or not np.isfinite(stop_range).all()
            or speed_range[0] <= 0 or speed_range[1] < speed_range[0]
            or stop_range[0] < 0 or stop_range[1] < stop_range[0] or not callable(feasible)):
        raise ValueError('invalid path generation parameters')
    radius = float(np.linalg.norm(ext))
    rng = np.random.default_rng(seed)
    result = []
    for route in routes:
        points = np.asarray(route, float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
            raise ValueError('invalid route')
        if np.any(points-radius < low) or np.any(points+radius > high):
            raise ValueError('route vehicle exceeds bounds')
        times, path, yaw = [0.], [tuple(points[0])], []
        for index, (start, end) in enumerate(zip(points, points[1:])):
            delta = end-start
            distance = float(np.linalg.norm(delta))
            if distance < 1e-9 or abs(delta[2]) > 1e-9:
                raise ValueError('route legs must be nonzero and horizontal')
            if segment_distance_squared(start, end, launch, launch) <= (radius+exclusion_m)**2:
                raise ValueError('route enters launch exclusion')
            heading = math.atan2(delta[1], delta[0])
            speed = float(rng.uniform(*speed_range))
            times.append(times[-1]+distance/speed)
            path.append(tuple(end))
            yaw.append(heading)
            if index < len(points)-2:
                stop = float(rng.uniform(*stop_range))
                if stop > 0:
                    times.append(times[-1]+stop)
                    path.append(tuple(end))
                    yaw.append(heading)
        vehicle = VehiclePath(tuple(times), tuple(path), tuple(ext), tuple(yaw))
        if not feasible(vehicle):
            raise ValueError('route rejected by feasibility callback')
        result.append(vehicle)
    return result


def generate_random_vehicle_paths(seed, count, legs, leg_length_range_m, turn_range_rad,
                                  half_extents_m, speed_range_m_s, stop_range_s,
                                  bounds_min, bounds_max, launch, exclusion_m,
                                  *, protected_paths, corridor_radius_m, feasible=None, max_attempts=10000,
                                  vehicle_center_z_m=None, obstacle_separation_m=None,
                                  existing_obstacles=(), horizon_s=0.):
    """Create seeded horizontal turning/stopping paths outside protected tubes.

    Whole paths use conservative vehicle bounding spheres. Optional envelope
    separation ignores time and can reject paths that would not actually meet.
    Geometry/road feasibility and official random distributions remain unknown.
    """
    low, high, launch_v, ext = map(vec, (bounds_min, bounds_max, launch, half_extents_m))
    lengths, turns = np.asarray(leg_length_range_m, float), np.asarray(turn_range_rad, float)
    speeds, stops = np.asarray(speed_range_m_s, float), np.asarray(stop_range_s, float)
    if (type(seed) is not int or seed < 0 or type(count) is not int or count < 0
            or type(legs) is not int or legs < 1 or type(max_attempts) is not int or max_attempts < 1
            or np.any(low >= high) or np.any(ext <= 0)
            or not math.isfinite(exclusion_m) or exclusion_m < 0
            or not math.isfinite(corridor_radius_m) or corridor_radius_m <= 0
            or any(x.shape != (2,) or not np.isfinite(x).all() for x in (lengths, turns, speeds, stops))
            or lengths[0] <= 0 or lengths[1] < lengths[0]
            or turns[0] < 0 or turns[1] < turns[0] or turns[1] > math.pi
            or speeds[0] <= 0 or speeds[1] < speeds[0]
            or stops[0] < 0 or stops[1] < stops[0]
            or not math.isfinite(horizon_s) or horizon_s < 0
            or (obstacle_separation_m is not None and (not math.isfinite(obstacle_separation_m) or obstacle_separation_m < 0))
            or (feasible is not None and not callable(feasible))):
        raise ValueError('invalid random vehicle path configuration')
    protected = []
    for path in protected_paths:
        p = np.asarray(path, float)
        if p.ndim != 2 or p.shape[1] != 3 or len(p) < 2 or not np.isfinite(p).all() or np.any(p < low) or np.any(p > high):
            raise ValueError('invalid protected path')
        protected.extend(zip(p[:-1], p[1:]))
    if not protected:
        raise ValueError('at least one protected path is required')
    radius = float(np.linalg.norm(ext))
    if np.any(low + radius >= high - radius):
        raise ValueError('bounds too small for vehicle')
    if vehicle_center_z_m is not None and (not math.isfinite(vehicle_center_z_m)
            or not low[2]+radius <= vehicle_center_z_m <= high[2]-radius):
        raise ValueError('vehicle center height is outside conservative bounds')
    rng = np.random.default_rng(seed)
    existing = tuple(existing_obstacles)
    if obstacle_separation_m is not None:
        for obstacle in existing:
            _obstacle_envelope(obstacle, horizon_s)
    result = []
    for _ in range(count):
        for _ in range(max_attempts):
            point = rng.uniform(low + radius, high - radius)
            if vehicle_center_z_m is not None:
                point[2] = vehicle_center_z_m
            route = [point]
            heading = float(rng.uniform(-math.pi, math.pi))
            valid = True
            for leg in range(legs):
                if leg:
                    heading += float(rng.uniform(*turns)) * (1 if rng.integers(2) else -1)
                length = float(rng.uniform(*lengths))
                target = point + np.array([length*math.cos(heading), length*math.sin(heading), 0.])
                if (np.any(target-radius < low) or np.any(target+radius > high)
                        or segment_distance_squared(point, target, launch_v, launch_v) <= (radius+exclusion_m)**2
                        or any(segment_distance_squared(point, target, a, b) <= (radius+corridor_radius_m)**2
                               for a, b in protected)):
                    valid = False
                    break
                route.append(target)
                point = target
            if not valid:
                continue
            # Route geometry is already checked above; the helper adds seeded
            # travel speed and stops, and repeats bounds/launch checks.
            path_seed = int(rng.integers(0, 0x80000000))
            vehicle = generate_vehicle_paths(path_seed, [route], ext, speeds, stops, low, high,
                                             launch_v, exclusion_m, feasible=lambda _: True)[0]
            if (obstacle_separation_m is not None and any(_obstacle_conflict(vehicle, prior, horizon_s, obstacle_separation_m)
                                                         for prior in (*existing, *result))):
                continue
            if feasible is None or feasible(vehicle):
                result.append(vehicle)
                break
        else:
            raise ValueError('cannot generate feasible turning vehicle path')
    return result
