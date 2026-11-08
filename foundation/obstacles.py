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
             *, line_directions=((0, 0, 1),), feasible=None):
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
            candidate = Vehicle(tuple(p), tuple(ext), (speed*math.cos(yaw), speed*math.sin(yaw), 0.), yaw)
            if not candidate.hit_swept(launch, launch, 0, exclusion_m) and (feasible is None or feasible(candidate)):
                vehicles.append(candidate)
                break
        else:
            raise ValueError('cannot place feasible vehicle')
    return lines, vehicles


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
