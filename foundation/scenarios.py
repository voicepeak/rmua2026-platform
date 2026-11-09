"""Explicit seeded obstacle profiles for the candidate PWM simulator.

The caller chooses dimensions, routes and optional feasibility checks; no
official obstacle distribution or static-map clearance is implied.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import numpy as np

from foundation.obstacles import CandidateMapObstacleFilter, generate_constrained, generate_random_vehicle_paths
from foundation.runtime import RmuaPhysics


@dataclass(frozen=True)
class ObstacleProfile:
    n_lines: int
    n_vehicles: int
    line_radius_m: float
    line_length_m: float
    vehicle_half_extents_m: tuple
    speed_range_m_s: tuple
    line_directions: tuple = ((0., 0., 1.),)
    vehicle_route_legs: int = 0
    route_leg_length_m: tuple = (3., 6.)
    turn_range_rad: tuple = (.1, 1.)
    stop_range_s: tuple = (0., 0.)
    vehicle_center_z_m: float | None = None

    def __post_init__(self):
        if (type(self.n_lines) is not int or type(self.n_vehicles) is not int
                or self.n_lines < 0 or self.n_vehicles < 0
                or type(self.vehicle_route_legs) is not int or self.vehicle_route_legs < 0):
            raise ValueError('obstacle counts must be nonnegative integers')
        if self.n_vehicles and (self.vehicle_center_z_m is None or not math.isfinite(self.vehicle_center_z_m)):
            raise ValueError('vehicle_center_z_m must be explicitly configured for vehicles')
        if self.vehicle_center_z_m is not None and not math.isfinite(self.vehicle_center_z_m):
            raise ValueError('vehicle_center_z_m must be finite')
        extents = np.asarray(self.vehicle_half_extents_m, float)
        speed = np.asarray(self.speed_range_m_s, float)
        directions = np.asarray(self.line_directions, float)
        lengths = np.asarray(self.route_leg_length_m, float)
        turns = np.asarray(self.turn_range_rad, float)
        stops = np.asarray(self.stop_range_s, float)
        if (not math.isfinite(self.line_radius_m) or self.line_radius_m < 0
                or not math.isfinite(self.line_length_m) or self.line_length_m <= 0
                or extents.shape != (3,) or not np.isfinite(extents).all() or np.any(extents <= 0)
                or speed.shape != (2,) or not np.isfinite(speed).all() or speed[0] < 0 or speed[1] < speed[0]
                or directions.ndim != 2 or directions.shape[1:] != (3,) or not len(directions)
                or not np.isfinite(directions).all() or np.any(np.linalg.norm(directions, axis=1) < 1e-12)
                or any(x.shape != (2,) or not np.isfinite(x).all() for x in (lengths, turns, stops))
                or lengths[0] <= 0 or lengths[1] < lengths[0]
                or turns[0] < 0 or turns[1] < turns[0] or turns[1] > math.pi
                or stops[0] < 0 or stops[1] < stops[0]
                or (self.vehicle_route_legs > 0 and speed[0] <= 0)):
            raise ValueError('invalid obstacle profile')
        object.__setattr__(self, 'vehicle_half_extents_m', tuple(float(x) for x in extents))
        object.__setattr__(self, 'speed_range_m_s', tuple(float(x) for x in speed))
        object.__setattr__(self, 'line_directions', tuple(tuple(float(x) for x in d) for d in directions))
        object.__setattr__(self, 'route_leg_length_m', tuple(float(x) for x in lengths))
        object.__setattr__(self, 'turn_range_rad', tuple(float(x) for x in turns))
        object.__setattr__(self, 'stop_range_s', tuple(float(x) for x in stops))


class ScenarioFactory:
    def __init__(self, *, bounds_min, bounds_max, launch, exclusion_m, protected_paths,
                 corridor_radius_m, horizon_s, profiles, fixed_seed, feasible=None,
                 obstacle_separation_m=0., vehicle_subdivisions=2):
        if type(fixed_seed) is not int or not 0 <= fixed_seed <= 0x7fffffff:
            raise ValueError('fixed_seed must be a nonnegative 32-bit integer')
        if type(vehicle_subdivisions) is not int or vehicle_subdivisions < 1:
            raise ValueError('vehicle_subdivisions must be a positive integer')
        if set(profiles) != {'fixed', 'random', 'difficult'} or any(not isinstance(x, ObstacleProfile) for x in profiles.values()):
            raise ValueError('provide explicit fixed, random and difficult profiles')
        if feasible is not None and not callable(feasible):
            raise ValueError('feasible must be callable')
        self.bounds_min = tuple(float(x) for x in bounds_min)
        self.bounds_max = tuple(float(x) for x in bounds_max)
        self.launch = tuple(float(x) for x in launch)
        self.paths = tuple(tuple(tuple(float(x) for x in p) for p in path) for path in protected_paths)
        self.exclusion = float(exclusion_m)
        self.corridor_radius = float(corridor_radius_m)
        self.horizon = float(horizon_s)
        self.obstacle_separation = float(obstacle_separation_m)
        self.vehicle_subdivisions = vehicle_subdivisions
        if (not all(math.isfinite(x) for x in (self.exclusion, self.corridor_radius, self.horizon, self.obstacle_separation))
                or self.horizon <= 0 or self.obstacle_separation < 0):
            raise ValueError('scene constraints must be finite and horizon must be positive')
        self.profiles = dict(profiles)
        self.fixed_seed = fixed_seed
        self.feasible = feasible

    def build(self, params, profile, seed, *, map_clearance_m=None, **runtime_options):
        if profile not in self.profiles:
            raise ValueError('unknown scenario profile')
        if type(seed) is not int or not 0 <= seed <= 0x7fffffff:
            raise ValueError('seed must be a nonnegative 32-bit integer')
        if 'lines' in runtime_options or 'vehicles' in runtime_options:
            raise ValueError('scenario factory owns lines and vehicles')
        options = dict(runtime_options)
        map_filter = None
        if map_clearance_m is not None:
            map_filter = CandidateMapObstacleFilter(options.get('candidate_map'),
                origin_ue_cm=options.get('map_origin_ue_cm'), horizon_s=self.horizon,
                clearance_m=map_clearance_m, vehicle_subdivisions=self.vehicle_subdivisions)
        def feasible(obj):
            return ((map_filter is None or map_filter(obj)) and
                    (self.feasible is None or self.feasible(obj)))
        drone_radius = options.get('drone_radius_m', .25)
        if not math.isfinite(drone_radius) or drone_radius < 0 or self.corridor_radius < drone_radius:
            raise ValueError('corridor radius must include the drone collision radius')
        # The generator only certifies obstacle motion over this horizon. Do
        # not silently let the episode continue after that guarantee expires.
        if 'timeout_s' not in options:
            options['timeout_s'] = self.horizon
        timeout = options['timeout_s']
        if timeout is None or not math.isfinite(timeout) or timeout <= 0 or timeout > self.horizon:
            raise ValueError('episode timeout must be positive and within the vehicle motion horizon')
        p = self.profiles[profile]
        effective_seed = self.fixed_seed if profile == 'fixed' else seed
        lines, vehicles = generate_constrained(
            effective_seed, self.bounds_min, self.bounds_max, self.launch, self.exclusion,
            p.n_lines, p.n_vehicles if p.vehicle_route_legs == 0 else 0,
            p.line_radius_m, p.line_length_m, p.vehicle_half_extents_m,
            p.speed_range_m_s, protected_paths=self.paths, corridor_radius_m=self.corridor_radius,
            horizon_s=self.horizon, line_directions=p.line_directions, feasible=feasible,
            vehicle_center_z_m=p.vehicle_center_z_m, obstacle_separation_m=self.obstacle_separation)
        if p.vehicle_route_legs:
            vehicles = generate_random_vehicle_paths(
                effective_seed ^ 0x5A17, p.n_vehicles, p.vehicle_route_legs, p.route_leg_length_m,
                p.turn_range_rad, p.vehicle_half_extents_m, p.speed_range_m_s, p.stop_range_s,
                self.bounds_min, self.bounds_max, self.launch, self.exclusion,
                protected_paths=self.paths, corridor_radius_m=self.corridor_radius, feasible=feasible,
                vehicle_center_z_m=p.vehicle_center_z_m, obstacle_separation_m=self.obstacle_separation,
                existing_obstacles=lines, horizon_s=self.horizon)
        geometry = {'lines': [asdict(x) for x in lines], 'vehicles': [asdict(x) for x in vehicles]}
        payload = json.dumps(geometry, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
        metadata = {'status': 'candidate; official distribution and map feasibility unverified',
                    'profile': profile, 'requested_seed': seed, 'effective_seed': effective_seed,
                    'profile_configuration': asdict(p), 'bounds_min_ned_m': self.bounds_min,
                    'bounds_max_ned_m': self.bounds_max, 'launch_ned_m': self.launch,
                    'launch_exclusion_m': self.exclusion, 'protected_paths_ned_m': self.paths,
                    'corridor_radius_m': self.corridor_radius, 'vehicle_motion_horizon_s': self.horizon,
                    'drone_radius_m': drone_radius, 'episode_timeout_s': timeout,
                    'obstacle_separation_m': self.obstacle_separation,
                    'vehicle_subdivisions': self.vehicle_subdivisions,
                    'external_feasibility_callback_used': self.feasible is not None,
                    'candidate_map_clearance_m': None if map_filter is None else map_filter.clearance_m,
                    'candidate_map_source_sha256': (None if map_filter is None else
                                                     getattr(map_filter.scene, 'source_sha256', None)),
                    'candidate_map_origin_ue_cm': None if map_filter is None else map_filter.origin.tolist(),
                    'geometry': geometry, 'geometry_sha256': hashlib.sha256(payload).hexdigest(),
                    'limitations': ['protected route and caller callback do not establish official map clearance',
                                    'constant configured vehicle center height does not establish road contact or terrain following',
                                    'straight vehicle trajectories are checked only over the specified horizon',
                                    'obstacle path envelopes are conservatively separated, independent of time',
                                    'candidate-map feasibility is partial and vehicle spheres may intersect the road',
                                    'wind and custom callbacks require separate seeding and state isolation']}
        return RmuaPhysics(params, lines=lines, vehicles=vehicles, **options), metadata
