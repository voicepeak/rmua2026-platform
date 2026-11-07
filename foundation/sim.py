"""Minimal single-vehicle research simulator; official parity is unverified.

State frame: NED metres, body angular rates FRD radians/s. This simulation does
not yet implement verified official static-map contact, wind-to-force coupling, force
degradation, gate scoring, or official stage logic. Collision checks below are
training proxies, and the terminal pose is the end of the substep containing
the hit, not the exact time of contact.
"""
from dataclasses import replace
import math
import numpy as np
from foundation.quad import Quad


class PhysicsSim:
    def __init__(self, params, *, lines=(), vehicles=(), force_replay=None,
                 drone_radius_m=.25, max_substep_s=.0025, candidate_map=None,
                 map_origin_ue_cm=None, aero_model=None):
        if (not math.isfinite(drone_radius_m) or drone_radius_m < 0 or
                not math.isfinite(max_substep_s) or not 0 < max_substep_s <= .1):
            raise ValueError('invalid radius or substep')
        self.quad = Quad(params)
        self.lines = tuple(lines)
        self.initial_vehicles = tuple(vehicles)
        self.force_replay = force_replay
        if (candidate_map is None) != (map_origin_ue_cm is None):
            raise ValueError('candidate map and explicit origin must be supplied together')
        if map_origin_ue_cm is not None:
            origin = np.asarray(map_origin_ue_cm, float)
            if origin.shape != (3,) or not np.isfinite(origin).all():
                raise ValueError('invalid map origin')
            self.map_origin_ue_cm = origin.copy()
        else:
            self.map_origin_ue_cm = None
        self.candidate_map = candidate_map
        self.aero_model = aero_model
        self.drone_radius_m = drone_radius_m
        self.max_substep_s = max_substep_s
        self.terminal = False

    def reset(self, initial_state=None):
        self.quad.reset(**(initial_state or {}))
        self.terminal = False
        return self.quad.state()

    def step(self, pwm4, dt, *, rotor_target_scale=1.0, wind_world_m_s=None):
        if self.terminal:
            raise RuntimeError('simulation is terminal; reset before stepping')
        pwm = np.asarray(pwm4, float)
        if (pwm.shape != (4,) or not np.isfinite(pwm).all() or np.any((pwm < 0) | (pwm > 1))
                or not math.isfinite(dt) or not 0 < dt <= 1
                or not math.isfinite(rotor_target_scale) or not 0 <= rotor_target_scale <= 1):
            raise ValueError('invalid PWM or timestep')
        if wind_world_m_s is not None and self.aero_model is None:
            raise ValueError('wind requires an explicit candidate aerodynamic model')
        wind = np.zeros(3) if wind_world_m_s is None else np.asarray(wind_world_m_s, float)
        if wind.shape != (3,) or not np.isfinite(wind).all():
            raise ValueError('invalid wind vector')
        if self.force_replay is not None and self.force_replay.enabled:
            t = self.quad.time
            if t < self.force_replay.times_s[0] or t + dt > self.force_replay.times_s[-1] + 1e-12:
                raise ValueError('step exceeds force replay interval')
        count = math.ceil(dt / self.max_substep_s)
        step = dt / count
        hit = None
        for _ in range(count):
            start = self.quad.position.copy()
            current_time = self.quad.time
            force = (self.force_replay.sample(current_time, start) if self.force_replay is not None
                     else np.zeros(3))
            torque = np.zeros(3)
            if self.aero_model is not None:
                drag_force, torque = self.aero_model.wrench(self.quad.rotation, self.quad.velocity,
                                                            self.quad.omega, wind)
                force = force + drag_force
            self.quad.step(pwm, step, external_force_world=force, external_torque_body=torque,
                           rotor_target_scale=rotor_target_scale)
            end = self.quad.position
            if self.candidate_map is not None:
                static = self.candidate_map.sweep_sphere_ned(start, end, self.drone_radius_m,
                                                             origin_ue_cm=self.map_origin_ue_cm)
                if static['hits']:
                    hit = {'kind': 'static_candidate', **static['hits'][0]}
            if hit is None:
                for index, line in enumerate(self.lines):
                    if line.hit_swept(start, end, self.drone_radius_m):
                        hit = {'kind': 'line', 'index': index}
                        break
            if hit is None:
                for index, vehicle in enumerate(self.initial_vehicles):
                    offset = np.asarray(vehicle.velocity_m_s, float) * current_time
                    moving = replace(vehicle, center=tuple(np.asarray(vehicle.center, float) + offset))
                    if moving.hit_swept(start, end, step, self.drone_radius_m):
                        hit = {'kind': 'vehicle', 'index': index}
                        break
            if hit is not None:
                self.terminal = True
                break
        state = self.quad.state()
        info = {'collision': hit, 'terminal': self.terminal, 'sim_time': state['t'],
                'static_map_collision': ('unverified_candidate_surfaces' if self.candidate_map is not None else 'unimplemented'),
                'wind_force': ('uncalibrated_drag_face_candidate' if self.aero_model is not None else 'unidentified'),
                'force_degradation': 'unidentified', 'physics_status': 'uncalibrated_candidate'}
        return state, info
