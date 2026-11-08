"""Minimal single-vehicle research simulator; official parity is unverified.

State frame: NED metres, body angular rates FRD radians/s. Optional candidate
map, gate-plane and aerodynamic models are not officially validated. Wind and
rotor target scale must be supplied externally; gate events are not official
scores. Collision checks are proxies and the terminal pose is the end of the
substep containing the hit, not the exact time of contact.
"""
from dataclasses import replace
import math
import numpy as np
from foundation.quad import Quad
from foundation.obstacles import VehiclePath


class PhysicsSim:
    def __init__(self, params, *, lines=(), vehicles=(), force_replay=None,
                 drone_radius_m=.25, max_substep_s=.0025, candidate_map=None,
                 map_origin_ue_cm=None, aero_model=None, gate_volumes=None,
                 recorded_effects=None, out_time_counter=None, candidate_field=None, wind_field=None,
                 field_rotor_target_scale=None):
        if (not math.isfinite(drone_radius_m) or drone_radius_m < 0 or
                not math.isfinite(max_substep_s) or not 0 < max_substep_s <= .1):
            raise ValueError('invalid radius or substep')
        self.quad = Quad(params)
        self.lines = tuple(lines)
        self.initial_vehicles = tuple(vehicles)
        self.force_replay = force_replay
        if ((candidate_map is not None or gate_volumes is not None) != (map_origin_ue_cm is not None)):
            raise ValueError('candidate map or gates and explicit origin must be supplied together')
        if map_origin_ue_cm is not None:
            origin = np.asarray(map_origin_ue_cm, float)
            if origin.shape != (3,) or not np.isfinite(origin).all():
                raise ValueError('invalid map origin')
            self.map_origin_ue_cm = origin.copy()
        else:
            self.map_origin_ue_cm = None
        self.candidate_map = candidate_map
        self.gate_volumes = gate_volumes
        self.aero_model = aero_model
        if wind_field is not None and (not callable(getattr(wind_field, 'sample', None))
                                       or not callable(getattr(wind_field, 'check_window', None))):
            raise ValueError('wind field must provide sample and check_window')
        if wind_field is not None and aero_model is None:
            raise ValueError('wind field requires an explicit aerodynamic model')
        if wind_field is not None and recorded_effects is not None and recorded_effects.wind_ned_m_s is not None:
            raise ValueError('choose candidate wind field or recorded wind')
        self.wind_field = wind_field
        if (recorded_effects is not None and recorded_effects.enabled
                and recorded_effects.wind_ned_m_s is not None and aero_model is None):
            raise ValueError('recorded wind requires an aerodynamic model')
        if recorded_effects is not None and candidate_field is not None:
            raise ValueError('choose recorded flags or candidate spatial field')
        if out_time_counter is not None and recorded_effects is None and candidate_field is None:
            raise ValueError('out-time counter requires field flags')
        if field_rotor_target_scale is not None:
            if not math.isfinite(field_rotor_target_scale) or not 0 <= field_rotor_target_scale <= 1:
                raise ValueError('invalid field rotor target scale')
            field_rotor_target_scale = float(field_rotor_target_scale)
        needs_scale = (candidate_field is not None or
                       (recorded_effects is not None and recorded_effects.enabled and np.any(recorded_effects.flags == 0)))
        if needs_scale and field_rotor_target_scale is None:
            raise ValueError('field rotor target scale must be selected explicitly')
        self.field_rotor_target_scale = field_rotor_target_scale
        self.recorded_effects = recorded_effects
        self.candidate_field = candidate_field
        self.out_time_counter = out_time_counter
        self.initial_out_time = out_time_counter.elapsed_s if out_time_counter is not None else None
        self.out_time_exceeded = False
        self.drone_radius_m = drone_radius_m
        self.max_substep_s = max_substep_s
        self.terminal = False

    def reset(self, initial_state=None):
        self.quad.reset(**(initial_state or {}))
        self.terminal = False
        self.out_time_exceeded = False
        if self.out_time_counter is not None:
            self.out_time_counter.elapsed_s = self.initial_out_time
        return self.quad.state()

    def step(self, pwm4, dt, *, rotor_target_scale=None, wind_world_m_s=None, timeout_suppressed=False):
        if self.terminal:
            raise RuntimeError('simulation is terminal; reset before stepping')
        pwm = np.asarray(pwm4, float)
        if (self.recorded_effects is not None or self.candidate_field is not None) and rotor_target_scale is not None:
            raise ValueError('field model cannot be combined with explicit rotor scale')
        if (self.recorded_effects is not None and self.recorded_effects.wind_ned_m_s is not None
                and wind_world_m_s is not None):
            raise ValueError('recorded wind cannot be combined with explicit wind')
        if self.wind_field is not None and wind_world_m_s is not None:
            raise ValueError('candidate wind field cannot be combined with explicit wind')
        if rotor_target_scale is None:
            rotor_target_scale = 1.0
        if (pwm.shape != (4,) or not np.isfinite(pwm).all() or np.any((pwm < 0) | (pwm > 1))
                or not math.isfinite(dt) or not 0 < dt <= 1 or not isinstance(timeout_suppressed, bool)
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
        start_time = self.quad.time
        end_time = start_time + dt
        if self.wind_field is not None:
            self.wind_field.check_window(start_time, end_time)
        boundaries = list(np.linspace(start_time, end_time, count+1))
        if self.recorded_effects is not None:
            self.recorded_effects.check_window(start_time, end_time)
            boundaries.extend(self.recorded_effects.transitions(start_time, end_time))
        boundaries = sorted(set(boundaries))
        hit = None
        gate_events = []
        ambiguous_gate_events = []
        field_state = None
        containment_unknown = {}
        for begin, finish in zip(boundaries, boundaries[1:]):
            step = finish-begin
            if step <= 1e-12:
                continue
            start = self.quad.position.copy()
            current_time = self.quad.time
            field_flag = 1
            if self.recorded_effects is not None:
                scheduled_wind, field_flag = self.recorded_effects.sample(begin)
                if self.recorded_effects.wind_ned_m_s is not None:
                    wind = scheduled_wind
                rotor_target_scale = 1.0 if field_flag else self.field_rotor_target_scale
            elif self.candidate_field is not None:
                field_state = self.candidate_field.evaluate(start, rotation=self.quad.rotation)
                field_flag = field_state['flag']
                rotor_target_scale = 1.0 if field_flag else self.field_rotor_target_scale
            if rotor_target_scale is None:
                raise ValueError('field rotor target scale must be selected explicitly')
            if self.wind_field is not None:
                wind = self.wind_field.sample(begin, start)
            force = (self.force_replay.sample(current_time, start) if self.force_replay is not None
                     else np.zeros(3))
            torque = np.zeros(3)
            if self.aero_model is not None:
                drag_force, torque = self.aero_model.wrench(self.quad.rotation, self.quad.velocity,
                                                            self.quad.omega, wind)
                force = force + drag_force
            self.quad.step(pwm, step, external_force_world=force, external_torque_body=torque,
                           rotor_target_scale=rotor_target_scale)
            if self.out_time_counter is not None:
                self.out_time_exceeded |= self.out_time_counter.tick(field_flag, step, timeout_suppressed=timeout_suppressed)
            end = self.quad.position
            step_gates = []
            if self.gate_volumes is not None:
                for event in self.gate_volumes.crossings_ned(start, end, origin_ue_cm=self.map_origin_ue_cm):
                    step_gates.append({**event, 'time_s': current_time + event['fraction']*step})
            if self.candidate_map is not None:
                static = self.candidate_map.sweep_sphere_ned(start, end, self.drone_radius_m,
                                                             origin_ue_cm=self.map_origin_ue_cm)
                for item in static.get('containment_unknown', []):
                    key = (item.get('actor'), item.get('component'), item.get('mesh'), item.get('reason'))
                    containment_unknown[key] = item
                if static['hits']:
                    hit = {'kind': 'static_candidate', **static['hits'][0]}
            if hit is None:
                for index, line in enumerate(self.lines):
                    if line.hit_swept(start, end, self.drone_radius_m):
                        hit = {'kind': 'line', 'index': index}
                        break
            if hit is None:
                for index, vehicle in enumerate(self.initial_vehicles):
                    if isinstance(vehicle, VehiclePath):
                        vehicle_hit = vehicle.hit_swept_interval(start, end, current_time, step, self.drone_radius_m)
                    else:
                        offset = np.asarray(vehicle.velocity_m_s, float) * current_time
                        moving = replace(vehicle, center=tuple(np.asarray(vehicle.center, float) + offset))
                        vehicle_hit = moving.hit_swept(start, end, step, self.drone_radius_m)
                    if vehicle_hit:
                        hit = {'kind': 'vehicle', 'index': index}
                        break
            if hit is not None:
                # Contact has no time of impact; crossings in this substep may
                # be after contact and therefore must not count as crossings.
                ambiguous_gate_events.extend({**event, 'collision_order': 'unknown'} for event in step_gates)
                self.terminal = True
                break
            gate_events.extend(step_gates)
        state = self.quad.state()
        info = {'collision': hit, 'terminal': self.terminal, 'sim_time': state['t'],
                'gate_events': gate_events,
                'ambiguous_gate_events': ambiguous_gate_events,
                'gate_event_status': ('candidate_geometry_unverified' if self.gate_volumes is not None else 'disabled'),
                'static_map_collision': ('unverified_candidate_surfaces' if self.candidate_map is not None else 'unimplemented'),
                'map_containment_unknown': list(containment_unknown.values()),
                'wind_force': ('uncalibrated_drag_face_candidate' if self.aero_model is not None else 'unidentified'),
                'wind_field_status': ('unverified_spatial_wind_candidate' if self.wind_field is not None else 'disabled'),
                'force_degradation': ('recorded_flag_explicit_target_candidate' if self.recorded_effects is not None and self.field_rotor_target_scale is not None else
                                      'recorded_effects_without_field_scaling' if self.recorded_effects is not None else
                                      'unverified_spatial_union_explicit_target_candidate' if self.candidate_field is not None else 'unidentified'),
                'field_rotor_target_scale': self.field_rotor_target_scale,
                'candidate_field_state': field_state,
                'out_time_s': self.out_time_counter.elapsed_s if self.out_time_counter is not None else None,
                'out_time_exceeded': self.out_time_exceeded,
                'physics_status': 'uncalibrated_candidate'}
        return state, info
