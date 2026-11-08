"""PWM-only single-environment API over the uncalibrated research simulator.

Only PWM is a control action. Physics and command periods are separate: each
call holds one command for an integral number of fixed physics steps. State
truth is an explicit research-only method; no deployable estimator is implied.
"""
from __future__ import annotations

import math

import numpy as np

from foundation.sim import PhysicsSim


class RmuaPhysics:
    def __init__(self, params, *, physics_dt=0.0025, timeout_s=None, flight_bounds_ned_m=None, **scenario):
        if not math.isfinite(physics_dt) or not 0 < physics_dt <= 0.1:
            raise ValueError('invalid physics_dt')
        if 'max_substep_s' in scenario:
            raise ValueError('use physics_dt rather than max_substep_s')
        if timeout_s is not None:
            if not math.isfinite(timeout_s) or timeout_s <= 0:
                raise ValueError('invalid timeout_s')
            ticks = timeout_s / physics_dt
            if round(ticks) < 1 or not math.isclose(ticks, round(ticks), rel_tol=0, abs_tol=1e-8):
                raise ValueError('timeout_s must be at least one whole physics step')
            self.timeout_ticks = round(ticks)
        else:
            self.timeout_ticks = None
        self.flight_bounds = None
        if flight_bounds_ned_m is not None:
            bounds = np.asarray(flight_bounds_ned_m, dtype=float)
            if bounds.shape != (2, 3) or not np.isfinite(bounds).all() or np.any(bounds[0] >= bounds[1]):
                raise ValueError('flight bounds must contain finite NED min/max vectors')
            self.flight_bounds = bounds.copy()
        self.physics_dt = float(physics_dt)
        self.sim = PhysicsSim(params, max_substep_s=self.physics_dt, **scenario)
        self.ticks = 0
        self.terminal = False

    def _observation(self):
        return {'time_s': self.sim.quad.time, 'observable_state': None}

    def truth_state(self):
        """Explicit debug/training truth; never returned in default observations."""
        return self.sim.quad.state()

    def reset(self, initial_state=None):
        self.sim.reset(initial_state)
        self.ticks = 0
        self.terminal = False
        if self.flight_bounds is not None and self._outside_bounds():
            self.terminal = True
            raise ValueError('initial position is outside candidate flight bounds')
        return self._observation()

    def _outside_bounds(self):
        position = self.sim.quad.position
        return bool(np.any(position < self.flight_bounds[0]) or np.any(position > self.flight_bounds[1]))

    def _prepare_step(self, pwm, dt):
        if self.terminal:
            raise RuntimeError('simulation is terminal; reset before stepping')
        if self.flight_bounds is not None and self._outside_bounds():
            raise ValueError('initial position is outside candidate flight bounds')
        command = np.asarray(pwm, dtype=float)
        if (command.shape != (4,) or not np.isfinite(command).all()
                or np.any((command < 0) | (command > 1))):
            raise ValueError('PWM must contain four finite values in candidate range [0, 1]')
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('invalid command duration')
        count = dt / self.physics_dt
        if not math.isclose(count, round(count), rel_tol=0, abs_tol=1e-8) or round(count) < 1:
            raise ValueError('duration must be a whole number of physics steps')
        steps = round(count)
        if self.timeout_ticks is not None:
            steps = min(steps, self.timeout_ticks - self.ticks)
        # Reject uncovered schedules before mutating state. The preflight is
        # conservative if a collision would have ended this command earlier.
        start = self.sim.quad.time
        end = start + steps * self.physics_dt
        replay = self.sim.force_replay
        if replay is not None and replay.enabled:
            if start < replay.times_s[0] or end > replay.times_s[-1] + 1e-12:
                raise ValueError('step exceeds force replay interval')
        if self.sim.recorded_effects is not None:
            self.sim.recorded_effects.check_window(start, end)
        if self.sim.wind_field is not None:
            self.sim.wind_field.check_window(start, end)
        return command.copy(), steps

    def step(self, pwm, dt):
        command, steps = self._prepare_step(pwm, dt)
        gates = []
        ambiguous_gates = []
        containment_unknown = {}
        last_info = None
        out_of_bounds = False
        for _ in range(steps):
            _, last_info = self.sim.step(command, self.physics_dt)
            self.ticks += 1
            out_of_bounds = self.flight_bounds is not None and self._outside_bounds()
            if out_of_bounds:
                # The physics step does not resolve the boundary-exit time.
                ambiguous_gates.extend({**event, 'boundary_order': 'unknown'} for event in last_info['gate_events'])
            else:
                gates.extend(last_info['gate_events'])
            ambiguous_gates.extend(last_info['ambiguous_gate_events'])
            for item in last_info['map_containment_unknown']:
                key = (item.get('actor'), item.get('component'), item.get('mesh'), item.get('reason'))
                containment_unknown[key] = item
            if last_info['terminal'] or out_of_bounds or (self.timeout_ticks is not None and self.ticks >= self.timeout_ticks):
                self.terminal = True
                break
        events = {'contact': last_info['collision'], 'gate_crossing_candidate': gates,
                  'ambiguous_gate_crossing': ambiguous_gates,
                  'force_zone': last_info['candidate_field_state'],
                  'force_zone_elapsed_s': last_info['out_time_s'],
                  'force_zone_timeout_candidate': last_info['out_time_exceeded'],
                  'timeout': self.timeout_ticks is not None and self.ticks >= self.timeout_ticks,
                  'out_of_bounds_candidate': out_of_bounds,
                  'terminal': self.terminal, 'physics_status': last_info['physics_status'],
                  'map_status': last_info['static_map_collision'],
                  'map_containment_unknown': list(containment_unknown.values()),
                  'wind_status': last_info['wind_force'],
                  'wind_field_status': last_info['wind_field_status'],
                  'force_degradation_status': last_info['force_degradation'],
                  'field_rotor_target_scale': last_info['field_rotor_target_scale'],
                  'completed_physics_steps': self.ticks}
        return self._observation(), events
