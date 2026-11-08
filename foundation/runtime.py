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
    def __init__(self, params, *, physics_dt=0.0025, timeout_s=None, **scenario):
        if not math.isfinite(physics_dt) or not 0 < physics_dt <= 0.1:
            raise ValueError('invalid physics_dt')
        if 'max_substep_s' in scenario:
            raise ValueError('use physics_dt rather than max_substep_s')
        if timeout_s is not None:
            if not math.isfinite(timeout_s) or timeout_s <= 0:
                raise ValueError('invalid timeout_s')
            ticks = timeout_s / physics_dt
            if not math.isclose(ticks, round(ticks), rel_tol=0, abs_tol=1e-8):
                raise ValueError('timeout_s must be a whole number of physics steps')
            self.timeout_ticks = round(ticks)
        else:
            self.timeout_ticks = None
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
        return self._observation()

    def step(self, pwm, dt):
        if self.terminal:
            raise RuntimeError('simulation is terminal; reset before stepping')
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
        gates = []
        last_info = None
        for _ in range(steps):
            _, last_info = self.sim.step(command, self.physics_dt)
            self.ticks += 1
            gates.extend(last_info['gate_events'])
            if last_info['terminal'] or (self.timeout_ticks is not None and self.ticks >= self.timeout_ticks):
                self.terminal = True
                break
        events = {'contact': last_info['collision'], 'gate_crossing_candidate': gates,
                  'force_zone': last_info['candidate_field_state'],
                  'force_zone_elapsed_s': last_info['out_time_s'],
                  'force_zone_timeout_candidate': last_info['out_time_exceeded'],
                  'timeout': self.timeout_ticks is not None and self.ticks >= self.timeout_ticks,
                  'terminal': self.terminal, 'physics_status': last_info['physics_status'],
                  'completed_physics_steps': self.ticks}
        return self._observation(), events
