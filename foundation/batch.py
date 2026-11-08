"""Batch PWM environments with a vectorized no-effects free-flight fast path.

Scenarios with maps, effects, obstacles, gates or timeout fall back to the
single-environment implementation; no official parity is implied.
"""
import numpy as np
from scipy.spatial.transform import Rotation

from foundation.runtime import RmuaPhysics


class BatchRmuaPhysics:
    def __init__(self, environments, *, backend='auto'):
        self.envs = tuple(environments)
        if not self.envs or any(not isinstance(env, RmuaPhysics) for env in self.envs):
            raise ValueError('expected at least one RmuaPhysics environment')
        if len({id(env) for env in self.envs}) != len(self.envs):
            raise ValueError('environments must be distinct instances')
        # Counters are mutable. Sharing one silently corrupts per-env state.
        counters = [e.sim.out_time_counter for e in self.envs if e.sim.out_time_counter is not None]
        if len({id(x) for x in counters}) != len(counters):
            raise ValueError('out-time counters must not be shared')
        if backend not in ('auto', 'reference'):
            raise ValueError('backend must be auto or reference')
        self.backend = backend
        self.last_backend = None

    def _mask(self, active):
        if active is None:
            return np.ones(len(self.envs), dtype=bool)
        mask = np.asarray(active)
        if mask.shape != (len(self.envs),) or mask.dtype.kind != 'b':
            raise ValueError('active mask must be a boolean per environment')
        return mask

    def reset(self, initial_states=None, *, active=None):
        mask = self._mask(active)
        states = [None] * len(self.envs) if initial_states is None else list(initial_states)
        if len(states) != len(self.envs):
            raise ValueError('expected one initial state per environment')
        return [env.reset(state) if enabled else env._observation()
                for env, state, enabled in zip(self.envs, states, mask)]

    def batch_step(self, pwm, dt, *, active=None):
        commands = np.asarray(pwm, dtype=float)
        if commands.shape != (len(self.envs), 4):
            raise ValueError('expected one four-value PWM action per environment')
        mask = self._mask(active)
        # Preflight every active action before advancing any environment.
        prepared = []
        for env, command, enabled in zip(self.envs, commands, mask):
            if enabled:
                prepared.append(env._prepare_step(command, dt))
        selected = [env for env, enabled in zip(self.envs, mask) if enabled]
        if self.backend == 'auto' and self._can_vectorize(selected, prepared):
            self._vectorized_free_flight(selected, prepared)
            self.last_backend = 'vectorized_free_flight'
            observations, events = [], []
            for env, enabled in zip(self.envs, mask):
                observations.append(env._observation())
                events.append({'contact': None, 'gate_crossing_candidate': [], 'ambiguous_gate_crossing': [], 'force_zone': None,
                               'force_zone_elapsed_s': None, 'force_zone_timeout_candidate': False,
                               'timeout': False, 'terminal': False, 'physics_status': 'uncalibrated_candidate',
                               'out_of_bounds_candidate': False,
                               'map_status': 'unimplemented', 'map_containment_unknown': [], 'wind_status': 'unidentified',
                               'wind_field_status': 'disabled',
                               'force_degradation_status': 'unidentified', 'field_rotor_target_scale': env.sim.field_rotor_target_scale,
                               'completed_physics_steps': env.ticks} if enabled else None)
            return observations, events
        self.last_backend = 'reference'
        observations, events = [], []
        for env, command, enabled in zip(self.envs, commands, mask):
            if enabled:
                obs, event = env.step(command, dt)
            else:
                obs, event = env._observation(), None
            observations.append(obs)
            events.append(event)
        return observations, events

    @staticmethod
    def _can_vectorize(envs, prepared):
        if len(envs) < 2 or len({x.physics_dt for x in envs}) != 1 or len({x[1] for x in prepared}) != 1:
            return False
        for env in envs:
            sim = env.sim
            if (sim.terminal or sim.max_substep_s != env.physics_dt
                    or (sim.quad.time + env.physics_dt) - sim.quad.time <= 1e-12
                    or env.timeout_ticks is not None or env.flight_bounds is not None or sim.lines or sim.initial_vehicles or sim.candidate_map is not None
                    or sim.gate_volumes is not None or sim.aero_model is not None or sim.wind_field is not None or sim.force_replay is not None
                    or sim.recorded_effects is not None or sim.candidate_field is not None or sim.out_time_counter is not None):
                return False
        return True

    @staticmethod
    def _vectorized_free_flight(envs, prepared):
        quads = [e.sim.quad for e in envs]
        pwm = np.array([p[0] for p in prepared])
        steps = prepared[0][1]
        p = np.stack([q.position for q in quads]); v = np.stack([q.velocity for q in quads])
        w = np.stack([q.omega for q in quads]); signals = np.stack([q.signal for q in quads])
        rot = Rotation.from_quat(np.stack([q.rotation.as_quat() for q in quads]))
        t = np.array([q.time for q in quads]); inertia = np.stack([q.inertia for q in quads])
        arms = np.stack([q.arms for q in quads]); signs = np.stack([q.yaw_sign for q in quads])
        mass = np.array([q.p.mass for q in quads]); gravity = np.array([q.p.gravity for q in quads])
        lift = np.array([q.p.max_thrust for q in quads]); drag = np.array([q.p.linear_drag for q in quads])
        torque_scale = np.array([q.p.max_reaction_torque for q in quads])
        angular_drag = np.array([q.p.angular_drag for q in quads]); tau = np.array([q.p.motor_tau for q in quads])
        for _ in range(steps):
            # Match the single environment's absolute-time substep subtraction.
            delta = (t + envs[0].physics_dt) - t
            thrust = signals * lift[:, None]
            forces = np.zeros((len(envs), 4, 3)); forces[:, :, 2] = -thrust
            torque = np.cross(arms, forces).sum(axis=1)
            torque[:, 2] += np.sum(signals * signs, axis=1) * torque_scale
            force = rot.apply(forces.sum(axis=1))
            force[:, 2] += mass * gravity
            force -= drag[:, None] * v
            v += force / mass[:, None] * delta[:, None]
            p += v * delta[:, None]
            domega = (torque - np.cross(w, inertia * w) - angular_drag[:, None] * w) / inertia
            w += domega * delta[:, None]
            rot = rot * Rotation.from_rotvec(w * delta[:, None])
            signals += (pwm - signals) * (-np.expm1(-delta / tau))[:, None]
            t += delta
        for i, (env, q) in enumerate(zip(envs, quads)):
            q.position, q.velocity, q.omega, q.signal = p[i].copy(), v[i].copy(), w[i].copy(), signals[i].copy()
            q.rotation, q.time = rot[i], float(t[i])
            env.ticks += steps

    def truth_state(self, index):
        """Explicit research-only truth for one selected environment."""
        return self.envs[index].truth_state()
