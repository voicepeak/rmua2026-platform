"""Approximate batched PWM training sandbox for RmuaPhysics.

This is an explicitly labelled *approximate* sandbox, not an official-equivalent
environment.  Actions are four PWM channels only; dynamics come from the
uncalibrated ``PhysicsSim`` candidate model.  Observations default to the
research truth interface because no IMU/GPS/EKF model exists yet; callers that
need a deployment-shaped observation must supply their own estimator and treat
``observable_state`` as a placeholder, never as truth.
"""
from __future__ import annotations

import math

import numpy as np

from foundation.runtime import RmuaPhysics


class RmuaPhysicsVecEnv:
    """Vectorised PWM-only 6DOF sandbox with explicit truth observations."""

    def __init__(self, params, n_envs, *, physics_dt=0.0025, command_dt=0.02,
                 seed=0, gates=None, gate_radius_m=2.0, initial_position_ned_m=(0, 0, -2),
                 initial_rotor_signal=0.18, collision_penalty=1.0, gate_bonus=1.0,
                 progress_scale=1.0, timeout_s=60.0, observation_mode='truth_research',
                 **scenario):
        if not isinstance(n_envs, int) or n_envs < 1:
            raise ValueError('n_envs must be a positive integer')
        if not math.isfinite(command_dt) or command_dt <= 0:
            raise ValueError('invalid command_dt')
        ratio = command_dt / physics_dt
        if not math.isclose(ratio, round(ratio), rel_tol=0, abs_tol=1e-8) or round(ratio) < 1:
            raise ValueError('command_dt must be a whole number of physics steps')
        if observation_mode not in ('truth_research', 'observable_placeholder'):
            raise ValueError('unknown observation_mode')
        if not math.isfinite(gate_radius_m) or gate_radius_m <= 0:
            raise ValueError('invalid gate_radius_m')
        self.params = params
        self.n_envs = n_envs
        self.physics_dt = float(physics_dt)
        self.command_dt = float(command_dt)
        self.steps_per_command = round(ratio)
        self.seed = int(seed)
        self.observation_mode = observation_mode
        self.gate_radius_m = float(gate_radius_m)
        self.collision_penalty = float(collision_penalty)
        self.gate_bonus = float(gate_bonus)
        self.progress_scale = float(progress_scale)
        self.initial_position = np.asarray(initial_position_ned_m, dtype=float)
        if self.initial_position.shape != (3,) or not np.isfinite(self.initial_position).all():
            raise ValueError('invalid initial position')
        self.initial_rotor_signal = float(initial_rotor_signal)
        self.gates = None if gates is None else np.asarray(gates, dtype=float)
        if self.gates is not None:
            if self.gates.ndim != 2 or self.gates.shape[1] != 3 or not np.isfinite(self.gates).all():
                raise ValueError('gates must be an (N, 3) finite array')
        self._rngs = [np.random.default_rng(self.seed + i) for i in range(n_envs)]
        self._envs = [RmuaPhysics(params, physics_dt=physics_dt, timeout_s=timeout_s, **scenario)
                      for _ in range(n_envs)]
        self._last_action = np.zeros((n_envs, 4), dtype=float)
        self._gate_index = np.zeros(n_envs, dtype=int)
        self._previous_distance = np.full(n_envs, np.nan)
        self._done = np.zeros(n_envs, dtype=bool)
        self._truncated = np.zeros(n_envs, dtype=bool)
        self._episode_steps = np.zeros(n_envs, dtype=int)

    # ------------------------------------------------------------------ reset
    def reset(self, initial_states=None, indices=None):
        """Reset selected environments; each env keeps an independent RNG stream."""
        targets = range(self.n_envs) if indices is None else list(indices)
        if initial_states is not None and len(initial_states) != len(targets):
            raise ValueError('initial_states must match the reset index count')
        for slot, index in enumerate(targets):
            state = None
            if initial_states is not None and initial_states[slot] is not None:
                state = dict(initial_states[slot])
            else:
                jitter = self._rngs[index].uniform(-0.02, 0.02, 3)
                state = {'position': (self.initial_position + jitter).tolist(),
                         'velocity': [0.0, 0.0, 0.0],
                         'rotor_signal': [self.initial_rotor_signal] * 4}
            self._envs[index].reset(state)
            self._last_action[index] = 0.0
            self._gate_index[index] = 0
            self._previous_distance[index] = np.nan
            self._done[index] = False
            self._truncated[index] = False
            self._episode_steps[index] = 0
        return self.observations()

    # -------------------------------------------------------------- observation
    def _truth_vector(self, index):
        state = self._envs[index].truth_state()
        return np.concatenate([
            np.asarray(state['position'], dtype=float),
            np.asarray(state['velocity'], dtype=float),
            np.asarray(state['quaternion_wxyz'], dtype=float),
            np.asarray(state['omega'], dtype=float),
            np.asarray(state['rotor_signal'], dtype=float),
            self._last_action[index],
            np.asarray([state['t']], dtype=float),
        ])

    def observations(self):
        if self.observation_mode == 'truth_research':
            return np.stack([self._truth_vector(i) for i in range(self.n_envs)])
        # Placeholder only: no estimator is implemented.  A fixed-width zero
        # vector keeps the interface shape stable without leaking truth.
        return np.zeros((self.n_envs, 22), dtype=float)

    def observable_state(self):
        """Explicit placeholder for a future sensor/EKF observation interface."""
        return None

    # ------------------------------------------------------------------- reward
    def _target_gate(self, index):
        if self.gates is None or len(self.gates) == 0:
            return None
        return self.gates[min(self._gate_index[index], len(self.gates) - 1)]

    def _reward(self, index, events):
        state = self._envs[index].truth_state()
        position = np.asarray(state['position'], dtype=float)
        reward = 0.0
        gate_reached = False
        target = self._target_gate(index)
        if target is not None:
            distance = float(np.linalg.norm(position - target))
            if np.isfinite(self._previous_distance[index]):
                reward += self.progress_scale * (self._previous_distance[index] - distance)
            self._previous_distance[index] = distance
            if distance <= self.gate_radius_m:
                reward += self.gate_bonus
                gate_reached = True
                self._gate_index[index] = min(self._gate_index[index] + 1, len(self.gates))
                self._previous_distance[index] = np.nan
        if events.get('contact') or events.get('map_status') == 'contact':
            reward -= self.collision_penalty
        if events.get('out_of_bounds_candidate'):
            reward -= self.collision_penalty
        return reward, gate_reached

    # --------------------------------------------------------------------- step
    def step(self, pwm):
        actions = np.asarray(pwm, dtype=float)
        if actions.shape != (self.n_envs, 4) or not np.isfinite(actions).all():
            raise ValueError('pwm must be a finite (n_envs, 4) array')
        if np.any((actions < 0) | (actions > 1)):
            raise ValueError('PWM must be in candidate range [0, 1]')
        rewards = np.zeros(self.n_envs, dtype=float)
        done = self._done.copy()
        truncated = self._truncated.copy()
        infos = []
        gate_reached = np.zeros(self.n_envs, dtype=bool)
        for index, env in enumerate(self._envs):
            info = {
                'gate_reached': False,
                'gate_index': int(self._gate_index[index]),
                'contact': False,
                'terminal': bool(done[index]),
                'truncated': bool(truncated[index]),
                'observable_state': None,
                'approximate_sandbox': True,
            }
            if done[index]:
                infos.append(info)
                continue
            observation, events = env.step(actions[index], self.command_dt)
            self._last_action[index] = actions[index].copy()
            self._episode_steps[index] += 1
            reward, reached = self._reward(index, events)
            rewards[index] = reward
            gate_reached[index] = reached
            done[index] = bool(events.get('terminal'))
            truncated[index] = bool(events.get('timeout')) and not done[index]
            info.update({
                'gate_reached': reached,
                'gate_index': int(self._gate_index[index]),
                'contact': bool(events.get('contact')),
                'terminal': bool(done[index]),
                'truncated': bool(truncated[index]),
                'events': events,
                'observable_state': None,
            })
            infos.append(info)
        self._done = done
        self._truncated = truncated
        observations = self.observations()
        return observations, rewards, done, truncated, gate_reached, infos

    # ------------------------------------------------------------------- state
    def truth_state(self, index):
        return self._envs[index].truth_state()

    @property
    def gates_total(self):
        return None if self.gates is None else len(self.gates)

    def close(self):
        self._envs = []


def make_rmua_physics_vec_env(params, n_envs=1, **kwargs):
    """Convenience constructor; the result is an approximate PWM sandbox."""
    return RmuaPhysicsVecEnv(params, n_envs, **kwargs)
