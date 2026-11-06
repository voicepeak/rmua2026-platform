"""Node 1: vectorized state-only sim-fast environment.

One env == one official segment. The env exposes the official rule state
machine (30 s start, gates in order, segment timer, force field, wind, factory
prerequisite, collision) with a configurable plant:

    dynamics="kinematic": body velocity command -> integrated pose (fast, stable)
    dynamics="quad":      4 PWM -> 6-DOF rigid body + velocity controller

No rendering, no ROS, no per-step Python loop over environments.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..core.constants import DEFAULT_CONTROL_HZ, DEFAULT_PHYSICS_HZ, QuadParams
from ..core.control import VelocityController
from ..core import quat as quat_utils
from ..core.frames import wrap_pi, yaw_deg_to_forward
from ..rules.model import Rules, SegmentSpec, load_rules
from ..scene.forcefield import ForceField
from ..scene.map_data import MapData, load_map_data
from ..scene.obstacles import LineObstacles, VehicleObstacles, generate_lines, generate_vehicles
from ..scene.track import SegmentTrack, build_track
from ..scene.wind import WindField

OBS_NAMES = (
    "pos_x",
    "pos_y",
    "pos_z",
    "vel_x",
    "vel_y",
    "vel_z",
    "quat_w",
    "quat_x",
    "quat_y",
    "quat_z",
    "omega_x",
    "omega_y",
    "omega_z",
    "next_gate_x_body",
    "next_gate_y_body",
    "next_gate_z_body",
    "gate_normal_x_body",
    "gate_normal_y_body",
    "gate_normal_z_body",
    "path_ahead_x_body",
    "path_ahead_y_body",
    "path_ahead_z_body",
    "progress",
    "time_left",
    "seg_1",
    "seg_2",
    "seg_3",
    "seg_4",
    "seg_5",
    "seg_6",
    "gates_passed",
    "force_field",
    "start_phase",
)
OBS_DIM = len(OBS_NAMES)

PHASE_WAIT_START = 0
PHASE_FLYING = 1
PHASE_DONE = 2
PHASE_FAILED = 3


@dataclass
class SimFastConfig:
    map_name: str = "RMUA2024OL1"
    segment_id: int = 1
    num_envs: int = 64
    seed: int = 1
    control_hz: float = DEFAULT_CONTROL_HZ
    physics_hz: float = DEFAULT_PHYSICS_HZ
    action_mode: str = "vel"  # vel | pwm
    action_frame: str = "body"  # body | world (kinematic only)
    dynamics: str = "kinematic"  # kinematic | quad
    max_speed_mps: float = 25.0
    max_accel_mps2: float = 8.0
    max_steps: int | None = None
    corridor_half_width_m: float = 5.0
    corridor_height_m: float = 5.0
    start_timeout_s: float = 30.0
    force_field_limit_s: float = 2.0
    force_field_thrust_scale: float = 0.35
    line_max: int = 8
    vehicle_max: int = 3
    auto_factory_inspect: bool = True
    factory_radius_m: float = 15.0
    reward_progress: float = 1.0
    reward_gate: float = 10.0
    reward_complete: float = 50.0
    reward_time: float = 0.2
    reward_force: float = 0.5
    penalty_collision: float = 20.0
    penalty_timeout: float = 20.0
    penalty_force_fail: float = 20.0
    penalty_wrong_report: float = 50.0
    penalty_missed_gate: float = 20.0
    noisy_obs: bool = False
    gps_noise_m: float = 0.1
    obs_noise_m: float = 0.02


class SimFastVecEnv:
    def __init__(self, config: SimFastConfig | None = None, rules: Rules | None = None):
        self.config = config or SimFastConfig()
        self.rules = rules or load_rules()
        if not 1 <= self.config.segment_id <= 6:
            raise ValueError("segment_id must be 1..6")
        self.spec: SegmentSpec = self.rules.segment_by_id[self.config.segment_id]
        self.map_data: MapData = load_map_data(self.config.map_name)
        self.track: SegmentTrack = build_track(self.map_data, self.spec)
        self.track.corridor_half_width_m = self.config.corridor_half_width_m
        self.track.corridor_height_m = self.config.corridor_height_m
        self.n = int(self.config.num_envs)
        if self.n <= 0:
            raise ValueError("num_envs must be positive")
        self.dt = 1.0 / float(self.config.control_hz)
        self.substeps = max(1, int(round(self.config.physics_hz / self.config.control_hz)))
        self.quad = None
        self.controller = None
        if self.config.dynamics == "quad":
            self.quad = _QuadAdapter(self.n)
            self.controller = VelocityController(QuadParams())
        elif self.config.dynamics != "kinematic":
            raise ValueError("dynamics must be 'kinematic' or 'quad'")
        if self.config.action_mode not in ("vel", "pwm"):
            raise ValueError("action_mode must be 'vel' or 'pwm'")
        if self.config.action_frame not in ("body", "world"):
            raise ValueError("action_frame must be 'body' or 'world'")
        if self.config.action_frame == "world" and self.config.dynamics != "kinematic":
            raise ValueError("world-frame velocity action requires kinematic dynamics")
        if self.config.action_mode == "pwm" and self.config.dynamics != "quad":
            raise ValueError("pwm action requires dynamics='quad'")

        self.max_steps = int(
            self.config.max_steps
            if self.config.max_steps is not None
            else round(self.spec.time_limit_s * self.config.control_hz) + 1
        )
        self.pos = np.zeros((self.n, 3), dtype=np.float64)
        self.vel = np.zeros((self.n, 3), dtype=np.float64)
        self.quat = quat_utils.identity((self.n,))
        self.omega = np.zeros((self.n, 3), dtype=np.float64)
        self.yaw = np.zeros(self.n, dtype=np.float64)
        self.prev_pos = self.pos.copy()
        self.time = np.zeros(self.n, dtype=np.float64)
        self.segment_time = np.zeros(self.n, dtype=np.float64)
        self.start_time = np.zeros(self.n, dtype=np.float64)
        self.phase = np.zeros(self.n, dtype=np.int32)
        self.gate_index = np.zeros(self.n, dtype=np.int32)
        self.start_inside = np.zeros(self.n, dtype=bool)
        self.end_inside = np.zeros(self.n, dtype=bool)
        self.gates_passed = np.zeros(self.n, dtype=np.int32)
        self.total_gates = np.zeros(self.n, dtype=np.int32)
        self.factory_reports = np.zeros(self.n, dtype=np.int32)
        self.factory_hold = np.zeros(self.n, dtype=np.float64)
        self.force_time = np.zeros(self.n, dtype=np.float64)
        self.steps = np.zeros(self.n, dtype=np.int64)
        self.episode_return = np.zeros(self.n, dtype=np.float64)
        self.failure_reason = np.full(self.n, "", dtype=object)
        self.lines: LineObstacles | None = None
        self.vehicles: VehicleObstacles | None = None
        self.wind: WindField | None = None
        self.rngs: list[np.random.Generator] = []
        self.base_seed = int(self.config.seed)
        self.episode_count = np.zeros(self.n, dtype=np.int64)
        self.reset()

    # ------------------------------------------------------------------
    @property
    def observation_dim(self) -> int:
        return OBS_DIM

    @property
    def action_dim(self) -> int:
        return 4

    def _seed_rngs(self, seed: int | None = None):
        base = self.base_seed if seed is None else int(seed)
        self.base_seed = base
        self.rngs = [
            np.random.default_rng(np.random.SeedSequence([base, env_index]))
            for env_index in range(self.n)
        ]

    def reset(self, seed: int | None = None):
        self._seed_rngs(seed)
        self.episode_count.fill(0)
        start = self.track.start_position
        normal = self.track.start_gate.normal
        yaw = float(np.arctan2(normal[1], normal[0]))
        for env_index, rng in enumerate(self.rngs):
            jitter = rng.normal(0.0, 0.16, size=3)
            jitter[2] *= 0.25
            self.pos[env_index] = start + jitter
            self.vel[env_index] = 0.0
            self.yaw[env_index] = yaw + float(rng.normal(0.0, 0.01))
            self.omega[env_index] = 0.0
        self.quat = quat_utils.from_euler(self.yaw)
        if self.quad is not None:
            rpm = np.full((self.n, 4), self.quad.params.max_rpm * self.quad.params.hover_pwm)
            self.quad.reset(self.pos, self.vel, self.quat, self.omega, rpm)
        self.prev_pos = self.pos.copy()
        self.gate_prev_pos = self.pos.copy()
        self.time.fill(0.0)
        self.segment_time.fill(0.0)
        self.start_time.fill(0.0)
        self.phase.fill(PHASE_WAIT_START)
        self.gate_index.fill(0)
        self.start_inside.fill(False)
        self.end_inside.fill(False)
        self.gates_passed.fill(0)
        self.total_gates.fill(0)
        self.factory_reports.fill(0)
        self.factory_hold.fill(0.0)
        self.force_time.fill(0.0)
        self.steps.fill(0)
        self.episode_return.fill(0.0)
        self.failure_reason.fill("")
        self.lines = generate_lines(self.track, self.rngs, max_count=self.config.line_max)
        self.vehicles = generate_vehicles(self.track, self.rngs, max_count=self.config.vehicle_max)
        self.wind = WindField.create(self.rngs, self.spec.id, self.spec.wind)
        return self._obs()

    def reset_indices(self, indices):
        """Reset only the selected environments after a terminal transition."""
        indices = np.asarray(indices, dtype=np.int64)
        if indices.size == 0:
            return self._obs()
        start = self.track.start_position
        normal = self.track.start_gate.normal
        yaw = float(np.arctan2(normal[1], normal[0]))
        rngs = []
        for env_index in indices:
            seed = int(self.base_seed) + int(env_index) * 1000003 + int(self.episode_count[env_index])
            self.episode_count[env_index] += 1
            rng = np.random.default_rng(np.random.SeedSequence([seed, int(env_index)]))
            rngs.append(rng)
            jitter = rng.normal(0.0, 0.16, size=3)
            jitter[2] *= 0.25
            self.pos[env_index] = start + jitter
            self.vel[env_index] = 0.0
            self.yaw[env_index] = yaw + float(rng.normal(0.0, 0.01))
            self.omega[env_index] = 0.0
            self.quat[env_index] = quat_utils.from_euler(np.array([self.yaw[env_index]]))[0]
            self.prev_pos[env_index] = self.pos[env_index]
            self.gate_prev_pos[env_index] = self.pos[env_index]
            self.time[env_index] = 0.0
            self.segment_time[env_index] = 0.0
            self.start_time[env_index] = 0.0
            self.phase[env_index] = PHASE_WAIT_START
            self.gate_index[env_index] = 0
            self.start_inside[env_index] = False
            self.end_inside[env_index] = False
            self.gates_passed[env_index] = 0
            self.total_gates[env_index] = 0
            self.factory_reports[env_index] = 0
            self.factory_hold[env_index] = 0.0
            self.force_time[env_index] = 0.0
            self.steps[env_index] = 0
            self.episode_return[env_index] = 0.0
            self.failure_reason[env_index] = ""
        new_lines = generate_lines(self.track, rngs, max_count=self.config.line_max)
        self.lines.start[indices] = new_lines.start
        self.lines.end[indices] = new_lines.end
        self.lines.radius[indices] = new_lines.radius
        self.lines.active[indices] = new_lines.active
        new_vehicles = generate_vehicles(self.track, rngs, max_count=self.config.vehicle_max)
        self.vehicles.center[indices] = new_vehicles.center
        self.vehicles.yaw[indices] = new_vehicles.yaw
        self.vehicles.velocity[indices] = new_vehicles.velocity
        self.vehicles.active[indices] = new_vehicles.active
        new_wind = WindField.create(rngs, self.spec.id, self.spec.wind)
        self.wind.speed[indices] = new_wind.speed
        self.wind.direction[indices] = new_wind.direction
        self.wind.amplitude[indices] = new_wind.amplitude
        self.wind.period[indices] = new_wind.period
        self.wind.phase[indices] = new_wind.phase
        if self.quad is not None:
            rpm = np.full((self.n, 4), self.quad.params.max_rpm * self.quad.params.hover_pwm)
            self.quad.reset(self.pos, self.vel, self.quat, self.omega, rpm)
        return self._obs()

    # ------------------------------------------------------------------
    def step(self, actions):
        actions = np.asarray(actions, dtype=np.float64)
        if actions.shape != (self.n, self.action_dim):
            raise ValueError(f"actions must be ({self.n}, {self.action_dim}), got {actions.shape}")
        if self.config.action_mode == "vel":
            actions = actions.copy()
            actions[:, :3] = np.clip(
                actions[:, :3], -self.config.max_speed_mps, self.config.max_speed_mps
            )
            actions[:, 3] = np.clip(actions[:, 3], -2.5, 2.5)

        # Wind and force field are evaluated at the current state.
        wind = self.wind.sample(self.time) if self.wind is not None else np.zeros((self.n, 3))
        force = ForceField.evaluate(
            self.track, self.pos, self.config.force_field_thrust_scale
        )
        prev_pos = self.pos.copy()
        prev_vel = self.vel.copy()

        if self.config.dynamics == "kinematic":
            if self.config.action_frame == "world":
                v_world_des = actions[:, :3].copy()
                speed = np.linalg.norm(v_world_des[:, :2], axis=1)
                target_yaw = np.where(
                    speed > 0.3, np.arctan2(v_world_des[:, 1], v_world_des[:, 0]), self.yaw
                )
                delta_yaw = wrap_pi(target_yaw - self.yaw)
                max_turn = 3.0 * self.dt
                self.yaw = wrap_pi(self.yaw + np.clip(delta_yaw, -max_turn, max_turn))
                self.omega[:, 2] = np.clip(delta_yaw / self.dt, -3.0, 3.0)
            else:
                v_world_des = self._body_to_world(actions[:, :3])
                self.yaw = wrap_pi(self.yaw + actions[:, 3] * self.dt)
                self.omega[:, 2] = actions[:, 3]
            alpha = 1.0 - np.exp(-self.dt / 0.12)
            desired_vel = self.vel + (v_world_des - self.vel) * alpha
            # Acceleration limit 8 m/s^2, applied as a velocity delta cap.
            delta_v = desired_vel - prev_vel
            max_delta = self.config.max_accel_mps2 * self.dt
            norm = np.linalg.norm(delta_v, axis=1, keepdims=True)
            scale = np.minimum(1.0, max_delta / np.maximum(norm, 1e-9))
            self.vel = prev_vel + delta_v * scale
            self.pos += self.vel * self.dt
            self.quat = quat_utils.from_euler(self.yaw)
        else:
            if self.config.action_mode == "vel":
                pwm = self.controller.pwm(self.quad, actions[:, :3], actions[:, 3], self.dt)
            else:
                pwm = actions
            pwm = pwm * force.factor[:, None]
            self.quad.step_substeps(pwm, wind=wind, dt=self.dt, substeps=self.substeps)
            self.pos = self.quad.pos.copy()
            self.vel = self.quad.vel.copy()
            self.quat = self.quad.quat.copy()
            self.omega = self.quad.omega.copy()
            self.yaw, _, _ = quat_utils.to_euler(self.quat)

        self.prev_pos = prev_pos
        self.gate_prev_pos = prev_pos
        self.time += self.dt
        self.steps += 1

        # Vehicles move after the plant step; collision uses the new positions.
        if self.vehicles is not None:
            self.vehicles.step(self.dt)

        _, _, _, lateral, vertical = self.track.project(self.pos)
        inside_corridor = force.inside
        self.force_time = np.where(inside_corridor, 0.0, self.force_time + self.dt)

        # 30 s start rule: leave the start trigger zone.
        start_distance = np.linalg.norm(self.pos - self.track.start_position, axis=1)
        just_started = (self.phase == PHASE_WAIT_START) & (
            start_distance > self.track.start_trigger_length_m
        )
        self.phase = np.where(just_started, PHASE_FLYING, self.phase)
        self.start_time = np.where(self.phase == PHASE_WAIT_START, self.start_time + self.dt, self.start_time)
        self.segment_time = np.where(self.phase == PHASE_FLYING, self.segment_time + self.dt, self.segment_time)

        # Gate trigger volumes (node-1 proxy for the undecoded official trigger).
        start_inside = self.track.start_gate.contains_trigger(self.pos)
        end_inside = self.track.end_gate.contains_trigger(self.pos)
        start_cross = start_inside & (~self.start_inside)
        end_cross = end_inside & (~self.end_inside)
        self.start_inside = start_inside
        self.end_inside = end_inside
        start_pass = (self.phase == PHASE_FLYING) & (self.gate_index == 0) & start_cross
        end_ready = (self.phase == PHASE_FLYING) & (self.gate_index == 1) & end_cross
        factory_required = self.spec.factory_reports
        end_pass = end_ready & (self.factory_reports >= factory_required)
        self.gate_index = np.where(start_pass, 1, self.gate_index)
        self.gate_index = np.where(end_pass, 2, self.gate_index)
        self.gates_passed += start_pass.astype(np.int32) + end_pass.astype(np.int32)
        self.total_gates += start_pass.astype(np.int32) + end_pass.astype(np.int32)
        end_trigger = (
            np.linalg.norm(self.pos - self.track.end_position, axis=1) < 5.0
        )
        segment_done = (
            (self.gate_index == 2)
            & end_trigger
            & (self.phase == PHASE_FLYING)
        )
        progress_now = self.track.project(self.pos)[0]
        start_gate_s = float(
            self.track.project(self.track.start_gate.center[None, :])[0][0]
        )
        end_gate_s = float(
            self.track.project(self.track.end_gate.center[None, :])[0][0]
        )
        missed_start = (
            (self.phase == PHASE_FLYING)
            & (self.gate_index == 0)
            & (progress_now > start_gate_s + 10.0)
            & (~start_pass)
        )
        missed_end = (
            (self.phase == PHASE_FLYING)
            & (self.gate_index == 1)
            & (progress_now > end_gate_s + 5.0)
            & (~end_pass)
        )
        missed_gate = missed_start | missed_end

        # Factory inspection placeholder: hold near a factory target for 1 s.
        # Perception/report correctness is a node-2 concern.
        if self.config.auto_factory_inspect and factory_required > 0:
            near_factory = self._near_factory_targets()
            slow = np.linalg.norm(self.vel, axis=1) < 2.0
            inspecting = near_factory & slow & (self.phase == PHASE_FLYING) & (
                self.factory_reports < factory_required
            )
            self.factory_hold = np.where(inspecting, self.factory_hold + self.dt, 0.0)
            completed = inspecting & (self.factory_hold >= 1.0)
            self.factory_reports += completed.astype(np.int32)

        # Collision and hard failures.
        collision = np.zeros(self.n, dtype=bool)
        if self.lines is not None:
            collision |= self.lines.collides(self.pos)
        if self.vehicles is not None:
            collision |= self.vehicles.collides(self.pos)
        # Hitting the ground or leaving the road envelope is a hard collision.
        collision |= vertical < -2.0
        collision |= np.abs(lateral) > self.config.corridor_half_width_m + 3.0

        timeout = np.zeros(self.n, dtype=bool)
        timeout |= (self.phase == PHASE_WAIT_START) & (
            self.start_time > self.config.start_timeout_s
        )
        timeout |= (self.phase == PHASE_FLYING) & (
            self.segment_time > self.spec.time_limit_s
        )
        force_fail = self.force_time > self.config.force_field_limit_s
        failed = collision | timeout | force_fail | missed_gate
        done = segment_done | failed

        # Reward. Before the start gate, reward approach to the gate; path
        # progress is unlocked only after the start gate so the policy cannot
        # farm progress by climbing parallel to the path.
        path_progress = np.clip(
            progress_now - self.track.project(prev_pos)[0],
            -10.0,
            10.0,
        )
        start_distance = np.linalg.norm(self.pos - self.track.start_gate.center, axis=1)
        start_distance_prev = np.linalg.norm(
            prev_pos - self.track.start_gate.center, axis=1
        )
        gate_approach = np.clip(start_distance_prev - start_distance, -10.0, 10.0)
        progress = np.where(self.gate_index >= 1, path_progress, gate_approach)
        reward = self.config.reward_progress * progress
        reward += self.config.reward_gate * (
            start_pass.astype(np.float64) + end_pass.astype(np.float64)
        )
        reward += self.config.reward_complete * segment_done.astype(np.float64)
        reward -= self.config.reward_time * self.dt
        reward -= self.config.reward_force * force.outside.astype(np.float64) * self.dt
        reward -= self.config.penalty_collision * collision.astype(np.float64)
        reward -= self.config.penalty_timeout * timeout.astype(np.float64)
        reward -= self.config.penalty_force_fail * force_fail.astype(np.float64)
        reward -= self.config.penalty_missed_gate * missed_gate.astype(np.float64)
        self.episode_return += reward

        # Terminal bookkeeping.
        newly_done = done & (self.phase != PHASE_DONE) & (self.phase != PHASE_FAILED)
        self.phase = np.where(segment_done, PHASE_DONE, self.phase)
        self.phase = np.where(failed, PHASE_FAILED, self.phase)
        for env_index in np.nonzero(newly_done)[0]:
            if segment_done[env_index]:
                self.failure_reason[env_index] = "success"
            elif collision[env_index]:
                self.failure_reason[env_index] = "collision"
            elif timeout[env_index]:
                self.failure_reason[env_index] = "timeout"
            elif force_fail[env_index]:
                self.failure_reason[env_index] = "force_field"
            elif missed_gate[env_index]:
                self.failure_reason[env_index] = "missed_gate"

        truncated = self.steps >= self.max_steps
        return self._obs(), reward.astype(np.float32), done, truncated, self._info()

    # ------------------------------------------------------------------
    def _body_to_world(self, vec_body):
        yaw = self.yaw
        c = np.cos(yaw)
        s = np.sin(yaw)
        x = vec_body[:, 0] * c - vec_body[:, 1] * s
        y = vec_body[:, 0] * s + vec_body[:, 1] * c
        return np.stack((x, y, vec_body[:, 2]), axis=1)

    def _near_factory_targets(self):
        if not self.map_data.factories:
            return np.zeros(self.n, dtype=bool)
        factories = np.stack([f.center for f in self.map_data.factories.values()], axis=0)
        # Use the closest factory as a placeholder target for each env.
        distances = np.linalg.norm(self.pos[:, None, :] - factories[None, :, :], axis=2)
        closest = np.min(distances, axis=1)
        return closest <= self.config.factory_radius_m

    def _next_gate(self):
        is_start = (self.gate_index == 0)[:, None]
        centers = np.where(
            is_start,
            self.track.start_gate.center[None, :],
            self.track.end_gate.center[None, :],
        )
        normals = np.where(
            is_start,
            self.track.start_gate.normal[None, :],
            self.track.end_gate.normal[None, :],
        )
        return centers, normals

    def _obs(self):
        progress = self.track.project(self.pos)[0]
        progress_norm = progress / max(self.track.total_length_m, 1e-9)
        centers, normals = self._next_gate()
        rel_world = centers - self.pos
        rel_body = quat_utils.rotate_inverse(self.quat, rel_world)
        normal_body = quat_utils.rotate_inverse(self.quat, normals)
        ahead = self.track.points_at_distances(
            np.minimum(progress + 10.0, self.track.total_length_m)
        )
        if not self.track.route_mode:
            ahead = np.where(
                (self.gate_index == 0)[:, None],
                self.track.start_gate.center[None, :],
                ahead,
            )
        ahead_body = quat_utils.rotate_inverse(self.quat, ahead - self.pos)
        seg_onehot = np.zeros((self.n, 6), dtype=np.float64)
        seg_onehot[np.arange(self.n), self.spec.id - 1] = 1.0
        force_flag = (~self.track.in_corridor(self.pos)).astype(np.float64)
        start_phase = (self.phase == PHASE_WAIT_START).astype(np.float64)
        obs = np.concatenate(
            (
                self.pos,
                self.vel,
                self.quat,
                self.omega,
                rel_body,
                normal_body,
                ahead_body,
                progress_norm[:, None],
                np.maximum(self.spec.time_limit_s - self.segment_time, 0.0)[:, None],
                seg_onehot,
                self.gates_passed[:, None].astype(np.float64),
                force_flag[:, None],
                start_phase[:, None],
            ),
            axis=1,
        )
        if self.config.noisy_obs:
            obs = obs + np.random.default_rng(self.base_seed + 7919).normal(
                0.0, self.config.obs_noise_m, size=obs.shape
            )
        return obs.astype(np.float32)

    def _info(self):
        return {
            "gates_passed": self.gates_passed.copy(),
            "total_gates": self.total_gates.copy(),
            "segment_time": self.segment_time.copy(),
            "phase": self.phase.copy(),
            "success": (self.phase == PHASE_DONE),
            "failure_reason": self.failure_reason.copy(),
            "factory_reports": self.factory_reports.copy(),
            "episode_return": self.episode_return.copy(),
        }


class _QuadAdapter:
    """Small wrapper so the env can swap between kinematic and 6-DOF plants."""

    def __init__(self, n: int):
        from ..core.quad import QuadBatch

        self._quad = QuadBatch(n)
        self.params = self._quad.params

    def reset(self, pos, vel, quat, omega, rpm):
        self._quad.reset(pos, vel, quat, omega, rpm)

    def step_substeps(self, pwm, wind, dt, substeps):
        self._quad.step_substeps(pwm, wind=wind, dt=dt, substeps=substeps)

    @property
    def pos(self):
        return self._quad.pos

    @property
    def n(self):
        return self._quad.n

    @property
    def vel(self):
        return self._quad.vel

    @property
    def quat(self):
        return self._quad.quat

    @property
    def omega(self):
        return self._quad.omega

    def body_velocity(self):
        return self._quad.body_velocity()

    def body_z(self):
        return self._quad.body_z()

    def hover_pwm(self):
        return self._quad.hover_pwm()
