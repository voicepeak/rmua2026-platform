"""Cross-check batch events against independent single runs in one synthetic scene."""
import numpy as np
import pytest

from foundation.aero import DragFaces
from foundation.batch import BatchRmuaPhysics
from foundation.effects import RecordedEffects
from foundation.gate_geometry import GateMeshVolumes
from foundation.map_scene import CandidateMap
from foundation.obstacles import Line, Vehicle
from foundation.out_time import OutTimeCounter
from foundation.quad import Params
from foundation.runtime import RmuaPhysics


def make_env(index):
    params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    vertices = np.array([[100., 200., -500.], [100., 600., -500.], [100., 400., 500.]])
    obj = {'actor': 'wall', 'mesh': 'wall', 'translation_ue_cm': [0, 0, 0],
           'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1]}
    scene = CandidateMap([obj], {'wall': (vertices, np.array([[0, 1, 2]]))})
    gate = {'actor': 'gate', 'mesh_package': '/Engine/EngineMeshes/Cube',
            'translation_ue_cm': [0, 300, 0], 'quaternion_xyzw': [0, 0, 0, 1],
            'scale': [100, 500, 500]}
    gates = GateMeshVolumes([gate], ([-1, -1, -1], [1, 1, 1]), crossing_axis=0)
    normals = np.vstack([np.eye(3), -np.eye(3)])
    aero = DragFaces(np.zeros((6, 3)), normals, [.08]*6, 1.225, 0.)
    effects = RecordedEffects([0, .05, .11, 1.], [[0, 1, 0]]*4, [1, 0, 1, 1])
    lines = [Line((.5, 0, -5), (.5, 0, 5), .02)] if index == 0 else []
    vehicles = [Vehicle((.65, 2, 0), (.12, .12, 1), (-1, 0, 0), 0)] if index == 1 else []
    return RmuaPhysics(params, physics_dt=.0025, timeout_s=.45 if index == 3 else None,
                       candidate_map=scene, map_origin_ue_cm=[0, 0, 0], gate_volumes=gates,
                       aero_model=aero, recorded_effects=effects, field_rotor_target_scale=.25, out_time_counter=OutTimeCounter(.04),
                       lines=lines, vehicles=vehicles, drone_radius_m=.08)


def test_batch_full_synthetic_scenario_matches_single_events_and_states():
    count = 4
    batch = BatchRmuaPhysics([make_env(i) for i in range(count)])
    singles = [make_env(i) for i in range(count)]
    initial = [{'position': [-1, 2*i, 0], 'velocity': [5, 0, 0], 'rotor_signal': [.18]*4}
               for i in range(count)]
    batch.reset(initial)
    for sim, state in zip(singles, initial):
        sim.reset(state)
    rng = np.random.default_rng(1109)
    terminal_reasons, saw_gate, saw_field = [None]*count, [False]*count, [False]*count
    for _ in range(60):
        active = np.array([not sim.terminal for sim in singles], dtype=bool)
        if not active.any():
            break
        pwm = rng.uniform(.18, .20, (count, 4))
        obs, events = batch.batch_step(pwm, .01, active=active)
        assert batch.last_backend == 'reference'
        for i in np.flatnonzero(active):
            expected_obs, expected = singles[i].step(pwm[i], .01)
            assert obs[i] == expected_obs and events[i] == expected
            saw_gate[i] |= bool(expected['gate_crossing_candidate'])
            saw_field[i] |= (expected['force_zone_elapsed_s'] or 0) > 0
            if expected['terminal']:
                terminal_reasons[i] = expected['contact']['kind'] if expected['contact'] else 'timeout'
            for key in ('position', 'velocity', 'quaternion_wxyz', 'omega', 'rotor_signal'):
                np.testing.assert_allclose(batch.truth_state(i)[key], singles[i].truth_state()[key], atol=1e-13)
    assert terminal_reasons == ['line', 'vehicle', 'static_candidate', 'timeout']
    assert all(saw_gate) and all(saw_field)
