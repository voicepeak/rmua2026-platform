from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from foundation.gate_geometry import GateMeshVolumes
from foundation.quad import Params
from foundation.sim import PhysicsSim


def synthetic(rotation=None):
    obj = {'actor': 'gate', 'mesh_package': '/Engine/EngineMeshes/Cube',
           'translation_ue_cm': [100, 200, 300], 'scale': [2, 3, 4],
           'quaternion_xyzw': (rotation or Rotation.identity()).as_quat().tolist()}
    return GateMeshVolumes([obj], ([-1, -1, -1], [1, 1, 1]), crossing_axis=0)


def test_crossing_rotation_direction_aperture_and_boundary():
    scene = synthetic(Rotation.from_euler('z', 90, degrees=True))
    hit = scene.crossings_ue([100, 190, 300], [100, 210, 300])
    assert len(hit) == 1 and hit[0]['fraction'] == pytest.approx(.5)
    assert hit[0]['direction'] == 1
    assert scene.crossings_ue([100, 210, 300], [100, 190, 300])[0]['direction'] == -1
    assert not scene.crossings_ue([110, 190, 300], [110, 210, 300])
    assert len(scene.crossings_ue([100, 190, 300], [100, 200, 300])) == 1
    assert not scene.crossings_ue([100, 200, 300], [100, 210, 300])


def test_official_gate_candidates_and_sim_event():
    assets = Path(__file__).resolve().parents[1] / 'asset_analysis'
    gates = GateMeshVolumes.load(assets, crossing_axis=1)
    assert len(gates.gates) == 12
    assert np.allclose(gates.high - gates.low, [256, 256, 256], atol=.01)
    obj = {'actor': 'gate', 'mesh_package': '/Engine/EngineMeshes/Cube',
           'translation_ue_cm': [0, 0, 0], 'scale': [100, 100, 100],
           'quaternion_xyzw': [0, 0, 0, 1]}
    gate = GateMeshVolumes([obj], ([-1, -1, -1], [1, 1, 1]), crossing_axis=0)
    p = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    sim = PhysicsSim(p, gate_volumes=gate, map_origin_ue_cm=[0, 0, 0])
    sim.reset({'position': [-.1, 0, 0], 'velocity': [10, 0, 0]})
    _, info = sim.step([0]*4, .03)
    assert len(info['gate_events']) == 1
    assert info['gate_events'][0]['gate'] == 'gate'
    assert info['gate_event_status'] == 'candidate_geometry_unverified'


@pytest.mark.parametrize('line_x', [-.5, .5])
def test_collision_and_gate_in_same_substep_are_not_counted_as_crossing(line_x):
    from foundation.obstacles import Line
    obj = {'actor': 'gate', 'mesh_package': '/Engine/EngineMeshes/Cube',
           'translation_ue_cm': [0, 0, 0], 'scale': [100, 100, 100],
           'quaternion_xyzw': [0, 0, 0, 1]}
    gate = GateMeshVolumes([obj], ([-1, -1, -1], [1, 1, 1]), crossing_axis=0)
    p = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    line = Line((line_x, 0, -2), (line_x, 0, 2), .02)
    sim = PhysicsSim(p, gate_volumes=gate, map_origin_ue_cm=[0, 0, 0],
                     lines=[line], drone_radius_m=.01, max_substep_s=.1)
    sim.reset({'position': [-1, 0, 0], 'velocity': [20, 0, 0]})
    _, info = sim.step([0]*4, .1)
    assert info['collision']['kind'] == 'line'
    assert info['gate_events'] == []
    assert info['ambiguous_gate_events'][0]['collision_order'] == 'unknown'


def test_gate_in_prior_substep_remains_recorded_before_later_collision():
    from foundation.obstacles import Line
    obj = {'actor': 'gate', 'mesh_package': '/Engine/EngineMeshes/Cube',
           'translation_ue_cm': [0, 0, 0], 'scale': [100, 100, 100],
           'quaternion_xyzw': [0, 0, 0, 1]}
    gate = GateMeshVolumes([obj], ([-1, -1, -1], [1, 1, 1]), crossing_axis=0)
    p = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    sim = PhysicsSim(p, gate_volumes=gate, map_origin_ue_cm=[0, 0, 0],
                     lines=[Line((.6, 0, -2), (.6, 0, 2), .01)],
                     drone_radius_m=.01, max_substep_s=.025)
    sim.reset({'position': [-1, 0, 0], 'velocity': [20, 0, 0]})
    _, info = sim.step([0]*4, .1)
    assert info['collision']['kind'] == 'line'
    assert len(info['gate_events']) == 1
    assert not info['ambiguous_gate_events']


def test_runtime_exposes_ambiguous_gate_event_separately():
    from foundation.obstacles import Line
    from foundation.runtime import RmuaPhysics
    obj = {'actor': 'gate', 'mesh_package': '/Engine/EngineMeshes/Cube',
           'translation_ue_cm': [0, 0, 0], 'scale': [100, 100, 100],
           'quaternion_xyzw': [0, 0, 0, 1]}
    gate = GateMeshVolumes([obj], ([-1, -1, -1], [1, 1, 1]), crossing_axis=0)
    p = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    sim = RmuaPhysics(p, physics_dt=.01, gate_volumes=gate, map_origin_ue_cm=[0, 0, 0],
                      lines=[Line((-.05, 0, -2), (-.05, 0, 2), .005)], drone_radius_m=.005)
    sim.reset({'position': [-.1, 0, 0], 'velocity': [20, 0, 0]})
    _, events = sim.step([0]*4, .01)
    assert events['contact']['kind'] == 'line'
    assert events['gate_crossing_candidate'] == []
    assert events['ambiguous_gate_crossing'][0]['gate'] == 'gate'


def test_runtime_gate_crossing_during_boundary_exit_is_ambiguous():
    from foundation.runtime import RmuaPhysics
    obj = {'actor': 'gate', 'mesh_package': '/Engine/EngineMeshes/Cube',
           'translation_ue_cm': [0, 0, 0], 'scale': [100, 100, 100],
           'quaternion_xyzw': [0, 0, 0, 1]}
    gate = GateMeshVolumes([obj], ([-1, -1, -1], [1, 1, 1]), crossing_axis=0)
    p = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    sim = RmuaPhysics(p, physics_dt=.01, gate_volumes=gate, map_origin_ue_cm=[0, 0, 0],
                      flight_bounds_ned_m=[[-1, -1, -1], [.05, 1, 1]])
    sim.reset({'position': [-.1, 0, 0], 'velocity': [20, 0, 0]})
    _, events = sim.step([0]*4, .01)
    assert events['out_of_bounds_candidate'] and not events['gate_crossing_candidate']
    assert events['ambiguous_gate_crossing'][0]['boundary_order'] == 'unknown'
