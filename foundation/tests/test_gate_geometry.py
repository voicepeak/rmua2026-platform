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
