import numpy as np
import pytest
from foundation.swept_mesh import swept_sphere_triangle
from foundation.map_scene import CandidateMap
from foundation.sim import PhysicsSim
from foundation.quad import Params


TRI = np.array([[0., 0., 0.], [10., 0., 0.], [0., 10., 0.]])


def test_swept_sphere_face_edge_and_stationary():
    assert swept_sphere_triangle([2, 2, 5], [2, 2, -5], 0, TRI)
    assert swept_sphere_triangle([5, -1, 5], [5, -1, -5], 1, TRI)
    assert not swept_sphere_triangle([5, -1.1, 5], [5, -1.1, -5], 1, TRI)
    assert swept_sphere_triangle([2, 2, .5], [2, 2, .5], .5, TRI)
    assert not swept_sphere_triangle([2, 2, .6], [2, 2, .6], .5, TRI)
    assert swept_sphere_triangle([0, 0, 0], [0, 0, 0], 0, np.zeros((3, 3)))


def test_candidate_map_sweep_and_simulation_are_explicit():
    vertices = np.array([[0, -100, -100], [0, 100, -100], [0, 0, 100.]])
    obj = {'actor': 'wall', 'mesh': 'wall', 'translation_ue_cm': [0, 0, 0],
           'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1]}
    scene = CandidateMap([obj], {'wall': (vertices, np.array([[0, 1, 2]]))})
    p = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    with pytest.raises(ValueError, match='supplied together'):
        PhysicsSim(p, candidate_map=scene)
    sim = PhysicsSim(p, candidate_map=scene, map_origin_ue_cm=[0, 0, 0],
                     drone_radius_m=.1, max_substep_s=.05)
    sim.reset({'position': [-1, 0, 0], 'velocity': [50, 0, 0]})
    _, info = sim.step([0]*4, .05)
    assert info['collision']['kind'] == 'static_candidate'
    assert info['collision']['actor'] == 'wall'
    assert info['static_map_collision'] == 'unverified_candidate_surfaces'
