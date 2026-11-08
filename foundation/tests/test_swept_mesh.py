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


def test_opt_in_closed_mesh_contains_initial_position_and_open_mesh_stays_unknown():
    import itertools
    from scipy.spatial import ConvexHull
    from scipy.spatial.transform import Rotation
    vertices = np.array(list(itertools.product((-10., 10.), repeat=3)))
    triangles = ConvexHull(vertices).simplices
    obj = {'actor': 'cube', 'mesh': 'cube', 'translation_ue_cm': [1000, 2000, 300],
           'quaternion_xyzw': Rotation.from_euler('z', .4).as_quat(), 'scale': [2, 3, 4]}
    default = CandidateMap([obj], {'cube': (vertices, triangles)})
    solid = CandidateMap([obj], {'cube': (vertices, triangles)}, assume_closed_meshes_solid=True)
    point = [1000, 2000, 300]
    assert not default.sweep_sphere_ue(point, point, 1)['hits']
    result = solid.sweep_sphere_ue(point, point, 1)
    assert result['hits'][0]['reason'] == 'candidate_closed_mesh_containment'
    assert not result['containment_unknown']
    assert not solid.sweep_sphere_ue([2000, 2000, 300], [2000, 2000, 300], 1)['hits']
    open_mesh = CandidateMap([obj], {'cube': (vertices, triangles[:-1])}, assume_closed_meshes_solid=True)
    result = open_mesh.sweep_sphere_ue(point, point, 1)
    assert not result['hits']
    assert result['containment_unknown'][0]['reason'] == 'mesh_not_topologically_closed'
    from foundation.runtime import RmuaPhysics
    params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    sim = RmuaPhysics(params, physics_dt=.0025, candidate_map=open_mesh,
                      map_origin_ue_cm=point, drone_radius_m=.01)
    sim.reset()
    _, events = sim.step([0]*4, .01)
    assert not events['terminal']
    assert events['map_containment_unknown'][0]['reason'] == 'mesh_not_topologically_closed'
