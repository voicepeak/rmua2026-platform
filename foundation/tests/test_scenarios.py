import numpy as np
import pytest

from foundation.batch import BatchRmuaPhysics
from foundation.quad import Params
from foundation.scenarios import ObstacleProfile, ScenarioFactory


def factory():
    base = ObstacleProfile(2, 1, .03, 3., (1., .5, .5), (0., 2.), ((0., 0., 1.), (0., 1., 1.)),
                           vehicle_center_z_m=-1.)
    hard = ObstacleProfile(4, 2, .05, 4., (1., .5, .5), (1., 3.), ((0., 0., 1.),),
                           vehicle_route_legs=2, stop_range_s=(.2, .4), vehicle_center_z_m=-1.)
    return ScenarioFactory(bounds_min=[-20]*3, bounds_max=[20]*3, launch=[0, 0, 0],
                           exclusion_m=2, protected_paths=[[[-15, 0, 0], [15, 0, 0]]],
                           corridor_radius_m=1, horizon_s=2,
                           profiles={'fixed': base, 'random': base, 'difficult': hard}, fixed_seed=42)


def params():
    return Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)


def test_scenario_profiles_seed_and_metadata():
    f = factory()
    _, fixed_a = f.build(params(), 'fixed', 1)
    _, fixed_b = f.build(params(), 'fixed', 2)
    assert fixed_a['effective_seed'] == fixed_b['effective_seed'] == 42
    assert fixed_a['geometry_sha256'] == fixed_b['geometry_sha256']
    _, random_a = f.build(params(), 'random', 11)
    _, random_b = f.build(params(), 'random', 11)
    _, random_c = f.build(params(), 'random', 12)
    assert random_a['geometry_sha256'] == random_b['geometry_sha256']
    assert random_a['geometry_sha256'] != random_c['geometry_sha256']
    _, hard = f.build(params(), 'difficult', 11)
    assert len(hard['geometry']['lines']) == 4 and len(hard['geometry']['vehicles']) == 2
    assert hard['geometry']['vehicles'][0]['positions_m'][1] == hard['geometry']['vehicles'][0]['positions_m'][2]
    assert all(point[2] == -1 for vehicle in hard['geometry']['vehicles'] for point in vehicle['positions_m'])
    assert all(vehicle['center'][2] == -1 for vehicle in random_a['geometry']['vehicles'])
    _, hard_repeat = f.build(params(), 'difficult', 11)
    assert hard['geometry_sha256'] == hard_repeat['geometry_sha256']
    assert hard['episode_timeout_s'] == hard['vehicle_motion_horizon_s'] == 2
    assert hard['obstacle_separation_m'] == 0
    assert hard['vehicle_subdivisions'] == 2
    with pytest.raises(ValueError, match='vehicle_subdivisions'):
        ScenarioFactory(bounds_min=[-20]*3, bounds_max=[20]*3, launch=[0, 0, 0],
                        exclusion_m=2, protected_paths=[[[-15, 0, 0], [15, 0, 0]]],
                        corridor_radius_m=1, horizon_s=2, profiles=f.profiles,
                        fixed_seed=42, vehicle_subdivisions=0)
    with pytest.raises(ValueError, match='unknown scenario'):
        f.build(params(), 'other', 1)
    with pytest.raises(ValueError, match='owns lines'):
        f.build(params(), 'random', 1, lines=[])
    with pytest.raises(ValueError, match='within the vehicle motion horizon'):
        f.build(params(), 'random', 1, timeout_s=3)
    with pytest.raises(ValueError, match='collision radius'):
        f.build(params(), 'random', 1, drone_radius_m=1.1)
    with pytest.raises(ValueError, match='invalid obstacle profile'):
        ObstacleProfile(1, 1, .03, 3, (1, 1, 1), (2, 1), vehicle_center_z_m=0)
    with pytest.raises(ValueError, match='explicitly configured'):
        ObstacleProfile(1, 1, .03, 3, (1, 1, 1), (0, 1))
    short, _ = f.build(params(), 'random', 1, physics_dt=.01, timeout_s=.02)
    short.reset()
    _, event = short.step([.18]*4, .1)
    assert event['timeout'] and short.truth_state()['t'] == pytest.approx(.02)


@pytest.mark.parametrize('profile', ['random', 'difficult'])
def test_generated_obstacles_have_separated_full_path_envelopes(profile):
    import itertools
    from foundation.obstacles import Line, Vehicle, VehiclePath, segment_distance_squared

    sim, manifest = factory().build(params(), profile, 42)
    objects = [*sim.sim.lines, *sim.sim.initial_vehicles]
    horizon = manifest['vehicle_motion_horizon_s']

    def segments(obj):
        if isinstance(obj, Line):
            return [(np.asarray(obj.start), np.asarray(obj.end))], obj.radius_m
        if isinstance(obj, Vehicle):
            start = np.asarray(obj.center)
            return [(start, start + np.asarray(obj.velocity_m_s) * horizon)], np.linalg.norm(obj.half_extents_m)
        assert isinstance(obj, VehiclePath)
        return list(zip(obj.positions_m[:-1], obj.positions_m[1:])), np.linalg.norm(obj.half_extents_m)

    for a, b in itertools.combinations(objects, 2):
        sa, ra = segments(a); sb, rb = segments(b)
        for a0, a1 in sa:
            for b0, b1 in sb:
                assert segment_distance_squared(a0, a1, b0, b1) > (ra + rb)**2


@pytest.mark.parametrize('profile', ['random', 'difficult'])
def test_seeded_scenario_single_and_batch_agree(profile):
    f = factory()
    seeds = [101, 102]
    batch_pairs = [f.build(params(), profile, seed, physics_dt=.01) for seed in seeds]
    single_pairs = [f.build(params(), profile, seed, physics_dt=.01) for seed in seeds]
    assert [m['geometry_sha256'] for _, m in batch_pairs] == [m['geometry_sha256'] for _, m in single_pairs]
    batch = BatchRmuaPhysics([e for e, _ in batch_pairs])
    batch.reset()
    for env, _ in single_pairs:
        env.reset()
    for _ in range(4):
        obs, events = batch.batch_step([[.18]*4]*2, .02)
        assert batch.last_backend == 'reference'
        for i, (env, _) in enumerate(single_pairs):
            obs_single, event_single = env.step([.18]*4, .02)
            assert obs[i] == obs_single and events[i] == event_single
            np.testing.assert_allclose(batch.truth_state(i)['position'], env.truth_state()['position'])


def test_separately_seeded_wind_and_obstacles_replay_without_shared_state():
    import math
    from foundation.aero import DragFaces
    from foundation.wind_field import CandidateWindField
    from foundation.wind_generator import WindGenerator
    from vendor.rmua_re.wind import WindParameters, generate_keyframes

    constants = {'hold_min_s': 3., 'rand_denominator': 16777216., 'base_hold_s': 5.,
                 'transition_s': 2., 'speed_cap_delta': 1., 'speed_spread': 1.,
                 'speed_bias': -.5, 'angle_span': 360., 'zero': 0., 'radians_per_degree': math.pi/180}

    def construct(wind_seed):
        frames = generate_keyframes(WindParameters(30., 50., 4., 1.), seed=wind_seed, constants=constants)
        wind = CandidateWindField(WindGenerator(frames, constants), source_to_ned=np.eye(3),
                                  bounds_min_ned_m=[-20]*3, bounds_max_ned_m=[20]*3)
        normals = np.vstack([np.eye(3), -np.eye(3)])
        aero = DragFaces(np.zeros((6, 3)), normals, [.08]*6, 1.225, 0.)
        return factory().build(params(), 'random', 99, physics_dt=.01, timeout_s=.1,
                               wind_field=wind, aero_model=aero)

    a, ma = construct(123)
    b, mb = construct(123)
    c, mc = construct(456)
    assert ma['geometry_sha256'] == mb['geometry_sha256'] == mc['geometry_sha256']
    a.reset(); b.reset(); c.reset()
    for _ in range(4):
        for sim in (a, b, c):
            sim.step([.18]*4, .02)
    for key in ('position', 'velocity', 'quaternion_wxyz', 'omega', 'rotor_signal'):
        np.testing.assert_array_equal(a.truth_state()[key], b.truth_state()[key])
    assert not np.allclose(a.truth_state()['velocity'], c.truth_state()['velocity'], atol=1e-6)


def test_vehicle_ground_clearance_uses_subbox_spheres():
    import numpy as np
    from foundation.map_scene import CandidateMap
    from foundation.obstacles import CandidateMapObstacleFilter, Vehicle, VehiclePath

    vertices = np.array([[-1000, -1000, 0], [1000, -1000, 0],
                         [1000, 1000, 0], [-1000, 1000, 0]], float)
    obj = {'actor': 'ground', 'component': 'plane', 'mesh': 'ground',
           'translation_ue_cm': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1],
           'scale': [1, 1, 1]}
    scene = CandidateMap([obj], {'ground': (vertices, np.array([[0, 1, 2], [0, 2, 3]]))},
                         assume_closed_meshes_solid=True)
    vehicle = Vehicle((0, 0, -.3), (1., .5, .2), (0, 0, 0), 0)
    coarse = CandidateMapObstacleFilter(scene, origin_ue_cm=[0, 0, 0], horizon_s=1,
                                        vehicle_subdivisions=1)
    fine = CandidateMapObstacleFilter(scene, origin_ue_cm=[0, 0, 0], horizon_s=1,
                                      vehicle_subdivisions=10)
    assert not coarse(vehicle) and fine(vehicle)
    path = VehiclePath((0., 1.), ((0, 0, -.3), (0, 0, -.3)), (1., .5, .2), (0.,))
    assert not coarse(path) and fine(path)
    with pytest.raises(ValueError, match='vehicle_subdivisions'):
        CandidateMapObstacleFilter(scene, origin_ue_cm=[0, 0, 0], horizon_s=1,
                                   vehicle_subdivisions=0)


def test_candidate_map_filter_checks_full_obstacle_paths_and_unknown_containment():
    import itertools
    from scipy.spatial import ConvexHull
    from foundation.map_scene import CandidateMap
    from foundation.obstacles import CandidateMapObstacleFilter, Line, Vehicle, VehiclePath

    vertices = np.array(list(itertools.product((-100., 100.), repeat=3)))
    triangles = ConvexHull(vertices).simplices
    obj = {'actor': 'cube', 'mesh': 'cube', 'translation_ue_cm': [0, 0, 0],
           'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1]}
    default = CandidateMap([obj], {'cube': (vertices, triangles)})
    with pytest.raises(ValueError, match='solid assumption'):
        CandidateMapObstacleFilter(default, origin_ue_cm=[0, 0, 0], horizon_s=2)
    solid = CandidateMap([obj], {'cube': (vertices, triangles)}, assume_closed_meshes_solid=True)
    check = CandidateMapObstacleFilter(solid, origin_ue_cm=[0, 0, 0], horizon_s=2, clearance_m=.1)
    assert not check(Line((-2, 0, 0), (2, 0, 0), .01))
    assert not check(Line((0, 0, 0), (.1, 0, 0), .01))
    assert check(Line((3, 3, 0), (4, 3, 0), .01))
    assert not check(Vehicle((-3, 0, 0), (.2, .2, .2), (3, 0, 0), 0))
    path = VehiclePath((0., 1., 2.), ((-3, 2, 0), (-3, 0, 0), (3, 0, 0)), (.2, .2, .2), (0., 0.))
    assert not check(path)
    open_mesh = CandidateMap([obj], {'cube': (vertices, triangles[:-1])}, assume_closed_meshes_solid=True)
    unknown = CandidateMapObstacleFilter(open_mesh, origin_ue_cm=[0, 0, 0], horizon_s=2)
    assert not unknown(Line((0, 0, 0), (.1, 0, 0), .01))


def test_scenario_factory_optional_map_filter_uses_runtime_map():
    import itertools
    from scipy.spatial import ConvexHull
    from foundation.map_scene import CandidateMap
    from foundation.obstacles import CandidateMapObstacleFilter

    vertices = np.array(list(itertools.product((-100., 100.), repeat=3)))
    obj = {'actor': 'cube', 'mesh': 'cube', 'translation_ue_cm': [500, 500, 0],
           'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1]}
    scene = CandidateMap([obj], {'cube': (vertices, ConvexHull(vertices).simplices)},
                         assume_closed_meshes_solid=True)
    kwargs = dict(candidate_map=scene, map_origin_ue_cm=[0, 0, 0], map_clearance_m=.2)
    sim, manifest = factory().build(params(), 'difficult', 42, **kwargs)
    check = CandidateMapObstacleFilter(scene, origin_ue_cm=[0, 0, 0], horizon_s=2, clearance_m=.2)
    assert manifest['candidate_map_clearance_m'] == .2
    assert manifest['vehicle_subdivisions'] == 2
    assert all(check(obj) for obj in (*sim.sim.lines, *sim.sim.initial_vehicles))
    _, repeat = factory().build(params(), 'difficult', 42, **kwargs)
    assert manifest['geometry_sha256'] == repeat['geometry_sha256']


def test_candidate_map_filter_rejects_ol1_surface_crossing():
    from foundation.benchmark_batch_map_contact import initial_state
    from foundation.map_scene import CandidateMap
    from foundation.obstacles import CandidateMapObstacleFilter, Line

    state, _ = initial_state(CandidateMap.load('foundation/asset_analysis'))
    scene = CandidateMap.load('foundation/asset_analysis', assume_closed_meshes_solid=True)
    start = np.asarray(state['position'])
    end = start + np.asarray(state['velocity']) * .01
    check = CandidateMapObstacleFilter(scene, origin_ue_cm=[0, 0, 0], horizon_s=2)
    assert not check(Line(tuple(start), tuple(end), .1))
    assert check(Line((10000, 10000, -10000), (10001, 10000, -10000), .1))
