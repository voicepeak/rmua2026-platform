import pytest
from foundation.obstacles import (Line, Vehicle, VehiclePath, generate, generate_vehicle_paths,
                                  segment_distance_squared)


def test_fast_line_crossing_and_near_miss():
    line = Line((0, 0, -3.5), (0, 0, 3.5), .026)
    assert line.hit_swept((-10, 0, 0), (10, 0, 0), .2)
    assert not line.hit_swept((-10, 1, 0), (10, 1, 0), .2)
    assert segment_distance_squared((0, 0, 0), (0, 0, 0), (1, 0, 0), (1, 0, 0)) == pytest.approx(1)


def test_relative_vehicle_motion_and_parallel_miss():
    vehicle = Vehicle((0, 0, 0), (1, 1, 1), (10, 0, 0), 0)
    assert vehicle.hit_swept((5, 0, 0), (5, 0, 0), 1, .1)
    assert not vehicle.hit_swept((5, 5, 0), (5, 5, 0), 1, .1)


def test_sweeps_agree_when_step_is_split():
    line = Line((0, 0, -2), (0, 0, 2), .026)
    whole = line.hit_swept((-20, .1, 0), (20, .1, 0), .2)
    split = (line.hit_swept((-20, .1, 0), (0, .1, 0), .2) or
             line.hit_swept((0, .1, 0), (20, .1, 0), .2))
    assert whole and split
    v1 = Vehicle((-10, 0, 0), (1, 1, 1), (20, 0, 0), 0)
    v2 = Vehicle((0, 0, 0), (1, 1, 1), (20, 0, 0), 0)
    assert v1.hit_swept((0, 0, 0), (0, 0, 0), 1, .2)
    assert (v1.hit_swept((0, 0, 0), (0, 0, 0), .5, .2) or
            v2.hit_swept((0, 0, 0), (0, 0, 0), .5, .2))


def test_reproducible_generation_and_launch_clearance():
    args = (42, [-30]*3, [30]*3, [0]*3, 5, 3, 2, .026, 7, [2, 1, 1], [1, 6])
    assert generate(*args) == generate(*args)
    lines, vehicles = generate(*args)
    assert all(not line.hit_swept([0]*3, [0]*3, 5) for line in lines)
    assert all(not v.hit_swept([0]*3, [0]*3, 0, 5) for v in vehicles)


def test_generation_enforces_full_bounds_and_custom_feasibility():
    import numpy as np
    lines, vehicles = generate(12, [-10]*3, [10]*3, [0]*3, 2, 5, 3, .2, 6,
                               [1, 2, .5], [0, 3],
                               line_directions=[(1, 0, 0), (0, 1, 1)],
                               feasible=lambda obj: (obj.start[2] if isinstance(obj, Line) else obj.center[2]) > 0)
    for line in lines:
        assert np.all(np.minimum(line.start, line.end)-line.radius_m >= -10)
        assert np.all(np.maximum(line.start, line.end)+line.radius_m <= 10)
        assert line.start[2] > 0
    for v in vehicles:
        c, s = abs(np.cos(v.yaw_rad)), abs(np.sin(v.yaw_rad))
        margin = [c*v.half_extents_m[0]+s*v.half_extents_m[1],
                  s*v.half_extents_m[0]+c*v.half_extents_m[1], v.half_extents_m[2]]
        assert np.all(np.abs(v.center)+margin <= 10)
        assert v.center[2] > 0
    with pytest.raises(ValueError, match='feasible line'):
        generate(12, [-1]*3, [1]*3, [0]*3, 0, 1, 0, .2, 6, [1]*3, [0, 1])


def test_piecewise_vehicle_turn_stop_and_split():
    path = VehiclePath((0, 1, 2, 3), ((-5, 0, 0), (0, 0, 0), (0, 0, 0), (0, 5, 0)),
                       (.5, .5, .5), (0, 0, 1.5707963267948966))
    # Collision occurs in the final segment after a full second of stopped time.
    assert path.hit_swept_interval((0, 4, 0), (0, 4, 0), 0, 3, .1)
    assert not path.hit_swept_interval((0, 4, 0), (0, 4, 0), 0, 2, .1)
    assert path.hit_swept_interval((0, 4, 0), (0, 4, 0), 2, 1, .1)
    # Vehicle remains at the final position after the path terminates.
    assert path.hit_swept_interval((0, 5, 0), (0, 5, 0), 4, 0, .1)
    with pytest.raises(ValueError, match='path'):
        VehiclePath((0, 1, 1), ((0, 0, 0),)*3, (1, 1, 1), (0, 0))


def test_seeded_vehicle_routes_stop_and_fail_closed():
    route = [[5, 0, 0], [5, 5, 0], [9, 5, 0]]
    args = (9, [route], [.5, .5, .5], [1, 2], [.2, .4], [-20]*3, [20]*3, [0]*3, 1)
    vehicles = generate_vehicle_paths(*args, feasible=lambda path: True)
    assert vehicles == generate_vehicle_paths(*args, feasible=lambda path: True)
    assert vehicles[0].positions_m[1] == vehicles[0].positions_m[2]
    assert vehicles[0].times_s[2] > vehicles[0].times_s[1]
    with pytest.raises(ValueError, match='rejected'):
        generate_vehicle_paths(*args, feasible=lambda path: False)
    with pytest.raises(ValueError, match='launch exclusion'):
        generate_vehicle_paths(9, [[[-5, 0, 0], [5, 0, 0]]], *args[2:], feasible=lambda path: True)


def test_extracted_asset_bounds_enclose_every_serialized_vertex():
    import json
    import numpy as np
    from pathlib import Path
    assets = json.loads((Path(__file__).resolve().parents[1] / 'asset_analysis' /
                         'obstacle_geometry_candidates.json').read_text())['assets']
    for name in ('thinline', 'thickline'):
        row = assets[name]
        spec = row['conservative_capsule']
        line = Line(tuple(np.asarray(spec['start_cm'])/100), tuple(np.asarray(spec['end_cm'])/100),
                    spec['radius_m'] + 1e-12)
        assert all(line.hit_swept(np.asarray(v)/100, np.asarray(v)/100, 0)
                   for v in row['transformed_vertices_cm'])
    row = assets['HoverTruck']
    vehicle = Vehicle(tuple(np.asarray(row['bounds_center_cm'])/100),
                      tuple(np.asarray(row['bounds_half_extents_m']) + 1e-12), (0, 0, 0), 0)
    assert all(vehicle.hit_swept(np.asarray(v)/100, np.asarray(v)/100, 0, 0)
               for v in row['transformed_vertices_cm'])


def test_asset_instances_apply_parent_transform_and_ned_origin():
    import numpy as np
    from pathlib import Path
    from scipy.spatial.transform import Rotation
    from foundation.frames import ue_cm_to_ned_m
    from foundation.obstacle_asset_instances import load, line_from_asset, vehicle_from_asset
    assets = load(Path(__file__).resolve().parents[1] / 'asset_analysis' / 'obstacle_geometry_candidates.json')
    translation, scale, origin = np.array([120, -80, 50]), np.array([2, 1, 1.5]), [10, 20, 30]
    rotation = Rotation.from_euler('z', .7)
    line = line_from_asset(assets, 'thinline', parent_translation_ue_cm=translation,
                           parent_rotation_xyzw=rotation.as_quat(), parent_scale=scale, origin_ue_cm=origin)
    for vertex in assets['thinline']['transformed_vertices_cm']:
        point = ue_cm_to_ned_m(rotation.apply(np.asarray(vertex)*scale)+translation, origin)
        assert line.hit_swept(point, point, 1e-12)
    vehicle = vehicle_from_asset(assets, parent_translation_ue_cm=translation, parent_yaw_rad=.7,
                                 parent_scale=scale, origin_ue_cm=origin)
    for vertex in assets['HoverTruck']['transformed_vertices_cm']:
        point = ue_cm_to_ned_m(rotation.apply(np.asarray(vertex)*scale)+translation, origin)
        assert vehicle.hit_swept(point, point, 0, 1e-12)
