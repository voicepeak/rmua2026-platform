import numpy as np

from foundation.benchmark_map_sweep_miss import RADIUS_M, _cases
from foundation.map_scene import CandidateMap
from foundation.quad import Params
from foundation.sim import PhysicsSim


def test_actual_map_high_speed_crossings_do_not_miss():
    scene = CandidateMap.load(
        'foundation/asset_analysis', include_unverified_simple=True,
        include_unverified_sprites=True, include_unverified_racks=True,
        include_unverified_road_cubes=True, include_unverified_building_meshes=True,
        include_unverified_rack_meshes=True)
    cases = _cases(scene, count_per_group=2)
    params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    for speed in (10., 150.):
        duration = 2. / speed
        for substep in (.01, .1):
            for case in cases:
                start = np.asarray(case['start_ned_m'])
                end = np.asarray(case['end_ned_m'])
                sim = PhysicsSim(params, candidate_map=scene, map_origin_ue_cm=[0, 0, 0],
                                 drone_radius_m=RADIUS_M, max_substep_s=substep)
                sim.reset({'position': start, 'velocity': (end - start) / duration})
                _, info = sim.step([0] * 4, duration)
                assert info['collision'] is not None
                assert info['collision']['kind'] == 'static_candidate'
