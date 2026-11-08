import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from foundation.analyze_region_events import analyze
from foundation.region_volumes import CandidateBoxVolumes, CandidateRegionUnion, CandidateSphereRegionUnion


def test_scaled_rotated_box_signed_distance_and_ned():
    obj = {'actor': 'road', 'component': 'ChargeArea', 'translation_ue_cm': [100, 0, 0],
           'quaternion_xyzw': Rotation.from_euler('z', 90, degrees=True).as_quat().tolist(),
           'scale': [2, 1, 1], 'box_half_extents_cm': [10, 5, 3]}
    boxes = CandidateBoxVolumes([obj])
    assert boxes.query_ue([100, 0, 0])[0]['signed_distance_cm'] == pytest.approx(-3)
    assert boxes.query_ue([100, 25, 0])[0]['signed_distance_cm'] == pytest.approx(5)
    assert boxes.query_ned([1, .25, 0], origin_ue_cm=[0, 0, 0])[0]['contains'] is False
    with pytest.raises(ValueError, match='query'):
        boxes.query_ue([np.nan, 0, 0])


def test_event_comparison_only_uses_accepted_pairs():
    obj = {'actor': 'road', 'component': 'ChargeArea', 'translation_ue_cm': [0, 0, 0],
           'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1], 'box_half_extents_cm': [20]*3}
    event = {'nearest_gps': {'accepted': True, 'position': [0, 0, 0], 'arrival_delta_s': .01},
             'enter_flag': 1, 'event_flag': 0, 'segment_index': 2, 'spline': '0x1'}
    rows, counts = analyze([event, {**event, 'nearest_gps': {'accepted': False}}],
                           CandidateBoxVolumes([obj]), [0, 0, 0])
    assert len(rows) == 1
    assert counts['enter=1,event=0']['inside']['ChargeArea'] == 1
    mapped, _ = analyze([event], CandidateBoxVolumes([obj]), [0, 0, 0], {2: 'road'})
    assert mapped[0]['hypothesized_segment_actor'] == 'road'
    assert mapped[0]['hypothesized_segment_volumes']['ChargeArea']['contains']


def test_candidate_union_startup_port_and_spatial_sim():
    from pathlib import Path
    from foundation.out_time import OutTimeCounter
    from foundation.quad import Params
    from foundation.sim import PhysicsSim
    root = Path(__file__).resolve().parents[1] / 'asset_analysis'
    union = CandidateRegionUnion.from_candidates(root/'OL1_road_box_volumes_candidate.json',
                                                  root/'OL1_port_volumes_candidate.json',
                                                  root/'OL1_factory_volumes_candidate.json',
                                                  origin_ue_cm=[.0078, .001, -7])
    startup = union.evaluate([0, 0, .31])
    assert startup['flag'] == 1 and startup['matches']['port'][0]['actor'] == '1_1_Port'
    obj = {'actor': 'safe', 'component': 'ChargeArea', 'translation_ue_cm': [0, 0, 0],
           'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1], 'box_half_extents_cm': [100]*3}
    safe = CandidateRegionUnion({'road': CandidateBoxVolumes([obj])}, origin_ue_cm=[0, 0, 0])
    p = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    with pytest.raises(ValueError, match='scale must be selected explicitly'):
        PhysicsSim(p, candidate_field=safe)
    sim = PhysicsSim(p, candidate_field=safe, field_rotor_target_scale=.25, out_time_counter=OutTimeCounter(.02))
    sim.reset({'position': [5, 0, 0]})
    _, info = sim.step([.2]*4, .03)
    assert info['candidate_field_state']['flag'] == 0 and info['out_time_exceeded']
    assert info['force_degradation'] == 'unverified_spatial_union_explicit_target_candidate'
    assert info['field_rotor_target_scale'] == .25
    sim.reset({'position': [0, 0, 0]})
    _, info = sim.step([.2]*4, .03)
    assert info['candidate_field_state']['flag'] == 1 and not info['out_time_exceeded']


def test_field_snapshot_comparison_preserves_late_samples():
    from foundation.analyze_field_region_trace import analyze
    def box(actor, component, center):
        return {'actor': actor, 'component': component, 'translation_ue_cm': center,
                'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1], 'box_half_extents_cm': [10]*3}
    roads = CandidateBoxVolumes([box('road', 'ChargeArea', [1000, 0, 0])])
    ports = CandidateBoxVolumes([box('port', 'Extent', [0, 0, 0])])
    event = {'nearest_gps': {'accepted': True, 'position': [0, 0, 0], 'arrival_monotonic_delta_s': 1},
             'lists': {'array40': {'passing': [{'flag': 1}]}}, 'predicate': 1, 'monotonic_s': 0}
    rows, counts = analyze([event], roads, [0, 0, 0], .1, {'port': ports})
    assert rows[0]['candidate_union_inside'] and not rows[0]['within_arrival_gap']
    assert rows[0]['passing_objects']['array40'] == 1 and not counts


def test_sphere_region_respects_offset_rotation_and_box_corners():
    from foundation.quad import Params
    from foundation.sim import PhysicsSim
    obj = {'actor':'road', 'component':'ChargeArea', 'translation_ue_cm':[0,0,0],
           'quaternion_xyzw':[0,0,0,1], 'scale':[1,1,1], 'box_half_extents_cm':[10,10,10]}
    boxes = CandidateBoxVolumes([obj])
    field = CandidateSphereRegionUnion({'road': boxes}, origin_ue_cm=[0,0,0], offset_ue_cm=[0,0,6], radius_cm=2)
    assert field.evaluate([0,0,-.15], rotation=Rotation.identity())['flag'] == 0
    assert field.evaluate([0,0,-.05], rotation=Rotation.identity())['flag'] == 1
    field_zero = CandidateSphereRegionUnion({'road':boxes}, origin_ue_cm=[0,0,0], offset_ue_cm=[0,0,0], radius_cm=2)
    assert field_zero.evaluate([.115,.115,0], rotation=Rotation.identity())['flag'] == 0
    assert field_zero.evaluate([.11,.11,0], rotation=Rotation.identity())['flag'] == 1
    with pytest.raises(ValueError, match='rotation'):
        field.evaluate([0,0,0])
    params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    sim = PhysicsSim(params, candidate_field=field, field_rotor_target_scale=.25, max_substep_s=.01)
    sim.reset({'position':[0,0,-.05]})
    _, info = sim.step([.2]*4, .01)
    assert info['candidate_field_state']['flag'] == 1
    assert info['candidate_field_state']['status'].startswith('candidate sphere-based')
