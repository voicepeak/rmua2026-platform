import json

import pytest

from foundation.summarize_predicate_capture import summarize


def test_predicate_pairing_preserves_nearest_unaccepted_and_gap():
    event = {'monotonic_s': 10.1, 'predicate': 0}
    log = 'prefix RMUA_PREDICATE_CHANGE ' + json.dumps(event)
    gps = [{'arrival_monotonic_s': 10., 'position': [0, 0, 0], 'accepted': True},
           {'arrival_monotonic_s': 10.09, 'position': [1, 0, 0], 'accepted': False}]
    row = summarize(log, gps)[0]
    assert row['nearest_gps']['accepted'] is False
    assert row['nearest_gps']['arrival_monotonic_delta_s'] == pytest.approx(-.01)
    with pytest.raises(ValueError, match='monotonic'):
        summarize(log, list(reversed(gps)))


def test_interpolation_preserves_unclassified_events():
    from foundation.interpolate_field_observations import analyze
    class Field:
        def evaluate(self, position):
            return {'flag': int(position[0] > 0), 'matches': {}}
    events = [{'monotonic_s': 0, 'predicate': 0}, {'monotonic_s': .5, 'predicate': 1},
              {'monotonic_s': 3, 'predicate': 0}]
    gps = [{'arrival_monotonic_s': 0, 'accepted': True, 'position': [-1, 0, 0]},
           {'arrival_monotonic_s': 1, 'accepted': True, 'position': [3, 0, 0]}]
    rows, counts = analyze(events, gps, Field(), 1.1)
    assert not rows[0]['eligible'] and rows[1]['eligible'] and not rows[2]['eligible']
    assert counts == {'observed_1_candidate_1': 1}


def test_direct_pose_comparison_preserves_missing_pointers():
    from foundation.analyze_direct_field_pose import analyze
    from foundation.region_volumes import CandidateBoxVolumes
    box = {'actor': 'road', 'component': 'ChargeArea', 'translation_ue_cm': [0, 0, 0],
           'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1], 'box_half_extents_cm': [10]*3}
    events = [{'monotonic_s': 1., 'predicate': 1, 'lists': {}, 'drone_world_cm': [0, 0, 0]},
              {'monotonic_s': 2., 'predicate': 0, 'lists': {}, 'drone_read_error': 'unreadable'}]
    rows, counts = analyze(events, {'road': CandidateBoxVolumes([box])})
    assert rows[0]['available'] and rows[0]['candidate_inside']
    assert not rows[1]['available'] and rows[1]['reason'] == 'unreadable'
    assert counts == {'observed_1_inside_1': 1}
