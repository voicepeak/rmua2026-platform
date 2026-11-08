import base64
import struct

import pytest

from foundation.pwm_echo_audit import audit, decode


def message(pwm, *, frame=b'base', nsec=4):
    wire = struct.pack('<IIII', 7, 3, nsec, len(frame)) + frame + struct.pack('<4d', *pwm)
    return {'type': 'airsim_ros/RotorPWM', 'md5sum': '4967432b608c06d70b17be20124fa258',
            'wire_base64': base64.b64encode(wire).decode(), 'arrival_monotonic_ns': 100}


def test_decode_preserves_header_and_rejects_invalid_wire():
    row = decode(message([.1, .2, .3, .4]))
    assert row['seq'] == 7 and row['stamp_ns'] == 3_000_000_004
    assert row['frame_id'] == 'base' and row['pwm'] == [.1, .2, .3, .4]
    with pytest.raises(ValueError):
        decode(message([0]*4, nsec=1_000_000_000))
    with pytest.raises(ValueError):
        decode({**message([0]*4), 'wire_base64': 'AAAA'})


def test_edge_audit_does_not_claim_matching_prior_is_new_response():
    commands = [{'monotonic_ns': 100, 'stamp_ns': 1000, 'pwm': [0]*4},
                {'monotonic_ns': 200, 'stamp_ns': 1100, 'pwm': [1]*4},
                {'monotonic_ns': 300, 'stamp_ns': 1200, 'pwm': [0]*4},
                {'monotonic_ns': 400, 'stamp_ns': 1300, 'pwm': [0]*4}]
    observations = [{'arrival_monotonic_ns': t, 'stamp_ns': s, 'pwm': p}
                    for t, s, p in [(90, 900, [0]*4), (120, 1010, [0]*4),
                                    (230, 1130, [1]*4), (310, 1210, [1]*4),
                                    (320, 1220, [0]*4)]]
    result = audit(commands, observations)
    assert result[0]['prior_already_matches'] is True
    assert 'ambiguous' in result[0]['interpretation']
    assert result[1]['prior_already_matches'] is False
    assert result[1]['first_match_arrival_minus_record_ms'] == pytest.approx(.00003)
    assert 'physical application still unverified' in result[1]['interpretation']
    assert result[2]['prior_already_matches'] is False
