import base64
import json
import struct
import pytest
from foundation.analyze_pwm_edges import analyze


def test_edge_matching_does_not_claim_unobserved_response(tmp_path):
    commands = [{'kind': 'command', 'monotonic_ns': 100_000_000, 'pwm': [.2]*4},
                {'kind': 'command', 'monotonic_ns': 200_000_000, 'pwm': [.4]*4}]
    def out(t, pwm):
        raw = struct.pack('<IIII4d', 0, 0, 0, 0, *([pwm]*4))
        return {'topic': 'rotor_pwm', 'arrival_monotonic_ns': t, 'type': 'airsim_ros/RotorPWM',
                'md5sum': '4967432b608c06d70b17be20124fa258', 'wire_base64': base64.b64encode(raw).decode()}
    samples = [out(90_000_000, 0), out(125_000_000, .2), out(220_000_000, .2)]
    for name, rows in [('commands.jsonl', commands), ('capture.jsonl', samples)]:
        (tmp_path/name).write_text(''.join(json.dumps(x)+'\n' for x in rows))
    result = analyze(tmp_path)
    assert result['edges'][0]['first_match_delay_ms'] == pytest.approx(25)
    assert result['edges'][1]['first_match_delay_ms'] is None
