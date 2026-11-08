import base64
import json
import struct

import pytest

from foundation.audit_calibration_wind import analyze, decode


def wind(stamp_ns, vector):
    sec, nsec = divmod(stamp_ns, 1_000_000_000)
    wire = struct.pack('<IIII3d', 1, sec, nsec, 0, *vector)
    return {'topic': 'wind', 'type': 'geometry_msgs/Vector3Stamped',
            'md5sum': '7b324c7325e683bf02a9b14b01090ec7',
            'wire_base64': base64.b64encode(wire).decode('ascii')}


def test_wind_window_and_missing_samples(tmp_path):
    (tmp_path/'commands.jsonl').write_text(json.dumps({'kind': 'command', 'stamp_ns': 1_000_000_000})+'\n')
    rows = [wind(1_200_000_000, (9, 0, 0)), wind(1_400_000_000, (0, 3, 4)),
            wind(1_700_000_000, (0, 0, 0)), wind(1_900_000_000, (9, 0, 0))]
    (tmp_path/'capture.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in rows))
    result = analyze(tmp_path)
    assert result['window_sample_count'] == 2 and result['max_observed_vector_norm'] == 5
    assert result['max_adjacent_gap_s'] == pytest.approx(.3)
    (tmp_path/'capture.jsonl').write_text(json.dumps(rows[0])+'\n')
    result = analyze(tmp_path)
    assert result['window_sample_count'] == 0 and result['max_observed_vector_norm'] is None


def test_wind_decoder_rejects_invalid_message():
    row = wind(1_000_000_000, (0, 0, 0))
    row['wire_base64'] = base64.b64encode(b'bad').decode('ascii')
    with pytest.raises(ValueError, match='short'):
        decode(row)
