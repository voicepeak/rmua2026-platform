import base64
import json
import struct
import pytest
from foundation.analyze_collective import analyze, pwm_value


def test_pwm_wire_format_and_type_checks():
    data = struct.pack('<IIII4d', 7, 1, 2, 0, .1, .2, .3, .4)
    row = {'type': 'airsim_ros/RotorPWM', 'md5sum': '4967432b608c06d70b17be20124fa258',
           'wire_base64': base64.b64encode(data).decode()}
    assert pwm_value(row) == pytest.approx((.1, .2, .3, .4))
    row['md5sum'] = 'wrong'
    with pytest.raises(ValueError, match='type'):
        pwm_value(row)


def test_multiple_phases_preserve_rotor_stream(tmp_path):
    def command(t, pwm):
        return {'kind': 'command', 'monotonic_ns': t, 'pwm': pwm}
    commands = [command(0, [.2]*4), command(200_000_000, [.204, .2, .2, .2]),
                command(400_000_000, [.2, .204, .2, .2]),
                {'kind': 'end', 'monotonic_ns': 600_000_000, 'status': 'completed'}]
    records = []
    for t in range(0, 600_000_000, 20_000_000):
        wire = struct.pack('<IIII4d', 0, 0, 0, 0, .2, .2, .2, .2)
        records += [{'topic': 'imu', 'arrival_monotonic_ns': t, 'omega_body': [0, 0, 0],
                     'acceleration_body': [0, 0, -9.8]},
                    {'topic': 'rotor_pwm', 'arrival_monotonic_ns': t,
                     'type': 'airsim_ros/RotorPWM', 'md5sum': '4967432b608c06d70b17be20124fa258',
                     'wire_base64': base64.b64encode(wire).decode()},
                    {'topic': 'pose_gt', 'arrival_monotonic_ns': t, 'position': [0, 0, 0]}]
    for name, values in [('commands.jsonl', commands), ('capture.jsonl', records)]:
        (tmp_path / name).write_text(''.join(json.dumps(v)+'\n' for v in values))
    result = analyze(tmp_path)
    assert len(result['phases']) == 3
    assert result['phases'][1]['provisional_incremental_slope_rad_s2'][0] < 0
    assert result['phases'][2]['provisional_incremental_slope_rad_s2'][0] > 0
