import numpy as np

from foundation.analyze_wind_repeatability import compare


def test_repeatability_uses_common_interval_and_command_hold():
    def run(start, positions, commands):
        t = np.array([start, 1.0, 2.0])
        return {'pose': (t, np.array(positions, dtype=float)),
                'wind': (t, np.zeros((3, 3))),
                'command': (t, np.array(commands, dtype=float)), 'sha256': {}}

    zeros = [[0, 0, 0]] * 3
    commands = [[0, 0, 0, 0], [.2, .2, .2, .2], [.2, .2, .2, .2]]
    a = run(0., zeros, commands)
    b = run(.5, [[1, 0, 0]] * 3, commands)
    result = compare(a, b, duration=2., interval=.25)
    assert result['interval_s'] == [.5, 2.]
    assert result['pose_difference_m']['max'] == 1.
    assert result['wind_difference_m_s']['max'] == 0.
    assert result['command_difference_pwm']['max'] == 0.
