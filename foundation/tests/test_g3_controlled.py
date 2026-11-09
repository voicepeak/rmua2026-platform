import pytest

from foundation.benchmark_g3_controlled import f01, f02, w02


def test_controlled_w02_f01_f02_interfaces():
    wind = w02()
    assert wind['samples'] == 80
    assert 0 < wind['inside_samples'] < wind['samples']
    assert wind['first_nonzero_wind'] == [3., 0., 0.]
    scales = f01()
    assert scales['flag_one']['rotor_signal'][0] > scales['flag_zero']['rotor_signal'][0]
    assert scales['flag_zero']['rotor_signal'][0] == pytest.approx(.05, abs=1e-9)
    out = f02()
    assert out['final_elapsed_s'] > out['threshold_s']
    assert out['exceeded'] is True
