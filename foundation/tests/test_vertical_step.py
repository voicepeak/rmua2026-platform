import numpy as np
import pytest
from foundation.fit_vertical_step import curve, fit, evaluate


def test_fit_and_frozen_holdout_against_synthetic_step():
    t = np.linspace(-.18, .30, 97)
    y = -10 + 2 * curve(t, .040, .020)
    run = {'t': t, 'y': y, 'z_range': [0, 0], 'hashes': {}}
    result = fit(run)
    assert result['delay_s'] == pytest.approx(.040, abs=.0021)
    assert result['tau_s'] == pytest.approx(.020, abs=.0021)
    assert evaluate(run, result)['imu_z_rmse_m_s2'] < 1e-8
    shifted = dict(run, y=y+.5)
    assert evaluate(shifted, result)['imu_z_rmse_m_s2'] == pytest.approx(.5)
