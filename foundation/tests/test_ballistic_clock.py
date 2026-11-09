import numpy as np
import pytest

from foundation.audit_ballistic_clock import fit


def test_quadratic_gravity_fit():
    origin = 1_000_000_000_000
    samples = [{'stamp_ns': origin + int(i * 10_000_000),
                'position': [0., 0., 1. + 2. * (i * .01) + .5 * 9.80665 * (i * .01) ** 2]}
               for i in range(30)]
    result = fit(samples, 'stamp_ns', origin)
    assert result['samples'] == 30
    np.testing.assert_allclose(result['acceleration_m_s2'], 9.80665, atol=1e-9)
    np.testing.assert_allclose(result['initial_velocity_m_s'], 2., atol=1e-9)
    assert result['quadratic_rmse_m'] < 1e-12


def test_fit_rejects_nonfinite_or_nonmonotonic_samples():
    with pytest.raises(ValueError, match='invalid'):
        fit([{'stamp_ns': 0, 'position': [0, 0, 0]},
             {'stamp_ns': 1, 'position': [0, 0, float('nan')]},
             {'stamp_ns': 2, 'position': [0, 0, 0]}], 'stamp_ns', 0)
    with pytest.raises(ValueError, match='invalid'):
        fit([{'stamp_ns': 2, 'position': [0, 0, 0]},
             {'stamp_ns': 1, 'position': [0, 0, 0]},
             {'stamp_ns': 3, 'position': [0, 0, 0]}], 'stamp_ns', 0)
