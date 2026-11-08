import numpy as np
import pytest

from foundation.audit_initial_velocity import fit


def test_causal_fit_uses_only_samples_at_or_before_start():
    times = np.arange(-.15, .151, .01)
    positions = np.column_stack((2*times + 3*times**2, -times + times**2, np.ones(len(times))))
    causal, _ = fit(times, positions, 0., causal=True)
    centered, _ = fit(times, positions, 0., causal=False)
    np.testing.assert_allclose(causal, [2, -1, 0], atol=1e-12)
    np.testing.assert_allclose(centered, [2, -1, 0], atol=1e-12)
    altered = positions.copy()
    altered[times > 0] += 10
    np.testing.assert_allclose(fit(times, altered, 0., causal=True)[0], causal, atol=1e-12)
    assert not np.allclose(fit(times, altered, 0., causal=False)[0], centered)


def test_fit_rejects_insufficient_history():
    with pytest.raises(ValueError, match='insufficient'):
        fit(np.array([-.01, 0., .01]), np.zeros((3, 3)), 0., causal=True)
