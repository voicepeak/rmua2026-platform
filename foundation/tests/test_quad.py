import numpy as np
import pytest
from foundation.quad import Params, Quad


def params():
    return Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)


def test_hover_and_free_fall_are_repeatable():
    p = params()
    hover = p.mass * p.gravity / (4 * p.max_thrust)
    q = Quad(p)
    q.reset(rotor_signal=[hover] * 4)
    for _ in range(100):
        result = q.step([hover] * 4, .01)
    np.testing.assert_allclose(result["position"], [0, 0, 0], atol=1e-12)
    np.testing.assert_allclose(result["omega"], [0, 0, 0], atol=1e-12)
    q.reset()
    assert q.step([0] * 4, .01)["velocity"][2] == pytest.approx(p.gravity * .01)


@pytest.mark.parametrize("motor,signs", [(0, (-1, 1, 1)), (1, (1, -1, 1)),
                                         (2, (1, 1, -1)), (3, (-1, -1, -1))])
def test_candidate_motor_torque_signs(motor, signs):
    signal = np.zeros(4)
    signal[motor] = .2
    q = Quad(params())
    q.reset(rotor_signal=signal)
    np.testing.assert_array_equal(np.sign(q.step(signal, .001)["omega"]), signs)


def test_invalid_commands_rejected():
    with pytest.raises(ValueError):
        Quad(params()).step([2, 0, 0, 0], .01)


def test_filter_updates_after_current_rotor_force():
    p = params()
    q = Quad(p)
    first = q.step([.2] * 4, .01)
    assert first['velocity'][2] == pytest.approx(p.gravity * .01)
    filtered = .2 * (1 - np.exp(-1))
    np.testing.assert_allclose(first['rotor_signal'], [filtered] * 4)
    second = q.step([.2] * 4, .01)
    assert second['velocity'][2] == pytest.approx(
        p.gravity * .02 - 4 * p.max_thrust * filtered / p.mass * .01)


def test_explicit_degradation_scales_filter_target_and_recovers():
    q = Quad(params())
    first = q.step([.2]*4, .01, rotor_target_scale=.25)
    np.testing.assert_allclose(first['rotor_signal'], [.05*(1-np.exp(-1))]*4)
    second = q.step([.2]*4, .01, rotor_target_scale=1.)
    assert second['rotor_signal'][0] > first['rotor_signal'][0]
    with pytest.raises(ValueError):
        q.step([.2]*4, .01, rotor_target_scale=1.1)
