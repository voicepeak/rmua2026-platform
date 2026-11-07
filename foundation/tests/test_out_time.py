import pytest
from foundation.out_time import OutTimeCounter


def test_accumulates_across_return_and_uses_strict_threshold():
    counter = OutTimeCounter()
    assert not counter.tick(0, .5)
    assert not counter.tick(1, 10)
    assert counter.elapsed_s == .5
    assert not counter.tick(0, 1.5)
    assert counter.elapsed_s == 2.
    assert counter.tick(0, .125)


def test_suppression_does_not_stop_counter():
    counter = OutTimeCounter()
    assert not counter.tick(0, 2.5, timeout_suppressed=True)
    assert counter.elapsed_s == 2.5
    assert not counter.tick(1, .5)
    assert counter.tick(0, 0.)
    with pytest.raises(ValueError):
        counter.tick(2, .1)
