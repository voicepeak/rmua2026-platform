import numpy as np

from rmua.scene.map_data import load_map_data
from rmua.scene.track import GateCrossing


def _crossing():
    data = load_map_data("RMUA2024OL1")
    return GateCrossing.from_gate(data.gates["1_1_Gate"], np.array([1.0, 0.0, 0.0]))


def test_crossing_inside_aperture():
    gate = _crossing()
    previous = gate.center - gate.normal * 1.0
    current = gate.center + gate.normal * 1.0
    assert bool(gate.contains_crossing(previous, current)[0])


def test_crossing_outside_aperture_rejected():
    gate = _crossing()
    previous = gate.center + gate.right * (gate.width_m * 0.5 + 1.0) - gate.normal
    current = previous + gate.normal * 2.0
    assert not bool(gate.contains_crossing(previous, current)[0])


def test_wrong_direction_rejected():
    gate = _crossing()
    previous = gate.center + gate.normal * 1.0
    current = gate.center - gate.normal * 1.0
    assert not bool(gate.contains_crossing(previous, current)[0])
