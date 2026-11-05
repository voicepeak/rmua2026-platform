import numpy as np

from rmua.rules.model import load_rules
from rmua.scene.map_data import available_maps, load_map_data
from rmua.scene.track import build_track


def test_all_maps_load():
    maps = available_maps()
    assert len(maps) == 12
    assert maps[0] == "RMUA2024OL1"
    for name in maps:
        data = load_map_data(name)
        assert len(data.gates) == 12
        assert len(data.factories) == 12
        assert len(data.ports) == 12
        assert len(data.route_configs) == 12
        for gate in data.gates.values():
            assert np.all(np.isfinite(gate.center))
            assert np.isclose(np.linalg.norm(gate.normal), 1.0)


def test_ol1_gate_names_and_positions():
    data = load_map_data("RMUA2024OL1")
    assert "1_1_Gate" in data.gates
    assert "6_2_Gate" in data.gates
    gate = data.gates["1_1_Gate"]
    assert np.allclose(gate.center, [623.49585938, -87.76669922, 135.07503906], atol=1e-6)


def test_track_build_for_every_segment():
    data = load_map_data("RMUA2024OL1")
    rules = load_rules()
    for spec in rules.segments:
        track = build_track(data, spec)
        assert len(track.path) >= 4
        assert track.total_length_m > 50.0
        assert np.all(np.isfinite(track.path))
        # start position is on the outside of the start gate; end is beyond end gate.
        assert track.start_gate.signed_distance(track.start_position) < 0.0
        assert track.end_gate.signed_distance(track.end_position) > 0.0
