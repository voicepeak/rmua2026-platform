import struct

import pytest

from foundation.analyze_line_fixture_surface import analyze


def test_controlled_fixture_surface_and_index_matching():
    transform = struct.pack('<12f', 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0).hex()
    payload = bytearray(0x5c)
    struct.pack_into('<3f', payload, 0x18, .2, .2, .01)
    struct.pack_into('<i', payload, 0x58, 7)
    log = (f'LINE_FIXTURE selector=0 component=0x123 coordinate_source=fixed_world_origin '
           f'old={transform} new={transform}\nLINE_FIXTURE_RESULT index=7\n'
           f'HIT_RESULT component=0x123 bytes={payload.hex()}\n')
    assets = {'thinline': {'transformed_vertices_cm': [[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                           'triangles': [[0, 1, 2]]}}
    result = analyze(log, assets, 0)
    assert result['same_component_hits'] == 1
    assert result['surface_distance_cm']['max'] == pytest.approx(.01)
    struct.pack_into('<i', payload, 0x58, 8)
    wrong = log[:log.index('HIT_RESULT')] + f'HIT_RESULT component=0x123 bytes={payload.hex()}\n'
    with pytest.raises(ValueError, match='mismatched'):
        analyze(wrong, assets, 0)
