import struct
import numpy as np
import pytest
from foundation.extract_physx_mesh import decode, body_setups


def test_decode_triangle_and_reject_bad_indices():
    head = b'NXS\x01MESH' + struct.pack('<5I', 14, 0, 8, 3, 1)
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype='<f4').tobytes()
    blob = b'prefix' + head + vertices + struct.pack('<3H', 0, 1, 2)
    points, triangles, info = decode(blob)
    assert info['header_offset'] == 6
    np.testing.assert_array_equal(triangles, [[0, 1, 2]])
    np.testing.assert_allclose(points[1], [1, 0, 0])
    with pytest.raises(ValueError, match='out-of-bounds'):
        decode(blob[:-6] + struct.pack('<3H', 0, 1, 3))


def test_reject_unknown_format():
    with pytest.raises(ValueError, match='header'):
        decode(b'unknown')


def test_body_is_identified_by_class_not_name():
    asset = {'Imports': [{'ObjectName': 'BodySetup'}], 'Exports': [
        {'ObjectName': 'RB_BodySetup_0', 'ClassIndex': -1},
        {'ObjectName': 'BodySetup_0', 'ClassIndex': 0}]}
    assert [x['ObjectName'] for x in body_setups(asset)] == ['RB_BodySetup_0']
