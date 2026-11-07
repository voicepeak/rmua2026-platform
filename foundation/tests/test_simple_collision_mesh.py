import numpy as np
import pytest
from foundation.simple_collision_mesh import simple_mesh
from foundation.map_scene import CandidateMap


def test_box_candidate_transforms_and_raycast():
    mesh = simple_mesh({'boxes': [{'center_local_ue_cm': [5, 0, 0],
                                   'rotation_ue_deg': [0, 90, 0],
                                   'size_ue_cm': [4, 2, 2]}]})
    obj = {'actor': 'candidate', 'mesh': 'box', 'translation_ue_cm': [100, 0, 0],
           'quaternion_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1]}
    scene = CandidateMap([obj], {'box': mesh})
    hits = scene.raycast_ue([105, -10, 0], [105, 10, 0])['hits']
    assert hits[0]['point_ue_cm'] == pytest.approx([105, -2, 0])


def test_convex_transform_and_hull_without_indices():
    fields = lambda name, val: {'Name': name, 'Value': [{'Value': val}]}
    shape = {'vertices_local_ue_cm': [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
             'triangle_indices': [], 'element_transform_raw': {'Value': [
                 fields('Rotation', dict(X=0, Y=0, Z=0, W=1)),
                 fields('Translation', dict(X=10, Y=20, Z=30)),
                 fields('Scale3D', dict(X=2, Y=2, Z=2))]}}
    vertices, triangles = simple_mesh({'convex': [shape]})
    np.testing.assert_allclose(vertices.min(axis=0), [10, 20, 30])
    np.testing.assert_allclose(vertices.max(axis=0), [12, 22, 32])
    assert triangles.shape == (4, 3)


def test_sprite_candidate_at_observed_contact():
    from pathlib import Path
    assets = Path(__file__).resolve().parents[1] / 'asset_analysis'
    scene = CandidateMap.load(assets, include_unverified_sprites=True)
    hits = scene.sweep_sphere_ue([0, 0, -37.909], [0, 0, -37.909], 2)['hits']
    assert any(h['actor'] == '1_1_Port' and h['component'] == 'R_logo_Sprite1' for h in hits)
