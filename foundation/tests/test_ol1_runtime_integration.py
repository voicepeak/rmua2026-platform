from pathlib import Path

import numpy as np

from foundation.frames import ue_cm_to_ned_m
from foundation.map_scene import CandidateMap
from foundation.quad import Params
from foundation.runtime import RmuaPhysics


def test_ol1_loaded_surface_produces_runtime_contact():
    assets = Path(__file__).resolve().parents[1] / 'asset_analysis'
    scene = CandidateMap.load(assets)
    obj = scene.placements[0]
    rotation, translation, scale = scene.transforms[0]
    vertices, indices = scene.meshes[obj['mesh']]
    tri = rotation.apply(vertices[indices[0]] * scale) + translation
    normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
    normal /= np.linalg.norm(normal)
    center = tri.mean(axis=0)
    origin = [0, 0, 0]  # Synthetic origin for this integration check only.
    start = ue_cm_to_ned_m(center - normal * 100, origin)
    end = ue_cm_to_ned_m(center + normal * 100, origin)
    params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    sim = RmuaPhysics(params, physics_dt=.01, candidate_map=scene,
                      map_origin_ue_cm=origin, drone_radius_m=.05)
    sim.reset({'position': start, 'velocity': (end - start) / .1})
    observation, events = sim.step([0]*4, .1)
    assert events['contact']['kind'] == 'static_candidate'
    assert events['contact']['actor'] == obj['actor']
    assert events['terminal'] and 0 < observation['time_s'] < .1
