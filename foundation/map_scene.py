"""Queryable *partial* candidate static map in UE centimetres.

Only explicitly complex-as-simple mesh candidates are queried. A negative ray
result is never proof of free space: most building modes, road auxiliary
components, instance construction, transforms and collision channels are U.
Swept-sphere queries are candidate surface tests, not official drone contact.
"""
import hashlib
import itertools
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from foundation.frames import ned_m_to_ue_cm, ue_cm_to_ned_m
from foundation.road_scene import segment_intersections


class CandidateMap:
    def __init__(self, placements, meshes, excluded_count=0):
        self.meshes = meshes
        self.placements = placements
        self.excluded_count = excluded_count
        bounds = []
        self.transforms = []
        for obj in placements:
            vertices, _ = meshes[obj['mesh']]
            lo, hi = vertices.min(axis=0), vertices.max(axis=0)
            corners = np.array(list(itertools.product(*zip(lo, hi))), dtype=float)
            rotation = Rotation.from_quat(obj['quaternion_xyzw'])
            t, scale = np.asarray(obj['translation_ue_cm'], float), np.asarray(obj['scale'], float)
            if t.shape != (3,) or scale.shape != (3,) or not np.isfinite([t, scale]).all() or np.any(scale <= 0):
                raise ValueError('invalid candidate transform')
            world = rotation.apply(corners*scale)+t
            bounds.append((world.min(axis=0), world.max(axis=0)))
            self.transforms.append((rotation, t, scale))
        self.low = np.array([x[0] for x in bounds]).reshape(-1, 3)
        self.high = np.array([x[1] for x in bounds]).reshape(-1, 3)

    @classmethod
    def load(cls, directory, *, include_unverified_simple=False, include_unverified_sprites=False,
             include_unverified_racks=False, include_unverified_road_cubes=False):
        directory = Path(directory)
        road = json.loads((directory/'OL1_road_placements_candidate.json').read_text(encoding='utf-8'))
        buildings = json.loads((directory/'OL1_building_placements_candidate.json').read_text(encoding='utf-8'))
        if buildings['unresolved']:
            raise ValueError('unresolved building transforms require explicit review')
        admitted = [x for x in buildings['placements'] if x['collision_trace_flag'] == 'CTF_UseComplexAsSimple'
                    and x['mesh_status'] == 'decoded_prefix']
        placements = road['instances'] + admitted
        meshes = {}
        for name in {x['mesh'] for x in placements}:
            path = directory/'mesh_collision'/(name+'.npz')
            if not path.is_file():
                raise FileNotFoundError(path)
            with np.load(path, allow_pickle=False) as data:
                vertices, triangles = data['vertices'], data['triangles']
            if vertices.ndim != 2 or vertices.shape[1] != 3 or triangles.ndim != 2 or triangles.shape[1] != 3:
                raise ValueError('invalid mesh arrays')
            if not np.isfinite(vertices).all() or triangles.size == 0 or triangles.min() < 0 or triangles.max() >= len(vertices):
                raise ValueError('invalid mesh contents')
            meshes[name] = (vertices, triangles.astype(int))
        simple_file = directory/'OL1_simple_convex_candidates.json'
        included_simple = 0
        if include_unverified_simple:
            from foundation.simple_collision_mesh import simple_mesh
            assets = json.loads(simple_file.read_text(encoding='utf-8'))['assets']
            simple_meshes = {}
            for name, asset in assets.items():
                try:
                    simple_meshes[name] = simple_mesh(asset)
                except (KeyError, TypeError, ValueError):
                    continue
            for obj in buildings['placements']:
                if (obj['collision_trace_flag'] == 'CTF_UseComplexAsSimple'
                        or obj['mesh'] not in simple_meshes):
                    continue
                name = 'unverified_simple:' + obj['mesh']
                meshes[name] = simple_meshes[obj['mesh']]
                placements.append({**obj, 'mesh': name, 'collision_evidence': 'unverified simple candidate'})
                included_simple += 1
        sprite_files = [directory/'OL1_sprite_placements_candidate.json', directory/'sprite_collision_candidates.json']
        if include_unverified_sprites:
            from foundation.simple_collision_mesh import simple_mesh
            sprites = json.loads(sprite_files[0].read_text(encoding='utf-8'))
            assets = json.loads(sprite_files[1].read_text(encoding='utf-8'))['assets']
            for obj in sprites['placements']:
                package = obj.get('sprite_package')
                asset_name = package.rsplit('/', 1)[-1] if package else None
                if asset_name not in assets or assets[asset_name]['collision_trace_flag'] != 'CTF_UseSimpleAsComplex':
                    continue
                name = 'unverified_sprite:' + asset_name
                if name not in meshes:
                    meshes[name] = simple_mesh(assets[asset_name])
                placements.append({**obj, 'mesh': name, 'collision_evidence': 'sprite candidate; channel unverified'})
        rack_files = [directory/'OL1_rack_placements_candidate.json', directory/'rack_simple_candidates.json']
        if include_unverified_racks:
            from foundation.simple_collision_mesh import simple_mesh
            racks = json.loads(rack_files[0].read_text(encoding='utf-8'))
            if racks['unresolved']:
                raise ValueError('unresolved rack transforms require explicit review')
            assets = json.loads(rack_files[1].read_text(encoding='utf-8'))['assets']
            rack_meshes = {}
            for name, asset in assets.items():
                try:
                    rack_meshes[name] = simple_mesh(asset)
                except (KeyError, TypeError, ValueError):
                    continue
            for obj in racks['placements']:
                if obj['mesh'] not in rack_meshes:
                    continue
                name = 'unverified_rack:' + obj['mesh']
                meshes[name] = rack_meshes[obj['mesh']]
                placements.append({**obj, 'mesh': name, 'collision_evidence': 'rack candidate; channel unverified'})
        road_cube_file = directory/'OL1_road_cubes_candidate.json'
        if include_unverified_road_cubes:
            from foundation.simple_collision_mesh import simple_mesh
            cubes = json.loads(road_cube_file.read_text(encoding='utf-8'))
            if cubes['unresolved']:
                raise ValueError('unresolved road cube transforms require explicit review')
            asset = json.loads(simple_file.read_text(encoding='utf-8'))['assets']['Engine_BasicShapes_Cube']
            name = 'unverified_road_cube:Engine_BasicShapes_Cube'
            meshes[name] = simple_mesh(asset)
            for obj in cubes['placements']:
                if obj['mesh_package'] != '/Engine/BasicShapes/Cube':
                    raise ValueError('unexpected road cube mesh')
                placements.append({**obj, 'mesh': name, 'collision_evidence': 'road cube candidate; channel unverified'})
        result = cls(placements, meshes, len(buildings['placements'])-len(admitted)-included_simple)
        paths = [directory/'OL1_road_placements_candidate.json', directory/'OL1_building_placements_candidate.json']
        paths += [directory/'mesh_collision'/(name+'.npz') for name in sorted(meshes) if not name.startswith(('unverified_simple:', 'unverified_sprite:', 'unverified_rack:', 'unverified_road_cube:'))]
        if include_unverified_simple:
            paths.append(simple_file)
        if include_unverified_sprites:
            paths.extend(sprite_files)
        if include_unverified_racks:
            paths.extend(rack_files)
        if include_unverified_road_cubes:
            paths.extend([road_cube_file, simple_file])
        result.source_sha256 = {str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
        return result

    def raycast_ue(self, start, end):
        start, end = np.asarray(start, float), np.asarray(end, float)
        if start.shape != (3,) or end.shape != (3,) or not np.isfinite([start, end]).all() or np.linalg.norm(end-start) < 1e-12:
            raise ValueError('invalid segment')
        # Segment AABB overlap is conservative; narrow phase uses triangles.
        candidates = np.where(np.all(self.high >= np.minimum(start, end), axis=1) &
                              np.all(self.low <= np.maximum(start, end), axis=1))[0]
        hits = []
        for index in candidates:
            obj = self.placements[index]
            r, t, scale = self.transforms[index]
            a, b = r.inv().apply(start-t)/scale, r.inv().apply(end-t)/scale
            vertices, triangles = self.meshes[obj['mesh']]
            fractions = segment_intersections(a, b, vertices, triangles)
            if len(fractions):
                fraction = float(fractions.min())
                hits.append({'actor': obj['actor'], 'component': obj.get('component'), 'mesh': obj['mesh'],
                             'fraction': fraction, 'point_ue_cm': (start+fraction*(end-start)).tolist()})
        return {'hits': sorted(hits, key=lambda x: x['fraction']), 'candidate_count': len(candidates),
                'coverage': 'partial, transforms and collision channels unverified',
                'excluded_building_components': self.excluded_count}

    def raycast_ned(self, start, end, *, origin_ue_cm):
        """Caller supplies an independently established, axis-aligned origin."""
        result = self.raycast_ue(ned_m_to_ue_cm(start, origin_ue_cm), ned_m_to_ue_cm(end, origin_ue_cm))
        for hit in result['hits']:
            hit['point_ned_m'] = ue_cm_to_ned_m(hit['point_ue_cm'], origin_ue_cm).tolist()
        result['frame_assumption'] = 'axis-aligned NED; caller-supplied origin not validated'
        return result

    def sweep_sphere_ue(self, start, end, radius_cm):
        """Test candidate triangle surfaces; no time of impact or inside-solid test."""
        from foundation.swept_mesh import swept_sphere_triangle
        start, end = np.asarray(start, float), np.asarray(end, float)
        if (start.shape != (3,) or end.shape != (3,) or not np.isfinite([start, end]).all()
                or not np.isfinite(radius_cm) or radius_cm < 0):
            raise ValueError('invalid sphere sweep')
        lo, hi = np.minimum(start, end)-radius_cm, np.maximum(start, end)+radius_cm
        candidates = np.where(np.all(self.high >= lo, axis=1) & np.all(self.low <= hi, axis=1))[0]
        hits = []
        for index in candidates:
            obj = self.placements[index]
            rotation, translation, scale = self.transforms[index]
            vertices, indices = self.meshes[obj['mesh']]
            world = rotation.apply(vertices*scale) + translation
            triangles = world[indices]
            select = np.all(triangles.max(axis=1) >= lo, axis=1) & np.all(triangles.min(axis=1) <= hi, axis=1)
            if any(swept_sphere_triangle(start, end, radius_cm, tri) for tri in triangles[select]):
                hits.append({'actor': obj['actor'], 'component': obj.get('component'), 'mesh': obj['mesh']})
        return {'hits': hits, 'candidate_count': len(candidates),
                'coverage': 'partial candidate surfaces; no containment or time of impact',
                'excluded_building_components': self.excluded_count}

    def sweep_sphere_ned(self, start, end, radius_m, *, origin_ue_cm):
        result = self.sweep_sphere_ue(ned_m_to_ue_cm(start, origin_ue_cm),
                                      ned_m_to_ue_cm(end, origin_ue_cm), radius_m*100)
        result['frame_assumption'] = 'axis-aligned NED; caller-supplied origin not validated'
        return result
