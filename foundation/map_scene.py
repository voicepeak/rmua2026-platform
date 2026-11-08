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


def _closed_indexed_mesh(triangles):
    """Topological edge test only; it does not prove a physical solid."""
    indices = np.asarray(triangles)
    if (indices.ndim != 2 or indices.shape[1] != 3 or len(indices) == 0
            or np.any(indices[:, 0] == indices[:, 1]) or np.any(indices[:, 1] == indices[:, 2])
            or np.any(indices[:, 2] == indices[:, 0])):
        return False
    edges = np.sort(np.concatenate((indices[:, [0, 1]], indices[:, [1, 2]], indices[:, [2, 0]])), axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    return bool(np.all(counts == 2))


def _candidate_inside_closed_mesh(point, vertices, triangles):
    """Three parity rays; disagreement means unknown, not outside."""
    point = np.asarray(point, float)
    low, high = vertices.min(axis=0), vertices.max(axis=0)
    if np.any(point < low) or np.any(point > high):
        return False
    distance = 4 * np.linalg.norm(high - low) + 1
    directions = np.array([[.729, .461, .506], [-.379, .813, .442], [.326, -.589, .739]])
    votes = []
    for direction in directions:
        end = point + direction / np.linalg.norm(direction) * distance
        fractions = np.sort(segment_intersections(point, end, vertices, triangles))
        # A ray can intersect two triangles on a common edge at the same
        # location. Collapse only numerically coincident fractions.
        unique = fractions[np.r_[True, np.diff(fractions) > 1e-8]] if len(fractions) else fractions
        votes.append(bool(len(unique) % 2))
    return votes[0] if all(x == votes[0] for x in votes) else None


class CandidateMap:
    def __init__(self, placements, meshes, excluded_count=0, *, assume_closed_meshes_solid=False):
        self.meshes = meshes
        self.placements = placements
        self.excluded_count = excluded_count
        if not isinstance(assume_closed_meshes_solid, bool):
            raise ValueError('solid assumption must be explicit boolean')
        self.assume_closed_meshes_solid = assume_closed_meshes_solid
        self.closed_meshes = {name: _closed_indexed_mesh(mesh[1]) for name, mesh in meshes.items()} if assume_closed_meshes_solid else {}
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
    def load_serialized_candidates(cls, directory, mesh_directory, *, simple_shapes_file=None,
                                   road_cube_shapes_file=None, rack_shapes_file=None, sprite_shapes_file=None,
                                   assume_closed_meshes_solid=False):
        """Load a map's own road/building exports; all runtime behavior unverified."""
        directory, mesh_directory = Path(directory), Path(mesh_directory)
        road_path, building_path = directory/'road_candidates.json', directory/'building_candidates.json'
        road, buildings = (json.loads(p.read_bytes()) for p in (road_path, building_path))
        if (road['source_file'] != buildings['source_file'] or road['source_sha256'] != buildings['source_sha256']
                or road['generator_sha256'] != buildings['generator_sha256']
                or road['missing_blueprint_actors'] or buildings['missing_blueprint_actors']
                or buildings['unresolved']):
            raise ValueError('mismatched or unresolved serialized map candidates')
        admitted = [x for x in buildings['placements'] if x['collision_trace_flag'] == 'CTF_UseComplexAsSimple'
                    and x['mesh_status'] == 'decoded_prefix']
        placements = road['instances'] + admitted
        meshes, paths = {}, [road_path, building_path]
        for name in sorted({x['mesh'] for x in placements}):
            path = mesh_directory / (name + '.npz')
            with np.load(path, allow_pickle=False) as data:
                vertices, triangles = data['vertices'], data['triangles']
            if (vertices.ndim != 2 or vertices.shape[1] != 3 or triangles.ndim != 2 or triangles.shape[1] != 3
                    or not np.isfinite(vertices).all() or triangles.size == 0 or triangles.min() < 0
                    or triangles.max() >= len(vertices) or triangles.dtype.kind not in 'iu'):
                raise ValueError('invalid mesh contents')
            meshes[name] = (vertices, triangles.astype(int))
            paths.append(path)
        included_simple = 0
        if simple_shapes_file is not None:
            from foundation.simple_collision_mesh import simple_mesh
            simple_path = Path(simple_shapes_file)
            simple_assets = json.loads(simple_path.read_bytes())['assets']
            simple_meshes = {}
            for name, asset in simple_assets.items():
                try:
                    simple_meshes[name] = simple_mesh(asset)
                except (KeyError, TypeError, ValueError):
                    continue
            for obj in buildings['placements']:
                if obj['collision_trace_flag'] == 'CTF_UseComplexAsSimple' or obj['mesh'] not in simple_meshes:
                    continue
                name = 'unverified_simple:' + obj['mesh']
                meshes[name] = simple_meshes[obj['mesh']]
                placements.append({**obj, 'mesh': name, 'collision_evidence': 'unverified simple candidate'})
                included_simple += 1
            paths.append(simple_path)
        if road_cube_shapes_file is not None or rack_shapes_file is not None or sprite_shapes_file is not None:
            from foundation.simple_collision_mesh import simple_mesh

            def load_group(filename, actor_class, shape_file):
                candidate_path = directory / filename
                candidate = json.loads(candidate_path.read_bytes())
                if (candidate['unresolved'] or (actor_class is not None and candidate.get('actor_class') != actor_class)
                        or candidate['source_sha256']['map'] != road['source_sha256']['map_json']
                        or candidate['source_sha256']['blueprints'] != road['source_sha256']['building_blueprints']):
                    raise ValueError('mismatched or unresolved auxiliary candidates')
                shape_path = Path(shape_file)
                shapes = json.loads(shape_path.read_bytes())['assets']
                paths.extend([candidate_path, shape_path])
                return candidate, shapes

            if road_cube_shapes_file is not None:
                cubes, shapes = load_group('road_cube_candidates.json', 'BP_Segment_Rot_C', road_cube_shapes_file)
                if cubes.get('component_prefix') != 'Cube':
                    raise ValueError('unexpected road cube selection')
                name = 'unverified_road_cube:Engine_BasicShapes_Cube'
                meshes[name] = simple_mesh(shapes['Engine_BasicShapes_Cube'])
                for obj in cubes['placements']:
                    if obj['mesh_package'] != '/Engine/BasicShapes/Cube':
                        raise ValueError('unexpected road cube mesh')
                    placements.append({**obj, 'mesh': name, 'collision_evidence': 'road cube candidate; channel unverified'})
            if rack_shapes_file is not None:
                racks, shapes = load_group('rack_candidates.json', 'BP_Rack_C', rack_shapes_file)
                rack_meshes = {}
                for name, asset in shapes.items():
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
            if sprite_shapes_file is not None:
                sprites, shapes = load_group('sprite_candidates.json', None, sprite_shapes_file)
                for obj in sprites['placements']:
                    package = obj.get('sprite_package')
                    asset_name = package.rsplit('/', 1)[-1] if package else None
                    if asset_name not in shapes or shapes[asset_name]['collision_trace_flag'] != 'CTF_UseSimpleAsComplex':
                        continue
                    name = 'unverified_sprite:' + asset_name
                    if name not in meshes:
                        meshes[name] = simple_mesh(shapes[asset_name])
                    placements.append({**obj, 'mesh': name, 'collision_evidence': 'sprite candidate; channel unverified'})
        result = cls(placements, meshes, len(buildings['placements']) - len(admitted) - included_simple,
                     assume_closed_meshes_solid=assume_closed_meshes_solid)
        result.source_sha256 = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
        return result

    @classmethod
    def load(cls, directory, *, include_unverified_simple=False, include_unverified_sprites=False,
             include_unverified_racks=False, include_unverified_road_cubes=False,
             assume_closed_meshes_solid=False):
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
        result = cls(placements, meshes, len(buildings['placements'])-len(admitted)-included_simple,
                     assume_closed_meshes_solid=assume_closed_meshes_solid)
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
        """Test surfaces and, only if explicitly enabled, candidate containment."""
        from foundation.swept_mesh import swept_sphere_triangle
        start, end = np.asarray(start, float), np.asarray(end, float)
        if (start.shape != (3,) or end.shape != (3,) or not np.isfinite([start, end]).all()
                or not np.isfinite(radius_cm) or radius_cm < 0):
            raise ValueError('invalid sphere sweep')
        lo, hi = np.minimum(start, end)-radius_cm, np.maximum(start, end)+radius_cm
        candidates = np.where(np.all(self.high >= lo, axis=1) & np.all(self.low <= hi, axis=1))[0]
        hits = []
        containment_unknown = []
        for index in candidates:
            obj = self.placements[index]
            rotation, translation, scale = self.transforms[index]
            vertices, indices = self.meshes[obj['mesh']]
            world = rotation.apply(vertices*scale) + translation
            triangles = world[indices]
            select = np.all(triangles.max(axis=1) >= lo, axis=1) & np.all(triangles.min(axis=1) <= hi, axis=1)
            if any(swept_sphere_triangle(start, end, radius_cm, tri) for tri in triangles[select]):
                hits.append({'actor': obj['actor'], 'component': obj.get('component'), 'mesh': obj['mesh']})
            elif self.assume_closed_meshes_solid and self.closed_meshes[obj['mesh']]:
                local_start = rotation.inv().apply(start-translation) / scale
                local_end = rotation.inv().apply(end-translation) / scale
                inside = [_candidate_inside_closed_mesh(p, vertices, indices) for p in (local_start, local_end)]
                if True in inside:
                    hits.append({'actor': obj['actor'], 'component': obj.get('component'), 'mesh': obj['mesh'],
                                 'reason': 'candidate_closed_mesh_containment'})
                elif None in inside:
                    containment_unknown.append({'actor': obj['actor'], 'component': obj.get('component'),
                                                'mesh': obj['mesh'], 'reason': 'ray_parity_disagrees'})
            elif self.assume_closed_meshes_solid and (np.all(start >= self.low[index]) and np.all(start <= self.high[index])
                                                       or np.all(end >= self.low[index]) and np.all(end <= self.high[index])):
                containment_unknown.append({'actor': obj['actor'], 'component': obj.get('component'),
                                            'mesh': obj['mesh'], 'reason': 'mesh_not_topologically_closed'})
        return {'hits': hits, 'candidate_count': len(candidates),
                'coverage': ('partial candidate surfaces and assumed-solid closed meshes; no time of impact'
                             if self.assume_closed_meshes_solid else
                             'partial candidate surfaces; no containment or time of impact'),
                'containment_unknown': containment_unknown,
                'excluded_building_components': self.excluded_count}

    def sweep_sphere_ned(self, start, end, radius_m, *, origin_ue_cm):
        result = self.sweep_sphere_ue(ned_m_to_ue_cm(start, origin_ue_cm),
                                      ned_m_to_ue_cm(end, origin_ue_cm), radius_m*100)
        result['frame_assumption'] = 'axis-aligned NED; caller-supplied origin not validated'
        return result
