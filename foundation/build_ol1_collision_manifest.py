"""Create a reproducible inventory of the partial OL1 collision candidates."""
from collections import Counter
import hashlib
import json
from pathlib import Path

from foundation.map_scene import CandidateMap


ASSETS = Path('foundation/asset_analysis')
OUTPUT = Path('foundation/maps/OL1/collision_manifest.json')


def summarize(scene):
    groups = Counter(x['mesh'].split(':', 1)[0] if ':' in x['mesh'] else 'default'
                     for x in scene.placements)
    return {'instance_count': len(scene.placements), 'mesh_count': len(scene.meshes),
            'groups': dict(sorted(groups.items())),
            'excluded_building_components_only': scene.excluded_count,
            'source_sha256': dict(sorted((k.replace('\\', '/'), v) for k, v in scene.source_sha256.items()))}


def main():
    default = CandidateMap.load(ASSETS)
    optional = CandidateMap.load(ASSETS, include_unverified_simple=True,
                                 include_unverified_sprites=True, include_unverified_racks=True,
                                 include_unverified_road_cubes=True)
    result = {
        'status': 'PARTIAL', 'map_id': 'OL1',
        'source_asset_directory': '../../asset_analysis',
        'source_pak_sha256_recorded': '2407662b224ed28c64412402af044da85aced07d78ce6fc0948e6be11ea2a0c2',
        'default': summarize(default), 'all_optional_candidates': summarize(optional),
        'optional_groups_require_explicit_opt_in': ['unverified_simple', 'unverified_sprite',
                                                     'unverified_rack', 'unverified_road_cube'],
        'known_gaps': [
            'All physical collision channels and actual component inclusion remain unverified.',
            'Excluded-building count does not count other missing categories.',
            '480 rack paper/pointer components are not included in optional rack candidates.',
            'Road auxiliary cubes, building simple shapes, sprites and rack meshes are only candidates.',
            'Gate mesh transforms are not verified apertures or collision volumes.',
            'Scene origin, axis alignment, geometry coverage and initial containment remain unverified.',
            'Surface sweeps do not return time of impact; default mode does not check solid containment.',
            'Optional containment assumes topologically closed meshes are solid; open/ambiguous cases remain unknown.',
        ],
        'evidence': ['../../reports/road_collision_extraction.md',
                     '../../reports/gate_runtime_root_comparison.json',
                     '../../reports/gate_runtime_root_comparison_seed456.json',
                     '../../reports/contact_surface_seed123.json',
                     '../../reports/contact_surface_seed456.json'],
        'generator_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'loader_sha256': hashlib.sha256(Path('foundation/map_scene.py').read_bytes()).hexdigest(),
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print('default:', len(default.placements), 'all optional:', len(optional.placements),
          'excluded building components:', optional.excluded_count)


if __name__ == '__main__':
    main()
