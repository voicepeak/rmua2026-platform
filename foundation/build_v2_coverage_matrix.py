"""Build the V2.0 OL1 stage / OL2-OL12 coverage matrix.

The matrix is a coverage index, not an acceptance upgrade.  Each row records
whether a runtime detection-component root list, a runtime score sequence, an
official contact and a factory-entry observation exist for that scope.  Missing
items stay UNVERIFIED; static candidates alone never count as runtime coverage.
"""
import hashlib
import json
from pathlib import Path


ROOT = Path('.')
REPORTS = ROOT / 'foundation' / 'reports'
MAPS = ROOT / 'foundation' / 'maps'
OUTPUT_JSON = REPORTS / 'v2_coverage_matrix_20261110.json'
OUTPUT_MD = REPORTS / 'v2_coverage_matrix_20261110.md'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def gate_group_counts():
    data = load(MAPS / 'OL1' / 'detection_gate_candidates.json')
    return {name: group['count'] for name, group in data['groups'].items()}, digest(
        MAPS / 'OL1' / 'detection_gate_candidates.json')


def oL1_score():
    path = REPORTS / 'g2_score_route_seed123_20261109.json'
    data = load(path)
    return {
        'road1': {
            'score_start': data['score_start'],
            'score_end': data['score_end'],
            'increments': data['score_increment_count'],
            'observed_indices': data['observed_gate_indices'],
            'evidence': str(path).replace('\\', '/'),
            'sha256': digest(path),
            'limitations': data['limitations'],
        }
    }


def map_rows():
    rows = {}
    for map_name in ['OL%d' % i for i in range(2, 13)]:
        gate_path = MAPS / map_name / 'gate_candidates.json'
        gate = load(gate_path)
        runtime = None
        runtime_md = MAPS / map_name / 'gate_runtime_evidence.md'
        if runtime_md.is_file():
            runtime = {
                'static_gate_runtime_matches': gate['actor_count'],
                'evidence': str(runtime_md).replace('\\', '/'),
                'limitations': 'historical patched default-map startup observation; not detection gates, flight or scoring',
            }
        rows[map_name] = {
            'runtime_map_redirect': (
                {'status': 'OBSERVED', 'evidence': 'foundation/reports/g2_ol2_attempt_20261110.md',
                 'limitation': 'map URL redirect and teleport observed; no SEG/DUMP, no flyable state'}
                if map_name == 'OL2' else (
                    {'status': 'OBSERVED', 'evidence': 'foundation/reports/g2_ol3_attempt_20261110.md',
                     'limitation': 'map URL redirect and teleport observed; no SEG/DUMP, no flyable state'}
                    if map_name == 'OL3' else (
                        {'status': 'OBSERVED', 'evidence': 'foundation/reports/g2_ol4_attempt_20261110.md',
                         'limitation': 'map URL redirect and teleport observed; no SEG/DUMP'}
                        if map_name == 'OL4' else None))),
            'static_gate_candidates': gate['actor_count'],
            'static_gate_candidates_sha256': digest(gate_path),
            'runtime_gate_transform_evidence': runtime,
            'detection_components': {'status': 'UNVERIFIED', 'count': None},
            'score': {'status': 'UNVERIFIED', 'end': None},
            'official_contact': {'status': 'UNVERIFIED', 'kind': None},
            'factory_entry': {'status': 'UNVERIFIED', 'evidence': None},
        }
    return rows


def main():
    counts, gate_sha = gate_group_counts()
    score = oL1_score()
    road1_end = score['road1']['score_end']
    road1_indices = score['road1']['observed_indices']
    rows = {
        'OL1-stage1': {
            'routes': ['road1', 'road3'],
            'detection_components': {
                'status': 'PARTIAL',
                'road1': {'count': counts['road1'], 'source': 'foundation/maps/OL1/detection_gate_candidates.json'},
                'road3': {'count': counts['road3'], 'source': 'foundation/maps/OL1/detection_gate_candidates.json'},
                'sha256': gate_sha,
            },
            'score': {
                'status': 'PARTIAL',
                'road1': {'end': road1_end, 'increments': score['road1']['increments'], 'indices': road1_indices},
                'road3': {'end': 0, 'increments': 0, 'indices': [],
                          'component_triggers_observed': list(range(30)),
                          'trigger_evidence': 'foundation/reports/g2_road3_teleport_seed123_20261110.json',
                          'blocker': 'stage1 active scoring queue; stage2 injection required'},
                'evidence': score['road1']['evidence'],
                'sha256': score['road1']['sha256'],
            },
            'official_contact': {
                'status': 'PARTIAL',
                'road_side_rail': '939 grouped callbacks',
                'port_inner_wall': '115 callbacks',
                'hub_wall': '234 callbacks',
                'evidence': ['g2_contact_road_seed123_20261109.json',
                             'g2_contact_portwall_seed123_20261109.json',
                             'g2_contact_gate1_seed123_20261109_v2.json'],
            },
            'factory_entry': {'status': 'NOT_APPLICABLE', 'reason': 'stage 1-3 have no factory'},
            'limitations': 'single seed, single diagnostic route, GDB all-stop; road3 score still 0/30 after three geometry/stage-state failures',
            'attempts': 'foundation/reports/g2_road3_attempt_20261110.md',
            'route_manifest': 'foundation/maps/OL1/stage1_route_manifest.json',
            'waypoints': 'foundation/maps/OL1/stage1_waypoints.json',
        },
        'OL1-stage2': {
            'routes': ['reverse road3', 'route5'],
            'route_manifest': 'foundation/maps/OL1/stage2_route_manifest.json',
            'waypoints': 'foundation/maps/OL1/stage2_waypoints.json',
            'detection_components': {
                'status': 'PARTIAL',
                'road3': {'count': counts['road3'], 'source': 'runtime road3 component list'},
                'route5': {'count': None},
            },
            'score': {'status': 'PASS (controlled stage2 injection)', 'end': 30, 'observed_indices': list(range(30)),
                      'evidence': 'foundation/reports/g2_stage2_routexy_gate11_0_corridor_v12_seed123_20261110.json',
                      'limitation': 'controlled stage2 injection + GDB teleport; 30/30 road3, not official stage progression'},
            'official_contact': {'status': 'PARTIAL', 'kind': 'hub wall contact used by stage1 run; route5 contact not observed'},
            'factory_entry': {'status': 'NOT_APPLICABLE', 'reason': 'stage 1-3 have no factory'},
        },
        'OL1-stage3': {
            'route_manifest': 'foundation/maps/OL1/stage3_route_manifest.json',
            'waypoints': 'foundation/maps/OL1/stage3_waypoints.json',
            'routes': ['reverse route5', 'route8'],
            'detection_components': {'status': 'PARTIAL', 'declared_count': 31,
                                     'roots': 'unusable (nan transforms)',
                                     'evidence': 'foundation/reports/g2_stage3_return_20261110.json'},
            'score': {'status': 'UNVERIFIED', 'end': 0, 'observed_indices': list(range(16, 31))},
            'official_contact': {'status': 'UNVERIFIED'},
            'factory_entry': {'status': 'NOT_APPLICABLE', 'reason': 'stage 1-3 have no factory'},
        },
        'OL1-stage4': {
            'route_manifest': 'foundation/maps/OL1/stage4_route_manifest.json',
            'waypoints': 'foundation/maps/OL1/stage4_waypoints.json',
            'routes': ['reverse route8', 'route12'],
            'detection_components': {'status': 'UNVERIFIED', 'count': None,
                                     'attempt': 'foundation/reports/g2_stage4_return_20261110.md'},
            'score': {'status': 'UNVERIFIED', 'end': None,
                      'attempt': 'foundation/reports/g2_stage4_return_20261110.md'},
            'official_contact': {'status': 'UNVERIFIED'},
            'factory_entry': {'status': 'UNVERIFIED', 'expected': 'one factory before hub'},
        },
        'OL1-stage5': {
            'route_manifest': 'foundation/maps/OL1/stage5_route_manifest.json',
            'waypoints': 'foundation/maps/OL1/stage5_waypoints.json',
            'routes': ['reverse route12', 'route10'],
            'detection_components': {'status': 'UNVERIFIED', 'count': None},
            'score': {'status': 'UNVERIFIED', 'end': None},
            'official_contact': {'status': 'UNVERIFIED'},
            'factory_entry': {'status': 'UNVERIFIED', 'expected': 'one before hub and one after hub'},
        },
        'OL1-stage6': {
            'route_manifest': 'foundation/maps/OL1/stage6_route_manifest.json',
            'waypoints': 'foundation/maps/OL1/stage6_waypoints.json',
            'routes': ['reverse route10', 'route7'],
            'detection_components': {'status': 'UNVERIFIED', 'count': None},
            'score': {'status': 'UNVERIFIED', 'end': None},
            'official_contact': {'status': 'UNVERIFIED'},
            'factory_entry': {'status': 'UNVERIFIED', 'expected': 'one before hub and one after hub'},
        },
    }
    maps = map_rows()
    result = {
        'status': 'coverage index; PARTIAL/UNVERIFIED rows are not official equivalence',
        'baseline_commit': 'f1d6b7e',
        'scope': {
            'ol1_stages': list(rows),
            'maps': list(maps),
            'columns': ['detection_components', 'score', 'official_contact', 'factory_entry'],
        },
        'summary': {
            'ol1_detection_component_runtime_groups': 2,
            'ol1_detection_component_roots': counts['road1'] + counts['road3'],
            'ol1_score_components_verified': road1_end + counts['road3'],
            'ol1_score_components_pending': 0,
            'maps_with_static_gate_candidates': len(maps),
            'maps_with_runtime_gate_transform_evidence': sum(
                1 for row in maps.values() if row['runtime_gate_transform_evidence']),
            'maps_with_runtime_map_redirect': 3,
            'maps_with_runtime_detection_components': 0,
            'maps_with_runtime_score': 0,
            'maps_with_official_contact': 0,
            'factory_entrances_verified': 0,
            'factory_candidate_volumes': 12,
            'ol1_stage_route_manifests': 6,
            'ol1_stage3_declared_components': 31,
            'training_env': 'foundation/training_env.py (approximate sandbox)',
        },
        'ol1': rows,
        'maps': maps,
        'known_gaps': [
            '12 named *_Gate actors are solid route gates, not detection gates',
            'OL1 road3 score 30/30 under injected stage2 (controlled); stage1 direct remains 0/30 and component triggers 30/30',
            'OL1 stage2 score is 30/30 under controlled injection; stage3-6 runtime roots/score and route5 components remain unverified',
            'OL2-OL12 runtime detection roots and scores not yet audited',
            'factory entrance actor/trigger not located; 12 static candidate volumes exist, no event evidence',
            'OL1 stage1 direct road3 score is 0/30; stage2 injection reaches 30/30; this is not official stage progression',
        ],
        'map_layout_correction': 'OL2-OL12 are congruent copies of the same OL1 course under yaw+translation, not 12 different tracks; see foundation/reports/v2_map_clone_20261110.md. Runtime detection/score/config equivalence remains unverified.',
        'source_sha256': {
            'detection_gate_candidates': gate_sha,
            'score_evidence': score['road1']['sha256'],
        },
        'generator_sha256': digest(__file__),
    }
    OUTPUT_JSON.write_text(json.dumps(result, indent=1, ensure_ascii=False) + '\n', encoding='utf-8')
    lines = [
        '# V2.0 coverage matrix (2026-11-10)',
        '',
        'Coverage index only. Static candidates do not count as runtime detection components.',
        '',
        '> 2026-11-10 correction: OL2–OL12 与 OL1 的赛道几何在刚体变换下同构，不是 12 条',
        '> 不同赛道；见 [v2_map_clone_20261110.md](v2_map_clone_20261110.md)。下表中',
        '> OL2–OL12 的“0 runtime roots”仍表示运行时未审计，不表示静态布局不同。',
        '',
        '| scope | detection components | score | official contact | factory |',
        '|---|---|---|---|---|',
        '| OL1 stage1 road1 | %d/%d roots (runtime) | %d/%d, indices 0-32 | road rail 939 / port wall 115 / hub wall 234 | N/A |' % (
            counts['road1'], counts['road1'], road1_end, counts['road1']),
        '| OL1 stage1 road3 | %d/%d roots (runtime) | 0/%d score; 30/30 component triggers observed under stage1 (inactive queue) | none | N/A |' % (
            counts['road3'], counts['road3'], counts['road3']),
        '| OL1 stage2 | road3 30 roots known; route5 roots UNVERIFIED | 30/30 road3 score (injected stage2, controlled) | hub wall only | N/A |',
        '| OL1 stage3 | 31 declared components; roots unusable (nan) | 0/31 (observed indices 16-30) | UNVERIFIED | N/A |',
        '| OL1 stage4 | UNVERIFIED (attempt: no SEG) | UNVERIFIED (attempt: stuck at approach) | UNVERIFIED | UNVERIFIED (1 before hub) |',
        '| OL1 stage5 | UNVERIFIED | UNVERIFIED | UNVERIFIED | UNVERIFIED (before+after hub) |',
        '| OL1 stage6 | UNVERIFIED | UNVERIFIED | UNVERIFIED | UNVERIFIED (before+after hub) |',
    ]
    for map_name, row in maps.items():
        runtime = row['runtime_gate_transform_evidence']
        gate_cell = '12 static candidates' + (
            '; %d/%d runtime transform matches' % (runtime['static_gate_runtime_matches'],
                                                  row['static_gate_candidates'])
            if runtime else '')
        if row.get('runtime_map_redirect'):
            gate_cell += '; map redirect observed, no SEG/DUMP'
        lines.append('| %s | 0 runtime roots (%s) | UNVERIFIED | UNVERIFIED | UNVERIFIED |' % (
            map_name, gate_cell))
    lines += [
        '',
        '## Evidence',
        '',
        '- OL1 detection runtimes: `foundation/maps/OL1/detection_gate_candidates.json` '
        '(SHA-256 `%s`).' % gate_sha,
        '- OL1 road1 score: `foundation/reports/g2_score_route_seed123_20261109.json` '
        '(SHA-256 `%s`).' % score['road1']['sha256'],
        '- OL2 gate runtime transform matches: `foundation/maps/OL2/gate_runtime_evidence.md`.',
        '',
        '## Gaps',
        '',
    ] + ['- ' + gap for gap in result['known_gaps']] + [
        '',
        'Machine-readable: `foundation/reports/v2_coverage_matrix_20261110.json`.',
    ]
    OUTPUT_MD.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(json.dumps(result['summary'], indent=2))
    print('wrote', OUTPUT_JSON, OUTPUT_MD)


if __name__ == '__main__':
    main()
