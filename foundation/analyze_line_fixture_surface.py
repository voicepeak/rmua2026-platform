"""Compare candidate hit-result vector bytes with controlled line convex surfaces.

The candidate vector at 0x18 and instance index at 0x58 are hypotheses; fixture
injection and debugger timing prevent this from proving natural-course parity.
"""
import argparse
import hashlib
import json
import re
import struct
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from foundation.swept_mesh import point_triangle_distance_squared


FIXTURE = re.compile(r'LINE_FIXTURE selector=(\d+) component=(0x[0-9a-f]+) '
                     r'coordinate_source=fixed_world_origin old=[0-9a-f]{96} new=([0-9a-f]{96})')
HIT = re.compile(r'HIT_RESULT component=(0x[0-9a-f]+) bytes=([0-9a-f]+)')


def analyze(log, assets, selector):
    fixtures = FIXTURE.findall(log)
    indices = re.findall(r'LINE_FIXTURE_RESULT index=(-?\d+)', log)
    if len(fixtures) != 1 or len(indices) != 1:
        raise ValueError('expected exactly one fixture and return index')
    selected, component, raw = fixtures[0]
    if int(selected) != selector or int(indices[0]) < 0:
        raise ValueError('unexpected fixture selector or index')
    values = np.array(struct.unpack('<12f', bytes.fromhex(raw)))
    if not np.isfinite(values).all() or np.any(values[8:11] <= 0):
        raise ValueError('invalid fixture transform')
    row = assets['thickline' if selector else 'thinline']
    local = np.asarray(row['transformed_vertices_cm'], float)
    vertices = Rotation.from_quat(values[:4]).apply(local*values[8:11])+values[4:7]
    triangles = np.asarray(row['triangles'], int)
    results = []
    for observed_component, data in HIT.findall(log):
        if observed_component != component:
            continue
        payload = bytes.fromhex(data)
        if len(payload) < 0x5c:
            raise ValueError('truncated hit result')
        index = struct.unpack_from('<i', payload, 0x58)[0]
        if index != int(indices[0]):
            raise ValueError('fixture hit has mismatched candidate instance index')
        point = np.asarray(struct.unpack_from('<3f', payload, 0x18))
        if not np.isfinite(point).all():
            raise ValueError('nonfinite candidate hit vector')
        distance = min(point_triangle_distance_squared(point, vertices[tri]) for tri in triangles)**.5
        results.append({'candidate_point_cm': point.tolist(), 'surface_distance_cm': float(distance),
                        'candidate_index': index})
    if not results:
        raise ValueError('no fixture-component hits')
    d = np.asarray([r['surface_distance_cm'] for r in results])
    return {'fixture_component': component, 'fixture_return_index': int(indices[0]),
            'fixture_transform_hex': raw, 'same_component_hits': len(results),
            'surface_distance_cm': {'max': float(d.max()), 'median': float(np.median(d)),
                                    'p95': float(np.percentile(d, 95))}, 'hits': results}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('experiment', type=Path)
    p.add_argument('assets', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    manifest_raw = (a.experiment/'manifest.json').read_bytes()
    manifest = json.loads(manifest_raw)
    if manifest.get('binary_sha256') != '5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800':
        raise ValueError('unexpected binary hash')
    selector = manifest['line_fixture_selector']
    if selector not in (0, 1):
        raise ValueError('expected line fixture')
    assets_raw = a.assets.read_bytes()
    assets = json.loads(assets_raw)['assets']
    runs = {}
    for run in manifest['runs']:
        if run.get('status') != 'observed' or run.get('fixed_pwm_command_exit_code') != 0:
            raise ValueError('incomplete run in fixture matrix')
        name = f"seed{run['seed']}_repeat{run['repeat']}"
        raw = (a.experiment/name/'gdb.log').read_bytes()
        result = analyze(raw.decode('utf-8', 'replace'), assets, selector)
        result['log_sha256'] = hashlib.sha256(raw).hexdigest()
        runs[name] = result
    output = {'status': 'controlled fixture surface correspondence, not natural collision acceptance',
              'selector': selector, 'source_sha256': {
                  'manifest': hashlib.sha256(manifest_raw).hexdigest(),
                  'asset_candidates': hashlib.sha256(assets_raw).hexdigest()},
              'runs': runs,
              'limitations': ['candidate hit location offset 0x18 and instance index offset 0x58 not independently typed',
                              'fixture world transform is injected and presumed to apply directly to the collision convex',
                              'GDB and forced stage/state perturb timing and behavior',
                              'surface correspondence does not establish natural world transforms or collision response']}
    a.output.write_text(json.dumps(output, indent=2)+'\n', encoding='utf-8')
    print('runs', len(runs), 'hits', sum(r['same_component_hits'] for r in runs.values()),
          'maximum distance cm', max(r['surface_distance_cm']['max'] for r in runs.values()))


if __name__ == '__main__':
    main()
