"""Inventory and cautiously decode cooked triangles from parsed mesh assets."""
import argparse
import hashlib
import json
from pathlib import Path
from foundation.extract_physx_mesh import extract, body_setups


def process(directory, output):
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for source in sorted(directory.glob('*.json')):
        raw = source.read_bytes()
        asset = json.loads(raw)
        bodies = body_setups(asset)
        row = {'asset': source.stem, 'source_sha256': hashlib.sha256(raw).hexdigest(),
               'body_count': len(bodies), 'collision_trace_flag': None, 'simple_shapes': {},
               'triangle_status': 'not decoded'}
        if len(bodies) == 1:
            body = bodies[0]
            for p in body.get('Data', []):
                if p.get('Name') == 'CollisionTraceFlag':
                    row['collision_trace_flag'] = p.get('EnumValue')
                if p.get('Name') == 'AggGeom':
                    row['simple_shapes'] = {s.get('Name'): len(s.get('Value') or []) for s in p.get('Value') or []}
            try:
                info = extract(source, output/(source.stem+'.npz'))
                row.update(triangle_status='decoded_prefix', vertex_count=info['vertex_count'],
                           triangle_count=info['triangle_count'])
            except (KeyError, ValueError, StopIteration) as exc:
                row['triangle_status'] = 'unavailable: '+str(exc)
        rows.append(row)
    return {'assets': rows, 'limitations': ['decoded data is local mesh space, not world aligned',
                                             'an omitted collision trace flag is unknown, not an assumed default',
                                             'not every cooked prefix is necessarily used for runtime collision',
                                             'materials, channels and actor construction remain unverified']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = process(a.directory, a.output)
    (a.output/'inventory.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('assets', len(result['assets']), 'decoded', sum(x['triangle_status']=='decoded_prefix' for x in result['assets']))


if __name__ == '__main__':
    main()
