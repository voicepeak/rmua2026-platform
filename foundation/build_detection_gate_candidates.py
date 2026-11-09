"""Build the OL1 candidate detection-gate manifest from the runtime score audit.

The source audit recorded 33+30 runtime component roots associated with the two
stage-1 scoring splines. This manifest is a candidate trigger layer, not a
verified aperture, collision volume or official score.
"""
import hashlib
import json
from pathlib import Path


ROOT = Path('data/audits/ol1_seed123')
OUTPUT = Path('foundation/maps/OL1/detection_gate_candidates.json')
SOURCES = ('road1_components.json', 'road3_components.json')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    groups = {}
    source_sha256 = {}
    for name in SOURCES:
        path = ROOT / name
        raw = path.read_bytes()
        data = json.loads(raw)
        key = path.stem.replace('_components', '')
        if data['count'] != len(data['gates']):
            raise ValueError('count does not match gate list')
        groups[key] = {
            'pid': data['pid'],
            'spline_actor': data['spline_actor'],
            'count': data['count'],
            'gates': data['gates'],
        }
        source_sha256[name] = hashlib.sha256(raw).hexdigest()
    result = {
        'status': 'candidate detection-gate components; trigger volume and scoring unverified',
        'frame': 'UE world centimetres; quaternion xyzw; serialized runtime roots',
        'source_audit_directory': '../../data/audits/ol1_seed123',
        'source_sha256': source_sha256,
        'groups': groups,
        'limitations': [
            'runtime root transforms do not establish the trigger plane, aperture or extent',
            'the 12 named *_Gate actors are solid BlockAll route gates, not these detection components',
            'component identity is inferred from the scoring trace, not decoded object names',
            'the audit run used GDB all-stop and one seed/route',
            'no official scoring or collision equivalence is implied',
        ],
        'generator_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(result, separators=(',', ':'), allow_nan=False) + '\n', encoding='utf-8')
    print('groups', {key: group['count'] for key, group in groups.items()})


if __name__ == '__main__':
    main()
