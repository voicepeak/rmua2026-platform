"""List building Blueprint package imports from a UAssetAPI map dump."""
import argparse
import hashlib
import json
from pathlib import Path


def packages(asset):
    return sorted({x['ObjectName'] for x in asset['Imports']
                   if x.get('ClassName') == 'Package'
                   and x.get('ObjectName', '').startswith('/Game/Resources/Buildings/')
                   and x['ObjectName'].rsplit('/', 1)[-1].startswith('BP_')})


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    raw = a.source.read_bytes()
    data = {'source_sha256': hashlib.sha256(raw).hexdigest(), 'packages': packages(json.loads(raw)),
            'limitations': ['imports indicate references, not active instances or collision geometry',
                            'does not include dependencies from external Blueprints']}
    a.output.write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
    print('building blueprint packages:', len(data['packages']))


if __name__ == '__main__':
    main()
