"""Inventory imported static mesh package dependencies of parsed Blueprints."""
import argparse
import hashlib
import json
from pathlib import Path


def package_for_import(imports, index):
    seen = set()
    while index < 0:
        if -index > len(imports) or index in seen:
            raise ValueError('invalid or cyclic import chain')
        seen.add(index)
        node = imports[-index-1]
        if node.get('ClassName') == 'Package':
            return node['ObjectName']
        index = node.get('OuterIndex', 0)
    return None


def inspect(path):
    raw = path.read_bytes()
    asset = json.loads(raw)
    imports = asset['Imports']
    refs = []
    for index, obj in enumerate(imports, 1):
        if obj.get('ClassName') == 'StaticMesh':
            refs.append({'name': obj['ObjectName'], 'package': package_for_import(imports, -index),
                         'import_index': -index})
    return {'sha256': hashlib.sha256(raw).hexdigest(), 'mesh_imports': refs}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    blueprints = {f.name: inspect(f) for f in sorted(a.directory.glob('*.json'))}
    packages = sorted({x['package'] for b in blueprints.values() for x in b['mesh_imports'] if x['package']})
    data = {'blueprints': blueprints, 'packages': packages,
            'limitations': ['import presence does not prove instance use or collision channel',
                            'transitive child Blueprint dependencies not resolved']}
    a.output.write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
    print('blueprints:', len(blueprints), 'distinct mesh packages:', len(packages))


if __name__ == '__main__':
    main()
