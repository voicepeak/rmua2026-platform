"""Record exact packaged config lines without asserting Unreal merge semantics."""
import argparse
import hashlib
import json
from pathlib import Path


def inspect(paths):
    result = {}
    for path in paths:
        raw = path.read_bytes()
        lines = raw.decode('utf-8-sig').splitlines()
        result[str(path)] = {'sha256': hashlib.sha256(raw).hexdigest(),
                             'blockall_additions': [x for x in lines if x.lstrip().startswith('+Profiles=(Name="BlockAll"')],
                             'shape_complexity_lines': [x for x in lines if 'defaultshapecomplexity' in x.lower()]}
    return {'files': result,
            'limitations': ['raw config lines are not a merged runtime configuration',
                            'absence of a config key does not establish its engine default',
                            'component or runtime code may override collision profiles']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('output', type=Path)
    p.add_argument('configs', type=Path, nargs='+')
    a = p.parse_args()
    a.output.write_text(json.dumps(inspect(a.configs), indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
