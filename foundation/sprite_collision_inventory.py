"""Summarize decoded PaperSprite body setups and preserve source hashes."""
import argparse
import hashlib
import json
from pathlib import Path
from foundation.extract_simple_shapes import extract


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('json_directory', type=Path)
    p.add_argument('source_directory', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    names = ('Rlogo_Sprite', 'Longlogo_Sprite', 'a6_Sprite', 'AVATA_Sprite', 'Mavic_Sprite', 'p3_Sprite') + tuple(
        word + side + '_Sprite' for word in ('One', 'Two', 'Three', 'Four', 'Five', 'Six')
        for side in ('Left', 'Right'))
    assets = {name: extract(args.json_directory/(name+'.json')) for name in names}
    sources = {name+'.'+ext: hashlib.sha256((args.source_directory/(name+'.'+ext)).read_bytes()).hexdigest()
               for name in names for ext in ('uasset', 'uexp')}
    result = {'assets': assets, 'package_sha256': sources,
              'limitations': ['packaged body setup does not prove component collision channel or effective runtime use',
                              'sprite placements and object identity are candidate matches']}
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('assets', len(assets), 'boxes', sum(len(x['boxes']) for x in assets.values()))


if __name__ == '__main__':
    main()
