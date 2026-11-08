"""Check holdout metadata and timing coverage without evaluating trajectory."""
import argparse
import hashlib
import json
from pathlib import Path


def inspect(directory):
    paths = [directory / name for name in ('run.json', 'commands.jsonl', 'capture.jsonl', 'pool/pool.json')]
    raw = [p.read_bytes() for p in paths]
    run, pool = json.loads(raw[0]), json.loads(raw[3])
    events = [json.loads(x) for x in raw[1].splitlines() if x]
    cmds = [x for x in events if x.get('kind') == 'command']
    end = [x for x in events if x.get('kind') == 'end']
    if run.get('exit_status') != 0 or run.get('mode') != 'v2_holdout_axis' or len(end) != 1 or end[0].get('status') != 'completed':
        raise ValueError('run did not complete normally')
    if not cmds or list(directory.glob('*.gdb')):
        raise ValueError('missing commands or found debugger script')
    epoch = cmds[0]['stamp_ns']
    command_times = [(x['stamp_ns'] - epoch) * 1e-9 for x in cmds]
    capture = [json.loads(x) for x in raw[2].splitlines() if x]
    coverage = {}
    for topic in ('pose_gt', 'imu'):
        stamps = [(x['stamp_ns'] - epoch) * 1e-9 for x in capture if x.get('topic') == topic and x.get('stamp_ns')]
        if len(stamps) < 5 or any(b <= a for a, b in zip(stamps, stamps[1:])):
            raise ValueError(f'{topic}: invalid timestamps')
        near = [x for x in stamps if .18 <= x <= 1.42]
        coverage[topic] = {'first_s': stamps[0], 'last_s': stamps[-1],
                           'sample_count': len(near),
                           'max_gap_s': max((b - a for a, b in zip(near, near[1:])), default=None),
                           'covers_planned_fit_interval': stamps[0] <= .18 and stamps[-1] >= 1.42}
    return {'directory': str(directory), 'seed_requested': run.get('seed_requested'),
            'settings_sha256': pool.get('settings_sha256'), 'player_status': end[0]['status'],
            'command_last_s': command_times[-1], 'commands_cover_planned_window': command_times[-1] >= 1.30,
            'coverage': coverage, 'source_sha256': {str(p.relative_to(directory)).replace('\\', '/'): hashlib.sha256(b).hexdigest() for p, b in zip(paths, raw)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('directories', type=Path, nargs='+')
    args = parser.parse_args()
    rows = [inspect(directory) for directory in args.directories]
    result = {'status': 'sealed candidate holdout; no trajectories or model errors evaluated',
              'protocol': 'v2_blind_holdout_protocol.md', 'runs': rows,
              'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'limitations': ['timestamp coverage does not prove no collision or synchronized actuation',
                              'sampling interval and model assumptions require separate validation']}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for r in rows:
        print(r['seed_requested'], r['commands_cover_planned_window'],
              {k: v['covers_planned_fit_interval'] for k, v in r['coverage'].items()})


if __name__ == '__main__':
    main()
