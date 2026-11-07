"""Convert a foundation ROS capture to the exploratory replay input schema."""
import argparse
import hashlib
import json
from pathlib import Path


def read(path):
    return [json.loads(x) for x in path.read_text(encoding="utf-8").splitlines() if x.strip()]


def write(path, rows):
    path.write_text(''.join(json.dumps(row, separators=(',', ':')) + '\n' for row in rows), encoding='utf-8')


def convert(source, dest):
    capture = read(source / 'capture.jsonl')
    events = read(source / 'commands.jsonl')
    truth = [{'stamp': r['stamp_ns'] * 1e-9, 'position': r['position'],
              'orientation': r['quaternion_wxyz']} for r in capture if r['topic'] == 'pose_gt']
    imu = [{'stamp': r['stamp_ns'] * 1e-9, 'omega_body': r['omega_body']}
           for r in capture if r['topic'] == 'imu']
    commands = []
    for row in events:
        if row.get('kind') != 'command':
            continue
        sec, nsec = divmod(row['stamp_ns'], 1_000_000_000)
        commands.append({'data': {'header': {'stamp': {'secs': sec, 'nsecs': nsec}},
                                  **{f'rotorPWM{i}': value for i, value in enumerate(row['pwm'])}}})
    if min(len(truth), len(imu), len(commands)) < 2:
        raise ValueError('insufficient samples')
    dest.mkdir(parents=True, exist_ok=False)
    write(dest / 'evaluation_truth.jsonl', truth)
    write(dest / 'evaluation_imu.jsonl', imu)
    write(dest / 'evaluation_pwm_commands.jsonl', commands)
    manifest = {'source_sha256': {name: hashlib.sha256((source/name).read_bytes()).hexdigest()
                                  for name in ('capture.jsonl', 'commands.jsonl')},
                'limitations': ['command stamps are publish-side ROS time; physical actuation delay unknown',
                                'nanosecond epoch stamps converted to float seconds for exploratory replay']}
    (dest / 'conversion_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('dest', type=Path)
    a = p.parse_args()
    convert(a.source, a.dest)


if __name__ == '__main__':
    main()
