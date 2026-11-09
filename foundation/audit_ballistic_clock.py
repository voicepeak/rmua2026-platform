"""Exploratory gravity-consistency check for post-zero free-fall tails.

The window criterion is posthoc: a clean free-fall window requires low IMU
specific force, a smooth vertical drop and no reset. It is not a predeclared
acceptance test. Source and arrival fits are compared to show time-base
behaviour, not to identify command application latency.
"""
import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path('foundation/datasets')
OUTPUT = Path('foundation/reports/v2_ballistic_clock_audit.json')
G = 9.80665


def _read(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line]


def fit(samples, key, origin):
    t = np.array([(x[key] - origin) * 1e-9 for x in samples], float)
    z = np.array([x['position'][2] for x in samples], float)
    if len(t) < 3 or not np.isfinite(t).all() or not np.isfinite(z).all() or np.any(np.diff(t) <= 0):
        raise ValueError('invalid or nonmonotonic fit samples')
    center = t - t.mean()
    design = np.column_stack((np.ones(len(center)), center, center * center))
    coefficients = np.linalg.lstsq(design, z, rcond=None)[0]
    residual = z - design @ coefficients
    return {
        'samples': len(samples),
        'acceleration_m_s2': 2 * float(coefficients[2]),
        'initial_velocity_m_s': float(coefficients[1] + 2 * coefficients[2] * (t[0] - t.mean())),
        'quadratic_rmse_m': float(np.sqrt(np.mean(residual ** 2))),
        'duration_s': float(t[-1] - t[0]),
    }


def find_clean_window(pose, imu, command_start_ns, last_nonzero_ns):
    start_after = (last_nonzero_ns - command_start_ns) * 1e-9 + .05
    for index, first in enumerate(pose):
        start = (first['arrival_monotonic_ns'] - command_start_ns) * 1e-9
        if start < start_after:
            continue
        segment = [first]
        for sample in pose[index + 1:]:
            timestamp = (sample['arrival_monotonic_ns'] - command_start_ns) * 1e-9
            if timestamp - start > .35:
                break
            delta = sample['position'][2] - segment[-1]['position'][2]
            if abs(delta) > .1 or delta < -.02:
                break
            segment.append(sample)
        if len(segment) < 5:
            break
        end = (segment[-1]['arrival_monotonic_ns'] - command_start_ns) * 1e-9
        drop = segment[-1]['position'][2] - segment[0]['position'][2]
        specific = [x['acceleration_body'][2] for x in imu
                    if start <= (x['arrival_monotonic_ns'] - command_start_ns) * 1e-9 <= end]
        if (end - start >= .15 and .1 <= drop <= .6 and specific
                and abs(np.mean(specific)) < .05 and max(abs(x) for x in specific) < .5):
            return segment, specific, start, end
    return None


def analyze(directory):
    capture, commands = _read(directory / 'capture.jsonl'), _read(directory / 'commands.jsonl')
    pose = [x for x in capture if x.get('topic') == 'pose_gt']
    imu = [x for x in capture if x.get('topic') == 'imu']
    command = [x for x in commands if x.get('kind') == 'command']
    nonzero = [x for x in command if any(value > 0 for value in x['pwm'])]
    if len(pose) < 10 or len(imu) < 10 or not nonzero:
        return None
    command_start = command[0]['monotonic_ns']
    found = find_clean_window(pose, imu, command_start, nonzero[-1]['monotonic_ns'])
    if found is None:
        return None
    segment, specific, start, end = found
    source_origin = pose[0]['stamp_ns']
    source = fit(segment, 'stamp_ns', source_origin)
    arrival = fit(segment, 'arrival_monotonic_ns', command_start)
    return {
        'run': directory.name,
        'window_after_first_command_s': [start, end],
        'drop_m': float(segment[-1]['position'][2] - segment[0]['position'][2]),
        'imu_specific_force_mean_m_s2': float(np.mean(specific)),
        'imu_specific_force_max_abs_m_s2': float(max(abs(x) for x in specific)),
        'source_fit': source,
        'arrival_fit': arrival,
        'acceleration_deviation_from_g_fraction': {
            'source': source['acceleration_m_s2'] / G - 1,
            'arrival': arrival['acceleration_m_s2'] / G - 1,
        },
        'source_sha256': {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
                          for name in ('capture.jsonl', 'commands.jsonl')},
    }


def main():
    runs = []
    for directory in sorted(path for path in ROOT.iterdir() if path.is_dir()):
        if not (directory / 'capture.jsonl').is_file() or not (directory / 'commands.jsonl').is_file():
            continue
        result = analyze(directory)
        if result is not None:
            runs.append(result)
    result = {
        'status': 'exploratory posthoc free-fall gravity consistency; not clock calibration or G1 acceptance',
        'criterion': {'duration_s': [.15, .35], 'drop_m': [.1, .6],
                      'imu_specific_force_mean_abs_lt_m_s2': .05,
                      'imu_specific_force_max_abs_lt_m_s2': .5},
        'gravity_m_s2': G,
        'clean_run_count': len(runs),
        'runs': runs,
        'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'limitations': [
            'window criterion was chosen posthoc after inspecting existing runs',
            'heldout_mix seed 27182, 31415 and aborted seeds 260004/260006 met the clean free-fall criterion',
            'same waveform and similar initial conditions; not independent excitation',
            'residual thrust, drag and gravity-value uncertainty can bias the fitted acceleration',
            'does not identify command application time, motor state, initial velocity, contact or wind',
            'source and arrival timestamps are compared only over this short free-fall window',
        ],
    }
    OUTPUT.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for run in runs:
        print(run['run'], run['source_fit']['acceleration_m_s2'],
              run['arrival_fit']['acceleration_m_s2'])


if __name__ == '__main__':
    main()
