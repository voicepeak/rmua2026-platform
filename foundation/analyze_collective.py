"""Summarize issued PWM phases and coincident recorded IMU and PWM debug output.

Arrival times share a host monotonic clock but include transport delay. The
debug output is not proven to be actuator state. Ground contact is possible.
"""
import argparse
import base64
import hashlib
import json
import struct
from pathlib import Path
import numpy as np


def read(path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def pwm_value(row):
    if row.get("type") != "airsim_ros/RotorPWM" or row.get("md5sum") != "4967432b608c06d70b17be20124fa258":
        raise ValueError("unexpected rotor message type")
    data = base64.b64decode(row["wire_base64"], validate=True)
    _, _, _, n = struct.unpack_from('<IIII', data)
    offset = 16 + n
    if len(data) != offset + 32:
        raise ValueError("unexpected rotor message length")
    return struct.unpack_from('<4d', data, offset)


def analyze(directory):
    cmdfile, capfile = directory / 'commands.jsonl', directory / 'capture.jsonl'
    commands, records = read(cmdfile), read(capfile)
    sent = [x for x in commands if x.get('kind') == 'command']
    ends = [x for x in commands if x.get('kind') == 'end']
    if not sent or len(ends) != 1:
        raise ValueError("missing commands or end event")
    events = []
    for row in sent:
        if not events or row['pwm'] != events[-1]['pwm']:
            events.append(row)
    imu = [x for x in records if x['topic'] == 'imu']
    pose = [x for x in records if x['topic'] == 'pose_gt']
    rotor = [x for x in records if x['topic'] == 'rotor_pwm']
    sections = []
    for i, event in enumerate(events):
        start = event['monotonic_ns']
        end = events[i+1]['monotonic_ns'] if i+1 < len(events) else ends[0]['monotonic_ns']
        # Skip first 100 ms after an edge; report counts rather than inventing values.
        def select(values):
            return [v for v in values if start + 100_000_000 <= v['arrival_monotonic_ns'] < end]
        m, r, p = select(imu), select(rotor), select(pose)
        angular_samples = [v for v in imu if start + 30_000_000 <= v['arrival_monotonic_ns'] < end]
        if len(angular_samples) >= 3:
            times = np.array([(v['arrival_monotonic_ns'] - start)*1e-9 for v in angular_samples])
            angular = np.array([v['omega_body'] for v in angular_samples])
            slope = np.linalg.lstsq(np.column_stack((times, np.ones(len(times)))), angular, rcond=None)[0][0].tolist()
        else:
            slope = None
        delta = np.asarray(event['pwm'], dtype=float) - np.mean(event['pwm'])
        prediction = None
        if np.any(np.abs(delta) > 1e-9):
            # Provisional published coefficients treated as per-(rev/s)^2;
            # this is an instantaneous incremental prediction, no lag/gyro.
            d = .18 / np.sqrt(2)
            max_thrust = .000367717 * (11079.03 / 60.)**2
            max_torque = 4.888486266072161e-06 * (11079.03 / 60.)**2
            roll = np.dot([-d, d, d, -d], delta) * max_thrust / .0046890742
            pitch = np.dot([d, -d, d, -d], delta) * max_thrust / .0069312
            yaw = np.dot([1, 1, -1, -1], delta) * max_torque / .010421166
            prediction = [float(roll), float(pitch), float(yaw)]
        sections.append({'pwm': event['pwm'], 'duration_s': (end-start)*1e-9,
                         'imu_samples': len(m), 'rotor_samples': len(r), 'pose_samples': len(p),
                         'median_imu_accel_z_m_s2': float(np.median([v['acceleration_body'][2] for v in m])) if m else None,
                         'median_debug_rotor_pwm': np.median([pwm_value(v) for v in r], axis=0).tolist() if r else None,
                         'angular_rate_slope_rad_s2': slope, 'angular_slope_samples': len(angular_samples),
                         'provisional_incremental_slope_rad_s2': prediction,
                         'position_z_range_m': [min(v['position'][2] for v in p), max(v['position'][2] for v in p)] if p else None})
    return {'status': ends[0]['status'], 'command_count': len(sent), 'phases': sections,
            'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'sha256': {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in (cmdfile, capfile)},
            'limitations': ['host monotonic arrival time includes transport delay',
                            'debug PWM is not proven to be actuator state',
                            'ground contact and later free flight are not classified',
                            'angular-rate slopes include prior motion, gravity, coupling and unknown actuation latency',
                            'incremental prediction assumes RPS-squared coefficient units and ignores motor lag and prior motion',
                            'one exploratory waveform, not a calibrated or held-out experiment']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    args.output.write_text(json.dumps(analyze(args.directory), indent=2, allow_nan=False) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
