"""Compare controlled field-toggle free-motion candidates on host arrival time.

No fitted scale: compare fixed 0.25 with 1.0. Initial velocity is derived from
posthoc pose; GDB timing, contact, sensor and target sampling remain uncertain.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from foundation.compare import compare
from foundation.quad import Params, Quad
from foundation.replay_closed_loop import position_velocity
from foundation.effects import RecordedEffects
from foundation.sim import PhysicsSim

ROTOR = re.compile(r'TOGGLE_ROTOR mono=([\d.]+) rotor=(0x[0-9a-f]+) flag=(\d+) '
                   r'filtered=([^ ]+) target=([^ ]+) input=([^ ]+) thrust=([^\s]+)')
TOGGLE = re.compile(r'TOGGLE mono=([\d.]+) start=([^ ]+) elapsed=([^ ]+) natural=(\d+) forced=(\d+)')


def run(directory, start_offset=.35, end_offset=.83, with_recovery=False, engine='direct'):
    if engine not in ('direct', 'integrated'):
        raise ValueError('unknown replay engine')
    command_path, capture_path = directory/'commands.jsonl', directory/'capture.jsonl'
    log_path, = (directory/'pool').glob('*.log')
    commands = [json.loads(x) for x in command_path.read_text().splitlines() if x]
    marker = next(x for x in commands if x.get('kind') == 'start')
    t0 = marker['monotonic_ns']
    sent = [x for x in commands if x.get('kind') == 'command']
    active = [x for x in sent if start_offset <= (x['monotonic_ns']-t0)*1e-9 <= end_offset]
    if not active or any(not np.allclose(x['pwm'], [.2]*4) for x in active):
        raise ValueError('expected constant 0.20 waveform in replay interval')
    log = log_path.read_text(encoding='utf-8', errors='replace')
    toggles = [m for m in TOGGLE.finditer(log) if m[2] != 'None' and int(m[5]) == 0]
    if len(toggles) != 1:
        raise ValueError('expected one forced-zero event')
    forced = float(toggles[0][1])-t0*1e-9
    samples = [(float(m[1])-t0*1e-9, m[2], int(m[3]), float(m[5])) for m in ROTOR.finditer(log)]
    first = {}
    for t, rotor, flag, target in samples:
        if t >= forced and flag == 0 and target < .1:
            first.setdefault(rotor, t)
    if len(first) != 4:
        raise ValueError('missing low-target rotor samples')
    switch = float(np.median(list(first.values())))
    restore, restore_samples, forced_restore = None, {}, None
    if with_recovery:
        restores = [m for m in TOGGLE.finditer(log) if m[2] != 'None' and int(m[5]) == 1]
        if len(restores) != 1:
            raise ValueError('expected one forced restore event')
        forced_restore = float(restores[0][1])-t0*1e-9
        for t, rotor, flag, target in samples:
            if t >= forced_restore and flag == 1 and target > .1:
                restore_samples.setdefault(rotor, t)
        if len(restore_samples) != 4:
            raise ValueError('missing restored-target rotor samples')
        restore = float(np.median(list(restore_samples.values())))
        if not start_offset < switch < restore < end_offset:
            raise ValueError('replay window must cover both target transitions')
    records = [json.loads(x) for x in capture_path.read_text().splitlines() if x]
    pose = [x for x in records if x.get('topic') == 'pose_gt']
    imu = [x for x in records if x.get('topic') == 'imu']
    pt = np.array([(x['arrival_monotonic_ns']-t0)*1e-9 for x in pose])
    it = np.array([(x['arrival_monotonic_ns']-t0)*1e-9 for x in imu])
    xyz = np.array([x['position'] for x in pose])
    quat = np.array([x['quaternion_wxyz'] for x in pose])
    omega = np.array([x['omega_body'] for x in imu])
    if (np.any(np.diff(pt) <= 0) or np.any(np.diff(it) <= 0) or
            pt[0] > start_offset-.12 or pt[-1] < end_offset+.12 or it[0] > start_offset or it[-1] < end_offset):
        raise ValueError('insufficient or nonmonotonic sensor coverage')
    chosen = np.where((pt >= start_offset) & (pt <= end_offset))[0]
    if len(chosen) < 4 or np.max(np.diff(pt[chosen])) > .05:
        raise ValueError('insufficient pose samples')
    start = float(pt[chosen[0]])
    pos, vel = position_velocity(pt, xyz, start)
    q = Slerp(pt, Rotation.from_quat(quat[:, [1, 2, 3, 0]]))([start]).as_quat()[0]
    w = np.array([np.interp(start, it, omega[:, j]) for j in range(3)])
    params = Params(.9, (.0046890742, .0069312, .010421166), .18,
                    .000367717*(11079.03/60)**2, 4.888486266072161e-06*(11079.03/60)**2, .005)
    reference = []
    for i in chosen:
        _, v = position_velocity(pt, xyz, pt[i])
        reference.append({'t': float(pt[i]-start), 'position': xyz[i].tolist(), 'velocity': v.tolist(),
                          'quaternion_wxyz': quat[i].tolist(),
                          'omega': [float(np.interp(pt[i], it, omega[:, j])) for j in range(3)]})
    outcomes = {}
    candidates = [('scale_025', .25, with_recovery), ('no_degradation', 1., False)]
    if with_recovery:
        candidates.append(('no_recovery', .25, False))
    for label, low_scale, do_restore in candidates:
        if engine == 'integrated':
            times = [0., switch-start]
            flags = [1, 0 if low_scale == .25 else 1]
            if do_restore:
                times.append(restore-start)
                flags.append(1)
            times.append(end_offset-start+.1)
            flags.append(flags[-1])
            effects = RecordedEffects(times, None, flags)
            sim = PhysicsSim(params, recorded_effects=effects, field_rotor_target_scale=low_scale, max_substep_s=.002)
            sim.reset({'position':pos, 'velocity':vel, 'quaternion_wxyz':q[[3,0,1,2]],
                       'omega':w, 'rotor_signal':[.2]*4})
            plant = sim.quad
        else:
            plant = Quad(params)
            plant.reset(pos, vel, q[[3, 0, 1, 2]], w, [.2]*4)
        predicted = []
        for row in reference:
            while plant.time < row['t']-1e-10:
                now = start + plant.time
                step = min(.002, row['t']-plant.time)
                if now < switch < now+step:
                    step = switch-now
                if do_restore and now < restore < now+step:
                    step = restore-now
                in_low = now >= switch-1e-10 and (not do_restore or now < restore-1e-10)
                if engine == 'integrated':
                    sim.step([.2]*4, step)
                else:
                    plant.step([.2]*4, step, rotor_target_scale=low_scale if in_low else 1.)
            s = plant.state()
            predicted.append({k: (float(v) if k == 't' else v.tolist()) for k, v in s.items() if k != 'rotor_signal'})
        outcomes[label] = compare(reference, predicted, max_gap=.05)
    return {'window_start_end_s': [start, float(pt[chosen[-1]])], 'engine': engine, 'forced_flag_time_s': forced,
            'first_low_target_s_per_rotor': first, 'switch_median_s': switch,
            'forced_restore_time_s': forced_restore, 'first_restored_target_s_per_rotor': restore_samples,
            'restore_median_s': restore,
            'models': outcomes,
            'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (command_path, capture_path, log_path)},
            'limitations': ['exploratory same-run replay; not held out or acceptance',
                            'uses host subscriber arrival timestamps and posthoc pose-derived initial velocity',
                            'switch approximated by median of first low-target samples, at most 60 ms per rotor',
                            'initial rotor state assumed; command/physics delay and actual clock uncalibrated',
                            'candidate has no ground collision or wind; free-flight/contact status is not independently established',
                            'forced predicate, suppressed timeout and all-stop debugger change behavior']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--start', type=float, default=.35)
    p.add_argument('--end', type=float, default=.83)
    p.add_argument('--with-recovery', action='store_true')
    p.add_argument('--engine', choices=('direct', 'integrated'), default='direct')
    a = p.parse_args()
    result = run(a.run, a.start, a.end, a.with_recovery, a.engine)
    result['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    a.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print('window', result['window_start_end_s'], 'target switch', result['switch_median_s'])
    for name, value in result['models'].items():
        print(name, {key: val['max'] for key, val in value['metrics'].items()})


if __name__ == '__main__':
    main()
