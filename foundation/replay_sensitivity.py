"""Select candidate delay/lag on one recording and evaluate fixed holdouts.

This is sensitivity analysis, not a motor-system identification: initial rotor
state, clock alignment, output sampling and ground-start conditions are unknown.
"""
import argparse
import hashlib
import json
from pathlib import Path
from foundation.replay_closed_loop import run


def metrics(directory, duration, delay, tau):
    result = run(directory, .32, duration, experiment='delay sensitivity only; not acceptance',
                 command_delay_s=delay, motor_tau_s=tau)
    return {'attitude_deg': result['metrics']['attitude_deg'], 'omega': result['metrics']['omega'],
            'position': result['metrics']['position'], 'source_sha256': result['source_sha256']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fit', type=Path, required=True)
    p.add_argument('--roll-holdout', type=Path, required=True)
    p.add_argument('--yaw-holdout', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    candidates = []
    for delay in (0., .01, .02, .03, .04, .05, .06):
        for tau in (.002, .005, .01, .02, .04):
            result = metrics(args.fit, .45, delay, tau)
            candidates.append({'delay_s': delay, 'tau_s': tau, 'omega_rmse': result['omega']['rmse'],
                               'attitude_rmse_deg': result['attitude_deg']['rmse']})
    best = min(candidates, key=lambda x: x['omega_rmse'])
    datasets = {'fit': (args.fit, .45), 'roll_holdout': (args.roll_holdout, .45),
                'yaw_holdout': (args.yaw_holdout, .30)}
    evaluation = {name: {'baseline': metrics(path, duration, 0, .005),
                         'selected': metrics(path, duration, best['delay_s'], best['tau_s'])}
                  for name, (path, duration) in datasets.items()}
    result = {'selection': 'minimum omega norm RMSE on fit run only', 'best': best,
              'grid': candidates, 'evaluation': evaluation,
              'source_sha256': {'sensitivity_script': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                                'replay_script': hashlib.sha256(Path(run.__code__.co_filename).read_bytes()).hexdigest()},
              'limitations': ['candidate delay includes clock offset and ROS/IMU scheduling',
                              'initial rotor signal assumed equal to delayed command at replay start',
                              'same narrow ground-start regime; no physics-step or motor-lag claim',
                              'roll holdout has same pattern at smaller amplitude; yaw has a different pattern',
                              'grid, metric and window exploratory, no pre-frozen acceptance threshold']}
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'best': best, 'omega_rmse': {k: [v['baseline']['omega']['rmse'], v['selected']['omega']['rmse']]
                                                  for k,v in evaluation.items()}}, indent=2))


if __name__ == '__main__':
    main()
