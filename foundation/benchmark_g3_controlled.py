"""Controlled W02/F01/F02 interface checks with synthetic wind/field schedules.

These checks exercise spatial wind sampling and recorded force-zone flags. They
do not validate natural wind zones, official timing or W01/F01 acceptance.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from foundation.aero import DragFaces
from foundation.effects import RecordedEffects
from foundation.out_time import OutTimeCounter
from foundation.quad import Params
from foundation.runtime import RmuaPhysics
from foundation.wind_field import CandidateWindField
from foundation.wind_generator import WindGenerator
from vendor.rmua_re.wind import Keyframe


OUTPUT = Path('foundation/reports/v2_g3_controlled_benchmark.json')


class ObserveWind:
    def __init__(self):
        self.winds = []

    def wrench(self, rotation, velocity, omega, wind):
        self.winds.append(np.asarray(wind, float).copy())
        return np.zeros(3), np.zeros(3)


def params():
    return Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)


def w02():
    constants = {'radians_per_degree': math.pi / 180, 'zero': 0.}
    frames = [Keyframe(0., 3., 0., 1), Keyframe(10., 3., 0., 1)]
    field = CandidateWindField(WindGenerator(frames, constants), source_to_ned=np.eye(3),
                               bounds_min_ned_m=[-1., -1., -1.],
                               bounds_max_ned_m=[1., 1., 1.])
    observer = ObserveWind()
    sim = RmuaPhysics(params(), physics_dt=.01, wind_field=field, aero_model=observer)
    sim.reset({'position': [-2., 0., 0.], 'velocity': [5., 0., 0.], 'rotor_signal': [0.] * 4})
    sim.step([0.] * 4, .8)
    inside = [wind for wind in observer.winds if np.linalg.norm(wind) > 1e-12]
    outside = [wind for wind in observer.winds if np.linalg.norm(wind) <= 1e-12]
    return {
        'samples': len(observer.winds),
        'inside_samples': len(inside),
        'outside_samples': len(outside),
        'nonzero_inside_fraction': len(inside) / len(observer.winds),
        'first_nonzero_wind': next((w.tolist() for w in observer.winds if np.linalg.norm(w) > 1e-12), None),
        'last_wind': observer.winds[-1].tolist(),
        'status': 'spatial region sampling only; not natural zone or W02 acceptance',
    }


def f01():
    results = {}
    for label, flag in (('flag_one', 1), ('flag_zero', 0)):
        effects = RecordedEffects([0., 1.], None, [flag, flag])
        sim = RmuaPhysics(params(), physics_dt=.01, recorded_effects=effects,
                          field_rotor_target_scale=.25, timeout_s=1.)
        sim.reset({'rotor_signal': [.2] * 4})
        _, events = sim.step([.2] * 4, .2)
        results[label] = {
            'rotor_signal': sim.truth_state()['rotor_signal'].tolist(),
            'force_degradation_status': events['force_degradation_status'],
            'field_scale': events['field_rotor_target_scale'],
        }
    return results


def f02():
    effects = RecordedEffects([0., .1, .2, .3, .4, 1.], None, [1, 0, 1, 0, 1, 1])
    counter = OutTimeCounter(.15)
    sim = RmuaPhysics(params(), physics_dt=.01, recorded_effects=effects,
                      field_rotor_target_scale=.25, out_time_counter=counter, timeout_s=1.)
    sim.reset()
    history = []
    for _ in range(40):
        _, events = sim.step([.2] * 4, .01)
        history.append({'time_s': sim.truth_state()['t'],
                        'elapsed_s': events['force_zone_elapsed_s'],
                        'exceeded': events['force_zone_timeout_candidate']})
    return {
        'threshold_s': .15,
        'final_elapsed_s': history[-1]['elapsed_s'],
        'exceeded': history[-1]['exceeded'],
        'history': history,
        'status': 'controlled recorded flags only; not natural F02 acceptance',
    }


def main():
    result = {
        'status': 'controlled synthetic G3 interface checks; not natural wind/force-zone evidence',
        'w02_spatial_wind': w02(),
        'f01_flag_scale': f01(),
        'f02_recorded_flags': f02(),
        'limitations': [
            'wind frames and field flag schedule are synthetic',
            'no official natural zone, boundary or 2 s rule data',
            'no independent physical force/trajectory comparison',
            'field scale 0.25 is an explicit test input, not an official default',
        ],
        'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    OUTPUT.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({k: result[k] for k in ('w02_spatial_wind', 'f01_flag_scale', 'f02_recorded_flags')}, indent=2))


if __name__ == '__main__':
    main()
