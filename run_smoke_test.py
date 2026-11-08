"""Exercise PWM, candidate effects and a line contact without ROS/UE.

This synthetic integration check is not official calibration or acceptance.
"""
import json
import math

import numpy as np

from foundation.aero import DragFaces
from foundation.effects import RecordedEffects
from foundation.obstacles import Line
from foundation.out_time import OutTimeCounter
from foundation.quad import Params
from foundation.runtime import RmuaPhysics


def main():
    params = Params(.9, (.0046890742, .0069312, .010421166), .18, 12., .16, .01)
    normals = np.vstack([np.eye(3), -np.eye(3)])
    aero = DragFaces(np.zeros((6, 3)), normals, np.full(6, .08), 1.225, 0.)
    effects = RecordedEffects([0., .1, .3, 2.], [[0, 1, 0]] * 4, [1, 0, 1, 1])
    line = Line((.20, 0, -10), (.20, 0, 10), .01)
    sim = RmuaPhysics(params, physics_dt=.0025, timeout_s=2., aero_model=aero,
                      recorded_effects=effects, field_rotor_target_scale=.25, out_time_counter=OutTimeCounter(.5),
                      lines=[line], drone_radius_m=.02)
    theta = math.radians(-15)
    sim.reset({'quaternion_wxyz': [math.cos(theta/2), 0, math.sin(theta/2), 0],
               'rotor_signal': [.24] * 4})
    saw_force_zone = False
    min_z = 0.
    wind_force_seen = False
    steps = 0
    while not sim.terminal:
        truth = sim.truth_state()
        wind_force, _ = aero.wrench(sim.sim.quad.rotation, truth['velocity'], truth['omega'], [0, 1, 0])
        still_force, _ = aero.wrench(sim.sim.quad.rotation, truth['velocity'], truth['omega'], [0, 0, 0])
        wind_force_seen |= np.linalg.norm(wind_force - still_force) > 1e-6
        _, events = sim.step([.24] * 4, .02)
        steps += 1
        saw_force_zone |= (events['force_zone_elapsed_s'] or 0) > 0
        min_z = min(min_z, sim.truth_state()['position'][2])
    if not (wind_force_seen and saw_force_zone and min_z < 0 and events['contact']
            and events['contact']['kind'] == 'line' and not events['timeout']):
        raise RuntimeError('smoke scenario did not exercise every required stage')
    print(json.dumps({'status': 'PASS: synthetic integration smoke only',
                      'physics_status': events['physics_status'], 'commands': steps,
                      'minimum_ned_z_m': float(min_z), 'wind_force_seen': bool(wind_force_seen),
                      'force_zone_elapsed_s': events['force_zone_elapsed_s'],
                      'contact': events['contact'], 'terminal_time_s': sim.truth_state()['t'],
                      'limitations': ['synthetic parameters, wind and field flags; not official alignment',
                                      'no official map, sensor model, batch or policy transfer validation']}, indent=2))


if __name__ == '__main__':
    main()
