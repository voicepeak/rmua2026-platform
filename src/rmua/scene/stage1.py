"""OL1 stage 1 reconstructed from map splines and captured scoring components.

The hub connector and detection aperture are development approximations.
Positions are UE centimetres in the OL1 export, converted to metres, z up.
"""
from pathlib import Path
import json

import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation

from .map_data import GateData
from .track import GateCrossing, SegmentTrack

ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / "data" / "audits" / "ol1_seed123"
END_GOAL = np.array([545.5599505719422, 519.8547547745701, 30.21285629272461])


def _spline_points(config):
    spline = config['spline']
    points = spline['points']
    keys = np.array([p['in'] for p in points], dtype=float)
    def vec(v):
        return [v['X'], v['Y'], v['Z']]
    values = np.array([vec(p['pos']) for p in points], dtype=float)
    # UE stores separate arrive/leave tangents; evaluate each segment separately.
    out = []
    for a, b in zip(range(len(points)-1), range(1, len(points))):
        curve = CubicHermiteSpline(keys[[a, b]], values[[a, b]],
            np.array([vec(points[a]['leave']), vec(points[b]['arrive'])]))
        out.append(curve(np.linspace(keys[a], keys[b], 300, endpoint=False)))
    out.append(values[-1:])
    local = np.concatenate(out)
    rot = spline['rotation']
    matrix = Rotation.from_euler('ZYX', [rot['Yaw'], rot['Pitch'], rot['Roll']], degrees=True)
    world = matrix.apply(local) + np.array(vec(spline['location']))
    return world / 100.0


def _unit(v):
    return v / max(np.linalg.norm(v), 1e-12)


def _resample(points):
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.r_[True, lengths > 1e-8]
    points = points[keep]
    cumulative = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    distances = np.r_[np.arange(0, cumulative[-1], 1.0), cumulative[-1]]
    return np.column_stack([np.interp(distances, cumulative, points[:, i]) for i in range(3)]), distances


def _crossing(name, center, normal, width, height):
    normal = _unit(np.asarray(normal, dtype=float))
    right = _unit(np.cross(normal, [0., 0., 1.]))
    gate = GateData(name, np.asarray(center), 0., np.ones(3), normal, right,
                    np.array([0., 0., 1.]), width, height, 'approx')
    return GateCrossing.from_gate(gate, normal)


def build_stage1(map_data, spec, gate_width=10., gate_height=5.):
    if map_data.name != 'RMUA2024OL1':
        raise ValueError('audited stage 1 is available only for RMUA2024OL1')
    if min(gate_width, gate_height) <= 0:
        raise ValueError('detection gate dimensions must be positive')
    configs = {c['RouteIndex']: c for c in map_data.route_configs}
    incoming = _spline_points(configs[1])
    outgoing = _spline_points(configs[3])[::-1]
    # A tangent-following cubic connector is not official track geometry.
    a, b = incoming[-1], outgoing[0]
    distance = np.linalg.norm(b-a)
    curve = CubicHermiteSpline([0., 1.], [a, b],
        [_unit(incoming[-1]-incoming[-2])*distance,
         _unit(outgoing[1]-outgoing[0])*distance])
    connector = curve(np.linspace(0., 1., 200))
    points, cumulative = _resample(np.vstack(([0., 0., 0.], incoming, connector, outgoing, END_GOAL)))
    gates = []
    for filename, reverse in [('road1_components.json', True), ('road3_components.json', False)]:
        data = json.loads((AUDIT / filename).read_text(encoding='utf-8'))
        items = data['gates'][::-1] if reverse else data['gates']
        for item in items:
            center = np.asarray(item['position_cm'], dtype=float) / 100.
            nearest = int(np.argmin(np.sum((points-center)**2, axis=1)))
            tangent = points[min(nearest+1, len(points)-1)] - points[max(nearest-1, 0)]
            # Direction follows the spline; captured component quaternion is retained in audit data.
            crossing = _crossing(f"{filename}:{item['index']}", center, tangent, gate_width, gate_height)
            gates.append(crossing)
    start = _crossing('stage1_start', np.zeros(3), points[1]-points[0], 10., 5.)
    end = _crossing('stage1_end', END_GOAL, points[-1]-points[-2], 10., 5.)
    return SegmentTrack(spec, start, end, points, cumulative, float(cumulative[-1]),
                        route_mode=True, scoring_gates=tuple(gates), audited_stage1=True)
