"""Construct explicit NED training proxies from candidate asset bounds.

Caller supplies a verified parent transform and UE/NED origin. These bounds
are conservative approximations and do not establish official collision modes.
"""
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from foundation.frames import ue_cm_to_ned_m
from foundation.obstacles import Line, Vehicle, vec


def load(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    return data['assets']


def line_from_asset(assets, kind, *, parent_translation_ue_cm, parent_rotation_xyzw,
                    parent_scale, origin_ue_cm):
    if kind not in ('thinline', 'thickline'):
        raise ValueError('unknown line asset')
    t, scale, origin = map(vec, (parent_translation_ue_cm, parent_scale, origin_ue_cm))
    if np.any(scale <= 0):
        raise ValueError('invalid parent scale')
    rotation = Rotation.from_quat(parent_rotation_xyzw)
    capsule = assets[kind]['conservative_capsule']
    start = rotation.apply(np.asarray(capsule['start_cm'])*scale)+t
    end = rotation.apply(np.asarray(capsule['end_cm'])*scale)+t
    radius = float(capsule['radius_m'])*max(scale[0], scale[1])
    return Line(tuple(ue_cm_to_ned_m(start, origin)), tuple(ue_cm_to_ned_m(end, origin)), radius)


def vehicle_from_asset(assets, *, parent_translation_ue_cm, parent_yaw_rad,
                       parent_scale, origin_ue_cm, velocity_ned_m_s=(0, 0, 0)):
    t, scale, origin, velocity = map(vec, (parent_translation_ue_cm, parent_scale, origin_ue_cm, velocity_ned_m_s))
    if np.any(scale <= 0) or not math.isfinite(parent_yaw_rad):
        raise ValueError('invalid parent transform')
    row = assets['HoverTruck']
    center = Rotation.from_euler('z', parent_yaw_rad).apply(np.asarray(row['bounds_center_cm'])*scale)+t
    extents = np.asarray(row['bounds_half_extents_m'], float)*scale
    return Vehicle(tuple(ue_cm_to_ned_m(center, origin)), tuple(extents), tuple(velocity), parent_yaw_rad)


def line_from_submission(assets, submission, *, frame, origin_ue_cm, component_transform=None):
    """Interpret one logged candidate transform under an explicit frame assumption.

    ``component_transform`` is required for component-local submissions and
    must have a uniform positive scale; nonuniform composition can introduce
    shear and is deliberately not approximated. Neither frame choice is proven
    by the submitted bytes alone.
    """
    if submission.get('return_index', -1) < 0:
        raise ValueError('submission did not return a nonnegative instance index')
    rotation = Rotation.from_quat(submission['quaternion_xyzw'])
    translation = vec(submission['translation_candidate_cm'])
    scale = vec(submission['scale_candidate'])
    if frame == 'component_local':
        if component_transform is None:
            raise ValueError('component transform required')
        parent_scale = vec(component_transform['scale'])
        if np.any(parent_scale <= 0) or not np.allclose(parent_scale, parent_scale[0], rtol=0, atol=1e-9):
            raise ValueError('component scale must be positive and uniform')
        parent_rotation = Rotation.from_quat(component_transform['quaternion_xyzw'])
        translation = (parent_rotation.apply(translation*parent_scale)
                       + vec(component_transform['translation_ue_cm']))
        rotation = parent_rotation*rotation
        scale = scale*parent_scale
    elif frame != 'world' or component_transform is not None:
        raise ValueError('explicit world or component_local frame required')
    return line_from_asset(assets, submission['asset'], parent_translation_ue_cm=translation,
                           parent_rotation_xyzw=rotation.as_quat(), parent_scale=scale, origin_ue_cm=origin_ue_cm)
