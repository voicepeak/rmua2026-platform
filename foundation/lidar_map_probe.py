"""Exploratory LiDAR-return vs partial candidate-map ray comparison.

This uses historical latest GPS/IMU, an assumed 5 cm sensor offset, an
unverified start-area UE/NED translation, and collision geometry as a proxy
for LiDAR-visible surfaces. A match or miss is not collision acceptance.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from foundation.map_scene import CandidateMap


def probe(scene, scan_path, *, origin_ue_cm, count, seed, extension_m=2.):
    with np.load(scan_path, allow_pickle=False) as scan:
        cloud = np.asarray(scan['xyz'], float)
        meta = json.loads(str(scan['metadata']))
    if cloud.ndim != 2 or cloud.shape[1] != 3 or count < 1 or extension_m < 0:
        raise ValueError('invalid cloud or sampling configuration')
    distances = np.linalg.norm(cloud, axis=1)
    valid = np.where(np.isfinite(cloud).all(axis=1) & (distances >= 1.) & (distances <= 40.))[0]
    if not len(valid):
        raise ValueError('no usable lidar points')
    rng = np.random.default_rng(seed)
    indices = np.sort(rng.choice(valid, size=min(count, len(valid)), replace=False))
    r = Rotation.from_quat(meta['orientation_xyzw'])
    position = np.asarray(meta['position'], float)
    sensor = position + r.apply([0., 0., -.05])
    rows = []
    for index in indices:
        v = r.apply(cloud[index])
        observed = float(distances[index])
        end = sensor + v*(1+extension_m/observed)
        result = scene.raycast_ned(sensor, end, origin_ue_cm=origin_ue_cm)
        hit = result['hits'][0] if result['hits'] else None
        predicted = float(hit['fraction']*(observed+extension_m)) if hit else None
        rows.append({'point_index': int(index), 'observed_range_m': observed, 'predicted_range_m': predicted,
                     'signed_error_m': predicted-observed if hit else None,
                     'candidate_actor': hit['actor'] if hit else None})
    errors = np.array([x['signed_error_m'] for x in rows if x['signed_error_m'] is not None])
    return {'scan': scan_path.name, 'scan_sha256': hashlib.sha256(scan_path.read_bytes()).hexdigest(), 'sampling_seed': seed,
            'gps_cloud_offset_s': meta['gps_stamp_ros']-meta['cloud_stamp_ros'],
            'imu_cloud_offset_s': meta['imu_stamp_ros']-meta['cloud_stamp_ros'],
            'selected': len(rows), 'partial_map_hits': len(errors),
            'within_1m': int(np.count_nonzero(np.abs(errors) <= 1.)),
            'absolute_error_m_p50_p95': np.percentile(np.abs(errors), [50,95]).tolist() if len(errors) else None,
            'rays': rows}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets', type=Path, required=True)
    p.add_argument('--scan', type=Path, action='append', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--origin-ue-cm', type=float, nargs=3, required=True)
    p.add_argument('--count', type=int, default=128)
    p.add_argument('--include-unverified-simple', action='store_true',
                   help='add serialized simple shapes as unverified candidates')
    args = p.parse_args()
    scene = CandidateMap.load(args.assets, include_unverified_simple=args.include_unverified_simple)
    result = {'origin_ue_cm': args.origin_ue_cm, 'unverified_simple_included': args.include_unverified_simple,
              'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'source_map_sha256': scene.source_sha256,
              'excluded_building_components': scene.excluded_count,
              'scans': [probe(scene, path, origin_ue_cm=args.origin_ue_cm, count=args.count, seed=100+i)
                        for i,path in enumerate(args.scan)],
              'limitations': ['candidate origin only supported by short initial-area trials',
                              'GPS/IMU are latest samples, not synchronized to each cloud',
                              'sensor offset and coordinate convention are assumptions',
                              'collision meshes and lidar visual returns are different modalities',
                              'candidate collision modes, building transforms and other objects remain unverified',
                              'matches do not prove collision geometry; misses do not prove free space']}
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    for row in result['scans']:
        print(row['scan'], 'selected', row['selected'], 'hits', row['partial_map_hits'],
              'within 1m', row['within_1m'], 'error p50/p95', row['absolute_error_m_p50_p95'])


if __name__ == '__main__':
    main()
