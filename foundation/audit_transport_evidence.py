"""Inventory historical no-breakpoint ROS transport evidence; no model fitting."""
import argparse
import hashlib
import json
from pathlib import Path


EXPECTED_LIBRARY = '69d5ea3302bdd5f20b842ec3d3e84bafcb50ef3649ada42c4d346c9bcd389885'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(repo, library):
    paths = {'decompilation': repo/'artifacts/rea/ros_transport_functions_rea.json',
             'transport_matrix': repo/'experiments/transport_matrix_20261105/summary.json',
             'unhooked_comparison': repo/'experiments/transport_control_comparison_20261105/summary.json',
             'interposer_source': repo/'scripts/ros_publish_interpose.c', 'library': library}
    evidence = json.loads(paths['decompilation'].read_bytes())
    if digest(library) != EXPECTED_LIBRARY or evidence['subject']['digest']['sha256'] != EXPECTED_LIBRARY:
        raise ValueError('library digest does not match decompilation')
    items = evidence['normalized_result']['items']
    rotor = [x for x in items if 'PublishRotorPWM' in x['procedure']['name']]
    if len(rotor) != 1 or rotor[0]['status'] != 'ok':
        raise ValueError('missing successful rotor decompilation')
    code = rotor[0]['pseudocode']
    if not all(x in code for x in ('CMD_GET_ROTOR_PWM', 'CMD_GET_IMU_DATA', 'airsim_timestamp_to_ros')):
        raise ValueError('expected separate PWM and IMU requests absent')
    matrix = json.loads(paths['transport_matrix'].read_bytes())['runs']
    if len(matrix) != 6:
        raise ValueError('unexpected transport matrix size')
    p99, phases = [], []
    for name, row in matrix.items():
        measure = row['comparisons']['pwm_command_send_return_to_sim_callback']
        p99.append(measure['p99_ms'])
        phases.extend(x['callback_to_debug_publish_ms'] for x in row['phases']
                      if x['callback_to_debug_publish_ms'] is not None)
    return {'status': 'historical ROS-layer evidence, not actuator application or physical clock measurement',
            'library_sha256': EXPECTED_LIBRARY, 'rea_evidence_id': evidence['evidence_id'],
            'pwm_header_interpretation': 'rotor values use GET_ROTOR_PWM; header uses separate GET_IMU_DATA timestamp',
            'runs': len(matrix), 'seeds': sorted({x['seed'] for x in matrix.values()}),
            'sender_return_to_command_callback_p99_ms_range': [min(p99), max(p99)],
            'matched_callback_to_debug_publish_count': len(phases),
            'matched_callback_to_debug_publish_ms_range': [min(phases), max(phases)],
            'source_sha256': {name: {'path': str(path).replace('\\', '/'), 'sha256': digest(path)}
                              for name, path in paths.items()},
            'limitations': ['historical preload hook and ROS observers can change scheduling',
                            'published values matching commands are observational, not causally proven actuator state',
                            'sender-return timing omits time before publish returned',
                            'source header is a separate IMU request, not motor sample time',
                            'physical application time, contact and clock mapping remain unobserved']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('repo', type=Path)
    p.add_argument('library', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = run(a.repo, a.library)
    result['analysis_sha256'] = digest(Path(__file__))
    a.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print('runs', result['runs'], 'callback p99 ms', result['sender_return_to_command_callback_p99_ms_range'],
          'phase count', result['matched_callback_to_debug_publish_count'])


if __name__ == '__main__':
    main()
