"""Compare optimized and single free-flight executions on the local machine."""
import hashlib
import json
import platform
import statistics
import sys
from pathlib import Path

import numpy as np

from foundation.benchmark_batch_reference import exercise


def main():
    # A small warmup avoids timing first-use imports; three sequential trials
    # remain insufficient for a controlled performance study.
    exercise(8, commands=2, backend='auto')
    groups = {}
    for size in (64, 256):
        trials = [exercise(size, seed=20261109+i, commands=12, backend='auto') for i in range(3)]
        if any(r['backend_used'] != 'vectorized_free_flight' for r in trials):
            raise ValueError('expected vectorized free-flight path')
        if any(max(r['final_max_absolute_errors'].values()) > 1e-12 for r in trials):
            raise ValueError('batch/single final state parity failed')
        fast = statistics.median(r['batch_environment_commands_per_wall_s'] for r in trials)
        single = statistics.median(r['single_environment_commands_per_wall_s'] for r in trials)
        groups[str(size)] = {'trials': trials, 'median_batch_commands_per_s': fast,
                             'median_singles_commands_per_s': single,
                             'median_local_batch_to_singles_ratio': fast / single}
    result = {'status': 'vectorized free-flight local benchmark; not complete scene or official comparison',
              'platform': platform.platform(), 'processor': platform.processor(),
              'python_version': sys.version.split()[0], 'numpy_version': np.__version__, 'groups': groups,
              'code_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                              (Path(__file__), Path('foundation/benchmark_batch_reference.py'), Path('foundation/batch.py'),
                               Path('foundation/runtime.py'), Path('foundation/sim.py'), Path('foundation/quad.py'))},
              'limitations': ['free flight only; map, wind, field, gates, obstacles and timeouts use reference fallback',
                              'three local trials, 12 commands each, no CPU isolation',
                              'compares to local single environments, not official simulator or cross-machine performance',
                              'batch speed does not establish physical alignment']}
    Path('foundation/reports/v2_batch_vectorized_benchmark.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for size, group in groups.items():
        print(size, 'batch commands/s', round(group['median_batch_commands_per_s'], 1),
              'local single ratio', round(group['median_local_batch_to_singles_ratio'], 2),
              'max state error', max(max(r['final_max_absolute_errors'].values()) for r in group['trials']))


if __name__ == '__main__':
    main()
