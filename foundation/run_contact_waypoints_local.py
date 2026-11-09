#!/usr/bin/env python3
"""Run one isolated official contact waypoint probe on the local host.

The official simulator is launched in a disposable headless-pool container with
the read-only contact breakpoint.  The waypoint pilot uses debug pose truth and
is diagnostic only; this is not a race or policy result.
"""
import argparse
import hashlib
import json
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

SIMULATOR = Path('/home/tianbot/RUMA-by-helinjun/main/simulator/simulator_12.0.0.5')
SETTINGS = Path('/home/tianbot/RUMA-by-helinjun/main/configs/simulator_settings.json')
POOL = Path('/home/tianbot/桌面/RMUA2026-re/scripts/headless_pool.py')
BINARY = SIMULATOR / 'Build/LinuxNoEditor/RMUA/Binaries/Linux/RMUA-Linux-Shipping'
EXPECTED_BINARY_SHA256 = '5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800'
PROBE_IMAGE = 'rmua/re-probe:20261104'


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def run(args, **kwargs):
    return subprocess.run(args, text=True, check=True, **kwargs)


def docker_containers(prefix='rmua-re-'):
    result = subprocess.run(['docker', 'ps', '--format', '{{.Names}}'],
                            text=True, capture_output=True, check=True)
    return [name for name in result.stdout.splitlines() if name.startswith(prefix)]


def port_free(port):
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(('127.0.0.1', port))
        except OSError:
            return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('waypoints', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--seed', type=int, default=123)
    parser.add_argument('--port', type=int, default=11831)
    parser.add_argument('--player-timeout', type=int, default=90)
    parser.add_argument('--collector-seconds', type=int, default=60)
    parser.add_argument('--pool', type=Path, default=POOL)
    parser.add_argument('--simulator', type=Path, default=SIMULATOR)
    parser.add_argument('--settings', type=Path, default=SETTINGS)
    parser.add_argument('--image', default=PROBE_IMAGE)
    parser.add_argument('--gdb', type=Path, default=Path(__file__).parent / 'probes' / 'contact_readonly.gdb')
    args = parser.parse_args()
    if not 1 <= args.player_timeout <= 300:
        parser.error('player timeout must be 1-300 s')
    if not 5 <= args.collector_seconds <= 180:
        parser.error('collector duration must be 5-180 s')
    binary = args.simulator / 'Build/LinuxNoEditor/RMUA/Binaries/Linux/RMUA-Linux-Shipping'
    if not binary.is_file():
        parser.error('simulator binary not found')
    if not args.settings.is_file():
        parser.error('settings not found')
    if not args.pool.is_file():
        parser.error('headless pool not found')
    if args.output.exists():
        parser.error('output directory already exists')
    if not port_free(args.port):
        parser.error('port %d is already in use' % args.port)
    active = docker_containers()
    if active:
        parser.error('existing rmua-re containers: ' + ', '.join(active))
    actual_binary_hash = sha256(binary)
    if actual_binary_hash != EXPECTED_BINARY_SHA256:
        parser.error('unexpected simulator binary hash: ' + actual_binary_hash)
    args.output.mkdir(parents=True)
    scripts = args.output / 'scripts'
    scripts.mkdir()
    here = Path(__file__).parent
    probe = args.gdb
    player_source = here / 'probes' / 'contact_waypoints.py'
    collector_source = here / 'probes' / 'collect_ros.py'
    for source in (probe, player_source, collector_source):
        if not source.is_file():
            parser.error('missing probe source: ' + str(source))
    shutil.copy2(probe, scripts / 'crash_capture.gdb')
    shutil.copy2(args.pool, scripts / 'headless_pool.py')
    shutil.copy2(player_source, scripts / 'contact_waypoints.py')
    shutil.copy2(args.waypoints, args.output / 'waypoints.json')
    collector_text = collector_source.read_text(encoding='utf-8')
    old = 'if not 0 < duration <= 30:'
    if collector_text.count(old) != 1:
        parser.error('unexpected collector duration guard')
    collector_text = collector_text.replace(old, 'if not 0 < duration <= %d:' % args.collector_seconds)
    (scripts / 'collect_ros_long.py').write_text(collector_text, encoding='utf-8')
    manifest = {
        'mode': 'contact_waypoints',
        'seed': args.seed,
        'waypoints': str(args.waypoints),
        'waypoints_sha256': sha256(args.waypoints),
        'simulator': str(args.simulator),
        'settings': str(args.settings),
        'settings_sha256': sha256(args.settings),
        'binary_sha256': actual_binary_hash,
        'pool_source_sha256': sha256(args.pool),
        'image': args.image,
        'player_timeout_s': args.player_timeout,
        'collector_seconds': args.collector_seconds,
        'source_sha256': {
            'crash_capture.gdb': sha256(probe),
            'contact_waypoints.py': sha256(player_source),
            'collect_ros.py': sha256(collector_source),
            'collect_ros_long.py': sha256(scripts / 'collect_ros_long.py'),
        },
        'capture_state': 'started; result not yet received',
        'exit_status': None,
        'limitations': [
            'velocity control with debug pose truth; diagnostic only, not a policy result',
            'GDB all-stop contact breakpoint changes timing and may change collision response',
            'container is stopped immediately after the bounded pilot',
        ],
    }
    pool_script = scripts / 'headless_pool.py'
    manifest['pool_sha256'] = sha256(pool_script)
    manifest_path = args.output / 'run.json'
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    container = None
    collector = None

    def save():
        manifest_path.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')

    def stop_pool():
        pool_manifest = args.output / 'pool' / 'pool.json'
        if pool_manifest.is_file():
            subprocess.run([sys.executable, str(pool_script), 'stop', str(pool_manifest)],
                           text=True, capture_output=True, check=False)

    try:
        started = run([sys.executable, str(pool_script), 'start', '--simulator', str(args.simulator),
                       '--settings', str(args.settings), '--seeds', str(args.seed),
                       '--port', str(args.port), '--output', str(args.output / 'pool'),
                       '--timeout', '90', '--gdb-crash', '--image', args.image],
                      capture_output=True)
        (args.output / 'start.log').write_text(started.stdout + started.stderr, encoding='utf-8')
        pool_data = json.loads((args.output / 'pool' / 'pool.json').read_text(encoding='utf-8'))
        if len(pool_data['instances']) != 1:
            raise RuntimeError('expected one simulator instance')
        container = pool_data['instances'][0]['container']
        manifest['container'] = container
        manifest['container_id'] = pool_data['instances'][0]['id']
        save()
        for source, destination in ((player_source, '/tmp/contact_waypoints.py'),
                                    (args.waypoints, '/tmp/waypoints.json'),
                                    (scripts / 'collect_ros_long.py', '/tmp/collect_ros_long.py')):
            run(['docker', 'cp', str(source), container + ':' + destination], capture_output=True)
        collector_log = (args.output / 'collector.log').open('w', encoding='utf-8')
        collector = subprocess.Popen(
            ['docker', 'exec', container, 'bash', '-lc',
             'source /opt/ros/noetic/setup.bash; python3 /tmp/collect_ros_long.py /tmp/capture.jsonl %d'
             % args.collector_seconds],
            stdout=collector_log, stderr=subprocess.STDOUT, text=True)
        time.sleep(0.5)
        player = subprocess.run(
            ['docker', 'exec', container, 'bash', '-lc',
             'source /opt/ros/noetic/setup.bash; timeout --signal=TERM --kill-after=5s %ds '
             'python3 /tmp/contact_waypoints.py /tmp/waypoints.json /tmp/commands.jsonl'
             % args.player_timeout],
            text=True, capture_output=True, timeout=args.player_timeout + 20)
        (args.output / 'player.log').write_text(player.stdout + player.stderr, encoding='utf-8')
        manifest['player_exit_status'] = player.returncode
        save()
        stop = run(['docker', 'stop', '--time', '1', container], capture_output=True)
        (args.output / 'stop.log').write_text(stop.stdout + stop.stderr, encoding='utf-8')
        manifest['container_stop_status'] = stop.returncode
        save()
        try:
            collector.wait(timeout=15)
        except subprocess.TimeoutExpired:
            collector.kill()
        collector_log.close()
        manifest['collector_exit_status'] = collector.returncode
        for name in ('capture.jsonl', 'commands.jsonl'):
            result = subprocess.run(['docker', 'cp', container + ':/tmp/' + name,
                                     str(args.output / name)], text=True, capture_output=True)
            if result.returncode not in (0, 1):
                raise RuntimeError('docker cp failed for ' + name + ': ' + result.stderr)
        manifest['capture_state'] = 'player and collector finished; files copied'
        save()
        status = player.returncode
        manifest['exit_status'] = status
        save()
        for name in ('capture.jsonl', 'commands.jsonl'):
            path = args.output / name
            if not path.is_file() or path.stat().st_size == 0:
                manifest.setdefault('missing_required_files', []).append(name)
        if manifest.get('missing_required_files'):
            save()
            raise RuntimeError('required files missing: ' + ', '.join(manifest['missing_required_files']))
        print('exit_status:', status)
    except BaseException as exc:
        manifest['error'] = repr(exc)
        manifest['capture_state'] = 'failed; cleanup attempted'
        save()
        if container:
            subprocess.run(['docker', 'stop', '--time', '1', container],
                           text=True, capture_output=True, check=False)
        raise
    finally:
        if collector is not None and collector.poll() is None:
            collector.kill()
        if container:
            for name in ('capture.jsonl', 'commands.jsonl'):
                destination = args.output / name
                if not destination.is_file():
                    subprocess.run(['docker', 'cp', container + ':/tmp/' + name, str(destination)],
                                   text=True, capture_output=True, check=False)
        stop_pool()
        save()


if __name__ == '__main__':
    main()
