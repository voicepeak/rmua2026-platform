"""Run the official contact-waypoint probe on the remote host via SSH.

The remote-local runner is uploaded into a private /tmp directory, executed,
its artifacts are downloaded, and the temporary directory is removed. This is
a diagnostic contact probe, not a policy or race result.
"""
import argparse
import hashlib
import json
import posixpath
import tarfile
import time
import uuid
from pathlib import Path

from foundation.remote_preflight import connect


SOURCES = (
    'foundation/run_contact_waypoints_local.py',
    'foundation/probes/contact_readonly.gdb',
    'foundation/probes/contact_waypoints.py',
    'foundation/probes/collect_ros.py',
)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def upload_dir(sftp, local, remote):
    sftp.mkdir(remote, mode=0o700)
    for entry in sorted(Path(local).iterdir()):
        destination = posixpath.join(remote, entry.name)
        if entry.is_dir():
            upload_dir(sftp, entry, destination)
        else:
            sftp.put(str(entry), destination)


def download_dir(sftp, remote, local):
    Path(local).mkdir(parents=True, exist_ok=True)
    for entry in sftp.listdir_attr(remote):
        source = posixpath.join(remote, entry.filename)
        destination = Path(local) / entry.filename
        if entry.st_mode & 0o040000:
            download_dir(sftp, source, destination)
        else:
            sftp.get(source, str(destination))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('connection_script', type=Path)
    parser.add_argument('config', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--seed', type=int, default=123)
    parser.add_argument('--port', type=int, default=11841)
    parser.add_argument('--gdb', type=Path, default=Path('foundation/probes/contact_readonly.gdb'))
    parser.add_argument('--player-timeout', type=int, default=90)
    parser.add_argument('--collector-seconds', type=int, default=60)
    parser.add_argument('--timeout', type=int, default=240)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output directory already exists')
    sources = list(SOURCES)
    if str(args.gdb) not in sources:
        sources.append(str(args.gdb))
    if not all(Path(source).is_file() for source in sources):
        parser.error('missing local probe source')
    remote = '/tmp/foundation-contact-' + uuid.uuid4().hex
    client = None
    last_error = None
    for attempt in range(4):
        try:
            client = connect(args.connection_script)
            break
        except Exception as exc:
            last_error = exc
            if attempt == 3:
                raise
            time.sleep(5 * (attempt + 1))
    if client is None:
        raise last_error
    try:
        sftp = client.open_sftp()
        sftp.mkdir(remote, mode=0o700)
        for source in sources:
            local = Path(source)
            try:
                relative = local.relative_to('foundation').as_posix()
            except ValueError:
                relative = local.name
            destination = posixpath.join(remote, relative)
            if not destination.startswith(remote + '/'):
                raise ValueError('invalid destination')
            parent = posixpath.dirname(destination)
            try:
                sftp.stat(parent)
            except FileNotFoundError:
                sftp.mkdir(parent, mode=0o700)
            sftp.put(str(local), destination)
        try:
            gdb_relative = args.gdb.relative_to('foundation').as_posix()
        except ValueError:
            gdb_relative = args.gdb.name
        gdb_remote = posixpath.join(remote, gdb_relative)
        config_remote = posixpath.join(remote, 'waypoints.json')
        sftp.put(str(args.config), config_remote)
        output_remote = posixpath.join(remote, 'output')
        command = (
            'timeout --signal=TERM --kill-after=10s %ds python3 %s %s %s '
            '--seed %d --port %d --gdb %s --player-timeout %d --collector-seconds %d' % (
                args.timeout, posixpath.join(remote, 'run_contact_waypoints_local.py'),
                config_remote, output_remote, args.seed, args.port, gdb_remote,
                args.player_timeout, args.collector_seconds))
        _, stdout, stderr = client.exec_command(command, timeout=args.timeout + 30)
        out = stdout.read().decode('utf-8', 'replace')
        err = stderr.read().decode('utf-8', 'replace')
        status = stdout.channel.recv_exit_status()
        args.output.mkdir(parents=True)
        (args.output / 'remote_execution.json').write_text(json.dumps({
            'remote_directory': remote,
            'command': command,
            'exit_status': status,
            'stdout': out,
            'stderr': err,
            'source_sha256': {source: digest(source) for source in sources},
            'config_sha256': digest(args.config),
        }, indent=2) + '\n', encoding='utf-8')
        archive_remote = posixpath.join(remote, 'output.tar.gz')
        archive_local = args.output / 'run.tar.gz'
        made_archive = False
        try:
            _, tar_out, tar_err = client.exec_command(
                'tar -czf %s -C %s output' % (archive_remote, remote), timeout=60)
            tar_out.channel.recv_exit_status()
            sftp.get(archive_remote, str(archive_local))
            made_archive = archive_local.stat().st_size > 0
        except Exception as exc:
            (args.output / 'archive_error.txt').write_text(repr(exc) + chr(10), encoding='utf-8')
        if made_archive:
            tmp = args.output / 'tar_extract'
            tmp.mkdir()
            with tarfile.open(archive_local, 'r:gz') as archive:
                archive.extractall(tmp, filter='data')
            destination = args.output / 'run'
            if destination.exists():
                destination.rename(args.output / 'run_perfile')
            (tmp / 'output').rename(destination)
            tmp.rmdir()
            (args.output / 'artifact_mode.txt').write_text('tar' + chr(10), encoding='utf-8')
        else:
            download_dir(sftp, output_remote, args.output / 'run')
            (args.output / 'artifact_mode.txt').write_text('per-file' + chr(10), encoding='utf-8')
        sftp.close()
        print('exit_status:', status)
        print(out, err)
    finally:
        try:
            _, stdout, _ = client.exec_command('rm -rf ' + remote, timeout=15)
            stdout.channel.recv_exit_status()
        except Exception:
            pass
        client.close()


if __name__ == '__main__':
    main()
