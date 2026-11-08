"""Run a bounded independent official VEL segment trace in its own container."""
import argparse
import hashlib
import json
from pathlib import Path
import uuid

from foundation.remote_preflight import connect


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('connection_script', type=Path)
    p.add_argument('route', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--seconds', type=int, default=65)
    p.add_argument('--speed', type=float, default=4.)
    p.add_argument('--trace', choices=('segment', 'predicate'), default='segment')
    p.add_argument('--predicate-pose-probe', action='store_true')
    p.add_argument('--predicate-periodic', action='store_true')
    p.add_argument('--sphere-probe', action='store_true')
    p.add_argument('--seed', type=int, default=123)
    a = p.parse_args()
    if not 1 <= a.seconds <= 170 or not 0 < a.speed <= 6 or not 0 <= a.seed <= 2147483647:
        raise ValueError('invalid bounded duration or speed')
    if a.predicate_pose_probe and a.trace != 'predicate':
        raise ValueError('predicate pose probe requires predicate trace')
    if a.predicate_periodic and not a.predicate_pose_probe:
        raise ValueError('periodic predicate sampling requires pose probe')
    if a.sphere_probe and not a.predicate_pose_probe:
        raise ValueError('sphere probe requires predicate pose probe')
    files = [a.route/'route.json', a.route/'guides.json']
    if not all(f.is_file() for f in files):
        raise ValueError('route and guides required')
    a.output.mkdir(parents=True, exist_ok=False)
    remote = '/tmp/foundation-segment-'+uuid.uuid4().hex
    client = connect(a.connection_script)
    try:
        sftp = client.open_sftp()
        sftp.mkdir(remote, mode=0o700)
        sftp.mkdir(remote+'/route', mode=0o700)
        for source in files:
            sftp.put(str(source), remote+'/route/'+source.name)
        root = '/home/tianbot/桌面/RMUA2026-re/scripts/'
        if a.predicate_pose_probe or a.seed != 123:
            _, out, err = client.exec_command("cp -a '"+root.rstrip('/')+"' '"+remote+"/scripts'", timeout=30)
            code = out.channel.recv_exit_status()
            if code:
                raise RuntimeError('could not prepare isolated probe scripts: '+err.read().decode('utf-8', 'replace'))
            root = remote+'/scripts/'
        if a.seed != 123:
            with sftp.open(root+'run_beta_sensor_scout.py', 'r') as f:
                script = f.read().decode('utf-8')
            for old, new in (("'seed':123", "'seed':"+str(a.seed)),
                             ("'RMUA_SEED=123'", "'RMUA_SEED="+str(a.seed)+"'")):
                if script.count(old) != 1:
                    raise ValueError('unexpected seed configuration in runner')
                script = script.replace(old, new)
            with sftp.open(root+'run_beta_sensor_scout.py', 'w') as f:
                f.write(script)
        if a.predicate_pose_probe:
            probe = Path(__file__).parent/'probes/predicate_pose_probe.gdb'
            sftp.put(str(probe), root+probe.name)
            with sftp.open(root+'stage_trace_predicate.gdb', 'r') as f:
                gdb_script = f.read().decode('utf-8')
            anchor = 'source /analysis/scripts/charge_state_probe.py'
            if gdb_script.count(anchor) != 1:
                raise ValueError('unexpected predicate GDB script')
            gdb_script = gdb_script.replace(anchor, 'source /analysis/scripts/predicate_pose_probe.gdb\n'+anchor)
            with sftp.open(root+'stage_trace_predicate.gdb', 'w') as f:
                f.write(gdb_script)
            with sftp.open(root+'charge_state_probe.py', 'r') as f:
                predicate = f.read().decode('utf-8')
            anchor = "    for name,off,cnt,kind in [('array30'"
            injection = ("    try:\n"
                         "     drone=int(gdb.parse_and_eval('$foundation_drone_component'))\n"
                         "     actor=int(gdb.parse_and_eval('$foundation_drone_actor'))\n"
                         "     row['drone_component']=hex(drone);row['drone_actor']=hex(actor)\n"
                         "     if drone: row['drone_world_cm']=[f32(drone+0x1d0+i*4) for i in range(3)]\n"
                         "    except Exception as exc: row['drone_read_error']=repr(exc)\n")
            if a.sphere_probe:
                injection += ("    try:\n"
                              "     sphere=u64(actor+0x348) if actor else 0\n"
                              "     if sphere and u64(sphere+0x20)==actor:\n"
                              "      row['sphere_component']=hex(sphere)\n"
                              "      row['sphere_world_cm']=[f32(sphere+0x1d0+i*4) for i in range(3)]\n"
                              "      row['sphere_quat_xyzw']=[f32(sphere+0x1c0+i*4) for i in range(4)]\n"
                              "     else: row['sphere_read_error']='missing pointer or owner mismatch'\n"
                              "    except Exception as exc: row['sphere_read_error']=repr(exc)\n")
            if predicate.count(anchor) != 1:
                raise ValueError('unexpected predicate Python probe')
            predicate = predicate.replace(anchor, injection+anchor)
            if a.predicate_periodic:
                replacements = (("last=None\n", "last=None\nlast_sample=0.0\n"),
                                ("  global last\n", "  global last,last_sample\n"),
                                ("  if result!=last:\n", "  now=time.monotonic()\n  if result!=last or now-last_sample>=.2:\n"),
                                ("    try:\n     drone=int(gdb.parse_and_eval", "    row['is_transition']=(result!=last)\n    try:\n     drone=int(gdb.parse_and_eval"),
                                ("   last=result\n", "   last=result;last_sample=now\n"))
                for old, new in replacements:
                    if predicate.count(old) != 1:
                        raise ValueError('unexpected predicate periodic anchor')
                    predicate = predicate.replace(old, new)
            with sftp.open(root+'charge_state_probe.py', 'w') as f:
                f.write(predicate)
        runner = root+'run_beta_sensor_scout.py'
        command = ("timeout --signal=INT --kill-after=20s 270s python3 '"+runner+"' --output "+remote+
                   ' --seconds '+str(a.seconds)+' --speed '+str(a.speed)+' --'+a.trace+'-trace')
        _, stdout, stderr = client.exec_command(command, timeout=300)
        out, err = stdout.read().decode('utf-8', 'replace'), stderr.read().decode('utf-8', 'replace')
        status = stdout.channel.recv_exit_status()
        for name in ('manifest.json', 'gdb.log', 'pose_samples.jsonl', 'wind.jsonl', 'bridge.log', 'controller.log'):
            try:
                sftp.get(remote+'/'+name, str(a.output/name))
            except FileNotFoundError:
                pass
        for name in ('run_beta_sensor_scout.py', 'stage_trace_'+a.trace+'.gdb', 'charge_state_probe.py', 'analyze_segment_trace.py'):
            sftp.get(root+name, str(a.output/name))
        if a.predicate_pose_probe:
            sftp.get(root+'predicate_pose_probe.gdb', str(a.output/'predicate_pose_probe.gdb'))
        for source in files:
            (a.output/source.name).write_bytes(source.read_bytes())
        result = {'remote_directory': remote, 'ssh_exit_status': status,
                  'stdout': out, 'stderr': err, 'requested_seconds': a.seconds, 'requested_speed': a.speed, 'trace': a.trace, 'requested_seed': a.seed, 'predicate_periodic': a.predicate_periodic, 'sphere_probe': a.sphere_probe,
                  'route_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}
        (a.output/'execution.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
        print('ssh exit', status, 'local files', sorted(f.name for f in a.output.iterdir()))
        if status:
            raise SystemExit(status)
        manifest = json.loads((a.output/'manifest.json').read_bytes())
        if (manifest.get('status') != 'controller_exited' or manifest.get('seed') != a.seed or
                manifest.get('binary_sha256') != '5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800'):
            raise RuntimeError('diagnostic did not complete with the expected binary')
    finally:
        client.close()


if __name__ == '__main__':
    main()
