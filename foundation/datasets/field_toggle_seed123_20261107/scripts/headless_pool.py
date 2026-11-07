#!/usr/bin/env python3
"""Launch isolated official NullRHI scenes for diagnostics, not a lockstep RL API."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import os
from pathlib import Path
import secrets
import socket
import subprocess
import sys
import time

LABEL = 'rmua-re.headless-pool'
IMAGE = 'rmua/noetic-xal:driver-recovery-20261012'

LAUNCH = r'''set -e
source /opt/ros/noetic/setup.bash
export ROS_HOME=/tmp/ros
mkdir -p "$ROS_HOME"
roscore -p "$RMUA_PORT" > /tmp/roscore.log 2>&1 &
ros_pid=$!
trap 'kill "$ros_pid" 2>/dev/null || true' EXIT
for i in {1..120}; do
  if rosparam list >/dev/null 2>&1; then break; fi
  if ! kill -0 "$ros_pid" 2>/dev/null; then echo 'roscore failed' >&2; exit 1; fi
  sleep .25
done
rosparam list >/dev/null 2>&1 || { echo 'roscore not ready' >&2; exit 1; }
cd /sim/Build/LinuxNoEditor
./RMUA/Binaries/Linux/RMUA-Linux-Shipping RMUA seed "$RMUA_SEED" -nullrhi -settings=/configs/settings.json &
sim_pid=$!
trap 'kill "$sim_pid" "$ros_pid" 2>/dev/null || true' EXIT TERM INT
wait "$sim_pid"
'''

# The debugger is a parent of the unmodified executable. Its timing differs
# from the normal pool and its process exit code is not the simulator's status.
GDB_LAUNCH = LAUNCH.replace(
    './RMUA/Binaries/Linux/RMUA-Linux-Shipping RMUA seed',
    'gdb -nx -batch -x /analysis/scripts/crash_capture.gdb --args ./RMUA/Binaries/Linux/RMUA-Linux-Shipping RMUA seed')
FREQUENCY_GUARD_LAUNCH = LAUNCH.replace(
    './RMUA/Binaries/Linux/RMUA-Linux-Shipping RMUA seed',
    'LD_PRELOAD=/probe/frequency_guard.so RMUA_FREQ_GUARD_LOG=/out/frequency-${RMUA_PORT}.jsonl ./RMUA/Binaries/Linux/RMUA-Linux-Shipping RMUA seed')

def call(args, **kwargs):
    return subprocess.run(args, text=True, check=True, **kwargs)

def free_port(port):
    with socket.socket() as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('127.0.0.1', port))

def sha256(path):
    digest=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''): digest.update(chunk)
    return digest.hexdigest()

def inspect(name):
    p = subprocess.run(['docker','inspect',name], text=True, capture_output=True)
    return json.loads(p.stdout)[0] if p.returncode == 0 else None

def stop_pool(manifest_path):
    data = json.loads(manifest_path.read_text())
    if data.get('schema') != 'rmua-headless-pool-v1': raise ValueError('invalid manifest')
    for item in data['instances']:
        name=item['container']
        info=inspect(name)
        if not info: continue
        labels=info['Config'].get('Labels') or {}
        if labels.get(LABEL) != data['pool_id'] or info['Id'] != item['id']:
            raise RuntimeError('container identity changed; refusing to stop '+name)
        log=manifest_path.parent/(name+'.log')
        with log.open('wb') as f:
            subprocess.run(['docker','logs',name],stdout=f,stderr=subprocess.STDOUT,check=False)
        call(['docker','rm','-f',name],stdout=subprocess.DEVNULL)
        print('stopped',name,flush=True)

def start(args):
    sim=args.simulator.resolve(strict=True)
    settings=args.settings.resolve(strict=True)
    binary=sim/'Build/LinuxNoEditor/RMUA/Binaries/Linux/RMUA-Linux-Shipping'
    if not binary.is_file(): raise ValueError('invalid simulator directory')
    guard = args.frequency_guard.resolve(strict=True) if args.frequency_guard else None
    if guard and args.gdb_crash: raise ValueError('frequency guard and GDB crash mode cannot be combined')
    if guard:
        library=sim/'Build/LinuxNoEditor/RMUA/Plugins/AirSimROS/Binaries/Linux/libairsimros.so'
        if sha256(library) != '69d5ea3302bdd5f20b842ec3d3e84bafcb50ef3649ada42c4d346c9bcd389885':
            raise ValueError('frequency guard requires validated AirSim ROS library')
    seeds=[int(x) for x in args.seeds.split(',')]
    if not seeds or any(x<0 for x in seeds): raise ValueError('seeds must be nonnegative')
    if not (1 <= args.port <= 65535-len(seeds)+1): raise ValueError('invalid port range')
    if not 1 <= args.timeout <= 600: raise ValueError('timeout must be between 1 and 600 seconds')
    for i in range(len(seeds)): free_port(args.port+i)
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    pool_id=secrets.token_hex(6)
    manifest=output/'pool.json'
    image_id=call(['docker','image','inspect',args.image,'--format','{{.Id}}'],capture_output=True).stdout.strip()
    data={'schema':'rmua-headless-pool-v1','pool_id':pool_id,'simulator':str(sim),
          'binary_sha256':sha256(binary),'settings':str(settings),'settings_sha256':sha256(settings),
          'image':args.image,'image_id':image_id,'mode':'nullrhi+gdb' if args.gdb_crash else 'nullrhi','instances':[],'ready':False,
          'limitations':['No lockstep or reset guarantee','Camera images are empty in NullRHI',
                         'Diagnostics only; dynamic equivalence not established']}
    if args.gdb_crash:
        data['gdb_script_sha256']=sha256(Path(__file__).resolve().parent/'crash_capture.gdb')
        data['limitations'].append('GDB changes timing; GDB exit status is not the inferior exit status')
    if guard:
        data['frequency_guard_sha256']=sha256(guard)
        data['limitations'].append('frequency guard changes scheduling and covers only three exported functions')
    def save(): manifest.write_text(json.dumps(data,indent=2)+'\n')
    save()
    try:
        for i,seed in enumerate(seeds):
            name='rmua-re-'+pool_id+'-'+str(i)
            port=args.port+i
            cmd=['docker','run','-d','--init','--name',name,'--network','host','--gpus','all',
                 '--label',LABEL+'='+pool_id,'--user',str(os.getuid())+':'+str(os.getgid()),
                 '-e','NVIDIA_DRIVER_CAPABILITIES=all','-e','LD_LIBRARY_PATH=','-e','HOME=/tmp',
                 '-e','DISPLAY='+args.display,'-e','ROS_HOSTNAME=localhost',
                 '-e','ROS_MASTER_URI=http://127.0.0.1:'+str(port),
                 '-e','RMUA_PORT='+str(port),'-e','RMUA_SEED='+str(seed),
                 '-v',str(sim)+':/sim:ro','-v',str(settings)+':/configs/settings.json:ro',
                 '-v','/tmp/.X11-unix:/tmp/.X11-unix']
            if args.gdb_crash:
                cmd.extend(['--cap-add','SYS_PTRACE','--security-opt','seccomp=unconfined',
                            '-v',str(Path(__file__).resolve().parent.parent)+':/analysis:ro'])
            if guard:
                cmd.extend(['-v',str(guard)+':/probe/frequency_guard.so:ro','-v',str(output)+':/out'])
            launch = GDB_LAUNCH if args.gdb_crash else (FREQUENCY_GUARD_LAUNCH if guard else LAUNCH)
            cmd.extend(['--entrypoint','bash',args.image,'-c',launch])
            before=time.time()
            container_id=call(cmd,capture_output=True).stdout.strip()
            data['instances'].append({'seed':seed,'port':port,'container':name,'id':container_id,
                                      'docker_start_wall':before,'docker_created_wall':time.time()})
            save()
        deadline=time.monotonic()+args.timeout
        def wait_ready(item):
            name=item['container']
            while time.monotonic()<deadline:
                info=inspect(name)
                if not info or not info['State']['Running']: raise RuntimeError(name+' exited before readiness')
                cmd=['docker','exec','-e','ROS_HOME=/tmp/ros-health',name,'bash','-c',
                     'source /opt/ros/noetic/setup.bash; timeout 4 rostopic echo -n 1 /airsim_node/drone_1/imu/imu >/dev/null']
                if subprocess.run(cmd,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0:
                    return time.time()
                time.sleep(.5)
            raise TimeoutError('topic readiness timed out: '+name)
        with ThreadPoolExecutor(max_workers=len(data['instances'])) as executor:
            jobs={executor.submit(wait_ready,item):item for item in data['instances']}
            for job in as_completed(jobs):
                item=jobs[job]
                item['imu_ready_observed_wall']=job.result()
                item['startup_observed_s']=item['imu_ready_observed_wall']-item['docker_start_wall']
                save()
        data['ready']=True
        save()
        print(json.dumps({'manifest':str(manifest),'instances':data['instances']},indent=2),flush=True)
    except BaseException:
        stop_pool(manifest)
        raise

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('start')
    p.add_argument('--simulator',type=Path,required=True)
    p.add_argument('--settings',type=Path,required=True)
    p.add_argument('--seeds',required=True,help='comma-separated nonnegative seeds')
    p.add_argument('--port',type=int,default=11411)
    p.add_argument('--output',type=Path,required=True,help='new directory for manifest and logs')
    p.add_argument('--display',default=':1')
    p.add_argument('--image',default=IMAGE)
    p.add_argument('--timeout',type=float,default=75.)
    p.add_argument('--gdb-crash',action='store_true',help='debug build image required; stop and log SIGSEGV with GDB')
    p.add_argument('--frequency-guard',type=Path,help='development-only frequency-entry serialization interposer')
    p=sub.add_parser('stop')
    p.add_argument('manifest',type=Path)
    args=parser.parse_args()
    if args.command=='start': start(args)
    else: stop_pool(args.manifest.resolve(strict=True))

if __name__=='__main__':
    try: main()
    except (ValueError, OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print('headless-pool:',exc,file=sys.stderr)
        sys.exit(1)
