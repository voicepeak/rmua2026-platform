#!/usr/bin/env python3
"""Bounded diagnostic beta VEL scout using GPS+IMU bridge and static spline 1.

No debug truth subscribed as controller input. One fresh owned scene. Beta's
LiDAR/command deadline protections remain enabled; this is not race acceptance.
"""
import argparse,hashlib,json,os,secrets,subprocess,time
from pathlib import Path
from run_readonly_trace import LAUNCH,docker,sha
base=Path(__file__).resolve().parent.parent;workspace=Path('/home/tianbot/RUMA-by-helinjun');sim=workspace/'main/simulator/simulator_12.0.0.5';settings=workspace/'main/configs/simulator_settings.json'
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=int,default=35);p.add_argument('--speed',type=float,default=2.0);p.add_argument('--hub-observer',action='store_true');p.add_argument('--cause-trace',action='store_true');p.add_argument('--predicate-trace',action='store_true');p.add_argument('--inventory-trace',action='store_true');p.add_argument('--segment-trace',action='store_true');a=p.parse_args()
if not 1 <= a.seconds <= 180 or not 0 < a.speed <= 6:raise SystemExit('invalid duration or speed')
out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
if not (out/'route/route.json').is_file():raise SystemExit('output must contain pre-generated route/')
name='rmua-beta-'+secrets.token_hex(6);port=11761
meta={'seed':123,'mode':'VEL diagnostic','requested_duration_wall_s':a.seconds,'binary_sha256':sha(sim/'Build/LinuxNoEditor/RMUA/Binaries/Linux/RMUA-Linux-Shipping'),'settings_sha256':sha(settings),'bridge_sha256':sha(base/'scripts/gps_imu_pose_bridge.py'),'route_sha256':sha(out/'route/route.json'),'guides_sha256':sha(out/'route/guides.json'),'beta_controller_sha256':sha(workspace/'beta/rmua_ws/src/route_follower/scripts/route_follower.py'),'start_wall_unix_s':time.time(),'debug_truth_control':False,'debug_state_used_to_stop':False,'map':'default OL1','status':'starting'}
def save():(out/'manifest.json').write_text(json.dumps(meta,indent=2)+'\n')
save();bridge=None;wind=None;control=None;hub=None
try:
 args=['run','-d','--init','--name',name,'--network','host','--gpus','all','--cap-add','SYS_PTRACE','--security-opt','seccomp=unconfined','--user',f'{os.getuid()}:{os.getgid()}','-e','HOME=/tmp','-e','LD_LIBRARY_PATH=','-e','NVIDIA_DRIVER_CAPABILITIES=all','-e','DISPLAY=:1','-e','ROS_HOSTNAME=localhost','-e',f'ROS_MASTER_URI=http://127.0.0.1:{port}','-e',f'RMUA_PORT={port}','-e','RMUA_SEED=123','-v',str(sim)+':/sim:ro','-v',str(settings)+':/configs/settings.json:ro','-v',str(workspace)+':/workspace:ro','-v',str(base)+':/analysis:ro','-v',str(out)+':/out','-v','/tmp/.X11-unix:/tmp/.X11-unix','--entrypoint','bash','rmua/re-probe:20261104','-c',LAUNCH.replace('runtime_trace.gdb','stage_trace_segment.gdb' if a.segment_trace else ('stage_trace_inventory.gdb' if a.inventory_trace else ('stage_trace_predicate.gdb' if a.predicate_trace else ('stage_trace_cause.gdb' if a.cause_trace else 'stage_trace.gdb'))))]
 meta['container_id']=docker(*args,capture_output=True,check=True).stdout.strip();save()
 prefix='source /opt/ros/noetic/setup.bash; source /workspace/rmua_ws/devel/setup.bash; export OPENBLAS_NUM_THREADS=1; export ROS_HOME=/tmp/probe; '
 def cmd(s):return ['docker','exec','-u',f'{os.getuid()}:{os.getgid()}',name,'bash','-c',prefix+s]
 deadline=time.monotonic()+90
 while time.monotonic()<deadline:
  if docker('inspect',name,'--format','{{.State.Running}}',capture_output=True).stdout.strip()!='true':raise RuntimeError('simulator exited before ready')
  if subprocess.run(cmd('timeout 2 rostopic echo -n 1 /airsim_node/drone_1/gps >/dev/null'),stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0:break
  time.sleep(.5)
 else:raise TimeoutError('GPS not ready')
 meta['ready_wall_unix_s']=time.time();save()
 with (out/'bridge.log').open('w') as b,(out/'wind.log').open('w') as w,(out/'controller.log').open('w') as c:
  bridge=subprocess.Popen(cmd('exec python3 /analysis/scripts/gps_imu_pose_bridge.py /out/pose_samples.jsonl '+str(a.seconds+10)),stdout=b,stderr=subprocess.STDOUT)
  wind=subprocess.Popen(cmd('exec python3 /analysis/scripts/stage_wind_recorder.py /out/wind.jsonl'),stdout=w,stderr=subprocess.STDOUT)
  if a.hub_observer:
   hub=subprocess.Popen(cmd('exec python3 /analysis/scripts/hub_lidar_probe.py /out/hub'),stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
  time.sleep(2)
  if bridge.poll() is not None:raise RuntimeError('pose bridge exited before controller')
  # ROS remapping replaces the beta node's only debug pose subscription; the
  # bridge consumes only GPS and IMU. No truth publisher participates in control.
  control_cmd=('timeout --signal=INT --kill-after=5 '+str(a.seconds)+' rosrun route_follower route_follower.py '
   '/airsim_node/drone_1/debug/pose_gt:=/rmua/diagnostic/pose '
   '_route_file:=/out/route/route.json _route_name:=static_spline1 _guides_file:=/out/route/guides.json '
   '_gates_file:= _gate_z_uses_offset:=false _use_gate_map:=false _wait_for_first_map:=false _course_frame_calibration:=false '
   '_adaptive_speed:=true _lidar_fresh_publication_check:=true _lidar_stereo_obstacles:=false _cruise_speed:='+str(a.speed)+' _max_speed:='+str(a.speed)+' _normal_speed_floor:=1.0')
  meta['control_command']=control_cmd;meta['control_start_wall_unix_s']=time.time();save()
  control=subprocess.Popen(cmd(control_cmd),stdout=c,stderr=subprocess.STDOUT)
  try:
   meta['controller_exit_code']=control.wait(timeout=a.seconds+10);meta['status']='controller_exited'
  except subprocess.TimeoutExpired:control.terminate();meta['controller_exit_code']=control.wait(timeout=8);meta['status']='controller_outer_timeout'
except Exception as exc:meta.update(status='failed',error=repr(exc))
finally:
 for p in (control,bridge,wind,hub):
  if p and p.poll() is None:p.terminate()
 logs=docker('logs',name,capture_output=True);(out/'gdb.log').write_text(logs.stdout+logs.stderr)
 info=docker('inspect',name,'--format','{{.Id}}',capture_output=True)
 if info.returncode==0 and info.stdout.strip()==meta.get('container_id'):docker('rm','-f',name,capture_output=True)
 for p in (control,bridge,wind,hub):
  if p and p.poll() is None:
   try:p.wait(timeout=3)
   except subprocess.TimeoutExpired:p.kill();p.wait()
 meta['end_wall_unix_s']=time.time();save();print(json.dumps(meta,indent=2))
