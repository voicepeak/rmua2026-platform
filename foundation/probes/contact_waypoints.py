#!/usr/bin/env python3
"""Bounded velocity-waypoint pilot for isolated official contact probes.

Uses debug/pose_gt for closed-loop diagnostics only.  Not a deployable
controller and not a race result.  Body-frame commands are derived from the
reported yaw and clipped to the explicit safety envelope.
"""
import json
import math
import subprocess
import sys
import time

import genpy.dynamic
import rospy
from geometry_msgs.msg import PoseStamped

VEL_DEFINITION = ('std_msgs/Header header\nfloat64 vx\nfloat64 vy\nfloat64 vz\n'
                  'float64 yawRate\nuint8 va\nuint8 stop\n' + '=' * 80 +
                  '\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n')
VelCmd = genpy.dynamic.generate_dynamic('airsim_ros/VelCmd', VEL_DEFINITION)['airsim_ros/VelCmd']
if VelCmd._md5sum != '1a888a503939ba521aed68feee8eab6d':
    raise RuntimeError('VelCmd MD5 mismatch')


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def body_velocity(world, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * world[0] + s * world[1],
            -s * world[0] + c * world[1],
            world[2])


def main():
    waypoint_path, log_path = sys.argv[1], sys.argv[2]
    with open(waypoint_path, encoding='utf-8') as stream:
        plan = json.load(stream)
    safety = plan['safety']
    waypoints = plan['waypoints']
    if not waypoints:
        raise ValueError('empty waypoint plan')
    max_speed = float(safety['max_speed'])
    max_duration = float(safety['max_duration_s'])
    max_xy = float(safety['max_xy_m'])
    z_min = float(safety['min_z_m'])
    z_max = float(safety['max_z_m'])
    stale_limit = float(safety['stale_pose_s'])
    gain = float(safety.get('position_gain', 0.8))
    log_period = float(safety.get('log_period_s', 0.1))

    rospy.init_node('foundation_contact_waypoints', anonymous=True, disable_signals=True)
    latest = [None, 0.0]

    def pose_cb(msg):
        latest[0] = msg
        latest[1] = time.monotonic()

    rospy.Subscriber('/airsim_node/drone_1/debug/pose_gt', PoseStamped, pose_cb, queue_size=1)
    publisher = rospy.Publisher('/airsim_node/drone_1/vel_body_cmd', VelCmd, queue_size=1)

    start = time.monotonic()
    while time.monotonic() - latest[1] > stale_limit or publisher.get_num_connections() == 0:
        time.sleep(0.05)
        if time.monotonic() - start > 10.0:
            raise RuntimeError('pose or command subscriber unavailable')
    if latest[0] is None:
        raise RuntimeError('no initial pose')

    def command(world_velocity_ned, target_name, phase):
        pose = latest[0].pose
        q = pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        body = body_velocity(world_velocity_ned, yaw)
        msg = VelCmd()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'drone_1'
        msg.vx, msg.vy, msg.vz = body[0], body[1], -body[2]
        msg.yawRate = 0.0
        msg.va = 8
        msg.stop = 0
        publisher.publish(msg)
        event('command', target=target_name, phase=phase,
              desired_world_velocity_ned=list(world_velocity_ned),
              body_velocity_ned=list(body), command_velocity=[msg.vx, msg.vy, msg.vz],
              stamp_ns=msg.header.stamp.to_nsec(),
              position=[pose.position.x, pose.position.y, pose.position.z],
              quaternion_xyzw=[q.x, q.y, q.z, q.w])

    with open(log_path, 'w', buffering=1) as stream:
        def event(kind, **data):
            stream.write(json.dumps({'kind': kind, 'monotonic_ns': time.monotonic_ns(),
                                     'wall_ns': time.time_ns(), **data},
                                    separators=(',', ':')) + '\n')

        p = latest[0].pose
        initial = [p.position.x, p.position.y, p.position.z]
        q = p.orientation
        event('start', pose=initial, quaternion_xyzw=[q.x, q.y, q.z, q.w],
              plan=plan, max_speed=max_speed, max_duration_s=max_duration)
        if plan.get('takeoff'):
            try:
                result = subprocess.run(
                    ['bash', '-lc',
                     'source /opt/ros/noetic/setup.bash; source /workspace/rmua_ws/devel/setup.bash 2>/dev/null; '
                     'rosservice call /airsim_node/drone_1/takeoff "waitOnLastTask: false"'],
                    capture_output=True, text=True, timeout=15)
                event('takeoff', returncode=result.returncode,
                      stdout=result.stdout[-800:], stderr=result.stderr[-800:])
                time.sleep(6.0)
            except Exception as exc:
                event('takeoff_error', error=str(exc))
        if plan.get('reset'):
            try:
                result = subprocess.run(
                    ['bash', '-lc',
                     'source /opt/ros/noetic/setup.bash; source /workspace/rmua_ws/devel/setup.bash 2>/dev/null; '
                     'rosservice call /airsim_node/reset'],
                    capture_output=True, text=True, timeout=20)
                event('reset', returncode=result.returncode,
                      stdout=result.stdout[-800:], stderr=result.stderr[-800:])
                time.sleep(3.0)
            except Exception as exc:
                event('reset_error', error=str(exc))
        status = 'aborted'
        last_log = 0.0
        start = time.monotonic()
        try:
            for index, waypoint in enumerate(waypoints):
                name = str(waypoint.get('name', index))
                target = [float(x) for x in waypoint['position']]
                speed = min(max_speed, float(waypoint.get('speed', max_speed)))
                radius = float(waypoint.get('arrive_radius_m', 0.4))
                hold = float(waypoint.get('hold_s', 0.0))
                push = float(waypoint.get('push_s', 0.0))
                push_speed = min(max_speed, float(waypoint.get('push_speed', speed)))
                max_approach = float(waypoint.get('max_approach_s', 0.0))
                waypoint_start = time.monotonic()
                arrived_at = None
                event('waypoint', index=index, name=name, target=target, speed=speed,
                      arrive_radius_m=radius, hold_s=hold, push_s=push,
                      push_speed=push_speed, phase='approach')
                while True:
                    now = time.monotonic()
                    if now - start > max_duration:
                        raise RuntimeError('maximum flight duration exceeded')
                    pose_msg, arrived = latest
                    if pose_msg is None or now - arrived > stale_limit:
                        raise RuntimeError('stale pose')
                    pose = pose_msg.pose
                    position = [pose.position.x, pose.position.y, pose.position.z]
                    if not all(math.isfinite(v) for v in position):
                        raise RuntimeError('non-finite pose')
                    if z_min > position[2] or position[2] > z_max:
                        raise RuntimeError('altitude envelope exceeded')
                    if math.hypot(position[0], position[1]) > max_xy:
                        raise RuntimeError('horizontal envelope exceeded')
                    q = pose.orientation
                    yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
                    error = [target[i] - position[i] for i in range(3)]
                    distance = math.sqrt(sum(v * v for v in error))
                    if arrived_at is None and distance <= radius:
                        arrived_at = now
                        event('arrived', index=index, name=name, position=position)
                    elif arrived_at is None and max_approach > 0 and now - waypoint_start >= max_approach:
                        arrived_at = now
                        event('approach_timeout', index=index, name=name, position=position,
                              distance=distance, max_approach_s=max_approach)
                    if arrived_at is None:
                        desired_world = [min(1.0, gain * distance) * error[i] / max(distance, 1e-9)
                                         * speed for i in range(3)]
                        phase = 'approach'
                    elif now - arrived_at < hold:
                        desired_world = [0.0, 0.0, 0.0]
                        phase = 'hold'
                    elif now - arrived_at < hold + push:
                        direction = error
                        norm = math.sqrt(sum(v * v for v in direction))
                        desired_world = [push_speed * v / max(norm, 1e-9) for v in direction]
                        phase = 'push'
                    else:
                        break
                    command(desired_world, name, phase)
                    if now - last_log >= log_period:
                        event('state', index=index, name=name, phase=phase,
                              position=position, distance=distance)
                        last_log = now
                    time.sleep(0.05)
            status = 'completed'
        except Exception as exc:
            status = 'aborted: ' + str(exc)
            raise
        finally:
            for _ in range(20):
                command((0.0, 0.0, 0.0), 'shutdown', 'zero')
                time.sleep(0.02)
            event('finish', status=status)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('contact_waypoints:', exc, file=sys.stderr)
        sys.exit(1)
