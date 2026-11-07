#!/usr/bin/env python3
"""Bounded, debug-pose-guarded PWM step for an isolated simulator only."""
import json
import math
import sys
import threading
import time

import genpy.dynamic
import rospy
from geometry_msgs.msg import PoseStamped

DEFINITION = ('std_msgs/Header header\nfloat64 rotorPWM0\nfloat64 rotorPWM1\n'
              'float64 rotorPWM2\nfloat64 rotorPWM3\n' + '=' * 80 +
              '\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n')
def phases(mode):
    common = lambda value: [value] * 4
    if mode == 'collective':
        return ((.5, common(.12)), (.6, common(.20)), (.5, common(.16)), (.3, common(0.)))
    if mode == 'differential':
        pulses = []
        for i in range(4):
            values = common(.20)
            values[i] = .206
            pulses.append((.15, values))
        return ((.4, common(.20)), *pulses, (.4, common(.16)), (.3, common(0.)))
    raise ValueError('unknown waveform')


def main():
    waveform = phases(sys.argv[2] if len(sys.argv) > 2 else 'collective')
    pwm_type = genpy.dynamic.generate_dynamic('airsim_ros/RotorPWM', DEFINITION)['airsim_ros/RotorPWM']
    if pwm_type._md5sum != '4967432b608c06d70b17be20124fa258':
        raise RuntimeError('message MD5 mismatch')
    rospy.init_node('foundation_pwm_probe', anonymous=True, disable_signals=True)
    lock = threading.Lock()
    latest = [None, 0.]

    def pose(msg):
        p, q = msg.pose.position, msg.pose.orientation
        with lock:
            latest[0] = (p.x, p.y, p.z, q.w, q.x, q.y, q.z)
            latest[1] = time.monotonic()

    sub = rospy.Subscriber('/airsim_node/drone_1/debug/pose_gt', PoseStamped, pose, queue_size=1)
    pub = rospy.Publisher('/airsim_node/drone_1/rotor_pwm_cmd', pwm_type, queue_size=1)
    deadline = time.monotonic() + 8
    with open(sys.argv[1], 'w', buffering=1) as out:
        def event(kind, **data):
            out.write(json.dumps({'kind': kind, 'monotonic_ns': time.monotonic_ns(),
                                  'wall_ns': time.time_ns(), **data}, separators=(',', ':')) + '\n')

        def command(values):
            msg = pwm_type()
            msg.header.stamp = rospy.Time.now()
            msg.rotorPWM0, msg.rotorPWM1, msg.rotorPWM2, msg.rotorPWM3 = values
            pub.publish(msg)
            event('command', pwm=list(values), stamp_ns=msg.header.stamp.to_nsec())

        status = 'unknown'
        try:
            while time.monotonic() < deadline:
                with lock:
                    initial, arrived = latest[:]
                if initial is not None and time.monotonic() - arrived < .3 and pub.get_num_connections() > 0:
                    break
                time.sleep(.02)
            else:
                raise RuntimeError('pose or command subscriber unavailable')
            if not all(math.isfinite(x) for x in initial):
                raise RuntimeError('non-finite initial pose')
            event('start', pose=list(initial), phases=list(waveform), max_displacement_m=.8,
                  max_tilt_rad=.5, stale_pose_s=.3)
            start = time.monotonic()
            end = sum(x[0] for x in waveform)
            while not rospy.is_shutdown():
                now = time.monotonic()
                elapsed = now - start
                if elapsed >= end:
                    break
                with lock:
                    current, arrived = latest[:]
                if current is None or now - arrived > .3:
                    raise RuntimeError('stale pose')
                if not all(math.isfinite(x) for x in current):
                    raise RuntimeError('non-finite pose')
                displacement = math.sqrt(sum((current[i]-initial[i])**2 for i in range(3)))
                w, x, y, z = current[3:]
                qnorm = math.sqrt(w*w+x*x+y*y+z*z)
                if qnorm < 1e-8:
                    raise RuntimeError('invalid orientation')
                tilt = math.acos(max(-1., min(1., 1 - 2*(x*x+y*y)/(qnorm*qnorm))))
                if displacement > .8 or tilt > .5:
                    raise RuntimeError('motion guard exceeded')
                boundary = 0.
                for duration, values in waveform:
                    boundary += duration
                    if elapsed < boundary:
                        break
                command(values)
                time.sleep(.02)
            if rospy.is_shutdown():
                raise RuntimeError('ROS shutdown before waveform completed')
            status = 'completed'
        except Exception as exc:
            status = 'aborted: ' + str(exc)
        finally:
            # Zero is only an attempted command, not a braking guarantee; the
            # enclosing harness stops the dedicated simulator container.
            for _ in range(3):
                command([0.]*4)
                time.sleep(.02)
            event('end', status=status)
            sub.unregister()
        if status != 'completed':
            raise SystemExit(2)


if __name__ == '__main__':
    main()
