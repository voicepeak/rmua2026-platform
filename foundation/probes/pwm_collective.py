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
    if mode in ('field_toggle', 'field_trace'):
        return ((1.7, common(.12)), (.2, common(0.)))
    if mode == 'field_toggle_high':
        return ((1.7, common(.15)), (.2, common(0.)))
    if mode == 'field_toggle_air':
        return ((1.7, common(.20)), (.2, common(0.)))
    if mode == 'field_recovery':
        return ((1.35, common(.20)), (.2, common(0.)))
    if mode in ('collective', 'wind_stage4', 'wind_stage4_late', 'aero_probe', 'aero_force_probe', 'aero_wind0', 'aero_wind3', 'frame_probe', 'contact_probe', 'field_forced0', 'field_forced1', 'cadence20', 'cadence50', 'cadence100'):
        return ((.5, common(.12)), (.6, common(.20)), (.5, common(.16)), (.3, common(0.)))
    if mode == 'differential':
        pulses = []
        for i in range(4):
            values = common(.185)
            values[i] = .189
            pulses.append((.13, values))
        return ((.25, common(.20)), *pulses, (.25, common(.16)), (.2, common(0.)))
    if mode in ('yaw', 'kinematic_yaw'):
        return ((.25, common(.20)), (.20, [.188, .188, .182, .182]),
                (.20, [.182, .182, .188, .188]), (.25, common(.16)), (.2, common(0.)))
    if mode == 'yaw_small':
        return ((.25, common(.20)), (.20, [.187, .187, .183, .183]),
                (.20, [.183, .183, .187, .187]), (.25, common(.16)), (.2, common(0.)))
    if mode in ('roll_pitch', 'frame_lateral', 'aero_wind3_tilt'):
        return ((.25, common(.20)), (.14, [.182, .188, .188, .182]),
                (.14, [.188, .182, .182, .188]), (.14, [.188, .182, .188, .182]),
                (.14, [.182, .188, .182, .188]), (.25, common(.16)), (.2, common(0.)))
    if mode == 'roll_pitch_small':
        return ((.25, common(.20)), (.14, [.183, .187, .187, .183]),
                (.14, [.187, .183, .183, .187]), (.14, [.187, .183, .187, .183]),
                (.14, [.183, .187, .183, .187]), (.25, common(.16)), (.2, common(0.)))
    if mode == 'braking':
        # Opposing pitch pulses initiate translation and then arrest rotation;
        # a near-hover interval precedes the reduced-thrust interval.
        return ((.25, common(.20)), (.14, [.187, .183, .187, .183]),
                (.14, [.183, .187, .183, .187]), (.25, common(.18)),
                (.20, common(.16)), (.2, common(0.)))
    if mode == 'heldout_mix':
        # Predeclared combined-axis waveform, distinct from the identification
        # pulses. Ground-start safety guards and bounded shutdown still apply.
        return ((.25, common(.20)), (.12, [.186, .180, .184, .170]),
                (.12, [.176, .184, .174, .186]), (.12, [.180, .178, .188, .174]),
                (.12, [.177, .185, .183, .175]), (.20, common(.18)),
                (.18, common(.16)), (.20, common(0.)))
    if mode in ('heldout_mix_small', 'kinematic_probe'):
        mixed = phases('heldout_mix')
        return tuple((duration, [.18 + (x-.18)/3 for x in values]) if 1 <= i <= 4
                     else (duration, values) for i, (duration, values) in enumerate(mixed))
    if mode in ('small_steps', 'rotor_probe', 'kinematic_steps'):
        # Lift off first, then examine both directions of small collective
        # steps and returns while keeping the usual guard and shutdown path.
        return ((.22, common(.20)), (.25, common(.18)), (.25, common(.19)),
                (.25, common(.18)), (.25, common(.17)), (.20, common(.18)),
                (.2, common(0.)))
    raise ValueError('unknown waveform')


def main():
    mode = sys.argv[2] if len(sys.argv) > 2 else 'collective'
    waveform = phases(mode)
    rate_hz = {'cadence20': 20, 'cadence50': 50, 'cadence100': 100, 'small_steps': 100, 'rotor_probe': 100, 'kinematic_steps': 100}.get(mode, 50)
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
                  max_tilt_rad=.5, stale_pose_s=.3, requested_rate_hz=rate_hz)
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
                time.sleep(1.0 / rate_hz)
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
