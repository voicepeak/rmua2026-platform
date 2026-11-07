"""Diagnostic-only ROS bridge for the audited stage-1 state policy.

Uses debug/pose_gt and inferred gate state. Not a deployable actor or race
score. The caller must export actor tensors to NPZ and run one fresh simulator.
"""
import argparse
import json
import math
import time

import numpy as np


def actor_forward(obs, weights):
    x = np.asarray(obs, dtype=np.float32) / weights['obs_scale']
    for layer in (0, 2, 4):
        x = x @ weights['actor.%d.weight' % layer].T + weights['actor.%d.bias' % layer]
        if layer != 4:
            x = np.tanh(x)
    return x


def world_up_to_body_command(velocity, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c*velocity[0]+s*velocity[1],
                     -s*velocity[0]+c*velocity[1], velocity[2]])


def read_official_state(path, now, max_age=2.0):
    """Fail closed if the external read-only state monitor is stale or absent."""
    try:
        with open(path, encoding='utf-8') as f:
            state = json.load(f)
        age = now - float(state['monotonic'])
        if age < 0 or age > max_age:
            return None
        return state
    except (OSError, ValueError, KeyError, TypeError):
        return None


def departure_command(pos, elapsed):
    """Conservative development-only takeoff and departure in z-up world axes."""
    forward = min(2.0, max(0., elapsed) * .4) if pos[2] >= 1.2 else 0.
    climb = np.clip(.8 * (1.8 - pos[2]), -.6, .8)
    return np.array([forward, 0., climb])


def limited_planar_velocity(desired, previous, dt, max_speed=4., max_accel=1.):
    desired = np.asarray(desired, dtype=float)
    previous = np.asarray(previous, dtype=float)
    desired = desired * min(1., max_speed / max(np.linalg.norm(desired), 1e-12))
    delta = desired - previous
    delta *= min(1., max_accel * max(0., dt) / max(np.linalg.norm(delta), 1e-12))
    return previous + delta


def guarded_vertical_velocity(desired, altitude):
    """Limit policy climb/descent and keep a conservative 1.4–3 m band."""
    low = max(-.4, .8 * (1.4-altitude))
    high = min(.4, .8 * (3.0-altitude))
    if low > high:
        return float(np.clip(.8 * (2.2-altitude), -.4, .4))
    return float(np.clip(desired, low, high))


def road_vertical_velocity(desired, altitude, reference):
    """Diagnostic envelope around the local approximate road centerline."""
    low = max(-.7, .8 * (reference - .8 - altitude))
    high = min(.7, .8 * (reference + .8 - altitude))
    if low > high:
        return float(np.clip(.8 * (reference-altitude), -.7, .7))
    return float(np.clip(desired, low, high))


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from airsim_ros.msg import VelCmd
    from rmua.envs.sim_fast import SimFastConfig, SimFastVecEnv, PHASE_FLYING

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', required=True)
    parser.add_argument('--log', required=True)
    parser.add_argument('--duration', type=float, default=180.)
    parser.add_argument('--mode', choices=('policy', 'departure_shadow', 'horizontal_guarded',
                                           'vertical_guarded', 'road_guarded',
                                           'extended_guarded'), default='policy')
    parser.add_argument('--state-file', required=True,
                        help='fresh JSON written by the official-state monitor')
    args = parser.parse_args(rospy.myargv()[1:])
    weights = np.load(args.weights, allow_pickle=False)
    env = SimFastVecEnv(SimFastConfig(num_envs=1, track_mode='stage1_audited',
        action_frame='world', line_max=0, vehicle_max=0))
    rospy.init_node('rmua_policy_truth_diagnostic', anonymous=True)
    latest = [None, 0.]
    def pose_cb(msg):
        latest[0] = msg
        latest[1] = time.monotonic()
    rospy.Subscriber('/airsim_node/drone_1/debug/pose_gt', PoseStamped, pose_cb, queue_size=1)
    pub = rospy.Publisher('/airsim_node/drone_1/vel_body_cmd', VelCmd, queue_size=1)
    def publish_zero():
        cmd = VelCmd(vx=0., vy=0., vz=0., yawRate=0., va=8, stop=0)
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = 'drone_1'
        pub.publish(cmd)
    rate = rospy.Rate(20)
    prev_pos = None
    prev_t = None
    filtered_velocity = np.zeros(3)
    prev_seq = None
    takeoff_complete = False
    start_wall = time.monotonic()
    started_at = None
    planar_command = np.zeros(2)
    planar_time = None
    horizontal_active = False
    with open(args.log, 'w', buffering=1) as log:
        while not rospy.is_shutdown() and time.monotonic()-start_wall < args.duration:
            msg, arrival = latest
            now = time.monotonic()
            official = read_official_state(args.state_file, now)
            state = None if official is None else official.get('state')
            if state not in ('Pending1', 'State1'):
                publish_zero()
                if state in ('Finished', 'Pending2', 'State2'):
                    log.write(json.dumps(dict(monotonic=now, phase='official_stop',
                                              official=official))+'\n')
                    break
                rate.sleep()
                continue
            if msg is None or now-arrival > .5:
                publish_zero()
                rate.sleep()
                continue
            p, q = msg.pose.position, msg.pose.orientation
            pos = np.array([p.x, p.y, -p.z])
            # Explicit diagnostic takeoff preamble from the existing follower:
            # establish clearance before letting the learned policy command XY.
            if args.mode == 'policy' and not takeoff_complete and pos[2] < 1.65:
                cmd = VelCmd(vx=0., vy=0., vz=1., yawRate=0., va=8, stop=0)
                cmd.header.stamp = rospy.Time.now(); cmd.header.frame_id = 'drone_1'
                pub.publish(cmd)
                log.write(json.dumps(dict(elapsed=now-start_wall, monotonic=now,
                                          official=official, phase='takeoff_preamble',
                                          pos=pos.tolist()))+'\n')
                prev_pos, prev_t, prev_seq = pos.copy(), now, msg.header.seq
                rate.sleep()
                continue
            takeoff_complete = True
            # Similarity transform NED/FRD -> z-up/FRU.
            quat = np.array([q.w, -q.x, -q.y, q.z])
            quat /= max(np.linalg.norm(quat), 1e-12)
            yaw = math.atan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),
                             1-2*(quat[2]**2+quat[3]**2))
            if (prev_pos is not None and prev_t is not None and msg.header.seq != prev_seq
                    and now-prev_t > 1e-3):
                raw_velocity = (pos-prev_pos)/(now-prev_t)
                filtered_velocity = .7*filtered_velocity + .3*raw_velocity
                for i, gate in enumerate(env.track.scoring_gates):
                    if env.gate_index[0] == i and gate.contains_crossing(prev_pos, pos)[0]:
                        env.gate_index[0] += 1
                        env.gates_passed[0] += 1
                prev_pos, prev_t, prev_seq = pos.copy(), now, msg.header.seq
            elif prev_pos is None:
                prev_pos, prev_t, prev_seq = pos.copy(), now, msg.header.seq
            env.pos[0] = pos; env.vel[0] = filtered_velocity; env.quat[0] = quat
            env.yaw[0] = yaw; env.omega[0] = 0.
            if started_at is None and not env.track.start_gate.contains_trigger(pos)[0]:
                started_at = now
            if started_at is not None:
                env.phase[0] = PHASE_FLYING
                env.segment_time[0] = now-started_at
            obs = env._obs()[0]
            action = actor_forward(obs, weights)
            vertical_active = False
            if args.mode in ('departure_shadow', 'horizontal_guarded', 'vertical_guarded', 'road_guarded', 'extended_guarded'):
                velocity_cmd = departure_command(pos, now-start_wall)
                if args.mode in ('horizontal_guarded', 'vertical_guarded', 'road_guarded', 'extended_guarded'):
                    if pos[0] >= 12. and official.get('state') == 'State1':
                        horizontal_active = True
                    if horizontal_active:
                        progress, _, _, lateral, _ = env.track.project(pos)
                        reference_z = float(env.track.point_at_distance(float(progress[0])+3.)[2])
                        outside = (abs(float(lateral[0])) > 3.5 or pos[2] < .7 or
                                   (abs(pos[2]-reference_z) > 2. if args.mode in ('road_guarded', 'extended_guarded')
                                    else pos[2] > 3.6))
                        if outside:
                            log.write(json.dumps(dict(monotonic=now, phase='guard_stop',
                                pos=pos.tolist(), official=official))+'\n')
                            break
                        dt = min(.1, now-planar_time) if planar_time is not None else .05
                        limit = 8. if args.mode == 'extended_guarded' else 4.
                        planar_command = limited_planar_velocity(action[:2], planar_command, dt, max_speed=limit)
                        velocity_cmd[:2] = planar_command
                        if args.mode == 'vertical_guarded' and pos[0] >= 20.:
                            velocity_cmd[2] = guarded_vertical_velocity(action[2], pos[2])
                            vertical_active = True
                        if args.mode in ('road_guarded', 'extended_guarded') and pos[0] >= 20.:
                            velocity_cmd[2] = road_vertical_velocity(action[2], pos[2], reference_z)
                            vertical_active = True
                    else:
                        planar_command = velocity_cmd[:2].copy()
                    planar_time = now
            else:
                velocity_cmd = np.clip(action[:3], -20., 20.)
            body = world_up_to_body_command(velocity_cmd, yaw)
            speed = np.linalg.norm(velocity_cmd[:2])
            target_yaw = math.atan2(velocity_cmd[1], velocity_cmd[0]) if speed > .3 else yaw
            yaw_error = (target_yaw-yaw+math.pi)%(2*math.pi)-math.pi
            yaw_rate = math.degrees(np.clip(yaw_error*3., -1., 1.))
            cmd = VelCmd(vx=float(body[0]), vy=float(body[1]), vz=float(body[2]),
                         yawRate=float(yaw_rate), va=8, stop=0)
            cmd.header.stamp = rospy.Time.now(); cmd.header.frame_id = 'drone_1'
            pub.publish(cmd)
            log.write(json.dumps(dict(elapsed=now-start_wall, monotonic=now, official=official,
                pos=pos.tolist(), phase=args.mode, horizontal_active=horizontal_active,
                vertical_active=vertical_active,
                vel=filtered_velocity.tolist(),
                yaw=yaw, quat=quat.tolist(), obs=obs.tolist(), action=action.tolist(),
                cmd=[cmd.vx,cmd.vy,cmd.vz,cmd.yawRate],
                inferred_gates=int(env.gates_passed[0]),
                progress=float(env.track.project(pos)[0][0])))+'\n')
            if args.mode == 'departure_shadow' and pos[0] >= 12. and official.get('state') == 'State1':
                log.write(json.dumps(dict(monotonic=now, phase='departure_complete',
                                          official=official))+'\n')
                break
            stop_x = 60. if args.mode == 'road_guarded' else 30.
            min_score = 3 if args.mode == 'road_guarded' else 2
            if (args.mode in ('horizontal_guarded', 'vertical_guarded', 'road_guarded') and pos[0] >= stop_x and
                    official.get('state') == 'State1' and official.get('score', 0) >= min_score):
                log.write(json.dumps(dict(monotonic=now, phase='guarded_complete',
                                          official=official))+'\n')
                break
            if (args.mode == 'extended_guarded' and float(env.track.project(pos)[0][0]) >= 200.
                    and official.get('state') == 'State1' and official.get('score', 0) >= 9):
                log.write(json.dumps(dict(monotonic=now, phase='guarded_complete',
                                          official=official))+'\n')
                break
            rate.sleep()
    # Keep a freshly timestamped brake flowing briefly; the caller must still
    # stop the diagnostic simulator promptly after this process exits.
    with open(args.log, 'a', buffering=1) as log:
        for _ in range(20):
            if rospy.is_shutdown():
                break
            publish_zero()
            msg = latest[0]
            if msg is not None:
                p = msg.pose.position
                log.write(json.dumps(dict(phase='braking', monotonic=time.monotonic(),
                    pos=[p.x, p.y, -p.z], official=read_official_state(args.state_file, time.monotonic())))+'\n')
            rate.sleep()


if __name__ == '__main__':
    main()
