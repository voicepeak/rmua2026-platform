#!/usr/bin/env python3
"""Bounded passive ROS capture; debug truth is for offline diagnostics only."""
import base64
import json
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock


def main():
    output, duration = sys.argv[1], float(sys.argv[2])
    if not 0 < duration <= 60:
        raise ValueError("duration must be in (0, 30]")
    rospy.init_node("foundation_passive_capture", anonymous=True, disable_signals=True)
    lock = threading.Lock()
    start = time.monotonic()
    with open(output, "w", buffering=1) as stream:
        def record(topic, body):
            row = {"topic": topic, "arrival_monotonic_ns": time.monotonic_ns(),
                   "arrival_wall_ns": time.time_ns(), **body}
            with lock:
                stream.write(json.dumps(row, separators=(",", ":")) + "\n")

        def pose(msg):
            p, q = msg.pose.position, msg.pose.orientation
            record("pose_gt", {"stamp_ns": msg.header.stamp.to_nsec(), "frame_id": msg.header.frame_id,
                               "position": [p.x, p.y, p.z], "quaternion_wxyz": [q.w, q.x, q.y, q.z]})

        def gps(msg):
            p, q = msg.pose.position, msg.pose.orientation
            record("gps", {"stamp_ns": msg.header.stamp.to_nsec(), "frame_id": msg.header.frame_id,
                           "position": [p.x, p.y, p.z], "quaternion_wxyz": [q.w, q.x, q.y, q.z]})

        def clock(msg):
            record("clock", {"stamp_ns": msg.clock.to_nsec()})

        def imu(msg):
            a, w, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
            record("imu", {"stamp_ns": msg.header.stamp.to_nsec(), "frame_id": msg.header.frame_id,
                           "acceleration_body": [a.x, a.y, a.z], "omega_body": [w.x, w.y, w.z],
                           "quaternion_wxyz": [q.w, q.x, q.y, q.z]})

        def raw(msg, topic):
            header = msg._connection_header or {}
            record(topic, {"type": header.get("type"), "md5sum": header.get("md5sum"),
                           "wire_base64": base64.b64encode(bytes(msg._buff)).decode("ascii")})

        record("collector_meta", {"use_sim_time": bool(rospy.get_param('/use_sim_time', False)),
                                  "ros_now_ns": rospy.Time.now().to_nsec(),
                                  "published_topics_at_start": rospy.get_published_topics()})
        subs = [rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, pose, queue_size=200),
                rospy.Subscriber("/airsim_node/drone_1/gps", PoseStamped, gps, queue_size=200),
                rospy.Subscriber("/airsim_node/drone_1/imu/imu", Imu, imu, queue_size=200),
                rospy.Subscriber("/airsim_node/drone_1/debug/rotor_pwm", rospy.AnyMsg, raw, "rotor_pwm", queue_size=200),
                rospy.Subscriber("/airsim_node/drone_1/debug/wind", rospy.AnyMsg, raw, "wind", queue_size=200),
                rospy.Subscriber("/clock", Clock, clock, queue_size=200)]
        while not rospy.is_shutdown() and time.monotonic() - start < duration:
            time.sleep(.02)
        for sub in subs:
            sub.unregister()


if __name__ == "__main__":
    main()
