#!/usr/bin/env python3
"""Plays MAVROS + VRPN for the ros_io test: publishes /mavros/imu/data_raw
(200 Hz) and /vrpn_client_node/uav1/pose (100 Hz) for a 0.5 m circle at
0.8 rad/s with level attitude, and records /mavros/vision_pose/pose.
Also records /rigid_state_estimator/state as raw bytes (the test has no
Python classes for it) with the publisher's advertised type and md5.
Prints one JSON line: {"vision": n, "last": [t, x, y, z], "estimates": n,
"estimate_type": ..., "estimate_md5": ..., "running": n}."""
import json
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

G, R, W = 9.8066, 0.5, 0.8
duration = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
rospy.init_node("eskf_chain_player", disable_signals=True)
imu_pub = rospy.Publisher("/mavros/imu/data_raw", Imu, queue_size=50)
pose_pub = rospy.Publisher("/vrpn_client_node/uav1/pose", PoseStamped, queue_size=50)
got = []
rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped,
                 lambda m: got.append([m.header.stamp.to_sec(), m.pose.position.x, m.pose.position.y, m.pose.position.z]))
estimates = []
estimate_meta = {}


def on_estimate(m):
    estimate_meta.update(m._connection_header)
    # header: seq u32, stamp 2 x u32, frame_id (u32 length + bytes); then estimator_state u8
    frame_len = int.from_bytes(m._buff[12:16], "little")
    estimates.append(m._buff[16 + frame_len])


rospy.Subscriber("/rigid_state_estimator/state", rospy.AnyMsg, on_estimate)
rospy.sleep(1.0)  # connections
start = rospy.Time.now().to_sec()
rate = rospy.Rate(200)
i = 0
while rospy.Time.now().to_sec() - start < duration:
    now = rospy.Time.now()
    t = now.to_sec()
    imu = Imu()
    imu.header.stamp = now
    imu.linear_acceleration.x = -R * W * W * math.sin(W * t)
    imu.linear_acceleration.y = -R * W * W * math.cos(W * t)
    imu.linear_acceleration.z = G
    imu.orientation.w = 1.0
    imu_pub.publish(imu)
    if i % 2 == 0:
        p = PoseStamped()
        p.header.stamp = now
        p.pose.position.x = R * math.sin(W * t)
        p.pose.position.y = R * math.cos(W * t) - R
        p.pose.position.z = 1.0
        p.pose.orientation.w = 1.0
        pose_pub.publish(p)
    i += 1
    rate.sleep()
rospy.sleep(0.5)
print(json.dumps({"vision": len(got), "last": got[-1] if got else None, "estimates": len(estimates),
                  "estimate_type": estimate_meta.get("type"), "estimate_md5": estimate_meta.get("md5sum"),
                  "running": sum(1 for s in estimates if s == 3)}), flush=True)
