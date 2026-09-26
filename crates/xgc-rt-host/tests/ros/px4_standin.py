#!/usr/bin/env python3
"""Stand-in vehicle + PX4/MAVROS for the unchanged px4_multirotor_controller
running behind ros_io (tracking_backend px4_local, as in the TRO config).

Loop: the plant's true pose goes out as VRPN (100 Hz, also on /uav1/pose for
the controller's consistency check) and MAVROS raw IMU
(200 Hz) -> ros_io -> ESKF module in the aggregator -> ros_io ->
/uav1/mavros/vision_pose/pose. This stand-in PX4 uses that vision pose as
its local position (the role EKF2 plays on the real vehicle) and follows the
controller's /uav1/mavros/setpoint_raw/local with a simple position/velocity
loop. It answers arming (cmd/command 400) and set_mode, sends "takeoff",
then "land", and prints one JSON summary line.
"""
import json
import math
import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandLong, CommandLongResponse, SetMode, SetModeResponse
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import String

G = 9.8066
NS = "/uav1"
hover_s = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
timeout_s = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

rospy.init_node("px4_standin", disable_signals=True)
lock = threading.Lock()
plant = {"p": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "a": [0.0, 0.0, 0.0]}
fcu = {"armed": False, "mode": "POSCTL", "arm_calls": 0, "disarm_calls": 0, "mode_calls": []}
sp = {"msg": None, "count": 0}
vision = {"last": None, "prev": None, "vel": [0.0, 0.0, 0.0], "count": 0, "err": []}
truth_hist = []  # (t, p)
states = []
max_z = [0.0]

imu_raw_pub = rospy.Publisher(NS + "/mavros/imu/data_raw", Imu, queue_size=50)
imu_pub = rospy.Publisher(NS + "/mavros/imu/data", Imu, queue_size=50)
vrpn_pub = rospy.Publisher("/vrpn_client_node/uav1/pose", PoseStamped, queue_size=50)
canon_pub = rospy.Publisher(NS + "/pose", PoseStamped, queue_size=50)  # controller's consistency check
state_pub = rospy.Publisher(NS + "/mavros/state", State, queue_size=10)
lpos_pub = rospy.Publisher(NS + "/mavros/local_position/pose", PoseStamped, queue_size=50)
lvel_pub = rospy.Publisher(NS + "/mavros/local_position/velocity_local", TwistStamped, queue_size=50)
batt_pub = rospy.Publisher(NS + "/mavros/battery", BatteryState, queue_size=5)
cmd_pub = rospy.Publisher("/command", String, queue_size=5, latch=False)


def truth_at(t):
    best = None
    for tt, p in reversed(truth_hist[-400:]):
        if best is None or abs(tt - t) < abs(best[0] - t):
            best = (tt, p)
        if tt < t - 0.05:
            break
    return best


def on_vision(m):
    t = m.header.stamp.to_sec()
    p = [m.pose.position.x, m.pose.position.y, m.pose.position.z]
    with lock:
        if vision["last"] is not None:
            dt = t - vision["last"][0]
            if dt > 1e-4:
                raw = [(p[i] - vision["last"][1][i]) / dt for i in range(3)]
                vision["vel"] = [0.7 * vision["vel"][i] + 0.3 * raw[i] for i in range(3)]
        vision["last"] = (t, p)
        vision["count"] += 1
        tr = truth_at(t)
        if tr is not None and abs(tr[0] - t) < 0.006:
            vision["err"].append(math.dist(p, tr[1]))
    # Stand-in EKF2: local position = the vision pose from the aggregator's ESKF.
    lp = PoseStamped()
    lp.header.stamp = m.header.stamp
    lp.header.frame_id = "map"
    lp.pose = m.pose
    lpos_pub.publish(lp)
    lv = TwistStamped()
    lv.header.stamp = m.header.stamp
    lv.twist.linear.x, lv.twist.linear.y, lv.twist.linear.z = vision["vel"]
    lvel_pub.publish(lv)


def on_setpoint(m):
    with lock:
        sp["msg"] = m
        sp["count"] += 1


def on_status(m):
    if not states or states[-1] != m.data:
        states.append(m.data)


def on_command(req):
    if int(req.command) == 400:  # MAV_CMD_COMPONENT_ARM_DISARM
        with lock:
            arm = req.param1 > 0.5
            fcu["armed"] = arm
            fcu["arm_calls" if arm else "disarm_calls"] += 1
        return CommandLongResponse(success=True, result=0)
    return CommandLongResponse(success=False, result=4)


def on_set_mode(req):
    with lock:
        fcu["mode"] = req.custom_mode
        fcu["mode_calls"].append(req.custom_mode)
    return SetModeResponse(mode_sent=True)


rospy.Subscriber(NS + "/mavros/vision_pose/pose", PoseStamped, on_vision)
rospy.Subscriber(NS + "/mavros/setpoint_raw/local", PositionTarget, on_setpoint)
rospy.Subscriber(NS + "/custom/statustext", String, on_status)
rospy.Service(NS + "/mavros/cmd/command", CommandLong, on_command)
rospy.Service(NS + "/mavros/set_mode", SetMode, on_set_mode)


def step_plant(dt):
    """PX4 stand-in: follow position (+ velocity feedforward) or velocity setpoints."""
    with lock:
        p, v = plant["p"], plant["v"]
        m = sp["msg"]
        flying = fcu["armed"] and fcu["mode"] == "OFFBOARD" and m is not None
        if flying:
            mask = m.type_mask
            use_pos = not (mask & (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ))
            use_vel = not (mask & (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ))
            spp = [m.position.x, m.position.y, m.position.z]
            spv = [m.velocity.x, m.velocity.y, m.velocity.z] if use_vel else [0.0, 0.0, 0.0]
            vcmd = [(1.5 * (spp[i] - p[i]) if use_pos else 0.0) + spv[i] for i in range(3)]
            norm = math.sqrt(sum(c * c for c in vcmd))
            if norm > 1.5:
                vcmd = [c * 1.5 / norm for c in vcmd]
            a = [max(-3.0, min(3.0, (vcmd[i] - v[i]) / 0.3)) for i in range(3)]
        else:
            a = [-v[i] / 0.2 for i in range(3)] if p[2] > 0.0 else [0.0, 0.0, 0.0]
            if not fcu["armed"]:
                a = [0.0, 0.0, -G if p[2] > 0.0 else 0.0]
        for i in range(3):
            v[i] += a[i] * dt
            p[i] += v[i] * dt
        if p[2] <= 0.0:
            # Ground contact: the ground cancels any downward acceleration, so
            # the accelerometer reads +g, not the commanded descent.
            p[2] = 0.0
            v[2] = max(v[2], 0.0)
            a[2] = max(a[2], 0.0)
            if not flying:
                v[0] = v[1] = 0.0
                a = [0.0, 0.0, 0.0]
        plant["a"] = a
        max_z[0] = max(max_z[0], p[2])
        return list(p), list(a)


def publish_loop():
    rate = rospy.Rate(200)
    i = 0
    last = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and not done.is_set():
        now = rospy.Time.now()
        t = now.to_sec()
        p, a = step_plant(max(0.0, min(0.02, t - last)))
        last = t
        truth_hist.append((t, p))
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "base_link"
        imu.orientation.w = 1.0
        imu.linear_acceleration.x, imu.linear_acceleration.y = a[0], a[1]
        imu.linear_acceleration.z = a[2] + G
        imu_raw_pub.publish(imu)
        imu_pub.publish(imu)
        if i % 2 == 0:
            vp = PoseStamped()
            vp.header.stamp = now
            vp.header.frame_id = "world"
            vp.pose.position.x, vp.pose.position.y, vp.pose.position.z = p
            vp.pose.orientation.w = 1.0
            vrpn_pub.publish(vp)
            canon_pub.publish(vp)
        if i % 20 == 0:
            st = State()
            st.header.stamp = now
            with lock:
                st.connected, st.armed, st.guided, st.mode = True, fcu["armed"], True, fcu["mode"]
            state_pub.publish(st)
        if i % 200 == 0:
            b = BatteryState()
            b.header.stamp = now
            b.voltage, b.percentage = 16.4, 0.9
            batt_pub.publish(b)
        i += 1
        rate.sleep()


done = threading.Event()
threading.Thread(target=publish_loop, daemon=True).start()
start = rospy.Time.now().to_sec()


def wait_for(pred, limit):
    end = rospy.Time.now().to_sec() + limit
    while rospy.Time.now().to_sec() < end:
        if pred():
            return True
        rospy.sleep(0.05)
    return False


result = {}
# Ready only counts after this run's controller has been seen in SelfCheck, so
# a stale state from anything else on the master cannot start the flight.
ready = wait_for(lambda: "SelfCheck" in states and states[-1] == "Ready", 25.0)
result["ready_after_s"] = round(rospy.Time.now().to_sec() - start, 2) if ready else None
def command(text, accepted, limit=10.0):
    """/command is not latched: wait for the controller's subscription, then
    repeat once a second until the controller's state shows it was taken."""
    wait_for(lambda: cmd_pub.get_num_connections() > 0, 5.0)
    end = rospy.Time.now().to_sec() + limit
    while rospy.Time.now().to_sec() < end:
        cmd_pub.publish(String(text))
        if wait_for(accepted, 1.0):
            return True
    return False


if ready:
    command("takeoff", lambda: states[-1] != "Ready")
    took_off = wait_for(lambda: bool(states) and states[-1] == "Hover", 30.0)
    result["hover_at_z"] = round(plant["p"][2], 3) if took_off else None
    if took_off:
        rospy.sleep(hover_s)
        result["z_after_hover"] = round(plant["p"][2], 3)
        command("land", lambda: states[-1] == "Landing")
        wait_for(lambda: not fcu["armed"], 30.0)
done.set()
with lock:
    err = sorted(vision["err"])
    result.update({
        "states": states,
        "max_z": round(max_z[0], 3),
        "final_z": round(plant["p"][2], 3),
        "armed_at_end": fcu["armed"],
        "arm_calls": fcu["arm_calls"],
        "disarm_calls": fcu["disarm_calls"],
        "mode_calls": fcu["mode_calls"],
        "setpoints": sp["count"],
        "vision_poses": vision["count"],
        "eskf_err_p50_m": round(err[len(err) // 2], 4) if err else None,
        "eskf_err_max_m": round(err[-1], 4) if err else None,
        "eskf_err_samples": len(err),
    })
print(json.dumps(result), flush=True)
