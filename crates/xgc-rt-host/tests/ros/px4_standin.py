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

With TRACK_S > 0 it also flies the planner path (tracking_backend px4_local,
Custom1): in Hover it publishes a 10 Hz planner setpoint on
/uav1/alg/setpoint_raw/local that holds the hover point, sends "custom1",
moves the plan +x at 0.3 m/s for TRACK_S seconds, holds 2 s, sends "hover",
then lands as above.

With TRACKING "reference" (the NMPC and DFBC backends) the Custom1 phase
instead flies the reference the controller activates itself: "custom1"
makes it request an analytic reference from the reference trajectory
generator, and it then sends body-rate + thrust targets on
setpoint_raw/attitude. The stand-in PX4 then flies attitude: body rates
follow the command with a 50 ms lag, and normalized thrust maps to specific
thrust through HOVER_THRUST. It publishes a hover-thrust estimate (the
plant's own value) as the hover thrust estimator would.

Usage: px4_standin.py [HOVER_S [TIMEOUT_S [TRACK_S [planner|reference]]]]
"""
import json
import math
import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, PositionTarget, State
from mavros_msgs.srv import CommandLong, CommandLongResponse, SetMode, SetModeResponse
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import String

G = 9.8066
NS = "/uav1"
hover_s = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
timeout_s = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
track_s = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
tracking = sys.argv[4] if len(sys.argv) > 4 else "planner"
HOVER_THRUST = 0.5
try:
    from hover_thrust_estimator_msgs.msg import HoverThrustEstimate
except ImportError:
    HoverThrustEstimate = None
TRACK_SPEED = 0.3

rospy.init_node("px4_standin", disable_signals=True)
lock = threading.Lock()
plant = {"p": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "a": [0.0, 0.0, 0.0],
         "q": [1.0, 0.0, 0.0, 0.0], "w": [0.0, 0.0, 0.0]}  # q: world<-body, w x y z; w: body rates
fcu = {"armed": False, "mode": "POSCTL", "arm_calls": 0, "disarm_calls": 0, "mode_calls": []}
sp = {"msg": None, "count": 0, "t": 0.0}
att = {"msg": None, "count": 0, "t": 0.0}
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
plan_pub = rospy.Publisher(NS + "/alg/setpoint_raw/local", PositionTarget, queue_size=10)
plan = {"origin": None, "t_move": None, "count": 0, "err": []}


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
        sp["t"] = rospy.Time.now().to_sec()


def on_attitude(m):
    with lock:
        att["msg"] = m
        att["count"] += 1
        att["t"] = rospy.Time.now().to_sec()


def qmul(a, b):
    return [a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]]


def qnorm(q):
    n = math.sqrt(sum(c * c for c in q))
    return [c / n for c in q]


def rotate(q, v):
    """World <- body."""
    r = qmul(qmul(q, [0.0] + list(v)), [q[0], -q[1], -q[2], -q[3]])
    return r[1:]


def unrotate(q, v):
    """Body <- world."""
    return rotate([q[0], -q[1], -q[2], -q[3]], v)


def tilt_for(acc):
    """Level-yaw attitude whose body z is along the thrust (acc + g)."""
    f = [acc[0], acc[1], acc[2] + G]
    n = math.sqrt(sum(c * c for c in f))
    if n < 1e-6:
        return [1.0, 0.0, 0.0, 0.0]
    z = [c / n for c in f]
    # Shortest rotation from world z to z.
    axis = [-z[1], z[0], 0.0]
    s_ = math.sqrt(axis[0] ** 2 + axis[1] ** 2)
    if s_ < 1e-9:
        return [1.0, 0.0, 0.0, 0.0]
    ang = math.atan2(s_, z[2])
    k = math.sin(0.5 * ang) / s_
    return [math.cos(0.5 * ang), axis[0] * k, axis[1] * k, 0.0]


def rates_between(q0, q1, dt):
    d = qmul([q0[0], -q0[1], -q0[2], -q0[3]], q1)
    if d[0] < 0:
        d = [-c for c in d]
    return [2.0 * c / dt for c in d[1:]] if dt > 1e-6 else [0.0, 0.0, 0.0]


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
rospy.Subscriber(NS + "/mavros/setpoint_raw/attitude", AttitudeTarget, on_attitude)
hte_pub = rospy.Publisher(NS + "/hover_thrust/estimate_state", HoverThrustEstimate, queue_size=10) if HoverThrustEstimate else None
rospy.Subscriber(NS + "/custom/statustext", String, on_status)
rospy.Service(NS + "/mavros/cmd/command", CommandLong, on_command)
rospy.Service(NS + "/mavros/set_mode", SetMode, on_set_mode)


def step_plant(dt):
    """PX4 stand-in. Position mode: follow position (+ velocity feedforward) or
    velocity setpoints; the attitude follows the demanded acceleration.
    Attitude mode (a fresh setpoint_raw/attitude, newer than any position
    setpoint): body rates follow the command with a 50 ms lag, and thrust
    along body z is thrust / HOVER_THRUST * g."""
    with lock:
        p, v, q, w = plant["p"], plant["v"], plant["q"], plant["w"]
        now = rospy.Time.now().to_sec()
        m = sp["msg"]
        offboard = fcu["armed"] and fcu["mode"] == "OFFBOARD"
        use_att = offboard and att["msg"] is not None and now - att["t"] < 0.5 and att["t"] >= sp["t"]
        flying = offboard and (m is not None or use_att)
        if use_att:
            cmd = att["msg"]
            rate = [cmd.body_rate.x, cmd.body_rate.y, cmd.body_rate.z]
            for i in range(3):
                w[i] += (rate[i] - w[i]) * min(1.0, dt / 0.05)
            q[:] = qnorm(qmul(q, [1.0, 0.5 * w[0] * dt, 0.5 * w[1] * dt, 0.5 * w[2] * dt]))
            f = rotate(q, [0.0, 0.0, max(0.0, cmd.thrust) / HOVER_THRUST * G])
            a = [f[0], f[1], f[2] - G]
        else:
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
            q_new = tilt_for(a) if fcu["armed"] and p[2] > 0.0 else [1.0, 0.0, 0.0, 0.0]
            w[:] = rates_between(q, q_new, dt)
            q[:] = q_new
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
                q[:] = [1.0, 0.0, 0.0, 0.0]
                w[:] = [0.0, 0.0, 0.0]
        plant["a"] = a
        max_z[0] = max(max_z[0], p[2])
        return list(p), list(a), list(q), list(w)


def publish_loop():
    rate = rospy.Rate(200)
    i = 0
    last = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and not done.is_set():
        now = rospy.Time.now()
        t = now.to_sec()
        p, a, q, w = step_plant(max(0.0, min(0.02, t - last)))
        last = t
        truth_hist.append((t, p))
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "base_link"
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = q
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = w
        (imu.linear_acceleration.x, imu.linear_acceleration.y,
         imu.linear_acceleration.z) = unrotate(q, [a[0], a[1], a[2] + G])
        imu_raw_pub.publish(imu)
        imu_pub.publish(imu)
        if i % 2 == 0:
            vp = PoseStamped()
            vp.header.stamp = now
            vp.header.frame_id = "world"
            vp.pose.position.x, vp.pose.position.y, vp.pose.position.z = p
            vp.pose.orientation.w, vp.pose.orientation.x, vp.pose.orientation.y, vp.pose.orientation.z = q
            vrpn_pub.publish(vp)
            canon_pub.publish(vp)
        if i % 20 == 0:
            st = State()
            st.header.stamp = now
            with lock:
                st.connected, st.armed, st.guided, st.mode = True, fcu["armed"], True, fcu["mode"]
            state_pub.publish(st)
        if hte_pub is not None and tracking == "reference" and i % 4 == 0:
            h = HoverThrustEstimate()
            h.header.stamp = now
            h.state = HoverThrustEstimate.STATE_AIRBORNE if p[2] > 0.5 else HoverThrustEstimate.STATE_GROUND
            h.hover_thrust = HOVER_THRUST
            hte_pub.publish(h)
        if i % 200 == 0:
            b = BatteryState()
            b.header.stamp = now
            b.voltage, b.percentage = 16.4, 0.9
            batt_pub.publish(b)
        i += 1
        rate.sleep()


def plan_at(t):
    """The planner's path: hold the origin, then +x at TRACK_SPEED for track_s."""
    x0, y0, z0 = plan["origin"]
    tau = 0.0 if plan["t_move"] is None else max(0.0, min(track_s, t - plan["t_move"]))
    moving = plan["t_move"] is not None and 0.0 < t - plan["t_move"] < track_s
    return [x0 + TRACK_SPEED * tau, y0, z0], [TRACK_SPEED if moving else 0.0, 0.0, 0.0]


def plan_loop():
    """10 Hz planner setpoints (the DMPC planner's rate) while plan_on is set."""
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not done.is_set():
        if plan_on.is_set():
            now = rospy.Time.now()
            p, v = plan_at(now.to_sec())
            m = PositionTarget()
            m.header.stamp = now
            m.header.frame_id = "map"
            m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            m.type_mask = PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
            m.position.x, m.position.y, m.position.z = p
            m.velocity.x, m.velocity.y, m.velocity.z = v
            plan_pub.publish(m)
            plan["count"] += 1
            if plan["t_move"] is not None:
                with lock:
                    plan["err"].append(math.dist(p, plant["p"]))
        rate.sleep()


done = threading.Event()
plan_on = threading.Event()
threading.Thread(target=publish_loop, daemon=True).start()
threading.Thread(target=plan_loop, daemon=True).start()
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
    if took_off and track_s > 0.0 and tracking == "reference":
        origin = list(plant["p"])
        tracking_ok = command("custom1", lambda: states[-1] == "Custom1")
        result["custom1"] = tracking_ok
        if tracking_ok:
            reach, z_lo, z_hi = 0.0, 1e9, -1e9
            end = rospy.Time.now().to_sec() + track_s
            while rospy.Time.now().to_sec() < end and states[-1] == "Custom1":
                with lock:
                    pp = list(plant["p"])
                reach = max(reach, math.hypot(pp[0] - origin[0], pp[1] - origin[1]))
                z_lo, z_hi = min(z_lo, pp[2]), max(z_hi, pp[2])
                rospy.sleep(0.05)
            result["custom1_held"] = states[-1] == "Custom1"
            result["reach_xy_m"] = round(reach, 3)
            result["z_range_m"] = [round(z_lo, 3), round(z_hi, 3)]
            result["attitude_setpoints"] = att["count"]
            command("hover", lambda: states[-1] == "Hover")
    elif took_off and track_s > 0.0:
        with lock:
            plan["origin"] = list(plant["p"])
        plan_on.set()
        rospy.sleep(0.5)
        tracking = command("custom1", lambda: states[-1] == "Custom1")
        result["custom1"] = tracking
        if tracking:
            plan["t_move"] = rospy.Time.now().to_sec()
            rospy.sleep(track_s + 2.0)
            with lock:
                err = sorted(plan["err"])
                result["x_after_track"] = round(plant["p"][0] - plan["origin"][0], 3)
                result["track_err_p50_m"] = round(err[len(err) // 2], 4) if err else None
                result["track_err_max_m"] = round(err[-1], 4) if err else None
            command("hover", lambda: states[-1] == "Hover")
        plan_on.clear()
        result["plan_setpoints"] = plan["count"]
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
