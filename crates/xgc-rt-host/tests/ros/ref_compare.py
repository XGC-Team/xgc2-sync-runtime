#!/usr/bin/env python3
"""Send the same reference requests to the ref-trajectory module (through
ros_io, namespace /uav1) and to the unchanged multirotor_reference_trajectory
node (/uav2), and compare what each publishes.

Per trajectory id, the active references must be equal field for field and
bit for bit, except the times that depend on when each side received the
request (header stamps, start_time). The status state sequences must match.
Prints one JSON summary line.
"""
import json
import math
import struct
import sys
import threading

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from multirotor_reference_trajectory_msgs.msg import (ActivePolynomialReference, AnalyticReference,
                                                      FlatReferencePoint, ReferenceStatus, SampledReference,
                                                      WaypointReferenceRequest)
from std_msgs.msg import Empty

BASE = "alg/multirotor_reference_trajectory/"
SIDES = ("/uav1/", "/uav2/")  # module, node

rospy.init_node("ref_compare", disable_signals=True)
lock = threading.Lock()
active = {s: {} for s in SIDES}   # side -> (kind, id, revision) -> last message
states = {s: [] for s in SIDES}   # side -> deduplicated state sequence


def keep(side, kind):
    def cb(m):
        with lock:
            active[side][(kind, m.trajectory_id, m.revision)] = m
    return cb


def on_status(side):
    def cb(m):
        with lock:
            if not states[side] or states[side][-1] != m.state:
                states[side].append(m.state)
    return cb


pubs = {}
for s in SIDES:
    rospy.Subscriber(s + BASE + "active/analytic", AnalyticReference, keep(s, "analytic"))
    rospy.Subscriber(s + BASE + "active/polynomial", ActivePolynomialReference, keep(s, "polynomial"))
    rospy.Subscriber(s + BASE + "active/sampled", SampledReference, keep(s, "sampled"))
    rospy.Subscriber(s + BASE + "status", ReferenceStatus, on_status(s))
    pubs[s] = {
        "analytic": rospy.Publisher(s + BASE + "request/analytic", AnalyticReference, queue_size=5),
        "waypoint": rospy.Publisher(s + BASE + "request/waypoint", WaypointReferenceRequest, queue_size=5),
        "sampled": rospy.Publisher(s + BASE + "request/sampled", SampledReference, queue_size=5),
        "reset": rospy.Publisher(s + BASE + "reset", Empty, queue_size=5),
    }


def wait_for(pred, limit):
    end = rospy.Time.now().to_sec() + limit
    while rospy.Time.now().to_sec() < end:
        if pred():
            return True
        rospy.sleep(0.05)
    return False


# Both sides are up and subscribed (their status is latched and repeats).
ready = wait_for(lambda: all(states[s] for s in SIDES) and
                 all(p.get_num_connections() > 0 for s in SIDES for p in pubs[s].values()), 20.0)


def pose(x, y, z, yaw=0.0):
    return Pose(Point(x, y, z), Quaternion(0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)))


sent = []


def send(kind, msg, expect=None):
    """Publish to both sides; wait until both publish the expected active reference."""
    for s in SIDES:
        pubs[s][kind].publish(msg)
    if expect is None:
        rospy.sleep(1.0)
        return True
    ok = wait_for(lambda: all(expect in active[s] for s in SIDES), 10.0)
    sent.append({"kind": expect[0], "id": expect[1], "both": ok})
    return ok


def analytic(tid, kind, params, duration=6.0):
    m = AnalyticReference()
    m.header.stamp = rospy.Time.now()
    m.trajectory_id, m.revision, m.analytic_type = tid, 1, kind
    m.duration, m.origin, m.params = duration, pose(0.0, 0.0, 1.0, 0.3), params
    send("analytic", m, ("analytic", tid, 1))


def sampled(tid):
    m = SampledReference()
    m.header.stamp = rospy.Time.now()
    m.trajectory_id, m.revision, m.sample_dt = tid, 1, 0.1
    for i in range(40):
        s, w = i * 0.1, 0.5
        p = FlatReferencePoint()
        p.t_from_start = s
        p.position = Point(math.cos(w * s), math.sin(w * s), 1.0 + 0.1 * s)
        p.velocity = Vector3(-w * math.sin(w * s), w * math.cos(w * s), 0.1)
        p.yaw, p.yaw_rate = 0.2 * s, 0.2
        m.points.append(p)
    send("sampled", m, ("sampled", tid, 1))


def waypoints(tid, points, segment_times=(), constraints=(), sizes=(), revision=3):
    m = WaypointReferenceRequest()
    m.header.stamp = rospy.Time.now()
    m.trajectory_id, m.revision = tid, revision
    m.waypoints = [pose(*p) for p in points]
    m.constraint_types = list(constraints)
    m.region_size = [Vector3(*s) for s in sizes]
    m.segment_times = list(segment_times)
    m.desired_speed, m.time_weight, m.max_iterations, m.rel_cost_tol = 1.0, 0.1, 80, 1.0e-5
    m.objective = WaypointReferenceRequest.OBJECTIVE_MINCO
    send("waypoint", m, ("polynomial", tid, revision))


A, W = AnalyticReference, WaypointReferenceRequest
if ready:
    analytic(1, A.ANALYTIC_CIRCLE_ENTRY, [1.5, 1.0, 1.2, 0.2, 0.3, 2.0, 0.5, -0.5], duration=8.0)
    analytic(2, A.ANALYTIC_LINE, [2.0, 1.0, 1.5, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0], duration=4.0)
    analytic(3, A.ANALYTIC_TORUS_KNOT, [0.3, 0.3, 2.0, 0.2, 0.1, 1.6], duration=10.0)
    sampled(4)
    waypoints(5, [(0, 0, 1), (1, 0.5, 1.2), (2, 0, 1)], segment_times=(1.0, 1.0))
    waypoints(6, [(0, 0, 1), (1, 1, 1.5), (2, 0, 1.2), (3, 1, 1)],
              constraints=(W.CONSTRAINT_POINT, W.CONSTRAINT_SPHERE, W.CONSTRAINT_BOX, W.CONSTRAINT_POINT),
              sizes=((0, 0, 0), (0.2, 0.2, 0.2), (0.3, 0.2, 0.1), (0, 0, 0)))
    send("reset", Empty())
    analytic(7, A.ANALYTIC_HOLD, [], duration=1.0)
    rospy.sleep(2.5)  # expires on both sides -> Ready


def bits(v):
    return struct.pack("<d", v)


def same(a, b, path="", skip=("header", "start_time")):
    """Field-for-field equality; floats compared bit for bit."""
    if isinstance(a, float):
        return [] if bits(a) == bits(b) else [path]
    if isinstance(a, (list, tuple)):
        if len(a) != len(b):
            return [path + ".len"]
        out = []
        for i, (x, y) in enumerate(zip(a, b)):
            out += same(x, y, f"{path}[{i}]")
        return out
    if hasattr(a, "__slots__"):
        out = []
        for f in a.__slots__:
            if f in skip:
                continue
            out += same(getattr(a, f), getattr(b, f), path + "." + f)
        return out
    return [] if a == b else [path]


with lock:
    keys = sorted(set(active[SIDES[0]]) | set(active[SIDES[1]]))
    diffs = {}
    for k in keys:
        a, b = active[SIDES[0]].get(k), active[SIDES[1]].get(k)
        if a is None or b is None:
            diffs[f"{k[0]}:{k[1]}"] = ["missing on " + ("module" if a is None else "node")]
            continue
        d = same(a, b)
        if d:
            diffs[f"{k[0]}:{k[1]}"] = d[:5]
    poly = [active[SIDES[0]][k] for k in keys if k[0] == "polynomial" and k in active[SIDES[0]]]
    result = {
        "ready": ready,
        "sent": sent,
        "compared": len(keys),
        "diffs": diffs,
        "states_module": states[SIDES[0]],
        "states_node": states[SIDES[1]],
        "polynomial_coeffs": sum(len(p.coeff_x) + len(p.coeff_y) + len(p.coeff_z) + len(p.coeff_yaw) for p in poly),
    }
print(json.dumps(result), flush=True)
