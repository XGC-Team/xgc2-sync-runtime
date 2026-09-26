#!/usr/bin/env python3
"""Actual host + Noetic ros_io clock/pose loop. No plant or physical claim.
Run only against an isolated test ROS master, with the source rebuilt ELF.
"""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock

root = Path(sys.argv[1]).resolve()
host, plugin = (root / "artifacts/xgc-rt-host", root / "artifacts/libros_io.so")
root.mkdir(parents=True, exist_ok=True)
rospy.set_param("/use_sim_time", True)
rospy.init_node("clock_authority", anonymous=False, disable_signals=True)
clock_pub = rospy.Publisher("/clock", Clock, queue_size=256)
pose_pub = rospy.Publisher("/integration/pose", PoseStamped, queue_size=4)
state = {"ns": 0, "emit": True, "end": False}
received = []
lock = threading.Lock()

def receive(msg):
    with lock:
        received.append((time.monotonic(), msg.header.stamp.to_nsec(), msg.pose.position.x))

sub = rospy.Subscriber("/integration/vision", PoseStamped, receive, queue_size=256)

def pump():
    while not state["end"]:
        if state["emit"]:
            ns = state["ns"]
            stamp = rospy.Time(ns // 1000000000, ns % 1000000000)
            clock_pub.publish(Clock(stamp))
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "world"
            pose.pose.orientation.w = 1
            pose.pose.position.x = 3.25
            pose_pub.publish(pose)
        time.sleep(0.005)

thread = threading.Thread(target=pump)
thread.start()

def wait(test, what, timeout=6):
    deadline = time.monotonic() + timeout
    while not test():
        if time.monotonic() > deadline:
            raise AssertionError("timeout: " + what)
        time.sleep(0.005)

def events(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line]

def steps(path):
    return len(path.read_text().splitlines()) if path.exists() else 0

results = []
children = []
try:
    for scenario in ("pause_stop", "reset", "jump"):
        state.update(ns=0, emit=True)
        time.sleep(0.08)
        out = root / scenario
        out.mkdir(exist_ok=True)
        sha = hashlib.sha256(plugin.read_bytes()).hexdigest()
        manifest = f'''[session]
id = "sim-{scenario}"
node = "n1"
roster = ["n1"]
period_ms = 1
epoch_ns = 10000000
[transport]
kind = "loopback"
[audit]
dir = "audit"
[clock_source]
kind = "ros1_sim"
plugin = "ros_io"
topic = "/clock"
expected_publisher = "/clock_authority"
world_instance_id = "isolated-ros-clock-{scenario}"
startup_timeout_wall_ms = 3000
stale_after_wall_ms = 100
max_advance_ns = 50000000
poll_wall_ms = 5
queue_capacity = 256
[[channel]]
name = "pose"
qos = "state"
[[plugin]]
name = "ros_io"
path = "{plugin}"
sha256 = "{sha}"
trigger = "both"
step_budget_ms = 20
wake_ms = 0.1
[plugin.config]
node_name = "sim_host_{scenario}"
pose_topic = "/integration/pose"
vision_pose_topic = "/integration/vision"
slice_ms = 0.5
queue_size = 10
frame_id = "world"
[plugin.bind]
pose = {{ channel = "pose" }}
vision_pose = {{ channel = "pose", from = ["n1"], latest = true }}
'''
        manifest_path = out / "manifest.toml"
        manifest_path.write_text(manifest)
        stdout = (out / "stdout.json").open("w")
        stderr = (out / "stderr.log").open("w")
        process = subprocess.Popen([str(host), "--manifest", str(manifest_path)], stdout=stdout, stderr=stderr)
        children.append(process)
        health_path = out / "audit/n1/health.jsonl"
        steps_path = out / "audit/n1/steps.jsonl"
        wait(lambda: any(e.get("event") == "epoch" for e in events(health_path)) or process.poll() is not None, "epoch")
        assert process.poll() is None, stderr.name
        assert not any(e.get("event") == "first_round" for e in events(health_path)), "zero must stay before epoch"
        state["ns"] = 10000000
        wait(lambda: pose_pub.get_num_connections() > 0 and sub.get_num_connections() > 0, "ordinary ROS input/output handshake after epoch activation")
        with lock:
            received.clear()
        for ns in range(11000000, 41000000, 1000000):
            state["ns"] = ns
            time.sleep(0.015)
        wait(lambda: len(received) > 2, "real pose input->host->vision ROS output")
        with lock:
            stamps = [row[1] for row in received]
            xs = [row[2] for row in received]
        assert all(0 < ns < 100000000 for ns in stamps), stamps
        assert all(x == 3.25 for x in xs), xs
        if scenario == "pause_stop":
            time.sleep(0.2)
            with lock:
                before_outputs = len(received)
            # Step writer may buffer; final step stamps are the authoritative check.
            before_time = state["ns"]
            time.sleep(1.15)
            with lock:
                assert len(received) == before_outputs, "ROS output progressed during frozen duplicate time"
            assert any(e.get("event") == "host_liveness" and not e["clock_runnable"] for e in events(health_path))
            state["emit"] = False
            time.sleep(0.15)
            with lock:
                assert len(received) == before_outputs, "ROS output progressed with no clock"
            state.update(ns=41000000, emit=True)
            for ns in range(41000000, 51000000, 1000000):
                state["ns"] = ns
                time.sleep(0.015)
            wait(lambda: len(received) > before_outputs, "bounded clock resume")
            state["emit"] = False
            time.sleep(0.15)
            started = time.monotonic()
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=2) == 0
            stop_ms = 1000 * (time.monotonic() - started)
        else:
            state["ns"] = 39000000 if scenario == "reset" else 120000000
            assert process.wait(timeout=3) == 1
            stop_ms = None
        stdout.close()
        stderr.close()
        summary = json.loads((out / "stdout.json").read_text())
        log_steps = events(steps_path)
        times = [s["t0"] for s in log_steps]
        assert len(times) == len(set(times)), "duplicate Session time executed twice"
        assert summary["plugins"][0]["consumed"] > 0
        assert summary["plugins"][0]["published"] > 0
        assert summary["plugins"][0]["state"] == "inactive"
        if scenario == "pause_stop":
            assert summary["aborted"] is None
        elif scenario == "reset":
            assert "backward" in summary["aborted"]
        else:
            assert "advance" in summary["aborted"]
        results.append({"scenario":scenario,"returncode":process.returncode,"stop_wall_ms":stop_ms,
            "steps":len(log_steps),"published":summary["plugins"][0]["published"],"consumed":summary["plugins"][0]["consumed"],
            "sim_stamp_range_ns":[min(stamps),max(stamps)],"aborted":summary["aborted"],
            "manifest_sha256":hashlib.sha256(manifest.encode()).hexdigest()})
    result = {"scope":"isolated actual ROS clock/pose source and host; no plant/DMPC/flight claim", "results":results,
        "host_sha256":hashlib.sha256(host.read_bytes()).hexdigest(),"ros_io_sha256":hashlib.sha256(plugin.read_bytes()).hexdigest()}
    (root / "ros-host-result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
finally:
    state["end"] = True
    thread.join(timeout=2)
    for process in children:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    rospy.signal_shutdown("integration complete")
