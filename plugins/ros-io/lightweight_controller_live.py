#!/usr/bin/env python3
"""Drive the real ctl-px4 SMC plugin against the lightweight FS150 plant."""

import argparse
import json
from pathlib import Path
import signal
import subprocess
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import String


STEP_NS = 100_000_000
TRAJECTORY_NS = 6_000_000_000
TRACKING_NS = 10_000_000_000
RUN_LIMIT_S = 60.0


def absolute_file(parser, option, value):
    try:
        path = value.expanduser().resolve(strict=True)
    except (OSError, RuntimeError) as error:
        parser.error(f"{option} does not resolve to an existing file: {error}")
    if not path.is_file():
        parser.error(f"{option} must name a file: {path}")
    return path


def make_manifest(plant_path, controller_path, ros_io_path, audit_dir, epoch_ns):
    quoted = lambda value: json.dumps(str(value))
    return f'''[session]
id = "private-lightweight-controller-live"
node = "uav1"
roster = ["uav1"]
period_ms = 1
epoch_ns = {epoch_ns}
run_for_ms = 60000

[transport]
kind = "loopback"

[audit]
dir = {quoted(audit_dir)}

[[channel]]
name = "pose"
qos = "state"
[[channel]]
name = "velocity"
qos = "state"
[[channel]]
name = "imu"
qos = "state"
[[channel]]
name = "fcu_state"
qos = "state"
[[channel]]
name = "setpoint"
qos = "control"
[[channel]]
name = "fcu_request"
qos = "event"
[[channel]]
name = "command"
qos = "event"
[[channel]]
name = "alg_setpoint"
qos = "control"
[[channel]]
name = "status"
qos = "state"

[[plugin]]
name = "plant"
path = {quoted(plant_path)}
trigger = "on_round"
config = {{ model = "fs150", epoch_ns = {epoch_ns}, step_ms = 1, output_ms = 10, initial_pose = [0.0, 0.0, 0.0, 0.0] }}
bind = {{ pose = {{ channel = "pose" }}, velocity = {{ channel = "velocity" }}, imu = {{ channel = "imu" }}, fcu_state = {{ channel = "fcu_state" }}, setpoint = {{ channel = "setpoint", from = ["uav1"] }}, fcu_request = {{ channel = "fcu_request", from = ["uav1"] }} }}

[[plugin]]
name = "controller"
path = {quoted(controller_path)}
trigger = "on_round"
config = {{ time_source = "session", tracking_backend = "smc", takeoff_altitude = 1, planning_period = 0.1 }}
bind = {{ local_pose = {{ channel = "pose", from = ["uav1"] }}, vrpn_pose = {{ channel = "pose", from = ["uav1"] }}, local_velocity = {{ channel = "velocity", from = ["uav1"] }}, imu = {{ channel = "imu", from = ["uav1"] }}, fcu_state = {{ channel = "fcu_state", from = ["uav1"] }}, command = {{ channel = "command", from = ["uav1"] }}, alg_setpoint = {{ channel = "alg_setpoint", from = ["uav1"] }}, setpoint = {{ channel = "setpoint" }}, fcu_request = {{ channel = "fcu_request" }}, status = {{ channel = "status" }} }}

[[plugin]]
name = "ros-io"
path = {quoted(ros_io_path)}
trigger = "on_round"
config = {{ node_name = "xgc_ros_io_uav1", frame_id = "world", sim_pose_topic = "/uav1/mavros/local_position/pose", sim_velocity_topic = "/uav1/mavros/local_position/velocity_local", sim_fcu_state_topic = "/uav1/mavros/state", command_topic = "/command", alg_setpoint_topic = "/uav1/alg/setpoint_raw/local", status_topic = "/uav1/custom/statustext" }}
bind = {{ sim_pose = {{ channel = "pose", from = ["uav1"] }}, sim_velocity = {{ channel = "velocity", from = ["uav1"] }}, sim_fcu_state = {{ channel = "fcu_state", from = ["uav1"] }}, command = {{ channel = "command" }}, alg_setpoint = {{ channel = "alg_setpoint" }}, status = {{ channel = "status", from = ["uav1"] }} }}
'''


def position_target(elapsed_ns, stamp_ns):
    q = min(1.0, max(0.0, float(elapsed_ns) / float(TRAJECTORY_NS)))
    message = PositionTarget()
    message.header.stamp = rospy.Time(stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000)
    message.header.frame_id = "map"
    message.coordinate_frame = 1
    message.type_mask = 3072
    message.position.x = 10.0 * q**3 - 15.0 * q**4 + 6.0 * q**5
    message.position.y = 0.0
    message.position.z = 1.0
    if q < 1.0:
        message.velocity.x = (30.0 * q**2 - 60.0 * q**3 + 30.0 * q**4) / 6.0
        message.acceleration_or_force.x = (60.0 * q - 180.0 * q**2 + 120.0 * q**3) / 36.0
    return message


def main():
    parser = argparse.ArgumentParser(
        description="Exercise ctl-px4 SMC with the lightweight FS150 plant through real ROS topics."
    )
    parser.add_argument("--host", required=True, type=Path, help="path to xgc-rt-host")
    parser.add_argument("--plant", required=True, type=Path, help="path to liblightweight_vehicle.so")
    parser.add_argument("--controller", required=True, type=Path, help="path to libctl_px4.so")
    parser.add_argument("--ros-io", required=True, type=Path, help="path to libros_io.so")
    parser.add_argument("--output-dir", required=True, type=Path, help="directory for manifest, logs, and result JSON")
    args = parser.parse_args()

    host_path = absolute_file(parser, "--host", args.host)
    plant_path = absolute_file(parser, "--plant", args.plant)
    controller_path = absolute_file(parser, "--controller", args.controller)
    ros_io_path = absolute_file(parser, "--ros-io", args.ros_io)
    work = args.output_dir.expanduser().resolve()
    work.mkdir(parents=True, exist_ok=True)
    audit_dir = work / "audit"
    audit_dir.mkdir(parents=True, exist_ok=True)

    started_wall_ns = time.time_ns()
    epoch_ns = started_wall_ns + 3_000_000_000
    deadline = time.monotonic() + RUN_LIMIT_S
    manifest_path = work / "controller-live.toml"
    manifest_path.write_text(
        make_manifest(plant_path, controller_path, ros_io_path, audit_dir, epoch_ns),
        encoding="utf-8",
    )

    capture_lock = threading.Lock()
    capture = {
        "controller_state": None,
        "controller_states": [],
        "fcu_state": None,
        "fcu_state_changes": [],
        "poses": [],
        "velocities": [],
        "actions": [],
    }

    def stamp_ns(message):
        return message.header.stamp.to_nsec()

    def on_controller_state(message):
        received_ns = time.time_ns()
        with capture_lock:
            if message.data != capture["controller_state"]:
                capture["controller_states"].append(
                    {"state": message.data, "received_wall_time_ns": received_ns}
                )
                capture["controller_state"] = message.data

    def on_fcu_state(message):
        received_ns = time.time_ns()
        current = {
            "connected": bool(message.connected),
            "armed": bool(message.armed),
            "mode": message.mode,
            "stamp_ns": stamp_ns(message),
            "received_wall_time_ns": received_ns,
        }
        with capture_lock:
            previous = capture["fcu_state"]
            if previous is None or any(
                current[key] != previous[key] for key in ("connected", "armed", "mode")
            ):
                capture["fcu_state_changes"].append(current)
            capture["fcu_state"] = current

    def on_pose(message):
        item = {
            "stamp_ns": stamp_ns(message),
            "received_wall_time_ns": time.time_ns(),
            "frame_id": message.header.frame_id,
            "position": [message.pose.position.x, message.pose.position.y, message.pose.position.z],
        }
        with capture_lock:
            capture["poses"].append(item)

    def on_velocity(message):
        item = {
            "stamp_ns": stamp_ns(message),
            "received_wall_time_ns": time.time_ns(),
            "frame_id": message.header.frame_id,
            "linear": [message.twist.linear.x, message.twist.linear.y, message.twist.linear.z],
        }
        with capture_lock:
            capture["velocities"].append(item)

    host_log_path = work / "controller-live-host.log"
    host_log = host_log_path.open("w", encoding="utf-8")
    host = subprocess.Popen(
        [str(host_path), "--manifest", str(manifest_path)],
        stdout=host_log,
        stderr=subprocess.STDOUT,
    )
    host_shutdown_escalation = None
    endpoint = None
    endpoint_error = None
    failure = None

    def check_deadline(wait_description=None):
        if host.poll() is not None:
            raise RuntimeError(f"xgc-rt-host exited with status {host.returncode}")
        if rospy.is_shutdown():
            raise RuntimeError("ROS node shut down before the flight sequence completed")
        if time.monotonic() >= deadline:
            if wait_description:
                raise TimeoutError(f"timeout waiting for {wait_description}")
            raise TimeoutError("controller live fixture exceeded its 60 second limit")

    def wait_for(predicate, description):
        while True:
            check_deadline(description)
            if predicate():
                return
            time.sleep(0.01)

    def current(name):
        with capture_lock:
            value = capture[name]
            if name in ("poses", "velocities"):
                return value[-1] if value else None
            return value

    def record_action(action, stamp_ns_value=None):
        with capture_lock:
            capture["actions"].append(
                {"action": action, "wall_time_ns": time.time_ns(), "target_stamp_ns": stamp_ns_value}
            )

    def result_payload():
        with capture_lock:
            controller_states = list(capture["controller_states"])
            fcu_state_changes = list(capture["fcu_state_changes"])
            poses = list(capture["poses"])
            velocities = list(capture["velocities"])
            actions = list(capture["actions"])
            final_fcu_state = capture["fcu_state"]
        return {
            "scope": "lightweight FS150 plant + existing ctl-px4 SMC + ros_io; no DMPC/planner/SITL/Gazebo validation",
            "passed": failure is None and endpoint_error is not None and endpoint_error < 0.03,
            "failure": failure,
            "epoch_ns": epoch_ns,
            "input_paths": {
                "host": str(host_path),
                "plant": str(plant_path),
                "controller": str(controller_path),
                "ros_io": str(ros_io_path),
            },
            "controller_state_sequence": controller_states,
            "fcu_state_changes": fcu_state_changes,
            "final_fcu_state": final_fcu_state,
            "endpoint": endpoint,
            "endpoint_error_m": endpoint_error,
            "pose_output_stamps_ns": [sample["stamp_ns"] for sample in poses],
            "velocity_output_stamps_ns": [sample["stamp_ns"] for sample in velocities],
            "pose_outputs": poses,
            "velocity_outputs": velocities,
            "actions": actions,
            "host_exit_code": host.returncode,
            "host_shutdown_escalation": host_shutdown_escalation,
        }

    def print_summary(result):
        print(
            json.dumps(
                {
                    "passed": result["passed"],
                    "failure": result["failure"],
                    "endpoint_error_m": result["endpoint_error_m"],
                    "state_sequences": {
                        "controller": result["controller_state_sequence"],
                        "fcu": result["fcu_state_changes"],
                    },
                    "exit_summary": {
                        "host_exit_code": result["host_exit_code"],
                        "host_shutdown_escalation": result["host_shutdown_escalation"],
                    },
                },
                indent=2,
                sort_keys=True,
            )
        )

    try:
        rospy.init_node("private_lightweight_controller_live", anonymous=False)
        rospy.Subscriber("/uav1/custom/statustext", String, on_controller_state, queue_size=100)
        rospy.Subscriber("/uav1/mavros/state", State, on_fcu_state, queue_size=100)
        rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, on_pose, queue_size=100)
        rospy.Subscriber("/uav1/mavros/local_position/velocity_local", TwistStamped, on_velocity, queue_size=100)
        command = rospy.Publisher("/command", String, queue_size=10)
        reference = rospy.Publisher("/uav1/alg/setpoint_raw/local", PositionTarget, queue_size=10)

        wait_for(
            lambda: command.get_num_connections() > 0
            and reference.get_num_connections() > 0
            and current("fcu_state") is not None
            and current("poses") is not None
            and current("velocities") is not None,
            "ROS edge connections and initial plant feedback",
        )
        wait_for(lambda: current("controller_state") == "Ready", "controller Ready")
        record_action("takeoff")
        command.publish(String(data="takeoff"))

        wait_for(lambda: current("controller_state") == "Hover", "controller Hover")
        custom_start_ns = time.time_ns()
        record_action("custom1", custom_start_ns)
        command.publish(String(data="custom1"))

        next_setpoint_ns = custom_start_ns
        while True:
            check_deadline()
            now_ns = time.time_ns()
            elapsed_ns = max(0, now_ns - custom_start_ns)
            if now_ns >= next_setpoint_ns:
                setpoint = position_target(elapsed_ns, now_ns)
                reference.publish(setpoint)
                if elapsed_ns >= TRACKING_NS:
                    target_stamp_ns = setpoint.header.stamp.to_nsec()
                    wait_for(
                        lambda: current("poses") is not None
                        and current("poses")["stamp_ns"] >= target_stamp_ns,
                        "pose output at the 10 second tracking endpoint",
                    )
                    latest_pose = current("poses")
                    if current("controller_state") != "Custom1":
                        raise AssertionError(
                            f"controller left Custom1 before endpoint measurement: {current('controller_state')}"
                        )
                    endpoint = {
                        "stamp_ns": latest_pose["stamp_ns"],
                        "received_wall_time_ns": latest_pose["received_wall_time_ns"],
                        "position": latest_pose["position"],
                    }
                    endpoint_error = (
                        (latest_pose["position"][0] - 1.0) ** 2
                        + latest_pose["position"][1] ** 2
                        + (latest_pose["position"][2] - 1.0) ** 2
                    ) ** 0.5
                    record_action("land")
                    command.publish(String(data="land"))
                    break
                next_setpoint_ns += STEP_NS
                if next_setpoint_ns <= now_ns:
                    next_setpoint_ns = now_ns + STEP_NS
            time.sleep(0.005)

        if endpoint_error >= 0.03:
            raise AssertionError(f"10 second endpoint error {endpoint_error:.6f} m is not below 0.03 m")

        wait_for(
            lambda: current("fcu_state") is not None
            and not current("fcu_state")["armed"]
            and current("poses") is not None
            and current("poses")["position"][2] < 0.03,
            "plant ground state and armed=false",
        )
        record_action("landed_from_plant_feedback")
    except BaseException as error:
        failure = f"{type(error).__name__}: {error}"
        raise
    finally:
        if host.poll() is None:
            try:
                host.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            if host.poll() is None:
                try:
                    host.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    host_shutdown_escalation = "terminate"
                    host.terminate()
                    try:
                        host.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        host_shutdown_escalation = "kill"
                        host.kill()
                        host.wait()
        host_log.close()
        result = result_payload()
        (work / "controller-live-result.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print_summary(result)


if __name__ == "__main__":
    main()
