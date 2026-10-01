#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandLong, SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def absolute_file(parser, option, value):
    try:
        path = value.expanduser().resolve(strict=True)
    except (OSError, RuntimeError) as error:
        parser.error(f'{option} does not resolve to an existing file: {error}')
    if not path.is_file():
        parser.error(f'{option} must name a file: {path}')
    return path


def main():
    parser = argparse.ArgumentParser(
        description='Exercise the ROS edge and three native lightweight vehicle models.'
    )
    parser.add_argument('--host', required=True, type=Path, help='absolute path to xgc-rt-host')
    parser.add_argument('--plant', required=True, type=Path, help='absolute path to liblightweight_vehicle.so')
    parser.add_argument('--ros-io', required=True, type=Path, help='absolute path to libros_io.so')
    parser.add_argument('--output-dir', required=True, type=Path, help='directory for manifest, logs, and result JSON')
    args = parser.parse_args()

    host_path = absolute_file(parser, '--host', args.host)
    plant_path = absolute_file(parser, '--plant', args.plant)
    ros_io_path = absolute_file(parser, '--ros-io', args.ros_io)
    work = args.output_dir.expanduser().resolve()
    work.mkdir(parents=True, exist_ok=True)
    audit_dir = work / 'ros-model-audit'
    audit_dir.mkdir(parents=True, exist_ok=True)

    epoch = time.time_ns() + 3_000_000_000
    names = ['fs150', 'scout', 'mecanum']
    manifest = (
        f'[session]\n'
        f'id = "private-lightweight-ros"\n'
        f'node = "models"\n'
        f'roster = ["models"]\n'
        f'period_ms = 1\n'
        f'epoch_ns = {epoch}\n'
        f'run_for_ms = 16000\n'
        f'[transport]\n'
        f'kind = "loopback"\n'
        f'[audit]\n'
        f'dir = {json.dumps(str(audit_dir))}\n'
    )
    ports = [('pose', 'state'), ('velocity', 'state'), ('imu', 'state'), ('state', 'state'), ('request', 'event'), ('control', 'control')]
    for name in names:
        for port, qos in ports:
            manifest += f'\n[[channel]]\nname = "{name}-{port}"\nqos = "{qos}"\n'
    for name in names:
        inputs = ('setpoint', 'control') if name == 'fs150' else ('cmd_vel', 'control')
        bindings = [f'{inputs[0]}={{channel="{name}-{inputs[1]}",from=["models"]}}', f'pose={{channel="{name}-pose"}}', f'velocity={{channel="{name}-velocity"}}']
        if name == 'fs150':
            bindings += [f'fcu_request={{channel="{name}-request",from=["models"]}}', f'imu={{channel="{name}-imu"}}', f'fcu_state={{channel="{name}-state"}}']
        yaw = 0.0 if name == 'fs150' else 1.5707963267948966
        manifest += (
            f'\n[[plugin]]\nname = "plant-{name}"\n'
            f'path = {json.dumps(str(plant_path))}\n'
            f'trigger = "on_round"\n'
            f'config = {{ model="{name}", epoch_ns={epoch}, step_ms=1, output_ms=10, initial_pose=[0.0,0.0,0.0,{yaw}] }}\n'
            f'bind = {{ {",".join(bindings)} }}\n'
        )
        config = [f'sim_pose_topic="/{name}/pose"', f'sim_velocity_topic="/{name}/velocity"',
                  f'sim_odometry_topic="/{name}/odom"', f'sim_odometry_child_frame="{name}/base_link"',
                  'node_name="private_lightweight_models"', 'frame_id="world"']
        bindings = [f'sim_pose={{channel="{name}-pose",from=["models"]}}', f'sim_velocity={{channel="{name}-velocity",from=["models"]}}']
        if name == 'fs150':
            config += ['alg_setpoint_topic="/fs150/setpoint"', 'sim_fcu_request_topic="/fs150/mavros"', 'sim_fcu_state_topic="/fs150/state"', 'sim_imu_topic="/fs150/imu"']
            bindings += ['alg_setpoint={channel="fs150-control"}', 'sim_fcu_request={channel="fs150-request"}', 'sim_fcu_state={channel="fs150-state",from=["models"]}', 'sim_imu={channel="fs150-imu",from=["models"]}']
        else:
            config += [f'cmd_vel_topic="/{name}/cmd_vel"']
            bindings += [f'cmd_vel={{channel="{name}-control"}}']
        manifest += (
            f'\n[[plugin]]\nname = "ros-{name}"\n'
            f'path = {json.dumps(str(ros_io_path))}\n'
            f'trigger = "on_round"\n'
            f'config = {{ {",".join(config)} }}\n'
            f'bind = {{ {",".join(bindings)} }}\n'
        )
    manifest_path = work / 'ros-model.toml'
    manifest_path.write_text(manifest)
    out = (work / 'ros-model-host.log').open('w')
    host = subprocess.Popen([str(host_path), '--manifest', str(manifest_path)], stdout=out, stderr=subprocess.STDOUT)
    try:
        rospy.init_node('private_model_probe', anonymous=False)
        poses, velocities, odometry, states, imus = {}, {}, {}, [], []

        def save_pose(message, name):
            poses.setdefault(name, []).append(message)

        def save_velocity(message, name):
            velocities.setdefault(name, []).append(message)

        def save_odometry(message, name):
            odometry.setdefault(name, []).append(message)

        subscriptions = []
        for name in names:
            subscriptions.append(rospy.Subscriber('/' + name + '/pose', PoseStamped, save_pose, name))
            subscriptions.append(rospy.Subscriber('/' + name + '/velocity', TwistStamped, save_velocity, name))
            subscriptions.append(rospy.Subscriber('/' + name + '/odom', Odometry, save_odometry, name))
        subscriptions.append(rospy.Subscriber('/fs150/state', State, states.append))
        subscriptions.append(rospy.Subscriber('/fs150/imu', Imu, imus.append))
        publishers = {name:rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size=1) for name in names[1:]}
        setpoint = rospy.Publisher('/fs150/setpoint', PositionTarget, queue_size=1)

        def wait_for(predicate, description, timeout=5):
            until = time.monotonic() + timeout
            while not predicate():
                if host.poll() is not None:
                    raise AssertionError('host exited: ' + str(host.returncode))
                if time.monotonic() > until:
                    raise AssertionError('timeout: ' + description)
                time.sleep(.01)

        wait_for(lambda:all(poses.get(n) and velocities.get(n) and odometry.get(n) for n in names) and states and imus, 'all model feedback', 8)
        assert states[-1].connected and not states[-1].armed
        assert imus[-1].orientation_covariance[0] == -1
        for name in names:
            assert abs(poses[name][-1].pose.position.x) < 1e-9
            assert poses[name][-1].header.frame_id == 'world'
        rospy.wait_for_service('/fs150/mavros/cmd/command', timeout=3)
        arm = rospy.ServiceProxy('/fs150/mavros/cmd/command', CommandLong)
        mode = rospy.ServiceProxy('/fs150/mavros/set_mode', SetMode)
        assert not arm(command=176).success
        assert arm(command=400, param1=1).success
        # Like PX4, the plant refuses OFFBOARD without a live setpoint stream.
        hold = PositionTarget()
        hold.coordinate_frame = 1
        hold.type_mask = 3135

        def streaming_offboard():
            hold.header.stamp = rospy.Time.now()
            setpoint.publish(hold)
            return states[-1].armed and states[-1].mode == 'OFFBOARD'

        for unused in range(10):
            streaming_offboard()
            time.sleep(.05)
        assert mode(custom_mode='OFFBOARD').mode_sent
        wait_for(streaming_offboard, 'actual model mode and arm feedback')
        scout = Twist()
        scout.linear.x = .5
        mecanum = Twist()
        mecanum.linear.y = .5
        acceleration = PositionTarget()
        acceleration.coordinate_frame = 1
        acceleration.type_mask = 3135
        acceleration.acceleration_or_force.x = .2
        acceleration.acceleration_or_force.z = .2
        start = time.monotonic()
        for unused in range(20):
            publishers['scout'].publish(scout)
            publishers['mecanum'].publish(mecanum)
            acceleration.header.stamp = rospy.Time.now()
            setpoint.publish(acceleration)
            time.sleep(.1)
        elapsed = time.monotonic() - start
        assert poses['scout'][-1].pose.position.y > .7 and abs(poses['scout'][-1].pose.position.x) < .01
        assert poses['mecanum'][-1].pose.position.x < -.7 and abs(poses['mecanum'][-1].pose.position.y) < .01
        assert poses['fs150'][-1].pose.position.x > .2 and poses['fs150'][-1].pose.position.z > .2
        assert velocities['fs150'][-1].twist.linear.x > .3
        for name in names:
            ps = {m.header.stamp.to_nsec() for m in poses[name]}
            vs = {m.header.stamp.to_nsec() for m in velocities[name]}
            assert len(ps & vs) > 100, (name, len(ps & vs))
            assert all(a.header.stamp < b.header.stamp for a, b in zip(poses[name], poses[name][1:]))
            by_pose = {m.header.stamp.to_nsec(): m for m in poses[name]}
            by_velocity = {m.header.stamp.to_nsec(): m for m in velocities[name]}
            matches = [m for m in odometry[name] if m.header.stamp.to_nsec() in ps & vs]
            assert len(matches) > 100, (name, 'same-step odometry', len(matches))
            assert all(a.header.stamp < b.header.stamp for a, b in zip(odometry[name], odometry[name][1:]))
            for m in matches:
                stamp = m.header.stamp.to_nsec()
                assert m.header.frame_id == 'world' and m.child_frame_id == name + '/base_link'
                assert m.pose.pose == by_pose[stamp].pose
                actual, measured = m.twist.twist.linear, by_velocity[stamp].twist.linear
                # Independently known fixture orientations: FS150 yaw=0;
                # both ground vehicles yaw=90 degrees. Twist is BODY, not world.
                expected = (measured.x, measured.y, measured.z) if name == 'fs150' else (measured.y, -measured.x, measured.z)
                assert max(abs(a-b) for a, b in zip((actual.x, actual.y, actual.z), expected)) < 1e-9
        for pub in publishers.values():
            pub.publish(Twist())
        # An in-air disarm is refused, as by PX4; AUTO.LAND lands and disarms.
        assert arm(command=400, param1=0).success
        time.sleep(.2)
        assert states[-1].armed
        assert mode(custom_mode='AUTO.LAND').mode_sent
        wait_for(lambda:not states[-1].armed, 'AUTO.LAND touchdown disarm')
        wait_for(lambda:poses['fs150'][-1].pose.position.z == 0.0, 'FS150 on the ground')
        assert states[-1].mode == 'AUTO.LAND'
        result = {
            'scope':'three real native plants and ROS edges; no planner/controller/SITL/Gazebo',
            'elapsed':elapsed,
            'robots':{
                n:{'samples':len(poses[n]),'odometrySamples':len(odometry[n]),'last_position':[poses[n][-1].pose.position.x, poses[n][-1].pose.position.y, poses[n][-1].pose.position.z]}
                for n in names
            },
        }
        (work / 'ros-model-result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result))
    finally:
        if host.poll() is None:
            host.send_signal(signal.SIGINT)
        code = host.wait(timeout=10)
        out.close()
        assert code == 0, (code, (work / 'ros-model-host.log').read_text()[-4000:])


if __name__ == '__main__':
    main()
