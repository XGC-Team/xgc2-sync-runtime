# Lightweight vehicle live check

`lightweight_vehicle_live.py` checks the ROS/Host boundary and the native FS150, Scout, and Mecanum lightweight vehicle models. It covers their ROS state/command topics, simulated MAVROS arm/mode services, motion, and pose/velocity timestamp alignment. It does not run a controller, planner, SITL, or Gazebo.

Source the ROS environment and prepare a reachable ROS master before running. The script does not start `roscore` or Docker. All four arguments are required; the host and plugin paths are resolved to absolute paths, and the output directory is created if needed.

Run from the `sync-runtime` directory with the built artifacts:

```sh
python3 plugins/ros-io/lightweight_vehicle_live.py \
  --host /absolute/path/to/xgc-rt-host \
  --plant /absolute/path/to/liblightweight_vehicle.so \
  --ros-io /absolute/path/to/libros_io.so \
  --output-dir /absolute/path/to/lightweight-vehicle-live-output
```

The output directory receives the generated manifest, Host log, model audit data, and `ros-model-result.json`.

An optional `sim_odometry_topic` publishes MAVROS-compatible `nav_msgs/Odometry`
from the existing measured `sim_pose` and `sim_velocity` channels. It pairs only
equal, increasing model timestamps. The pose uses `frame_id`; measured world
velocity is rotated into `sim_odometry_child_frame` (default `base_link`). It
does not differentiate positions or expose a new ABI port. The live check also
checks same-step stamps, measured poses and body twists at zero and 90-degree
yaw. `bash plugins/ros-io/run-odometry-test.sh` checks coordinate and rejection
cases without ROS; neither test certifies an algorithm experiment.

## Lightweight controller live check

`lightweight_controller_live.py` runs the FS150 plant and the real ctl-px4 SMC controller in separate `xgc-rt-host` processes. It sends the plant/controller channels over Zenoh TCP and uses the existing ROS master only for test commands, PVA setpoints, and observed ROS outputs. Source the ROS environment and prepare a reachable ROS master first; this script does not start ROS, MAVROS, or Docker. It checks the existing takeoff, 10-second trajectory tracking, endpoint error, and landing sequence. It does not validate DMPC or a complete experiment.

Run from the `sync-runtime` directory with the built artifacts and two unused loopback TCP ports:

```sh
python3 plugins/ros-io/lightweight_controller_live.py \
  --host /absolute/path/to/xgc-rt-host \
  --plant /absolute/path/to/liblightweight_vehicle.so \
  --controller /absolute/path/to/libctl_px4.so \
  --ros-io /absolute/path/to/libros_io.so \
  --plant-endpoint tcp/127.0.0.1:17441 \
  --controller-endpoint tcp/127.0.0.1:17442 \
  --output-dir /absolute/path/to/lightweight-controller-live-output
```

The output directory receives separate plant and controller manifests, Host logs, audit directories, and `controller-live-result.json` with both process IDs and exit codes.
