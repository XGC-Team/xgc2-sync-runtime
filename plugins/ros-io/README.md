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
