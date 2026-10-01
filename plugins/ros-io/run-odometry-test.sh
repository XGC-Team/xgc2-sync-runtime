#!/usr/bin/env bash
# ROS-free coordinate and sampling checks; the ROS/Host check is
# lightweight_vehicle_live.py against an independently prepared ROS master.
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
output="$(mktemp -d)"
trap 'rm -rf -- "$output"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror \
  -I "$root/abi/include" -I "$root/plugins/ros-io" \
  "$root/plugins/ros-io/sim_odometry_test.cpp" -o "$output/odometry-test"
"$output/odometry-test"
