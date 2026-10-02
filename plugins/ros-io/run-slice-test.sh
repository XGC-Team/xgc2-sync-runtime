#!/usr/bin/env bash
# ROS-free checks of ros_io's per-step ROS service arithmetic (ros_slice.hpp).
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
output="$(mktemp -d)"
trap 'rm -rf -- "$output"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror \
  -I "$root/plugins/ros-io" \
  "$root/plugins/ros-io/ros_slice_test.cpp" -o "$output/slice-test"
"$output/slice-test"
