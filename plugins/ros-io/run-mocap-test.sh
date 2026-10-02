#!/usr/bin/env bash
# ROS-free checks for publication-side simulated mocap measurements.
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
output="$(mktemp -d)"
trap 'rm -rf -- "$output"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror \
  -I "$root/abi/include" -I "$root/plugins/ros-io" \
  "$root/plugins/ros-io/sim_mocap_test.cpp" -o "$output/mocap-test"
"$output/mocap-test"
