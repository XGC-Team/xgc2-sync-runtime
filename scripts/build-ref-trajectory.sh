#!/usr/bin/env bash
# Build the ref-trajectory plugin on the ROS-free reference trajectory core.
#
# Usage: scripts/build-ref-trajectory.sh OUT.so
#
#   REF_CORE_LIB_DIR   dir with libmultirotor_reference_trajectory_core.so
#                      (required; e.g. <catkin ws>/devel/lib) — built from
#                      xgc2-multirotor-controller (feat/ros-free-reference or later)
#   REF_ROOT           the multirotor_reference_trajectory package source
#                      (default: sibling ros1/controller/multirotor-controller)
#   XGC2_PREFIX        prefix of libxgc2-math / libxgc2-state-machine (default /usr)
#   EIGEN_INCLUDE      Eigen 3 headers (default /usr/include/eigen3)
#   CXX                C++17 compiler; use the toolchain that built the core
#
# The build fails if any header the compiler actually read is a ROS header:
# the module is ROS-free by construction, not by convention.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
products="$(cd "$root/../.." && pwd)"
src="${REF_ROOT:-$products/ros1/controller/multirotor-controller/multirotor_reference_trajectory}"
core_lib="${REF_CORE_LIB_DIR:?set REF_CORE_LIB_DIR to the dir with libmultirotor_reference_trajectory_core.so}"
prefix="${XGC2_PREFIX:-/usr}"
eigen="${EIGEN_INCLUDE:-/usr/include/eigen3}"
cxx="${CXX:-c++}"
deps="$(mktemp)"
trap 'rm -f "$deps"' EXIT
$cxx -std=c++17 -O2 -fPIC -Wall -Wextra -shared -fvisibility=hidden -MD -MF "$deps" \
  -I "$root/abi/include" -I "$root/plugins/common" -I "$src/include" -I "$prefix/include" -isystem "$eigen" \
  -o "$1" "$root/plugins/ref-trajectory/ref_trajectory.cpp" \
  -L "$core_lib" -Wl,-rpath,"$core_lib" -lmultirotor_reference_trajectory_core \
  -L "$prefix/lib" -Wl,-rpath,"$prefix/lib" -lxgc2_state_machine
if tr ' \\' '\n\n' < "$deps" | grep -E '/include/ros/|_msgs/|ros1_utils|rosconsole|roscpp|/rostime|xmlrpcpp' >&2; then
  echo "build-ref-trajectory: the headers above are ROS headers; ref-trajectory must not use ROS" >&2
  rm -f "$1"
  exit 1
fi
