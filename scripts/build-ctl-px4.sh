#!/usr/bin/env bash
# Build the ctl-px4 plugin on the ROS-free PX4 controller core.
#
# Usage: scripts/build-ctl-px4.sh OUT.so
#
#   PX4_CORE_LIB_DIR   dir with libpx4_multirotor_controller_core.so (required;
#                      e.g. <catkin ws>/devel/lib) — built from
#                      xgc2-multirotor-controller (feat/ros-free-core or later)
#   PX4_CONTROLLER_ROOT the px4_multirotor_controller package source
#                      (default: sibling ros1/controller/multirotor-controller)
#   XGC2_PREFIX        prefix of libxgc2-math / libxgc2-state-machine (default /usr)
#   XGC2_MATH_INCLUDE_OVERRIDE
#                      optional include dir that contains
#                      xgc2_math/control/smc_tracking_controller.hpp.
#                      Empty uses the headers under XGC2_PREFIX (APT math >= 0.5.9).
#   ACADOS_ROOT        acados install (default /usr/local)
#   EIGEN_INCLUDE      Eigen 3 headers (default /usr/include/eigen3)
#   CXX                C++17 compiler; use the toolchain that built the core
#
# The build fails if any header the compiler actually read is a ROS header
# (ros/, *_msgs/, ros1_utils, rosconsole...): the module is ROS-free by
# construction, not by convention.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
products="$(cd "$root/../.." && pwd)"
ctrl="${PX4_CONTROLLER_ROOT:-$products/ros1/controller/multirotor-controller/px4_multirotor_controller}"
core_lib="${PX4_CORE_LIB_DIR:?set PX4_CORE_LIB_DIR to the dir with libpx4_multirotor_controller_core.so}"
prefix="${XGC2_PREFIX:-/usr}"
math_include="${XGC2_MATH_INCLUDE_OVERRIDE:-}"
math_flag=()
if [[ -n "$math_include" ]]; then
  if [[ ! -f "$math_include/xgc2_math/control/smc_tracking_controller.hpp" ]]; then
    echo "build-ctl-px4: XGC2_MATH_INCLUDE_OVERRIDE has no smc_tracking_controller.hpp" >&2
    exit 1
  fi
  math_flag=(-I "$math_include")
fi
acados="${ACADOS_ROOT:-/usr/local}"
eigen="${EIGEN_INCLUDE:-/usr/include/eigen3}"
cxx="${CXX:-c++}"
deps="$(mktemp)"
trap 'rm -f "$deps"' EXIT
$cxx -std=c++17 -O2 -fPIC -Wall -Wextra -shared -fvisibility=hidden -MD -MF "$deps" \
  "${math_flag[@]}" -I "$root/abi/include" -I "$root/plugins/common" \
  -I "$ctrl/include" -I "$ctrl/generated/nmpc/uav_nmpc" \
  -I "$acados" -I "$acados/include" -I "$acados/include/blasfeo/include" -I "$acados/include/hpipm/include" \
  -I "$acados/interfaces" -I "$prefix/include" -isystem "$eigen" \
  -o "$1" "$root/plugins/ctl-px4/ctl_px4.cpp" \
  -L "$core_lib" -Wl,-rpath,"$core_lib" -lpx4_multirotor_controller_core -lpx4_multirotor_controller_uav_nmpc_runtime \
  -L "$prefix/lib" -Wl,-rpath,"$prefix/lib" -Wl,-rpath,"$acados/lib"
if tr ' \\' '\n\n' < "$deps" | grep -E '/include/ros/|_msgs/|ros1_utils|rosconsole|roscpp|/rostime|xmlrpcpp' >&2; then
  echo "build-ctl-px4: the headers above are ROS headers; ctl-px4 must not use ROS" >&2
  rm -f "$1"
  exit 1
fi
