#!/usr/bin/env bash
# Build the plan-dmpc plugin on the ROS-free TRO DMPC planner libraries.
#
# Usage: scripts/build-plan-dmpc.sh OUT.so
#
#   DMPC_LIB_DIR       dir with libformation_generator_dmpc_{core,params,config}.so
#                      (required): the academic formation_generator package's
#                      standalone build (standalone/CMakeLists.txt) or its catkin
#                      devel lib dir
#   FORMATION_GENERATOR_ROOT  the formation_generator package source (required;
#                      <academic>/ros1_ws/src/planner/formation_generator); the
#                      convex_geometry, formation_patterns, math_utils and
#                      reference_trajectory headers are read from its workspace
#   ACADOS_ROOT        acados install (default /usr/local)
#   YAML_CPP_LIB_DIR   dir with libyaml-cpp.so, when not on the default path
#   EIGEN_INCLUDE      Eigen 3 headers (default /usr/include/eigen3)
#   CXX                C++17 compiler; use the toolchain that built the libraries
#
# The build fails if any header the compiler actually read is a ROS header
# (ros/, *_msgs/, rosconsole...): the module is ROS-free by construction, not
# by convention.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
lib="${DMPC_LIB_DIR:?set DMPC_LIB_DIR to the dir with libformation_generator_dmpc_config.so}"
fg="${FORMATION_GENERATOR_ROOT:?set FORMATION_GENERATOR_ROOT to the formation_generator package source}"
common="$(cd "$fg/../../common" && pwd)"
acados="${ACADOS_ROOT:-/usr/local}"
eigen="${EIGEN_INCLUDE:-/usr/include/eigen3}"
cxx="${CXX:-c++}"
rpath_dirs=("$lib" "$acados/lib")
if [ -n "${YAML_CPP_LIB_DIR:-}" ]; then rpath_dirs+=("$YAML_CPP_LIB_DIR"); fi
link_dirs=()
for d in "${rpath_dirs[@]}"; do link_dirs+=(-Wl,-rpath,"$d" -Wl,-rpath-link,"$d"); done
deps="$(mktemp)"
trap 'rm -f "$deps"' EXIT
$cxx -std=c++17 -O2 -fPIC -Wall -Wextra -shared -fvisibility=hidden -MD -MF "$deps" \
  -DACADOS_WITH_QPOASES -DUSE_ACADOS_TYPES -DACADOS_WITH_OSQP \
  -I "$root/abi/include" -I "$root/plugins/common" -I "$fg/include" \
  -I "$common/convex_geometry/include" -I "$common/formation_patterns/include" \
  -I "$common/math_utils/include" -I "$common/reference_trajectory/include" \
  -I "$acados/include" -I "$acados/include/blasfeo/include" -I "$acados/include/hpipm/include" \
  -isystem "$eigen" \
  -o "$1" "$root/plugins/plan-dmpc/plan_dmpc.cpp" \
  -L "$lib" -lformation_generator_dmpc_config -lformation_generator_dmpc_params -lformation_generator_dmpc_core \
  "${link_dirs[@]}" -Wl,--no-undefined
if tr ' \\' '\n\n' < "$deps" | grep -E '/include/ros/|_msgs/|ros1_utils|rosconsole|roscpp|/rostime|xmlrpcpp' >&2; then
  echo "build-plan-dmpc: the headers above are ROS headers; plan-dmpc must not use ROS" >&2
  rm -f "$1"
  exit 1
fi
