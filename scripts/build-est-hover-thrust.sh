#!/usr/bin/env bash
# Build the est-hover-thrust plugin (and, with a second argument, its
# replay reference program) from the unmodified upstream sources.
#
# Usage: scripts/build-est-hover-thrust.sh OUT.so [REFERENCE_BIN]
#
# Source locations default to the devops monorepo layout this repo lives in
# (devops/products/common/sync-runtime) and can be overridden:
#   XGC2_MATH_INCLUDE   libxgc2-math headers        (common/math/include)
#   XGC2_SM_ROOT        libxgc2-state-machine tree  (common/state-machine)
#   HTE_ROOT            hover_thrust_estimator pkg  (ros1/perception/estimator/hover-thrust/hover_thrust_estimator)
# CXX selects the C++17 compiler (default c++).
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
products="$(cd "$root/../.." && pwd)"
math="${XGC2_MATH_INCLUDE:-$products/common/math/include}"
sm="${XGC2_SM_ROOT:-$products/common/state-machine}"
hte="${HTE_ROOT:-$products/ros1/perception/estimator/hover-thrust/hover_thrust_estimator}"
cxx="${CXX:-c++}"
# Only the ROS-free parts of hover_thrust_estimator: the runtime and its
# state-machine states. Node, input producer and output consumers stay ROS.
srcs=("$sm/src/state_machine.cpp" "$hte/src/hover_thrust_estimator_runtime.cpp" "$hte"/src/state_machine/*.cpp)
flags=(-std=c++17 -O2 -fPIC -Wall -Wextra -I "$root/abi/include" -I "$math" -I "$sm/include" -I "$hte/include")
$cxx "${flags[@]}" -shared -fvisibility=hidden -o "$1" "${srcs[@]}" "$root/plugins/est-hover-thrust/est_hover_thrust.cpp"
if [[ $# -ge 2 ]]; then
  $cxx "${flags[@]}" -o "$2" "${srcs[@]}" "$root/plugins/est-hover-thrust/reference_replay.cpp"
fi
