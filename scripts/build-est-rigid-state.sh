#!/usr/bin/env bash
# Build the est-rigid-state plugin (and, with a second argument, its replay
# reference program) from the unmodified upstream sources.
#
# Usage: scripts/build-est-rigid-state.sh OUT.so [REFERENCE_BIN]
#
# Source locations default to the devops monorepo layout and can be
# overridden:
#   XGC2_MATH_INCLUDE  libxgc2-math headers        (common/math/include)
#   XGC2_SM_ROOT       libxgc2-state-machine tree  (common/state-machine)
#   ESKF_ROOT          estimator package           (ros1/perception/estimator/rigid-state/estimator_vrpn_px4_rotor_state)
#   EIGEN_INCLUDE      Eigen headers               (/usr/include/eigen3)
# CXX selects the C++17 compiler (default c++).
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
products="$(cd "$root/../.." && pwd)"
math="${XGC2_MATH_INCLUDE:-$products/common/math/include}"
sm="${XGC2_SM_ROOT:-$products/common/state-machine}"
eskf="${ESKF_ROOT:-$products/ros1/perception/estimator/rigid-state/estimator_vrpn_px4_rotor_state}"
eigen="${EIGEN_INCLUDE:-/usr/include/eigen3}"
cxx="${CXX:-c++}"
# Only the ROS-free core (the package's *_core library): the runtime and its
# state-machine states. Node, input producer and output consumer stay ROS.
srcs=("$sm/src/state_machine.cpp" "$eskf/src/vrpn_px4_rotor_state_estimator_runtime.cpp" "$eskf"/src/state_machine/*.cpp)
flags=(-std=c++17 -O2 -fPIC -Wall -Wextra -I "$root/abi/include" -I "$math" -I "$sm/include" -I "$eskf/include" -isystem "$eigen")
$cxx "${flags[@]}" -shared -fvisibility=hidden -o "$1" "${srcs[@]}" "$root/plugins/est-rigid-state/est_rigid_state.cpp"
if [[ $# -ge 2 ]]; then
  $cxx "${flags[@]}" -o "$2" "${srcs[@]}" "$root/plugins/est-rigid-state/reference_replay.cpp"
fi
