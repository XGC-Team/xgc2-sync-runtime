#!/usr/bin/env bash
# Build the lightweight vehicle plugin without ROS or controller libraries.
#
# Usage: scripts/build-lightweight-vehicle.sh OUT.so
#
#   XGC2_MATH_INCLUDE  dir containing xgc2_math/ (default: sibling math/include)
#   EIGEN_INCLUDE      Eigen headers (default: /usr/include/eigen3)
#   CXX                C++17 compiler (default: c++)
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: scripts/build-lightweight-vehicle.sh OUT.so" >&2
  exit 2
fi

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
math_include="${XGC2_MATH_INCLUDE:-$root/../math/include}"
eigen="${EIGEN_INCLUDE:-/usr/include/eigen3}"
cxx="${CXX:-c++}"
out="$1"

if [[ ! -f "$math_include/xgc2_math/geometry/kinematics.hpp" ]]; then
  echo "build-lightweight-vehicle: XGC2_MATH_INCLUDE has no xgc2_math/geometry/kinematics.hpp: $math_include" >&2
  exit 1
fi
if [[ ! -f "$eigen/Eigen/Core" ]]; then
  echo "build-lightweight-vehicle: EIGEN_INCLUDE has no Eigen/Core: $eigen" >&2
  exit 1
fi

mkdir -p -- "$(dirname -- "$out")"
"$cxx" -std=c++17 -O2 -fPIC -fvisibility=hidden -Wall -Wextra -Werror -shared \
  -I "$root/abi/include" -I "$root/plugins/common" -I "$math_include" \
  -isystem "$eigen" \
  -o "$out" "$root/plugins/lightweight-vehicle/lightweight_vehicle.cpp"
