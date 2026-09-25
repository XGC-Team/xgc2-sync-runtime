#!/usr/bin/env bash
# Build the ctl-dfbc plugin (and optionally its replay reference).
# Usage: scripts/build-ctl-dfbc.sh OUT.so [REFERENCE_BIN]
#   XGC2_MATH_INCLUDE  libxgc2-math headers (default: sibling common/math/include)
#   EIGEN_INCLUDE      Eigen 3 headers      (default: /usr/include/eigen3)
#   CXX                C++17 compiler       (default: c++)
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
products="$(cd "$root/../.." && pwd)"
math="${XGC2_MATH_INCLUDE:-$products/common/math/include}"
eigen="${EIGEN_INCLUDE:-/usr/include/eigen3}"
cxx="${CXX:-c++}"
flags=(-std=c++17 -O2 -fPIC -Wall -Wextra -I "$root/abi/include" -I "$root/plugins/ctl-dfbc" -I "$math" -isystem "$eigen")
$cxx "${flags[@]}" -shared -fvisibility=hidden -o "$1" "$root/plugins/ctl-dfbc/ctl_dfbc.cpp"
if [[ $# -ge 2 ]]; then
  $cxx "${flags[@]}" -o "$2" "$root/plugins/ctl-dfbc/reference_replay.cpp"
fi
