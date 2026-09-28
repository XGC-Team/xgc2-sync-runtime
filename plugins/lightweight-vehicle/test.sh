#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
math_include="${XGC2_MATH_INCLUDE:-$root/../math/include}"
eigen_include="${EIGEN_INCLUDE:-/usr/include/eigen3}"
test_dir="$(mktemp -d)"
trap 'rm -rf "$test_dir"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror \
  -I "$math_include" -isystem "$eigen_include" \
  "$root/plugins/lightweight-vehicle/model_test.cpp" -o "$test_dir/model-test"
"$test_dir/model-test"
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror \
  -I "$root/abi/include" -I "$root/plugins/common" \
  -I "$math_include" -isystem "$eigen_include" \
  "$root/plugins/lightweight-vehicle/lightweight_vehicle.cpp" \
  "$root/plugins/lightweight-vehicle/abi_test.cpp" -o "$test_dir/abi-test"
"$test_dir/abi-test"
