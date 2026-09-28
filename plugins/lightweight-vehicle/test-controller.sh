#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /absolute/path/to/libctl_px4.so (with its libraries on LD_LIBRARY_PATH)" >&2
  exit 2
fi
root="$(cd "$(dirname "$0")/../.." && pwd)"
test_dir="$(mktemp -d)"
trap 'rm -rf "$test_dir"' EXIT
bash "$root/scripts/build-lightweight-vehicle.sh" "$test_dir/liblightweight_vehicle.so"
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Werror -I "$root/abi/include" \
  "$root/plugins/lightweight-vehicle/controller_test.cpp" -ldl -o "$test_dir/controller-test"
"$test_dir/controller-test" "$test_dir/liblightweight_vehicle.so" "$1"
