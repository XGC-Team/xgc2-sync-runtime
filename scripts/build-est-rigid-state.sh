#!/usr/bin/env bash
# Build the product-owned rigid-state adapter using its public CMake entry.
# Usage: scripts/build-est-rigid-state.sh OUT.so [REFERENCE_BIN]
#
# ESKF_ROOT overrides the owner package location; CMAKE_PREFIX_PATH selects
# installed Eigen3/xgc2_math/xgc2_state_machine dependencies. The canonical
# runtime SDK is installed privately, then consumed through XgcRuntime::SDK.
# No estimator or state-machine source list is maintained here.
# Output directories also receive the shared core and state-machine libraries.
# CXX selects the compiler; CMAKE_BUILD_PARALLEL_LEVEL defaults to one job.
set -euo pipefail
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 OUT.so [REFERENCE_BIN]" >&2
  exit 2
fi
root="$(cd "$(dirname "$0")/.." && pwd)"
products="$(cd "$root/../.." && pwd)"
eskf="${ESKF_ROOT:-$products/ros1/perception/estimator/rigid-state/estimator_vrpn_px4_rotor_state}"
if [[ ! -f "$eskf/CMakeLists.txt" ]]; then
  echo "Missing rigid-state owner CMake project: $eskf (set ESKF_ROOT)" >&2
  exit 2
fi
build="$(mktemp -d "${TMPDIR:-/tmp}/xgc-est-rigid-state.XXXXXXXX")"
trap 'rm -rf -- "$build"' EXIT
cmake -S "$root/abi" -B "$build/sdk-build" -DCMAKE_INSTALL_PREFIX="$build/sdk"
cmake --install "$build/sdk-build"

with_reference=OFF
if [[ $# -eq 2 ]]; then with_reference=ON; fi
cmake -S "$eskf" -B "$build/owner" \
  -DRIGID_STATE_BUILD_ROS=OFF -DRIGID_STATE_BUILD_NATIVE=ON \
  -DRIGID_STATE_BUILD_TESTING="$with_reference" \
  -DXgcRuntimeSDK_DIR="$build/sdk/share/cmake/XgcRuntimeSDK" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_RPATH_USE_ORIGIN=ON \
  -DCMAKE_INSTALL_PREFIX="$build/install" -DCMAKE_INSTALL_LIBDIR=lib
build_targets=(est_rigid_state)
if [[ $# -eq 2 ]]; then build_targets+=(rigid_state_reference_replay); fi
cmake --build "$build/owner" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-1}" --target "${build_targets[@]}"
cmake --install "$build/owner"

# Resolve the actual installed state-machine dependency selected by owner CMake.
# The build RPATH resolves it before the owner libraries are relocated together.
sm_dependency="$(ldd "$build/owner/libest_rigid_state.so" | awk '$1 ~ /^libxgc2_state_machine[.]so/ && $2 == "=>" { print $1 " " $3 }')"
read -r sm_soname sm_path <<< "$sm_dependency"
if [[ -z "${sm_soname:-}" || ! -f "${sm_path:-}" ]]; then
  echo "Owner build has no resolvable installed xgc2_state_machine shared library: $sm_dependency" >&2
  exit 1
fi
copy_dependencies() {
  local destination="$1"
  mkdir -p -- "$destination"
  cp -L -- "$build/install/lib/libestimator_vrpn_px4_rotor_state_core.so" "$destination/"
  cp -L -- "$sm_path" "$destination/$sm_soname"
}
copy_dependencies "$(dirname "$1")"
cp -- "$build/install/lib/libest_rigid_state.so" "$1"
if [[ $# -eq 2 ]]; then
  copy_dependencies "$(dirname "$2")"
  cp -- "$build/owner/rigid_state_reference_replay" "$2"
fi
