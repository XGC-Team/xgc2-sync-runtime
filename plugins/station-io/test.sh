#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$root"
export CARGO_BUILD_JOBS=2
cargo build --offline -j 2 -p station-io -p numeric-vehicle
export STATION_IO_CMD="${CARGO_TARGET_DIR:-$root/target}/debug/station-io-cmd"
export STATION_IO_ELF="${CARGO_TARGET_DIR:-$root/target}/debug/libstation_io.so"
export NUMERIC_VEHICLE_ELF="${CARGO_TARGET_DIR:-$root/target}/debug/libnumeric_vehicle.so"
cargo test --offline -j 2 -p station-io -- --nocapture --test-threads=1
