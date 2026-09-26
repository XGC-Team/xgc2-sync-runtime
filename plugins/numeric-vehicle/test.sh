#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$root"
cargo build --offline -p numeric-vehicle --release
export NUMERIC_VEHICLE_ELF="${CARGO_TARGET_DIR:-$root/target}/release/libnumeric_vehicle.so"
cargo test --offline -p numeric-vehicle -- --nocapture
