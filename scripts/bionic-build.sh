#!/usr/bin/env bash
# Release build of xgc2-module-host against glibc 2.27 (see bionic-env.sh). The binary runs on
# Ubuntu 18.04, 20.04 and 24.04; this is the one the Debian package ships.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/bionic-env.sh"
cd "$root"
cargo build --release --locked -j3 --target "$triple" --bins
binary="$CARGO_TARGET_DIR/$triple/release/xgc2-module-host"
echo "highest glibc symbol version needed: $(glibc_floor "$binary")" >&2
printf '%s\n' "$binary"
