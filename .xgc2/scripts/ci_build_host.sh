#!/usr/bin/env bash
# Release build of xgc2-module-host inside Ubuntu 18.04, so that the one binary needs glibc 2.27
# at most and serves bionic, focal and noble. Runs on a host of the target architecture.
#
#   ci_build_host.sh --output DIR        writes DIR/xgc2-module-host
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
image=ubuntu:18.04
# The release toolchain is the minimum supported Rust version, so releases and MSRV agree.
rust=1.85.0

output=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output) output="${2:?}"; shift 2;;
    *) echo 'usage: ci_build_host.sh --output DIR' >&2; exit 2;;
  esac
done
[[ -n "$output" ]] || { echo 'usage: ci_build_host.sh --output DIR' >&2; exit 2; }
mkdir -p -- "$output"
output="$(cd "$output" && pwd -P)"

docker pull "$image"
docker run --rm -e RUST_VERSION="$rust" -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
  -v "$root:/workspace/source:ro" -v "$output:/workspace/out" "$image" bash -euo pipefail -c '
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -qq
    apt-get install -y -qq --no-install-recommends build-essential ca-certificates curl git pkg-config > /dev/null
    curl --proto "=https" --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain "$RUST_VERSION" > /dev/null
    . /root/.cargo/env
    cd /workspace/source
    CARGO_TARGET_DIR=/tmp/target cargo build --release --locked --bins
    install -m 0755 /tmp/target/release/xgc2-module-host /workspace/out/xgc2-module-host
    chown "$HOST_UID:$HOST_GID" /workspace/out/xgc2-module-host
  '
printf '%s\n' "$output/xgc2-module-host"
