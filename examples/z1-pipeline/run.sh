#!/usr/bin/env bash
# Build (release), run the Z1 pipeline host, merge its audit.
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$root"
cargo build -q --release
scripts/build-c-stub.sh target/release/libc_stub.so
rm -rf out/z1-pipeline
target/release/xgc-rt-host --manifest examples/z1-pipeline/node.toml > out-z1-summary.json
target/release/xgc-rt-audit merge out/z1-pipeline
mv out-z1-summary.json out/z1-pipeline/run-summary.json
echo "run summary: out/z1-pipeline/run-summary.json"
