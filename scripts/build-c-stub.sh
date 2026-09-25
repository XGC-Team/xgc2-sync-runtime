#!/usr/bin/env bash
# Build the pure-C stub plugin against abi/include/xgc_rt.h.
# Usage: scripts/build-c-stub.sh OUT.so
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
"${CC:-cc}" -std=c11 -O2 -Wall -Wextra -Werror -fPIC -shared \
  -I "$root/abi/include" "$root/plugins/c-stub/c_stub.c" -o "$1"
