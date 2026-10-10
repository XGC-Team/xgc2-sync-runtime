#!/usr/bin/env bash
# Build the host against the Ubuntu 18.04 sysroot (glibc 2.27, gcc 7.5) in a separate target
# directory and run its tests there. Proves two things: the binary links against glibc 2.27 only,
# and the tests pass when the test processes run on the glibc 2.27 loader and libc.
#
#   scripts/bionic-test.sh [extra cargo test arguments]
#
# XGC2_BIONIC_TOOLCHAINS points at the directory with bin/bionic-gcc, bin/bionic-g++ and
# bionic-sysroot (default /home/node/workspace/toolchains).
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
toolchains="${XGC2_BIONIC_TOOLCHAINS:-/home/node/workspace/toolchains}"
sysroot="$toolchains/bionic-sysroot"
triple=x86_64-unknown-linux-gnu

export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$root/target/bionic}"
# Link through the Bionic gcc with the Bionic sysroot. The test modules are compiled by it too.
export CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_LINKER="$toolchains/bin/bionic-gcc"
export CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUSTFLAGS="-C link-arg=--sysroot=$sysroot"
export CC="$toolchains/bin/bionic-gcc" CXX="$toolchains/bin/bionic-g++"
# Run the test processes on the glibc 2.27 dynamic loader with the sysroot's libraries.
libs="$sysroot/lib/x86_64-linux-gnu:$sysroot/usr/lib/x86_64-linux-gnu"
export CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUNNER="$sysroot/lib/x86_64-linux-gnu/ld-2.27.so --library-path $libs"

cd "$root"
cargo build --locked -j3 --target "$triple" --bins
binary="$CARGO_TARGET_DIR/$triple/debug/xgc2-module-host"
echo "highest glibc symbol version needed by xgc2-module-host:"
objdump -T "$binary" | grep -o 'GLIBC_[0-9.]*' | sort -uV | tail -1
cargo test --locked -j3 --target "$triple" "$@"
