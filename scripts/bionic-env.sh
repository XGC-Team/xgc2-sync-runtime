# Sourced by bionic-build.sh and bionic-test.sh: configure cargo to link through the Ubuntu 18.04
# gcc with the Ubuntu 18.04 sysroot (glibc 2.27, gcc 7.5), in a target directory of its own.
#
# XGC2_BIONIC_TOOLCHAINS points at the directory with bin/bionic-gcc, bin/bionic-g++ and
# bionic-sysroot (default /home/node/workspace/toolchains).

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
toolchains="${XGC2_BIONIC_TOOLCHAINS:-/home/node/workspace/toolchains}"
sysroot="$toolchains/bionic-sysroot"
triple=x86_64-unknown-linux-gnu

export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$root/target/bionic}"
# The Bionic gcc links the Rust code against the sysroot, and compiles the test modules too.
export CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_LINKER="$toolchains/bin/bionic-gcc"
export CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUSTFLAGS="-C link-arg=--sysroot=$sysroot"
export CC="$toolchains/bin/bionic-gcc" CXX="$toolchains/bin/bionic-g++"
# Test processes run on the glibc 2.27 dynamic loader with the sysroot's libraries.
libs="$sysroot/lib/x86_64-linux-gnu:$sysroot/usr/lib/x86_64-linux-gnu"
export CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUNNER="$sysroot/lib/x86_64-linux-gnu/ld-2.27.so --library-path $libs"

# glibc_floor BINARY: the highest glibc symbol version the binary needs.
glibc_floor() {
  objdump -T "$1" | grep -o 'GLIBC_[0-9.]*' | sort -uV | tail -1
}
