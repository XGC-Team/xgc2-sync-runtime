#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
project=""
platform=""
output=""
dry_run=0

usage() {
  cat >&2 <<'EOF'
usage: scripts/build-onboard-artifacts.sh --project DIR --platform TARGET --output DIR [--dry-run]

TARGET:
  focal-noetic-amd64
  focal-noetic-arm64
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --project) project="${2-}"; shift 2 ;;
    --platform) platform="${2-}"; shift 2 ;;
    --output) output="${2-}"; shift 2 ;;
    --dry-run) dry_run=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "build-onboard-artifacts: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

[[ -n "$project" && -n "$platform" && -n "$output" ]] || { usage; exit 2; }
[[ -d "$project" ]] || { echo "build-onboard-artifacts: project directory not found: $project" >&2; exit 2; }

case "$platform" in
  focal-noetic-amd64)
    docker_platform="linux/amd64"
    rust_target="x86_64-unknown-linux-gnu"
    elf_machine="x86-64"
    ;;
  focal-noetic-arm64)
    docker_platform="linux/arm64"
    rust_target="aarch64-unknown-linux-gnu"
    elf_machine="ARM aarch64"
    ;;
  *)
    echo "build-onboard-artifacts: unsupported target platform: $platform" >&2
    exit 2
    ;;
esac

mkdir -p "$output"
output="$(cd "$output" && pwd)"
project="$(cd "$project" && pwd)"
cache_root="${XGC2_BUILD_CACHE_DIR:-$HOME/.cache/xgc2/onboard-build}"
cache_dir="$cache_root/$platform"
mkdir -p "$cache_dir"

builder_image="${XGC2_ONBOARD_BUILDER_IMAGE:-ghcr.io/xgc-team/xgc2-onboard-builder:focal-noetic}"

inner=$(cat <<'INNER'
set -euo pipefail
mkdir -p "$XGC2_OUTPUT/bin"
command -v file >/dev/null || { echo "builder is missing file(1)" >&2; exit 3; }
command -v cargo >/dev/null || { echo "builder is missing cargo" >&2; exit 3; }
cd "$XGC2_RUNTIME"
cargo build --locked --release --target "$XGC2_RUST_TARGET" -p xgc-rt-host -p xgc-rt-audit
install -m 0755 "$CARGO_TARGET_DIR/$XGC2_RUST_TARGET/release/xgc-rt-host" "$XGC2_OUTPUT/bin/"
install -m 0755 "$CARGO_TARGET_DIR/$XGC2_RUST_TARGET/release/xgc-rt-audit" "$XGC2_OUTPUT/bin/"
find "$XGC2_OUTPUT/bin" -maxdepth 1 -type f -print0 | xargs -0 -r file | tee "$XGC2_OUTPUT/ELF.txt"
grep -q "ELF" "$XGC2_OUTPUT/ELF.txt" || { echo "no ELF artifacts produced" >&2; exit 4; }
grep -v "$XGC2_ELF_MACHINE" "$XGC2_OUTPUT/ELF.txt" | grep "ELF" && { echo "wrong target architecture" >&2; exit 4; } || true
INNER
)

cmd=(docker run --rm
  --platform "$docker_platform"
  -v "$project:/workspace/project:ro"
  -v "$root:/workspace/runtime:ro"
  -v "$output:/workspace/output"
  -v "$cache_dir:/workspace/cache"
  -e "CARGO_HOME=/workspace/cache/cargo"
  -e "CARGO_TARGET_DIR=/workspace/cache/target"
  -e "XGC2_PROJECT=/workspace/project"
  -e "XGC2_RUNTIME=/workspace/runtime"
  -e "XGC2_OUTPUT=/workspace/output"
  -e "XGC2_RUST_TARGET=$rust_target"
  -e "XGC2_ELF_MACHINE=$elf_machine"
  "$builder_image" bash -lc "$inner")

printf '%q ' "${cmd[@]}" > "$output/command.txt"
printf '\n' >> "$output/command.txt"
cat > "$output/build-target.env" <<EOF
XGC2_TARGET_PLATFORM=$platform
DOCKER_PLATFORM=$docker_platform
RUST_TARGET=$rust_target
ELF_MACHINE=$elf_machine
BUILDER_IMAGE=$builder_image
CACHE_DIR=$cache_dir
EOF

if (( dry_run )); then
  cat "$output/build-target.env"
  cat "$output/command.txt"
  exit 0
fi

"${cmd[@]}" 2>&1 | tee "$output/build.log"
echo "build-onboard-artifacts: output=$output platform=$platform cache=$cache_dir"
