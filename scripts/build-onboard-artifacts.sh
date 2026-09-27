#!/usr/bin/env bash
# W08: build once per OS/ROS/CPU on the station, never on a robot.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
project=""; platform=""; output=""; dry_run=0
image="${XGC2_ONBOARD_BUILDER_IMAGE:-}"
acados_prefix="/opt/acados"; jobs=2
usage() {
  cat <<'HELP'
usage: bash scripts/build-onboard-artifacts.sh --project SRC_DIR \
  --platform focal-noetic-{amd64,arm64} --output DIR --builder-image LOCAL_IMAGE \
  [--acados-prefix /opt/acados] [--jobs 2] [--dry-run]

SRC_DIR is the reviewed ROS workspace's src directory. It must contain
planner/formation_generator/standalone/CMakeLists.txt and common/.
LOCAL_IMAGE must already exist in the local Docker daemon, for the requested
architecture, with Ubuntu 20.04, ROS Noetic, Rust/Cargo, CMake, C++, Python 3,
readelf, patchelf and the matching acados install. No image is pulled or published.

Output is DIR/PLATFORM/{bin,lib,plugins}, plus local logs/target/timing evidence.
This is W09 packager input, NOT an installed bundle or a runtime-load verdict.
Existing output is refused; use a fresh DIR for a second timed build. The
per-source, per-image, per-platform local build cache is reused automatically.
XGC2_BUILD_CACHE_DIR overrides the local cache root. --dry-run writes nothing
and does not contact Docker. See scripts/onboard-build.md for W09 handoff.
HELP
}
fail() { echo "build-onboard-artifacts: $*" >&2; exit 2; }
while (($#)); do
  case "$1" in
    --project|--platform|--output|--builder-image|--acados-prefix|--jobs)
      [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || fail "$1 requires a value"
      case "$1" in
        --project) project="$2";; --platform) platform="$2";; --output) output="$2";;
        --builder-image) image="$2";; --acados-prefix) acados_prefix="$2";; --jobs) jobs="$2";;
      esac
      shift 2;;
    --dry-run) dry_run=1; shift;;
    -h|--help) usage; exit 0;;
    *) fail "unknown argument: $1";;
  esac
done
[[ -n "$project" && -n "$platform" && -n "$output" ]] || { usage >&2; exit 2; }
case "$platform" in
  focal-noetic-amd64) docker_platform=linux/amd64; rust_target=x86_64-unknown-linux-gnu;;
  focal-noetic-arm64) docker_platform=linux/arm64; rust_target=aarch64-unknown-linux-gnu;;
  *) fail "unsupported target platform: $platform";;
esac
[[ -n "$image" && "$image" != -* && "$image" != *[[:space:]]* ]] || fail 'set --builder-image to a real, locally prepared target builder'
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || fail '--jobs must be a positive integer'
[[ "$acados_prefix" == /* && "$acados_prefix" != / ]] || fail '--acados-prefix must be an absolute install prefix inside the builder'
command -v python3 >/dev/null || fail 'python3 is required'
[[ -f "$project/planner/formation_generator/standalone/CMakeLists.txt" && -d "$project/common" ]] || fail '--project must contain planner/formation_generator/standalone/CMakeLists.txt and common/'
project="$(cd "$project" && pwd -P)"
canonical() { python3 -c 'import os,sys; print(os.path.realpath(sys.argv[1]))' "$1"; }
output="$(canonical "$output")"
cache_root="$(canonical "${XGC2_BUILD_CACHE_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}/xgc2/onboard-build}")"
# Commas/newlines are Docker --mount separators, not valid input paths here.
for path in "$root" "$project" "$output" "$cache_root"; do
  [[ "$path" != *','* && "$path" != *$'\n'* && "$path" != *$'\r'* ]] || fail 'mount paths may not contain commas or newlines'
done
# Keep writable exports/caches outside the read-only source mounts and each other.
for writable in "$output" "$cache_root"; do
  [[ "$writable" != / ]] || fail "output/cache may not be the filesystem root"
  for source in "$root" "$project"; do
    [[ "$writable" != "$source" && "$writable" != "$source/"* && "$source" != "$writable/"* ]] || fail 'source, output and cache directories must be separate'
  done
done
[[ "$output" != "$cache_root" && "$output" != "$cache_root/"* && "$cache_root" != "$output/"* ]] || fail 'output and cache directories must be separate'
final="$output/$platform"
[[ ! -e "$final" && ! -L "$final" ]] || fail "refusing existing output: $final (choose a fresh --output)"
if ((dry_run)); then
  printf 'TARGET_PLATFORM=%q\nDOCKER_PLATFORM=%q\nRUST_TARGET=%q\nBUILDER_IMAGE=%q\nOUTPUT=%q\n' "$platform" "$docker_platform" "$rust_target" "$image" "$final"
  printf 'Steps: verify local builder; build reviewed standalone core; Cargo runtime/plugins; build-plan-dmpc.sh; build-ros-io.sh; verify/export ELF.\n'
  exit 0
fi
command -v docker >/dev/null || fail 'docker is required for a real build'
# Respect the selected local context, but never send source paths to a remote daemon.
if [[ -n "${DOCKER_HOST:-}" && -z "${DOCKER_CONTEXT:-}" ]]; then
  endpoint="$DOCKER_HOST"
else
  endpoint="$(docker context inspect --format '{{.Endpoints.docker.Host}}')"
fi
[[ "$endpoint" == unix:///* && "$endpoint" != *$'\n'* ]] || fail "local Unix-socket Docker context required; got: $endpoint"
docker_cmd=(env -u DOCKER_CONTEXT -u DOCKER_HOST docker --host "$endpoint")
info="$("${docker_cmd[@]}" image inspect --format '{{.Id}} {{.Os}}/{{.Architecture}}' "$image")"
read -r image_id actual_platform extra <<< "$info"
[[ "$image_id" =~ ^sha256:[0-9a-f]{64}$ && -z "$extra" && "$info" != *$'\n'* ]] || fail 'invalid local image inspection result'
[[ "$actual_platform" == "$docker_platform" ]] || fail "builder architecture is $actual_platform, requested $docker_platform"
# This is a cache namespace, not a source-content hash/lock/registry. Edits reuse it.
cache_key="$(python3 -c 'import hashlib,sys; print(hashlib.sha256("\0".join(sys.argv[1:]).encode()).hexdigest()[:24])' "$root" "$project" "$image_id" "$acados_prefix")"
cache="$cache_root/$platform/$cache_key"
mkdir -p "$output" "$cache/home" "$cache/cargo" "$cache/target"
echo "build-onboard-artifacts: cache=$cache (compile network disabled)"
stage="$(mktemp -d "$output/.${platform}.XXXXXX")"
cmd=("${docker_cmd[@]}" run --rm --pull=never --network none --platform "$docker_platform"
  --user "$(id -u):$(id -g)" --cap-drop ALL --security-opt no-new-privileges
  --mount "type=bind,src=$root,dst=/workspace/runtime,readonly"
  --mount "type=bind,src=$project,dst=/workspace/project,readonly"
  --mount "type=bind,src=$stage,dst=/workspace/output"
  --mount "type=bind,src=$cache,dst=/workspace/cache"
  -e HOME=/workspace/cache/home -e CARGO_HOME=/workspace/cache/cargo
  -e CARGO_TARGET_DIR=/workspace/cache/target
  -e "XGC2_TARGET_PLATFORM=$platform" -e "XGC2_RUST_TARGET=$rust_target"
  -e "XGC2_ACADOS_PREFIX=$acados_prefix" -e "XGC2_BUILD_JOBS=$jobs"
)
# Current standalone CMake also configures the workspace's opt-in harness target.
# Keep its original adjacent source path available without copying or changing it.
if [[ -d "$project/../tests" ]]; then
  cmd+=(--mount "type=bind,src=$project/../tests,dst=/workspace/tests,readonly")
fi
cmd+=(--entrypoint /bin/bash "$image_id" /workspace/runtime/scripts/build-onboard-container.sh)
printf '%q ' "${cmd[@]}" > "$stage/command.txt"; printf '\n' >> "$stage/command.txt"
printf 'TARGET_PLATFORM=%q\nDOCKER_PLATFORM=%q\nRUST_TARGET=%q\nBUILDER_IMAGE_ID=%q\nCACHE_KEY=%q\n' "$platform" "$docker_platform" "$rust_target" "$image_id" "$cache_key" > "$stage/build-target.env"
# Record source provenance without recursively hashing or copying source. A dirty
# tree is allowed for local iteration, but is explicitly not a reviewed baseline.
source_identity() {
  local prefix="$1" directory="$2" revision changes state
  revision=unavailable; state=unavailable
  if command -v git >/dev/null && revision="$(git -C "$directory" rev-parse --verify HEAD 2>/dev/null)"; then
    if changes="$(git --no-optional-locks -C "$directory" status --porcelain --untracked-files=normal -- . 2>/dev/null)"; then
      state=clean; [[ -z "$changes" ]] || state=dirty
    fi
  else
    revision=unavailable
  fi
  printf '%s_REVISION=%q\n%s_WORKTREE=%q\n' "$prefix" "$revision" "$prefix" "$state"
}
source_identity RUNTIME "$root" >> "$stage/build-target.env"
source_identity PROJECT "$project" >> "$stage/build-target.env"
start="$(python3 -c 'import time; print(time.monotonic_ns())')"
set +e
"${cmd[@]}" 2>&1 | tee "$stage/build.log"
statuses=("${PIPESTATUS[@]}")
set -e
rc="${statuses[0]}"; ((rc != 0)) || rc="${statuses[1]}"
end="$(python3 -c 'import time; print(time.monotonic_ns())')"
printf 'docker_elapsed_ms=%s\ndocker_exit_code=%s\n' "$(((end-start)/1000000))" "$rc" > "$stage/timing.env"
if ((rc != 0)); then
  echo "build-onboard-artifacts: failed ($rc); unaccepted output/logs retained at $stage" >&2
  exit "$rc"
fi
# Check the export on the station too; Docker success alone is not artifact evidence.
required=(bin/xgc-rt-host bin/xgc-rt-render bin/xgc-rt-audit
  plugins/libplan_dmpc.so plugins/libros_io.so plugins/libdmpc_rounds.so
  plugins/libnumeric_vehicle.so plugins/libstation_io.so
  lib/libformation_generator_dmpc_core.so lib/libformation_generator_dmpc_params.so
  lib/libformation_generator_dmpc_config.so lib/libacados.so lib/libhpipm.so lib/libblasfeo.so)
for artifact in "${required[@]}"; do
  if [[ ! -f "$stage/$artifact" ]]; then
    echo 'artifact_exit_code=4' >> "$stage/timing.env"
    echo "build-onboard-artifacts: missing $artifact; logs retained at $stage" >&2
    exit 4
  fi
done
(cd "$stage"; python3 "$root/scripts/verify-onboard-elf.py" --platform "$platform" \
  bin/xgc-rt-host bin/xgc-rt-render bin/xgc-rt-audit plugins/*.so lib/*.so*) > "$stage/ELF.txt" || {
    echo "artifact_exit_code=4" >> "$stage/timing.env"
    echo "build-onboard-artifacts: export validation failed; logs retained at $stage" >&2; exit 4;
  }
echo "artifact_exit_code=0" >> "$stage/timing.env"
mv -T -n -- "$stage" "$final"
[[ ! -d "$stage" ]] || fail "output appeared during build; refusing replacement; evidence=$stage"
echo "build-onboard-artifacts: output=$final cache=$cache (W09 packaging/loading still required)"
