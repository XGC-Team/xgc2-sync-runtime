#!/usr/bin/env bash
# Internal build body, invoked only in the caller-selected target Docker image.
set -euo pipefail
runtime=/workspace/runtime; project=/workspace/project
cache=/workspace/cache; output=/workspace/output
: "${XGC2_TARGET_PLATFORM:?}" "${XGC2_RUST_TARGET:?}" "${XGC2_ACADOS_PREFIX:?}" "${XGC2_BUILD_JOBS:?}"
check=(python3 "$runtime/scripts/verify-onboard-elf.py" --platform "$XGC2_TARGET_PLATFORM")
for tool in cargo rustc cmake c++ python3 readelf dpkg install ldd cmp awk sort; do
  command -v "$tool" >/dev/null || { echo "builder is missing $tool" >&2; exit 3; }
done
# The Docker image tag is not proof of its userland/toolchain identity.
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 20.04 ]] || { echo 'builder must be Ubuntu 20.04 (focal)' >&2; exit 3; }
arch="${XGC2_TARGET_PLATFORM##*-}"
[[ "$(dpkg --print-architecture)" == "$arch" ]] || { echo 'builder dpkg architecture mismatch' >&2; exit 3; }
rust_host="$(rustc -vV | sed -n 's/^host: //p')"
[[ "$rust_host" == "$XGC2_RUST_TARGET" ]] || { echo "builder Rust host mismatch: $rust_host" >&2; exit 3; }
[[ -f /opt/ros/noetic/include/ros/ros.h ]] || { echo 'builder is missing ROS Noetic headers' >&2; exit 3; }
# Tool versions are local evidence, not a substitute for target loading.
{ rustc -vV; cargo --version; cmake --version; c++ --version;
  printf 'os=%s architecture=%s\n' "$PRETTY_NAME" "$arch";
} > "$output/toolchain.txt"
acados="$XGC2_ACADOS_PREFIX"
"${check[@]}" "$acados/lib/libacados.so" "$acados/lib/libhpipm.so" "$acados/lib/libblasfeo.so" \
  /opt/ros/noetic/lib/libroscpp.so /opt/ros/noetic/lib/libroscpp_serialization.so \
  /opt/ros/noetic/lib/librosconsole.so /opt/ros/noetic/lib/librostime.so /opt/ros/noetic/lib/libcpp_common.so
mkdir -p "$cache/core" "$cache/core-install" "$cache/plan" "$cache/ros" "$output/bin" "$output/plugins" "$output/lib"
printf 'extern "C" int xgc2_target_probe() { return 0; }\n' | c++ -x c++ -shared -fPIC -o "$cache/compiler-probe.so" -
"${check[@]}" "$cache/compiler-probe.so"
# Reuse the existing standalone build verbatim. No solver/control flags are changed.
cmake -S "$project/planner/formation_generator/standalone" -B "$cache/core" \
  -DACADOS_ROOT="$acados" -DXGC_ABI_INCLUDE="$runtime/abi/include" \
  -DCMAKE_INSTALL_PREFIX="$cache/core-install" -DCMAKE_INSTALL_LIBDIR=lib
cmake --build "$cache/core" --parallel "$XGC2_BUILD_JOBS" --target \
  formation_generator_dmpc_core formation_generator_dmpc_params formation_generator_dmpc_config
# Reuse CMake's own install list to remove retired files, without refreshing all
# header mtimes (which would force a full plugin rebuild on every invocation).
previous_install=()
[[ ! -f "$cache/core/install_manifest.txt" ]] || mapfile -t previous_install < "$cache/core/install_manifest.txt"
cmake --install "$cache/core"
[[ -f "$cache/core/install_manifest.txt" ]] || { echo 'core install has no CMake manifest' >&2; exit 4; }
for installed in "${previous_install[@]}"; do
  [[ "$installed" == "$cache/core-install/"* && "$installed" != *'/../'* ]] || { echo 'unexpected CMake install path' >&2; exit 4; }
  if ! grep -Fxq -- "$installed" "$cache/core/install_manifest.txt"; then rm -f -- "$installed"; fi
done
bash "$runtime/scripts/build-plan-dmpc.sh" --core-prefix "$cache/core-install" \
  --acados-prefix "$acados" --build-dir "$cache/plan"
# Generated ROS headers/objects stay in cache, never in the exported plugin dir.
# gencpp imports genmsg from the selected ROS environment. setup.bash is not nounset-safe.
set +u
source /opt/ros/noetic/setup.bash
set -u
ROS_PREFIX=/opt/ros/noetic CXX=c++ bash "$runtime/scripts/build-ros-io.sh" "$cache/ros/libros_io.so"
cd "$runtime"
cargo build --offline --locked --release --target "$XGC2_RUST_TARGET" --jobs "$XGC2_BUILD_JOBS" \
  -p xgc-rt-host -p xgc-rt-audit -p dmpc-rounds -p numeric-vehicle -p station-io
release="$CARGO_TARGET_DIR/$XGC2_RUST_TARGET/release"
for binary in xgc-rt-host xgc-rt-render xgc-rt-audit; do
  "${check[@]}" "$release/$binary"
  install -m 0755 "$release/$binary" "$output/bin/$binary"
done
# Do not export workspace stub/demo plugins or build products by a broad glob.
for plugin in libdmpc_rounds.so libnumeric_vehicle.so libstation_io.so; do
  "${check[@]}" "$release/$plugin"
  install -m 0755 "$release/$plugin" "$output/plugins/$plugin"
done
install -m 0755 "$cache/plan/libplan_dmpc.so" "$output/plugins/libplan_dmpc.so"
install -m 0755 "$cache/ros/libros_io.so" "$output/plugins/libros_io.so"
# W09 receives target core/acados libraries only, not headers, CMake exports or source.
shopt -s nullglob
for directory in "$cache/core-install/lib" "$acados/lib"; do
  libraries=("$directory/"*.so "$directory/"*.so.*)
  ((${#libraries[@]})) || { echo "no shared libraries in $directory" >&2; exit 4; }
  for library in "${libraries[@]}"; do
    "${check[@]}" "$library"
    name="${library##*/}"
    [[ ! -e "$output/lib/$name" ]] || { echo "duplicate exported library: $name" >&2; exit 4; }
    # Dereference validated same-directory aliases; no build-prefix symlinks escape.
    install -m 0755 "$library" "$output/lib/$name"
  done
done
# Export the target image's resolved ROS/native dependencies into the same lib/
# input consumed by the existing W09 packager. Keep its Focal base libc6 +
# libgcc-s1 + libstdc++6 contract external; never copy the builder's loader.
# ldd runs only inside the already-validated native target image.
export LD_LIBRARY_PATH="$output/lib:$acados/lib:/opt/ros/noetic/lib"
: > "$output/dependencies.txt"
for artifact in "$output/bin/"* "$output/plugins/"*.so "$output/lib/"*.so*; do
  ldd "$artifact" >> "$output/dependencies.txt"
done
if grep -q '=> not found' "$output/dependencies.txt"; then
  echo 'unresolved target dependency; see dependencies.txt' >&2; exit 4
fi
while read -r name source; do
  case "$name" in
    libc.so.6|libm.so.6|libdl.so.2|libpthread.so.0|librt.so.1|libresolv.so.2|libutil.so.1|libgcc_s.so.1|libstdc++.so.6) continue ;;
  esac
  "${check[@]}" "$source"
  if [[ -e "$output/lib/$name" ]]; then
    cmp -s "$source" "$output/lib/$name" || { echo "conflicting dependency: $name" >&2; exit 4; }
  else
    install -m 0755 "$source" "$output/lib/$name"
  fi
done < <(awk '$2 == "=>" && $3 ~ /^\// { print $1, $3 }' "$output/dependencies.txt" | sort -u)

"${check[@]}" "$output/bin/"* "$output/plugins/"*.so "$output/lib/"*.so* > "$output/ELF.txt"
echo 'Target ELF export complete; this is not a W09 bundle/load or robot-motion test.'
