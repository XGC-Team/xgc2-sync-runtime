#!/usr/bin/env bash
# Stage the sync-runtime bundle a robot image copies in (Z3, decision B:
# source-pinned, no APT package, no CI or Jenkins job).
#
#   scripts/stage-robot-bundle.sh --out DIR [--revision REV]
#       [--academic REPO --academic-revision AREV] [--with-ctl-px4]
#
# REV (default HEAD) is a commit of this repository; AREV a commit of the
# academic repository (formation_generator, the TRO DMPC planner). Both must
# be on a remote branch, so anyone can rebuild the same bundle. Each is
# exported with git archive and built from the export, never from a working
# tree:
#   bin/xgc-rt-host, bin/xgc-rt-render, bin/xgc-rt-audit   cargo --release --locked
#   plugins/libtransport_zenoh.so                          the Zenoh transport plugin
#   lib/libformation_generator_dmpc_{core,params,config}.so  AREV standalone/CMakeLists.txt
#   plugins/libplan_dmpc.so                                scripts/build-plan-dmpc.sh against them
#   plugins/libctl_px4.so (--with-ctl-px4)                 scripts/build-ctl-px4.sh; its PX4 core
#       comes from PX4_CORE_LIB_DIR as built outside this script and is recorded
#       as not source-pinned (see docs/z3-packaging.md)
# and writes SOURCE-PINS.json (revisions, toolchains, external libraries,
# every staged file's sha256) and SHA256SUMS (sha256sum --check format), the
# file the image build checks. DIR must not exist.
#
# Environment: ACADOS_ROOT, EIGEN_INCLUDE, YAML_CPP_LIB_DIR, CXX as for
# build-plan-dmpc.sh; CMAKE (default cmake); CARGO (default cargo). With
# --with-ctl-px4 also PX4_CORE_LIB_DIR, PX4_CONTROLLER_ROOT, XGC2_PREFIX.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
out="" revision=HEAD academic="" academic_revision="" with_ctl_px4=0
fail() { printf 'stage-robot-bundle: %s\n' "$*" >&2; exit 1; }
while (($#)); do
  case "$1" in
    --out) out="${2:?}"; shift 2 ;;
    --revision) revision="${2:?}"; shift 2 ;;
    --academic) academic="${2:?}"; shift 2 ;;
    --academic-revision) academic_revision="${2:?}"; shift 2 ;;
    --with-ctl-px4) with_ctl_px4=1; shift ;;
    *) fail "unknown argument $1" ;;
  esac
done
[[ -n "$out" ]] || fail "--out DIR is required"
[[ ! -e "$out" ]] || fail "$out exists"
[[ -n "$academic" && -n "$academic_revision" ]] || fail "--academic REPO and --academic-revision AREV are required (plan-dmpc)"
cmake="${CMAKE:-cmake}" cargo="${CARGO:-cargo}" cxx="${CXX:-c++}"

# A commit on a remote branch of REPO, or fail.
pinned() {
  local repo="$1" rev="$2" commit
  commit="$(git -C "$repo" rev-parse --verify --quiet "${rev}^{commit}")" || fail "$rev is not a commit in $repo"
  [[ -n "$(git -C "$repo" branch --remotes --contains "$commit")" ]] \
    || fail "$commit is on no remote branch of $repo; push it so the bundle can be rebuilt"
  printf '%s\n' "$commit"
}
revision="$(pinned "$root" "$revision")"
academic_revision="$(pinned "$academic" "$academic_revision")"

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
mkdir -p "$work/runtime" "$work/academic" "$out"/{bin,plugins,lib}
git -C "$root" archive --format=tar "$revision" | tar -x -C "$work/runtime"
git -C "$academic" archive --format=tar "$academic_revision" ros1_ws/src/planner ros1_ws/src/common \
  | tar -x -C "$work/academic"
fg="$work/academic/ros1_ws/src/planner/formation_generator"
export SOURCE_DATE_EPOCH="$(git -C "$root" show -s --format=%ct "$revision")"

printf 'runtime %s, academic %s\n' "$revision" "$academic_revision" >&2
(cd "$work/runtime" && CARGO_TARGET_DIR="$work/target" "$cargo" build --release --locked \
  -p xgc-rt-host -p xgc-rt-audit -p transport-zenoh >&2)
install -m 0755 "$work/target/release/"{xgc-rt-host,xgc-rt-render,xgc-rt-audit} "$out/bin/"
install -m 0644 "$work/target/release/libtransport_zenoh.so" "$out/plugins/"

"$cmake" -S "$fg/standalone" -B "$work/dmpc" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER="$cxx" \
  -DACADOS_ROOT="${ACADOS_ROOT:?set ACADOS_ROOT}" -DEIGEN3_INCLUDE_DIR="${EIGEN_INCLUDE:-/usr/include/eigen3}" \
  -DXGC_ABI_INCLUDE="$work/runtime/abi/include" >&2
"$cmake" --build "$work/dmpc" -j "$(nproc)" --target formation_generator_dmpc_core \
  formation_generator_dmpc_params formation_generator_dmpc_config >&2
install -m 0644 "$work/dmpc/"libformation_generator_dmpc_{core,params,config}.so "$out/lib/"
DMPC_LIB_DIR="$out/lib" FORMATION_GENERATOR_ROOT="$fg" "$work/runtime/scripts/build-plan-dmpc.sh" "$out/plugins/libplan_dmpc.so" >&2
if ((with_ctl_px4)); then
  "$work/runtime/scripts/build-ctl-px4.sh" "$out/plugins/libctl_px4.so" >&2
  install -m 0644 "${PX4_CORE_LIB_DIR:?}/libpx4_multirotor_controller_core.so" "$out/lib/"
fi

# Every library a staged ELF loads must resolve, inside the bundle (lib/)
# or to a recorded external file.
externals="$work/externals"
: >"$externals"
for elf in "$out"/bin/* "$out"/plugins/*.so "$out"/lib/*.so; do
  while read -r name _ path _; do
    [[ "$path" == "not" ]] && fail "$(basename "$elf"): $name not found"
    [[ -z "$path" || "$path" == "$out/lib/"* ]] && continue
    printf '%s\n' "$path" >>"$externals"
  done < <(LD_LIBRARY_PATH="$out/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ldd "$elf" | awk '/=>/ {print $1, $2, $3, $4}')
done

(cd "$out" && find bin plugins lib -type f | LC_ALL=C sort | xargs sha256sum) >"$out/SHA256SUMS"
python3 - "$out" "$revision" "$academic_revision" "$with_ctl_px4" "$externals" "$cxx" "$cargo" <<'EOF'
import json, os, subprocess, sys, hashlib
out, rev, arev, ctl, externals, cxx, cargo = sys.argv[1:]
def sha(p):
    return hashlib.sha256(open(p, 'rb').read()).hexdigest()
def version(cmd):
    return subprocess.run(cmd, capture_output=True, text=True).stdout.splitlines()[0].strip()
files = [{"path": l.split("  ", 1)[1].strip(), "sha256": l.split()[0]} for l in open(os.path.join(out, "SHA256SUMS"))]
ext = sorted({os.path.realpath(p.strip()) for p in open(externals) if p.strip()})
pins = {
    "schema_version": 1,
    "kind": "xgc2-sync-runtime-robot-bundle",
    "sources": {
        "xgc2-sync-runtime": {"revision": rev, "built": ["xgc-rt-host", "xgc-rt-render", "xgc-rt-audit", "transport-zenoh", "plan-dmpc"] + (["ctl-px4"] if ctl == "1" else [])},
        "academic": {"revision": arev, "paths": ["ros1_ws/src/planner", "ros1_ws/src/common"], "built": ["formation_generator_dmpc_core", "formation_generator_dmpc_params", "formation_generator_dmpc_config"]},
    },
    "not_source_pinned": ([{"file": "lib/libpx4_multirotor_controller_core.so", "why": "PX4 controller core built outside this script (PX4_CORE_LIB_DIR)"}] if ctl == "1" else []),
    "toolchains": {"cargo": version([cargo, "--version"]), "rustc": version(["rustc", "--version"]), "cxx": version([cxx, "--version"])},
    "external_libraries": [{"path": p, "sha256": sha(p)} for p in ext],
    "files": files,
}
json.dump(pins, open(os.path.join(out, "SOURCE-PINS.json"), "w"), indent=2)
open(os.path.join(out, "SOURCE-PINS.json"), "a").write("\n")
EOF
printf '%s\n' "$out/SOURCE-PINS.json"
