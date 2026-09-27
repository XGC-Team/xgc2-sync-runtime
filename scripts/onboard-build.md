# Local onboard artifacts (W08)

Related work: XGC-Team/xgc2-harness#173. Packaging/loading belongs to W09 (#174);
base recipes belong to W06 (#171). Neither a PR merge nor the offline tests
below establish a usable robot deployment.

## Inputs and target model

Run `bash scripts/build-onboard-artifacts.sh --help` from this repository.
`--project` is the **reviewed ROS workspace src directory**, containing
`planner/formation_generator/standalone/CMakeLists.txt` and `common/` siblings.
It is not a robot ID, a whole paper repository, or a path on a remote machine.
Select the successful algorithm checkout with Root/W33; this driver does not
fetch, change, patch or select algorithm revisions. Current HEAD is not evidence
of a successful historical algorithm.

The target set is deliberately only `focal-noetic-amd64` and
`focal-noetic-arm64`. Compilation is native **inside the requested target
userland**, on a matching local CPU or with already-configured local binfmt/QEMU.
This is not host compilation followed by a renamed artifact. The script does
not install emulators or create a remote builder. It refuses non-Unix-socket
Docker endpoints and runs the inspected local image by its immutable image ID,
with `--pull=never`. No Docker socket is mounted into the build container.

Pass a real local image using `--builder-image` (or
`XGC2_ONBOARD_BUILDER_IMAGE`). There is intentionally **no fictional default tag**.
The W06 runtime base is not automatically a compiler image. Root must prepare
and admit a local development image containing Ubuntu 20.04, target ROS Noetic
headers/libraries and gencpp, CMake, C++, Python 3, binutils, Eigen, yaml-cpp, and
Rust/Cargo capable of reading the checked-in workspace/lock file. Rustup/toolchain
files must be accessible to the invoking UID independently of `CARGO_HOME`;
the driver uses persistent writable Cargo/home caches. Dependency downloads by
Cargo are local build activity, not robot installation or APT publication.

The image must contain the reviewed **target-architecture** acados install at
`--acados-prefix` (default `/opt/acados`), including the original required solver
backends. W08 does not change solver build choices, algorithms or budgets.
Image platform, Ubuntu release, dpkg architecture, Rust host, a compiled C++
probe, and acados/ROS ELF identities are checked before the main build. Missing
libraries/tools produce their original diagnostics; there is no fallback to
host libraries, dummy modules or another architecture.

## Build and repeat

Use paths outside both source checkouts for output/cache. Spaces are supported;
commas/newlines in bind-mount paths are rejected. Example variables are explicit
operator inputs, not private workstation paths:

```bash
export XGC2_BUILD_CACHE_DIR="$HOME/.cache/xgc2/onboard-build"
# REVIEWED_WS_SRC and ARM64_BUILDER name existing local inputs admitted by Root.
bash scripts/build-onboard-artifacts.sh \
  --project "$REVIEWED_WS_SRC" --platform focal-noetic-arm64 \
  --builder-image "$ARM64_BUILDER" --output "$FIRST_OUTPUT" --dry-run

bash scripts/build-onboard-artifacts.sh \
  --project "$REVIEWED_WS_SRC" --platform focal-noetic-arm64 \
  --builder-image "$ARM64_BUILDER" --output "$FIRST_OUTPUT"
bash scripts/build-onboard-artifacts.sh \
  --project "$REVIEWED_WS_SRC" --platform focal-noetic-arm64 \
  --builder-image "$ARM64_BUILDER" --output "$SECOND_OUTPUT"
```

Use `focal-noetic-amd64` and the admitted amd64 image for the station/container
target. Do not produce a Cartesian product of robot IDs and targets. Two outputs
must be distinct; existing target output is never overwritten. An unsuccessful
attempt retains a hidden `.focal-noetic-*.XXXXXX` directory with logs but never
becomes an accepted target directory.

The cache namespace includes source locations, immutable builder ID, acados
prefix and target. Edits reuse CMake/Cargo object caches; changing image/target
isolates them. The reviewed standalone core install, `build-plan-dmpc.sh`,
`build-ros-io.sh` and existing Cargo packages are reused. CMake's own install
manifest removes retired installed files without refreshing all header mtimes.
The existing ROS helper still recompiles its generated bridge; W08 does not
claim every component is a no-op on a second invocation. Run one build per
source/image/target cache at a time; use a distinct cache root for independent
concurrent builds. No new lock service or executor is introduced.

## Output and W09 handoff

`OUTPUT/focal-noetic-{amd64,arm64}/` contains:

- `bin/`: `xgc-rt-host`, `xgc-rt-render`, `xgc-rt-audit`.
- `plugins/`: `libplan_dmpc.so`, `libros_io.so`, `libdmpc_rounds.so`,
  `libnumeric_vehicle.so`, `libstation_io.so`.
- `lib/`: installed standalone core/config/params and acados shared libraries;
  validated same-directory aliases are dereferenced. No host libc is copied.
- Local evidence: `build-target.env`, `command.txt`, `build.log`, `toolchain.txt`,
  `ELF.txt`, `timing.env`.

The current build surface is **planner/numeric plus ROS/station I/O**. It does
not build the separate C++ controller/estimator/reference plugins and must not
be described as a full control composition or full HIL implementation. Those
existing build entrypoints and W23/W24 integration need separately reviewed
inputs. No stub/demo libraries, generated ROS headers, source directories,
static archives or CMake exports are exported here.

These are **precompiled inputs**, not an installed bundle. No placeholder
`BUNDLE.json`, deployment descriptor or manifests are emitted. W09 must use the
real product `scripts/package-sync-runtime.py package` with `--host`,
`--renderer`, each admitted `--plugin`, `--lib-dir`, `--arch`, source revisions,
version and the selected renderer's actual composition identity/digest. Select
only plugins belonging to that composition; passing every exported plugin to
a composition with fewer roles is correctly rejected by the existing packager.
W09 produces the existing `bin/lib/plugins/manifests/BUNDLE.json` layout and
checks target dependency closure/GLIBC/loader compatibility. The whole raw
output directory, especially logs, must **not** be copied into a robot bundle.

Source Git revision and clean/dirty state are recorded when available; exported
snapshots say `unavailable`, not a fabricated revision. Dirty development builds
are allowed and labelled, not certified historical baselines. Build logs and
commands can contain private paths or compiler excerpts: keep them local and
review/redact before sharing. This script has no public-artifact upload path.

## Evidence and acceptance

`verify-onboard-elf.py` checks ELF64/little-endian, EXEC/DYN type, target machine
(amd64=62, arm64=183), readelf acceptance and escaping symlinks. It is **not**
a dependency-closure, ABI, dlopen, algorithm-success or motion validator.

`timing.env` distinguishes `docker_exit_code` and `artifact_exit_code`; Docker
exit zero alone is rejected for missing/mixed exports. `docker_elapsed_ms`
measures the container invocation, not image preparation, all pipeline stages,
or algorithm runtime. Compare two successful builds on the same host/image,
source state, job count and cache; preserve both tool logs. The offline cache
mount test does not prove a cache hit or performance improvement.

Offline tests (Bash, Python 3, Git, clang, ld.lld and readelf):

```bash
XGC2_IMAGES_SOURCE="$IMAGES_CHECKOUT" python3 scripts/test-onboard-build.py
```

They compile small real freestanding amd64/arm64 ELF fixtures, exercise target
checks and shell boundaries, and use a **Docker test double** for export/error
paths. They do not build the actual runtime, ROS or acados. Without
`XGC2_IMAGES_SOURCE`, the four image-script tests are explicitly skipped, not
reported as image validation. The initial isolated environment has no Docker,
Rust toolchain, ROS/acados or robot; actual dual-target builds, measured cache
reuse, W09 packaging/target loading and hardware validation remain unexecuted.
See `onboard-build-validation.txt` for the checked-in offline run output.

Root acceptance must consume W06's real development environment and W09's
reviewed target loader (including its outstanding renderer/ELF acceptance
integration), build both targets, repeat on the same machine, package and load
on the target. Do not substitute unit tests or an online process for those
checks. No APT/server publication, remote build service, station experiment
configuration or physical actuator operation is part of this driver.

## Companion local image entrypoint

In xgc2-images, omitted platform preserves the existing `:base-local` tag.
Explicit `linux/amd64` / `linux/arm64` use `:base-local-amd64` /
`:base-local-arm64`, preventing overwrite. The base profiles/package lists are
still W06-owned; ordinary builds consume the Dockerfile parent default rather
than overriding W06 PR #6 with a second ros-base list. The SITL layer is only admitted for amd64; its local parent
must match the requested architecture. All built images are inspected for the
requested platform. This does not assert that all W06 base profiles have been
functionally validated on arm64. Local base builds retain the existing explicit
recipe's dependency downloads; they do not push an image or publish APT.
