# Z3 packaging: the sync-runtime in the robot image (decision B)

Decision B (author, 2026-09-25) builds the robot image's own software from
exact source revisions and copies it in. Nothing comes from an APT
republish, and no CI or Jenkins job builds it. The FS150 robot image already
does this for the Agent (XGC-Team/xgc2#108: `robot-container/fs150/Dockerfile`,
`scripts/build-robot-container-image.sh`):

1. a revision is resolved and must be on a remote branch;
2. `git archive` exports it;
3. the Agent is built from the export in a pinned build image;
4. the image build refuses a binary whose sha256 differs, and labels the
   image with the pins.

Z3 extends the same image with the sync-runtime bundle, using the same rules.

## What goes in

`scripts/stage-robot-bundle.sh` stages one directory, `/opt/xgc2/sync-runtime`
in the image:

| Path | Built from |
|---|---|
| `bin/xgc-rt-host` | this repository at REV, `cargo build --release --locked` |
| `bin/xgc-rt-render` | same (native deployment renderer, [native-deployment.md](native-deployment.md)) |
| `bin/xgc-rt-audit` | same (`xgc-rt-audit merge` for the Session's SyncAudit run) |
| `plugins/libtransport_zenoh.so` | same (transport plugin, `xgc_rt_transport_v1`) |
| `plugins/libdmpc_rounds.so` | same (rounds and peer admission) |
| `lib/libformation_generator_dmpc_{core,params,config}.so` | academic at AREV (`ros1_ws/src/planner`, `src/common`, `tests`), `formation_generator/standalone`, installed to a private prefix |
| `plugins/libplan_dmpc.so` | `scripts/build-plan-dmpc.sh --core-prefix` that prefix |
| `plugins/libctl_px4.so` (`--with-ctl-px4`) | `scripts/build-ctl-px4.sh`, see gaps |
| `SHA256SUMS` | `sha256sum` of every file above |
| `SOURCE-PINS.json` | the revisions, toolchains, every external library a staged ELF loads (with sha256), and every staged file's sha256 |

Both REV and AREV must be commits on a remote branch. The script builds only
from `git archive` exports, never from a working tree, and refuses an
existing output directory. It fails if a staged ELF has an unresolved
library.

The manifests then name the plugins with `sha256`, as the host already
enforces for `[[plugin]]` and `[transport]` (`path` + `sha256`, Z2e). A
robot's node manifest therefore pins the exact binaries in the image.
`xgc-rt-render` binds the host, plugins and libraries of its frozen
compositions to `DEPLOYMENT-BUNDLE.json`. The DMPC fleet composition below
extends that index with `transport` and `planner` entries.

## Image fragment

`packaging/robot-image/sync-runtime.Dockerfile.fragment` goes after the
Agent in the FS150 Dockerfile. The image build script stages the bundle into
the build context's `.build/sync-runtime/` and passes three arguments:
`XGC2_SYNC_RUNTIME_REVISION`, `XGC2_ACADEMIC_REVISION`, and the sha256 of
`SOURCE-PINS.json`. The fragment then:

- checks that pins file hash;
- runs `sha256sum --check` over every file;
- requires both revisions in the pins file;
- rejects symlinks;
- makes the tree read-only;
- runs `xgc-rt-render describe` once as a load check;
- labels the image with the pins (`io.xgc2.robot-image.sync-runtime-*`).

In `scripts/build-robot-container-image.sh`, next to the Agent step, the
image build script gains a step that:

1. resolves `--sync-runtime-revision` and `--academic-revision` (both default
   to what the station's Core pins);
2. runs `stage-robot-bundle.sh` inside the same pinned focal build image;
3. passes the three arguments.

## Sandbox evidence (2026-09-27)

Command:
`stage-robot-bundle.sh --revision 6eab902 --academic-revision 895d453b --with-ctl-px4`
(runtime main tip, academic main tip).

- It staged 11 files in 4 min 47 s. Every library resolved on the launch
  search path (bundle `lib/`, acados `lib/`, system): the DMPC and PX4 cores
  from `lib/`, and 19 external libraries recorded with their sha256.
- `libctl_px4.so` built from the pinned export is byte-identical (sha256
  `e5840f49…`) to the one the fleet tests build from the working tree.
  `libplan_dmpc.so` differs only by the install-prefix rpath.
- The staged `bin/xgc-rt-host` ran with an empty environment and only
  `LD_LIBRARY_PATH=bundle/lib:acados/lib:toolchain`. It loaded the
  sha256-pinned `libtransport_zenoh.so`, `libdmpc_rounds.so` and
  `libplan_dmpc.so` from its manifest (Zenoh on TCP). plan-dmpc configured
  from a knot_fs150 manifest, dmpc-rounds ran 15 beats, and the host
  exited 0.
- The DMPC core's state-machine logger writes to stdout, after the host's
  JSON summary, which is also on stdout. An Agent that parses the summary
  must read one JSON value, or the core must log to stderr.

## Gaps before a station build (Z3 Phase B)

1. **Toolchain.** The sandbox bundle is built with conda gcc 15 and yaml-cpp
   0.8. The FS150 image is Ubuntu 20.04 / ROS Noetic (gcc 9, yaml-cpp 0.6).
   The bundle must be staged inside the pinned focal build image
   (`xgc2-build-focal-dev`, as for the Agent) with acados installed there. A
   sandbox-built bundle is not ABI-compatible with the image and must not be
   copied in.
2. **acados.** The runtime needs `libacados`, `libhpipm`, `libblasfeo` and
   the generated solver libraries. Where they come from in the image (a
   pinned acados source build in the builder stage, copied to
   `/opt/acados`) is not decided yet.
3. **PX4 controller core.** `ctl-px4` links `libpx4_multirotor_controller_core.so`,
   which this script copies from `PX4_CORE_LIB_DIR` and records under
   `not_source_pinned`. The current controller (`xgc2-multirotor-controller`
   noetic) also needs `xgc2_math >= 0.5.9` (SMC tracking),
   `xgc2_state_machine`, an `xgc2_acados` CMake package and, when its
   generated NMPC solver is absent, `casadi` plus `acados_template` to
   regenerate it. To be source-pinned, the script needs the controller and
   xgc2-math revisions as further pins, built from their ROS-free CMake in the
   builder stage. An `XGC2_MATH_INCLUDE_OVERRIDE` used for `ctl-px4` is also
   recorded under `not_source_pinned`.
4. **Composition.** `xgc-rt-render` knows only the two single-robot control
   compositions. The DMPC fleet robot needs a third frozen composition:
   `ros-io`, estimators, `ctl-px4`, `plan-dmpc`, `dmpc-rounds`, and the Zenoh transport on
   the radio address. Its roster, peers, `(E0, P)` and link come from the
   Session (see [z3-session-artifacts.md](z3-session-artifacts.md)), so Core
   renders one deployment JSON per robot instead of hand-written manifests.
