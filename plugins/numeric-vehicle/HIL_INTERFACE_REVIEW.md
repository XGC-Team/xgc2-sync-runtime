# W24: SMC input boundary — Root review required

Tracks [XGC-Team/xgc2-harness#189](https://github.com/XGC-Team/xgc2-harness/issues/189).
This is a **partial, pre-integration delivery**, not a completed full HIL graph.
The task explicitly requires a port/units/time review before wiring. No Root
approval was present in #189 or the parent #164 when the interface was inspected.

## Scope and source identity

Runtime base: `6eab902568ae5a393b4e3b23565edb04c13b6ff6` (`main`). The input
adapter below is deliberately NOT imported by `src/lib.rs`: only its tests
include it. No deployment/composition, model integration, controller, ABI,
solver budget, controller gain, dependency, estimator or physical endpoint is
changed. The existing planner-only graph and `Model` remain byte-for-byte intact.
There are no deleted product paths in this preparatory PR.

Read-only controller inspection: `XGC-Team/xgc2-multirotor-controller` at
`4074bb0daaafd366a4a8c90abd253558640ea783`. This is an observed source revision,
**not an assertion that it is the W33 historical-success binary or the binary
loaded in a deployment bundle**. Root must reconcile that correspondence first.

Implementation authority is the task's pinned architecture/implementation/work
package documents at `xgc2-dev-memory@a5f23ceaf716c2afddf0f792d6722ac0cd0f3d20`.
W23 has submitted [runtime #25](https://github.com/XGC-Team/xgc2-sync-runtime/pull/25)
(head `f3b042a3cf61511d89b8b3ebab28859f0467500c`) and
[xgc2 #121](https://github.com/XGC-Team/xgc2/pull/121). At review time runtime #25
is open/unmerged. This PR does not cherry-pick it or claim to have consumed its
runtime deployment. Final integration must consume the reviewed W23/W09 work.

## Observed interface, not an invented attitude/thrust controller

The [runtime wrapper](../ctl-px4/ctl_px4.cpp) publishes
`xgc.position_target/1` on `setpoint` from `controller.getSetpoint()` and assigns
`stamp = t`, the controller publication time in Session seconds.
The [SMC strategy](https://github.com/XGC-Team/xgc2-multirotor-controller/blob/4074bb0daaafd366a4a8c90abd253558640ea783/px4_multirotor_controller/src/tracking/smc_acceleration_strategy.cpp)
returns `LocalSetpoint`, fills `ax/ay/az`, sets frame 1, and reads measured local
position/velocity. Its [header](https://github.com/XGC-Team/xgc2-multirotor-controller/blob/4074bb0daaafd366a4a8c90abd253558640ea783/px4_multirotor_controller/include/px4_multirotor_controller/tracking/smc_acceleration_strategy.h)
describes world acceleration `a_ref + feedback`, without gravity. It does not
produce body rates or normalized thrust.

[Controller types](https://github.com/XGC-Team/xgc2-multirotor-controller/blob/4074bb0daaafd366a4a8c90abd253558640ea783/px4_multirotor_controller/include/px4_multirotor_controller/common/types.h)
define `kSmcAccelerationTypeMask = 3135`: all P/V axes and yaw/yaw-rate ignored,
all acceleration axes enabled, FORCE clear. Frame 1 means **local world ENU at
this repository's pre-MAVROS boundary**. Do not infer a NED conversion merely
from a MAVLink frame number. The [wire schema](../../abi/include/xgc_schemas_v1.h)
is little-endian, 104 bytes; acceleration begins at byte 56, mask at 96, frame at
98, and the five reserved bytes at 99 must be zero.

`src/controller_input.rs` is a side-effect-free decoder for this exact observed
Custom1 output shape. It preserves the stamp and acceleration, rejects PVA,
force/body-frame/other masks, invalid lengths/reserved bytes, nonfinite active
acceleration, and nonpositive/nonfinite timestamps. Disabled P/V/yaw fields are
not read. There is no fallback to planner PVA, extra feedback, gravity addition,
axis rotation, actuator call, hold duration, integration, or state injection.
Successful decoding alone does NOT establish source identity, freshness,
Session membership, representability in nanoseconds, or permission to apply a
command. Those checks belong at the eventual runtime/model boundary.

## Port diagram proposed for Root review (NOT connected in this PR)

```text
station authority command ---------> existing ctl-px4 state machine
station authority mission ---------> existing rounds / planner
planner position_target (PVA) ------> ctl-px4.alg_setpoint ONLY
model local_pose / local_velocity -> ctl-px4 measured-state inputs
ctl-px4.setpoint (acceleration) ----> reviewed model control input ONLY
model paired_state ----------------> planner + station telemetry
ctl-px4.status --------------------> planner + rounds + station
ctl-px4.fcu_request ----------------> model-local request handling, if approved
                                      NEVER ros_io / MAVROS / physical FCU
ros_io ----------------------------> scene/timeline I/O ONLY
```

| Boundary | Units/frame | Time/source | Status |
| --- | --- | --- | --- |
| Planner -> controller | PVA: m, m/s, m/s^2; world ENU frame 1 | Planner stage-effective `stamp`, not receipt time; explicit planner origin | Existing W23 interface; not rewired here |
| Controller -> model | Acceleration: m/s^2; frame 1, mask 3135; no gravity addition | Controller publication `stamp = t`; retain envelope produce/receive and model application times separately | Decoder candidate only; Root model/timing approval pending |
| Model -> controller | `xgc.pose/1`, `xgc.twist/1` from the SAME integrated state | Current shared Session time, explicit local model origin | New model outputs not implemented |
| Model -> planner/station | Existing `xgc.dmpc.paired_state/1`, 96 bytes | Same integrated p/v and current Session time | Existing planner-only path retained; no synthetic trajectory states |
| Controller -> planner/rounds/station | `xgc.controller_status/1` | Real control-region status from `ctl-px4` | Must replace model pseudo-controller status in the future full graph ONLY |
| Controller request / model FCU state | Existing `xgc.fcu_request/1`, `xgc.fcu_state/1` | Model-local state transitions, never a real service | Bootstrap/ack semantics require Root decision |

Pose schema uses `q_wxyz`; the current numeric paired-state packing writes an
identity quaternion as `[0,0,0,1]`. These byte layouts must not be blindly copied
into each other. A lightweight translational model has no attitude dynamics;
Root must approve the attitude/IMU facts needed by the original controller's
readiness checks rather than injecting a passing state.

## Root decisions before any full wiring

1. Confirm the exact loaded original controller and parameter/binary provenance;
   confirm its SMC output is the observed frame-1, mask-3135 net acceleration.
   Approve the existing dynamics to reuse at this boundary. Do not add the
   planner-only `w=4` tracking loop on top of SMC or tune gains to make HIL pass.
2. Resolve **non-Custom1 states**. The original controller also handles takeoff,
   hover and landing; the strict acceleration decoder intentionally rejects
   other output masks. Decide the supported original model-side behavior and
   sensor/readiness/request feedback, without a new controller or fake paired
   state. The wrapper's command table does not include numeric-model `prepare`
   or `stop`; these cannot silently be treated as original controller commands.
3. Approve output application, expiry and missing-output behavior. The host
   updates at 1 ms; that does not mean a new control output every 1 ms. Inspected
   controller types contain a 0.01 s strategy period and a 30 Hz local-setpoint
   publish constant. Record the actual loaded controller cadence. Do NOT reuse
   the planner's fixed 100 ms segment lifetime, the model's 10 ms wake, or a
   guessed one-tick timeout as a controller-validity contract.
4. Confirm model-only FCU-request handling and actual status/feedback ordering.
   No `ros_io.setpoint`, `ros_io.fcu_request`, `attitude_rate`, estimator,
   hover-thrust or actuator binding is permitted in the future full HIL graph.

If a separately identified full HIL composition is approved, registration in
`deployment/mod.rs`/the release catalog and bundle acceptance is a minimal hunk
for Root/W09, **not this owner's write area**. W24 will own its HIL renderer and
composition. No speculative registration or fake production API is added here.
Existing `uav-dmpc-numeric-hil/v1` must remain an explicitly planner-only test.

## Tests and actual evidence

Executed in the isolated worker environment:

```sh
python -m py_compile plugins/numeric-vehicle/tests/test_planner_only_isolation.py
python -m unittest discover -s plugins/numeric-vehicle/tests -p 'test_*.py' -v
```

Result: **12 Python tests passed**. They parse the actual existing TOML template,
check the planner-only role set, scene-only ROS edge, model feedback/command
origins and unchanged period/wake/budget. Mutation negatives reject hardware
ports (including innocently named channels), unknown ROS ports, a real controller
injected into the old graph, duplicate/missing/unknown roles and a silently added
model control port. These are static graph tests, NOT actual controller execution,
plant motion, loaded-binary isolation or deployment acceptance.

The local read-only TOML copy was checked against the GitHub blob object ID
`48dbbc0ed164eea42d4e78ad942bd1b37bf6fa57` before testing. This verification is
worker evidence only, not a new runtime hash/signature mechanism. Direct git clone
failed with `Could not resolve host: github.com`; this was a source reconstruction
for targeted testing, not a claimed complete checkout/build.

The Rust decoder has 10 unit tests, including all 65,536 masks and all 256 frame
values. The Cargo test target adds 2 checks against the ACTUAL existing
`numeric_vehicle::model::Segment`: SMC output is not a PVA segment, and planner
PVA is not controller output. **These Rust tests were NOT executed here**:
`rustc` and `cargo` are absent. No successful compilation is claimed.

Root/build-environment replay (from the repository root):

```sh
# Standard-library-only decoder test; this does not start a plugin or model.
rustc --edition=2021 --test plugins/numeric-vehicle/src/controller_input.rs -o /tmp/w24-controller-input
/tmp/w24-controller-input
# Includes the old-model boundary negatives (12 Rust tests total).
cargo test -p numeric-vehicle --test controller_boundary
# Preserve the existing real-host / cdylib planner-only suite as a separate result.
plugins/numeric-vehicle/test.sh
```

## Full-HIL acceptance still outstanding

After interface approval and actual W23 integration, record an isolated run of
the real planner -> original controller -> reviewed model. For identical initial
state/mission, disconnect only the controller output, zero/change one acceleration
axis, and poison ignored P/V fields in the controller wire payload. Model motion
must follow the acceleration perturbation, not the planner PVA or ignored fields;
missing output must follow the approved policy. Decoder tests do not satisfy this
motion requirement.

Use the existing Session/Run audit for planner effective/produce times, controller
input/application and output stamps, model application/dt and feedback stamps,
controller status and runtime budgets. Distinguish wire loss from solve failure.
Reject duplicate/spoofed source bindings and any physical service/actuator edge
in the fully rendered graph; no budget/ABI/solver/SMC adjustments to obtain a pass.
Replay the same approved composition/package in container and on board. Actual
board solving/control/integration timing, radio behavior, task completion and
hardware isolation remain **unmeasured**. Root alone schedules the no-power
hardware HIL verification and closes #189 after independent acceptance.
