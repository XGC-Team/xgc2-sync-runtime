# Native deployment renderer v1

The release-owned `xgc-rt-render` selects one of two frozen, single-robot five-module compositions. Deployment validation and native control acceptance are separate gates. Both profiles use a 1 ms period and a 10 ms controller watchdog budget; this does not establish a 1 ms execution deadline. Native functional evidence is listed in [the validation record](validation/native-20260926/README.md).

Executable: `xgc-rt-render` (same bundle/bin directory as xgc-rt-host). The existing xgc-rt-host `--manifest` CLI is unchanged.

```
xgc-rt-render describe
xgc-rt-render describe --composition-id uav-control-px4-local-native-hover/v1
xgc-rt-render prepare --bundle-root /opt/xgc2/sync-runtime --state-root /absolute/private/test-state --deployment-json JSON
xgc-rt-render run --bundle-root /opt/xgc2/sync-runtime --deployment-json JSON
```

`describe` without a selector retains `uav-control-dfbc-native-hover/v1`. The
explicit selector accepts only that ID and `uav-control-px4-local-native-hover/v1`.
The latter uses the controller's PX4_LOCAL backend to consume PositionTarget
messages on the declared `alg_setpoint_topic`; it has no built-in DMPC planner.
The reference module remains available in this five-module graph, but its
analytic stream does not supply the PX4_LOCAL tracking input. The DFBC profile
uses its existing reference-generator path and ignores algorithm PositionTargets.

Each bundle pins exactly one composition SHA. Use the selected `describe` result
when packaging and verifying it; a renderer supporting both profiles does not
allow either graph to run against a bundle pinned to the other. `prepare` and
`run` select exclusively from the deployment JSON ID/SHA, with the same strict
configuration and five artifact roles for both profiles. Arbitrary backend,
graph or plugin-role overrides are not accepted.

`run` selects exactly one nonempty target-owned `XGC_AGENT_MANAGED_ROOT` or `XGC_CORE_MANAGED_ROOT` (both permitted only if equal), creates `<managed-root>/sync-runtime/<session_id>/<node_id>/generations/<nonce>/node.toml`, and execs the verified bundle host with that manifest. The launcher invokes no shell, removes inherited ROS_HOSTNAME, sets ROS_MASTER_URI/ROS_IP from the frozen configuration and sets ROS_NAMESPACE to the selected namespace. The host inherits an exclusive identity lock for its entire process lifetime. `prepare` is the isolated render/validation command and releases the lock on exit; it is not the managed launch entry.

Deployment JSON, deny unknown/duplicate fields, nulls and coercion. The envelope and its embedded JSON configuration each contain exactly one JSON value. The full CLI input is at most 64 KiB. All SHA fields are 64 lowercase hexadecimal characters.

- `session_id` and `node_id`: `[A-Za-z0-9][A-Za-z0-9_-]{0,63}`.
- `robot_namespace`: one segment, `[A-Za-z][A-Za-z0-9_]{0,63}`.
- Absolute topic/service names: leading `/`, then one or more `/`-separated segments `[A-Za-z][A-Za-z0-9_]*`, at most 128 bytes total. `/command` is valid. All 16 roles have distinct names.
- Provenance identifiers: nonempty, trimmed, at most 256 UTF-8 bytes, no ASCII control bytes or DEL.
- ROS master URI: `http://IPv4:port`, a literal IPv4 address and decimal port in 1..65535, with no trailing slash, userinfo or hostname. `ros_ip` is an IPv4 literal. Unspecified, multicast and broadcast addresses are rejected; loopback is permitted for a local deployment.



```json
{
  "schema_version": 1,
  "session_id": "real-session-uuid",
  "node_id": "uav2",
  "robot_namespace": "uav2",
  "platform": "linux-amd64",
  "bundle_sha256": "<SHA256 exact DEPLOYMENT-BUNDLE.json bytes>",
  "composition_id": "uav-control-dfbc-native-hover/v1",
  "composition_sha256": "<from xgc-rt-render describe>",
  "configuration_sha256": "<SHA256 exact UTF-8 configuration_json string bytes>",
  "configuration_json": "<JSON string conforming to Configuration below>"
}
```

Configuration (the fields shown are required; `simulation` is present only for `ros1-sim`; no default calibration or ROS addresses):

```json
{
  "input_time_domain": "wall-unix",
  "ros_master_uri": "http://172.30.251.251:11311",
  "ros_ip": "172.30.251.102",
  "takeoff_altitude_m": 2.3,
  "topics": {
    "imu_topic": "/uav2/mavros/imu/data_raw",
    "pose_topic": "/uav2/pose",
    "vision_pose_topic": "/uav2/mavros/vision_pose/pose",
    "rigid_state_estimate_topic": "/uav2/alg/state_estimator/state",
    "fcu_state_topic": "/uav2/mavros/state",
    "local_pose_topic": "/uav2/mavros/local_position/pose",
    "local_velocity_topic": "/uav2/mavros/local_position/velocity_local",
    "fcu_imu_topic": "/uav2/mavros/imu/data",
    "battery_topic": "/uav2/mavros/battery",
    "command_topic": "/uav2/command",
    "alg_setpoint_topic": "/uav2/alg/setpoint_raw/local",
    "attitude_target_topic": "/uav2/mavros/setpoint_raw/target_attitude",
    "setpoint_topic": "/uav2/mavros/setpoint_raw/local",
    "attitude_rate_topic": "/uav2/mavros/setpoint_raw/attitude",
    "status_topic": "/uav2/custom/statustext",
    "fcu_request_topic": "/uav2/mavros"
  },
  "calibration": {
    "verified": false,
    "field_offset_xyz": [0.0, 0.0, 0.0],
    "field_offset_rpy": [0.0, 0.0, 0.0],
    "imu_to_vrpn_marker_xyz": [0.0, 0.0, 0.0],
    "imu_to_vrpn_marker_rpy": [0.0, 0.0, 0.0],
    "provenance": {
      "kind": "unverified",
      "source_id": "frozen-robot-configuration-id",
      "source_sha256": "<64 lowercase hex>",
      "robot_asset_id": "selected-robot-asset-id"
    }
  }
}
```

Provenance kind is `unverified`, `simulation-model`, or `measured-calibration`. `verified` must be false exactly for `unverified`; other kinds require true plus explicit provenance. This records caller-supplied frozen evidence, not independent verification of its physical truth. The example intentionally does not claim a verified transform. All non-pose/non-command topic roles must be under the selected robot namespace; actual pose and command may deliberately name external/shared routes. All names are canonical absolute ROS names. No topic-role value is inferred from the test fixture.

Packaging writes a separate **DEPLOYMENT-BUNDLE.json** (exact bytes SHA binds it) next to the existing BUNDLE.json:

```json
{
  "schema_version": 1,
  "platform": "linux-amd64",
  "composition_sha256": "<from describe>",
  "host": {"path": "bin/xgc-rt-host", "sha256": "<sha>"},
  "plugins": {
    "ros_io": {"path": "plugins/libros_io.so", "sha256": "<sha>"},
    "rigid_state": {"path": "plugins/libest_rigid_state.so", "sha256": "<sha>"},
    "hover_thrust": {"path": "plugins/libest_hover_thrust.so", "sha256": "<sha>"},
    "controller": {"path": "plugins/libctl_px4.so", "sha256": "<sha>"},
    "reference": {"path": "plugins/libref_trajectory.so", "sha256": "<sha>"}
  },
  "libraries": [{"path": "lib/ACTUAL-FILE.so", "sha256": "<sha>"}],
  "links": [{"path": "lib/SONAME.so.0", "target": "ACTUAL-FILE.so"}]
}
```

Host/plugins/libraries are indexed actual regular ELF files. Bundle links are separately indexed, constrained to the same directory, and resolve to an indexed real file; source/target cannot escape the bundle. Every indexed byte is verified before generation/exec. External ROS and system dependencies remain installed OS dependencies, not forged vendored files. The runtime launch sets LD_LIBRARY_PATH to bundle/lib plus /opt/ros/noetic/lib.

`describe` supplies the selected canonical composition document and SHA. Each composition is built into the renderer, sets a 1 ms period and a controller 10 ms step budget, self-only roster, five actual module roles and the fixed backend settings plus explicit takeoff/calibration values. No observer, run_for_ms, fixed ROS namespace or automatic extrinsic verification. The optional simulator clock changes Session time without changing either graph. These control-only profiles do not include distributed synchronization, DMPC or UGV control.

State contract: same Session/node identity with different frozen deployment JSON is refused (including bundle/config/composition changes). Same identity/input creates a fresh audit generation; an active `run` blocks a second writer. Identity and generation writes are atomic under a no-symlink directory descriptor lock. Receipt includes deployment/manifest/bundle/config/composition hashes, generation and exact manifest/audit paths; prepare success is not live-module readiness.

Catalog can use ordinary direct executable `xgc-rt-render run`, the fixed bundle-root argument and `${deploymentJson}` as one argv value. Put normal strict parameter policy on deploymentJson and scalar namespace; require namespace to equal the envelope. The controller claim remains namespace-only, so different Sessions cannot own two actuator producers in one namespace. No Core-only Materializer, custom loader or target-dependent normalized absolute path is necessary. Same definition/policy must be installed on Core and Agent. Artifact pins must identify installed binaries; source revisions alone do not establish binary equivalence.

## Filesystem and process ownership

Use an absolute, clean target-owned private state directory. Every directory component is opened through a descriptor with O_NOFOLLOW; published files are regular files created exclusively with mode 0600, directories use 0700. Dot/parent/empty components and symlinked state/artifacts are rejected. Named pipes cannot block the artifact check. Directory rename uses Linux renameat2(RENAME_NOREPLACE), followed by directory fsync. These mechanisms require Linux amd64 and /proc mounted.

Identity deployment.json is frozen before creating a generation. A failed render/publish can leave the identity record or a hidden staging directory, which is never reused; an interrupted launch cannot overwrite an older manifest or audit. The same input can be retried; changed input requires a new Session or node identity. The release also keeps an exclusive flock while the host is running. Its FD is close-on-exec by default and becomes inheritable only for that selected launch; unrelated subprocesses cannot keep another identity locked.

The bundle root is a trusted immutable installation, supplied by the executable definition rather than the deployment JSON. The target must prevent concurrent in-place writes to that installation. All indexed bytes and declared library symlinks are checked before publication, and plugin SHA pins are passed into the real Manifest. Host exec uses the same verified file descriptor, so replacing its directory entry cannot redirect that launch. The descriptor is retained in the host. This is integrity validation against caller-pinned content, not a signature or independent approval of that caller's artifact/calibration claims.

Only the declared library links may be symlinks; each points directly to an indexed regular file in the same directory. This is not an ELF dependency resolver: packaging must index the actual custom dependency closure, and the target must supply compatible OS/ROS dependencies. Public file paths may be logged; configuration must not contain credentials or secret ROS URI components.

## Time, readiness and acceptance boundaries

The caller declares `input_time_domain: wall-unix` for physical wall-time inputs, with no `simulation` field. For simulator-stamped inputs it declares `ros1-sim` and supplies the following frozen block inside the same configuration JSON:

```json
"simulation": {
  "epoch_ns": 2000000000,
  "topic": "/clock",
  "expected_publisher": "/experiment_world",
  "world_instance_id": "selected-world-generation",
  "startup_timeout_wall_ms": 5000,
  "stale_after_wall_ms": 100,
  "max_advance_ns": 50000000,
  "poll_wall_ms": 5,
  "queue_capacity": 256
}
```

`epoch_ns` is the same future simulator time frozen by the experiment's Run coordinator for every participant. It is never derived from each node's message arrival. The remaining fields map directly to the host [clock source](clock-source.md); its kind is fixed to `ros1_sim` and its pinned plugin is `ros_io`. Manifest validation applies the existing authority and timing limits. Explicit null, a missing block for `ros1-sim`, or a simulation block for `wall-unix` is rejected. The generation receipt reports the selected domain. The unchanged `describe` result still describes the default wall-time profile.

The ESKF, hover estimator, controller and reference generator use the selected Session clock. The ROS source requires `/use_sim_time=true` and preserves sensor stamps. Pause prevents domain steps and outputs while Stop/liveness use steady time; a reset or authority fault ends the Session. A separate experiment ROS master/world and common clock authority work for both colocated and per-robot placements. Keep clock/plant traffic separate from impaired neighbor-radio traffic when evaluating neighbor communication. Accepting configuration does not independently verify that every live publisher uses the selected time domain.

`prepare` validates schema, files and the real Manifest resolver; its `live_readiness` is always false. A `run` receipt is also not module readiness: host module lifecycle/health (under `<audit_path>/<node_id>/health.jsonl`) and actual output/input evidence must be checked by the managing workflow. The process remains alive until the ordinary managed-process stop signal or an actual host failure; there is no fixture observer or 180-second deadline.

Both compositions are self-only, with native ESKF, native hover-thrust, the selected controller backend, reference generation and ROS I/O. Native hover-thrust owns its internal channel; the ROS adapter is not a second hover-thrust producer. Neither profile includes DMPC rounds/planning, multi-robot synchronization, UGV control or physical-flight acceptance. The surrounding catalog must claim the robot namespace exclusively across Sessions, not claim only the Session/node identity.

## Build and boundary tests

```
cargo build -p xgc-rt-host --bin xgc-rt-render --release
cargo test -p xgc-rt-host --test deployment
```

The deployment suite has no optional environment skip. It checks frozen identities, topology, explicit topics/transforms, strict JSON, wrong pins and unsupported time/platform/composition, path/symlink/FIFO rejection, atomic generations, concurrent writers and lifecycle locks. A tiny compiled C ELF probe verifies actual exec/PID/ROS environment and lock lifetime; it is deliberately not a ROS/native control acceptance claim. The separate native module gates and Agent/robot deployment acceptance must run against the exact final built plugins.

## Numerical HIL composition

`uav-dmpc-numeric-hil/v1` is a fixed model-only graph: `ros_io` supplies the
scene/clock edge, `station_io` supplies the authorized command/mission edge,
`dmpc_rounds` exchanges plans over the host's explicit Zenoh transport,
`plan_dmpc` produces PositionTarget, and `numeric_vehicle` consumes that PVA
and returns its own paired state and controller state. There is no actuator,
Arm interface, IMU, estimator, hover estimator, or physical controller in this
composition. The receipt has `role: "numeric-hil"` and `actuator_namespace: null`.
The two existing control compositions retain their template bytes and
`role: "control"`; their actuator namespace is the selected robot namespace.

The HIL configuration is selected by its compiled composition ID. It does not
accept the control profile's calibration, takeoff, or sensor-topic fields:

```json
{
  "input_time_domain": "wall-unix",
  "ros_master_uri": "http://127.0.0.1:11311",
  "ros_ip": "127.0.0.1",
  "epoch_ns": 1900000000000000000,
  "members": [
    {"uav_id": 1, "robot_namespace": "uav1", "planner_node": "board-a", "control_node": "board-a"},
    {"uav_id": 2, "robot_namespace": "uav2", "planner_node": "board-b", "control_node": "board-b"}
  ],
  "mission_authority_node": "board-a",
  "radio": {"listen": ["tcp/127.0.0.1:17442"], "connect": ["tcp/127.0.0.1:17441"]},
  "station": {"robot_id": "xgc2e-12345678901234567890", "zenoh_connect": "tcp/127.0.0.1:17457", "command_socket": "/run/xgc2/hil-board-b.sock"},
  "scene": {
    "snapshot_topic": "/experiment/scene/snapshot",
    "state_topic": "/experiment/scene/state",
    "timeline_ack_topic": "/uav2/dmpc/timeline_ack",
    "timeline_status_topic": "/uav2/dmpc/timeline_status"
  },
  "planner": {"manifest": "scenarios/dmpc_comprehensive/uav2.yaml", "algorithm": "legacy", "scene_id": "dmpc-uav8_comprehensive", "chain_n": 3, "state_dim": 9, "horizon": 40, "sampling_time": 0.1},
  "initial_position": [1.2, -3.4, 0.0],
  "initial_velocity": [0.1, 0.2, 0.0]
}
```

This two-member example shows the renderer's membership contract. The rendered
`plan_dmpc` configure text always has `manifest`, `self_id`, `timeline_authority`,
and `scene_id`. `manifest` is the ParamManifest yaml path (`namespace`,
`node_namespace`, `args`, `loads`, `params`). `scene_id` is that object's scene
document id, `dmpc-uav8_comprehensive` for the Comprehensive document. Optional
`algorithm`, `chain_n`, `state_dim`, `horizon`, and `sampling_time` are copied
when the object sets them. `fleet_count` is not written; `num_uavs` in the
manifest params or args is the fleet, and Comprehensive is 8. Use the real scene
slots, explicit initial state, and one shared Run epoch for a runnable deployment. A ROS simulation additionally supplies
`simulation` as described above, with its `epoch_ns` equal to the planner epoch.
Wall time forbids that block. The 1 ms host base period is unchanged; rounds use
100 ms planner time and the shared epoch.

Members are ordered experiment slots, not a discovery result. Robot identity
and node identity differ: the member record names a planner node and a control
node. This HIL graph requires them to coincide because it contains both the
planner and model. A remote planner/control pair can be represented by the
same member type, but needs its actual role-specific composition; selecting
HIL for a split pair is rejected. Existing control configurations may additionally supply `robot_member`
with those four fields; the renderer then requires `node_id == control_node`
and the matching robot namespace. The planner node cannot select that control
profile or acquire its actuator claim. Missing `robot_member` retains the
legacy single-node input contract. This identity check does not select a DMPC graph. The fixed planner, full,
and onboard SMC compositions are separate compiled IDs. Only a composition
that contains the control role can claim the actuator namespace; namespace
does not contain Session ID.

The HIL bundle has exactly `ros_io`, `numeric_vehicle`, `plan_dmpc`,
`dmpc_rounds`, and `station_io` plugin pins. Mixed control/HIL bundles are
rejected. The authority node alone binds the station command and mission
outputs; every consumer filters that same envelope origin. The rendered
station config sets `command` and `mission` from those remaining binds. Neighbor-plan
origins are planner-node roster indices, while local controller origin is the
control-node index. Legacy plan admission expects nine state components,
N+1 predicted columns, and no terminal-rest vector. Initial model state is
never overwritten with a reference pose.

The station command socket is an explicit per-instance target path (at most
100 bytes); its parent must exist before launch. Clock and scene ROS traffic
use the experiment's ROS authority. Neighbor plans use `radio` endpoints and
station telemetry uses its separate GCS endpoint. Co-located comparison runs
still use real explicit Zenoh endpoints between processes. Keep ROS world/clock
on the physics path when impairing neighbor radio; the renderer does not alter
network interfaces or create a clock bridge.

## DMPC planner, full, and onboard SMC

Three more fixed graphs use the same membership record. They do not add
hover-thrust or an attitude-rate channel. The two hover templates and the
numerical HIL graph are unchanged.

| ID | Plugins | Actuator namespace |
| --- | --- | --- |
| `uav-dmpc-native/v1` | `ros_io`, `controller`, `plan_dmpc`, `dmpc_rounds`, `station_io` | selected robot |
| `uav-dmpc-planner/v1` | `ros_io`, `plan_dmpc`, `dmpc_rounds`, `station_io` | none |
| `uav-dmpc-smc/v1` | `ros_io`, `controller`, `station_io` | selected robot |

`uav-dmpc-native/v1` requires `planner_node == control_node`. The planner and
SMC graphs require those nodes to differ, and each process must be the node
its composition names. These three graphs do not include `rigid_state` or the
`estimate` channel. SMC reads MAVROS `local_pose` and `local_velocity` from
`ros_io`, plus the planner PositionTarget through the existing 100 ms lifter
(`planning_period = 0.1`). It does not bind `hover_thrust` or `attitude_rate`.
The only actuator bindings are the existing `setpoint` and `fcu_request`
ports. The controller, not this graph, fills that setpoint as an
acceleration-only world-frame PositionTarget. State estimation and NMPC are
not part of this acceptance.

`paired_state` is the existing `ros_io` output. On the full graph,
`plan_dmpc` and `station_io` read it from `SELF`. On the split graphs the
control node's `ros_io` publishes it, and both the planner and `station_io`
read it from `control_node`. The controller has no `paired_state` port. There
is no new schema and no extra sync module. HIL still uses the 96-byte record
from `numeric_vehicle`. Frame and origin stay on the existing PX4 local pose
chain; this graph does not add a conversion.

`ros_io` `vision_pose` (`xgc.pose/1`, input) is bound to the existing `pose`
channel from `SELF` and republishes that external pose on `vision_pose_topic`.
It is not fed from `local_pose`. Full and SMC `station_io` also reads
`imu` from the `fcu_imu` channel (`xgc.imu/1`), `battery` (`xgc.battery/1`),
and `fcu_state` (`xgc.fcu_state/1`). Missing samples stay absent.

Takeoff altitude is explicit on the full and SMC configurations. There is no
rigid-state role and no calibration block. The renderer does not invent IMU
samples.
Current planner ELF for a later load is
`plan-dmpc-build/libplan_dmpc.so`
`f89b1ff176c4575e4a3c4b8b159c96bcfc5ee11754d55566af1292eba6d475b5`,
against ABI `xgc_dmpc_planner_v1.h`
`ea8b00fe4726a7cc7f7ec5b642f1335a19456c0daf8c4caeb13f3c7012560733`
(scene wire header 136, obstacle 192, part 120, vertex 24). The older bundle
copy `55c3005f04c97ef574a714dfeb09f541b84cfb3b7e11290ce99d7abd65f34ecc` is not
that ABI. This change does not launch those ELFs.
