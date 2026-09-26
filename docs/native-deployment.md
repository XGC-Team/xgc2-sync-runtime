# Native deployment renderer v1

The release-owned `xgc-rt-render` renders a frozen, single-robot five-module composition. Deployment validation and native control acceptance are separate gates. This profile uses a 1 ms period and a 10 ms controller watchdog budget; it does not establish a 1 ms execution deadline. Native functional evidence is listed in [the validation record](validation/native-20260926/README.md).

Executable: `xgc-rt-render` (same bundle/bin directory as xgc-rt-host). The existing xgc-rt-host `--manifest` CLI is unchanged.

```
xgc-rt-render describe
xgc-rt-render prepare --bundle-root /opt/xgc2/sync-runtime --state-root /absolute/private/test-state --deployment-json JSON
xgc-rt-render run --bundle-root /opt/xgc2/sync-runtime --deployment-json JSON
```

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

Configuration (all fields required; no default calibration or ROS addresses):

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

`describe` supplies the canonical composition document and SHA. Composition is built into the renderer, sets a 1 ms period and a controller 10 ms step budget, self-only roster, five actual module roles and source-default algorithm settings except the explicit takeoff/calibration values. No observer, run_for_ms, fixed ROS namespace or automatic extrinsic verification. No Sim clock, distributed synchronization, DMPC or UGV claim.

State contract: same Session/node identity with different frozen deployment JSON is refused (including bundle/config/composition changes). Same identity/input creates a fresh audit generation; an active `run` blocks a second writer. Identity and generation writes are atomic under a no-symlink directory descriptor lock. Receipt includes deployment/manifest/bundle/config/composition hashes, generation and exact manifest/audit paths; prepare success is not live-module readiness.

Catalog can use ordinary direct executable `xgc-rt-render run`, the fixed bundle-root argument and `${deploymentJson}` as one argv value. Put normal strict parameter policy on deploymentJson and scalar namespace; require namespace to equal the envelope. The controller claim remains namespace-only, so different Sessions cannot own two actuator producers in one namespace. No Core-only Materializer, custom loader or target-dependent normalized absolute path is necessary. Same definition/policy must be installed on Core and Agent. Artifact pins must identify installed binaries; source revisions alone do not establish binary equivalence.

## Filesystem and process ownership

Use an absolute, clean target-owned private state directory. Every directory component is opened through a descriptor with O_NOFOLLOW; published files are regular files created exclusively with mode 0600, directories use 0700. Dot/parent/empty components and symlinked state/artifacts are rejected. Named pipes cannot block the artifact check. Directory rename uses Linux renameat2(RENAME_NOREPLACE), followed by directory fsync. These mechanisms require Linux amd64 and /proc mounted.

Identity deployment.json is frozen before creating a generation. A failed render/publish can leave the identity record or a hidden staging directory, which is never reused; an interrupted launch cannot overwrite an older manifest or audit. The same input can be retried; changed input requires a new Session or node identity. The release also keeps an exclusive flock while the host is running. Its FD is close-on-exec by default and becomes inheritable only for that selected launch; unrelated subprocesses cannot keep another identity locked.

The bundle root is a trusted immutable installation, supplied by the executable definition rather than the deployment JSON. The target must prevent concurrent in-place writes to that installation. All indexed bytes and declared library symlinks are checked before publication, and plugin SHA pins are passed into the real Manifest. Host exec uses the same verified file descriptor, so replacing its directory entry cannot redirect that launch. The descriptor is retained in the host. This is integrity validation against caller-pinned content, not a signature or independent approval of that caller's artifact/calibration claims.

Only the declared library links may be symlinks; each points directly to an indexed regular file in the same directory. This is not an ELF dependency resolver: packaging must index the actual custom dependency closure, and the target must supply compatible OS/ROS dependencies. Public file paths may be logged; configuration must not contain credentials or secret ROS URI components.

## Time, readiness and acceptance boundaries

The caller explicitly declares `input_time_domain: wall-unix`. The ESKF, hover-thrust estimator, controller and reference generator use the host Session clock. This release does not inspect /use_sim_time or translate Gazebo /clock, and accepting the declaration does not prove a live upstream publisher actually obeys it. Bring-up must check its actual timestamp domain independently before control use.

`prepare` validates schema, files and the real Manifest resolver; its `live_readiness` is always false. A `run` receipt is also not module readiness: host module lifecycle/health (under `<audit_path>/<node_id>/health.jsonl`) and actual output/input evidence must be checked by the managing workflow. The process remains alive until the ordinary managed-process stop signal or an actual host failure; there is no fixture observer or 180-second deadline.

The composition is self-only, with native ESKF, native hover-thrust, DFBC controller, reference generation and ROS I/O. Native hover-thrust owns its internal channel; the ROS adapter is not a second hover-thrust producer. This profile does not include DMPC rounds/planning, multi-robot synchronization, UGV control, physical-flight acceptance, or a simulation-clock profile. The surrounding catalog must claim the robot namespace exclusively across Sessions, not claim only the Session/node identity.

## Build and boundary tests

```
cargo build -p xgc-rt-host --bin xgc-rt-render --release
cargo test -p xgc-rt-host --test deployment
```

The deployment suite has no optional environment skip. It checks frozen identities, topology, explicit topics/transforms, strict JSON, wrong pins and unsupported time/platform/composition, path/symlink/FIFO rejection, atomic generations, concurrent writers and lifecycle locks. A tiny compiled C ELF probe verifies actual exec/PID/ROS environment and lock lifetime; it is deliberately not a ROS/native control acceptance claim. The separate native module gates and Agent/robot deployment acceptance must run against the exact final built plugins.
