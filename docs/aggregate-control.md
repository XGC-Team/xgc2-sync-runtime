# Aggregate module administration

The aggregate process owns module loading, configuration, module health and
shutdown. Modules retain the native `xgc_rt_plugin_v1` C ABI and same-process
sample handoff. Real-time steps never dispatch RPC, parse management JSON or
wait for a controller's network response.

The process owner passes `--control-socket`, `--module-root`, `--document-root`
and `--audit-root`. All grants are absolute existing directories. An initial
`--manifest` is a trusted bootstrap document: its parent supplies the default
document/module grant and its audit allocation supplies the default audit
grant. An installed bundle outside that parent requires its separate module
grant. Administrative requests cannot grant more paths. The endpoint parent
is owned by the effective user, mode 0700; the SDK owns the 0600 socket and
exclusive endpoint lease. The host never creates a provider process.

`GET /v1/describe` returns the fresh `sync-runtime/v1/http.v1` ServiceRef.
Every internal query and mutation consumes that instance; discovery is the
only unbound route. Queries are GET, mutations are POST.

| Route | Body and result |
| --- | --- |
| `/v1/load` | Either `{manifest_path}` or `{manifest_toml,base_dir}`; prepares one loaded graph. An existing graph must be unloaded first. |
| `/v1/configure` | `{expected_revision,modules:{name:config},persist:false}` atomically replaces desired configuration for named modules while stopped. |
| `/v1/start` | `{}`; accepted starts native creation/configuration/activation on existing module executor threads. |
| `/v1/stop` | `{}`; accepted requests native deactivation/destruction. Completion is observed separately. |
| `/v1/unload` | `{expected_revision}`; requires a stopped graph and releases its module libraries and definition. |
| `/v1/health`, `/v1/modules` | Actual phase, module names, native liveness, errors, configuration and audit capacity/loss. |
| `/v1/configuration` | Desired/applied revisions and native configuration evidence. |
| `/v1/policy` | Effective shared startup policy, field source and product ceilings. |
| `/v1/observe/{after_event_revision}` | Held observation returns after a native/manager event. Caller deadline bounds the wait; at most two are admitted, with one connection and call reserved for management. A one-call/connection policy admits no held observer. |

Load validates manifest structure, path grants and actual library descriptors;
it does not execute a module's configure callback. A desired configuration can
therefore remain pending. Module-owned unknown fields and native constraints
are checked by its configure callback on start. `configuration_applied` lists
actual module states and failures; `applied_revision` advances only when every
configure succeeds. Native configuration has partial-effect semantics across
modules and does not promise rollback. `persist:true` is rejected. Desired
configuration and administrative revisions are ephemeral and disappear on
process exit; the process owner supplies the frozen manifest at the next boot.
Each new native start attempt clears its previous applied evidence, even when
the desired revision is unchanged. Unload/new load clear old graph evidence.

`source_manifest_sha256` is the digest of the original manifest bytes, not a
re-serialization. Configuring/unloading clears that frozen identity. Native
deployment readiness requires generation 1, matching digest, desired/applied
revision 1 and fresh actual active module liveness. A running PID, loaded
library, accepted start or old audit file does not establish readiness.

The manager admits four fixed jobs. Shared runtime policy ceilings are eight
connections, four in-flight calls and 1 MiB request/response documents. Module
configuration must fit 64 KiB of JSON before load/configure commits, including
file-loaded TOML whose JSON escaping expands its size. The response allowance
must be 1 MiB; a smaller startup policy is rejected. Native string details have
a 512-byte JSON preview and report `native_detail_truncated`; the native audit
retains full evidence. Session identities are at most 256 bytes and module
instance names 128 bytes. The
composition root snapshots `XGC2_XRPC_` once and delegates parsing to the SDK;
unsupported or unenforced settings fail startup. Module graphs have at most
128 module declarations. Manifests must be regular files, bounded to 1 MiB;
FIFOs, symlinks at the file entry and oversized documents are rejected.

The SDK-owned blocking closure retains the endpoint lease until each admitted
manager job actually completes, including after caller cancellation. Shutdown
joins manager/native supervisor work before endpoint close. No arbitrary native
code can be safely killed in-process. An abandoned module marks
`restart_required`; unloading/replacing/restarting that graph is refused and
the process owner must end the process. No mutation is automatically replayed.
Native destruction and audit writer joins run outside the query state lock;
health remains readable during a pending unload. Manager unwinding retains
and joins its supervisor; a supervisor panic latches a restart-required error.
An abandoned executor can continue native work after its supervisor reports
failure. That report does not release process ownership. On terminal abandon
or an unknown manager-join failure, the control listener, SDK Runtime and
endpoint lease remain held until the process actually exits, including while
final stdout is blocked. Health retains the failed graph; mutations remain
fenced. A replacement process cannot acquire the same endpoint while the old
lease is held. After actual process exit, the SDK can reclaim an unreachable
stale socket only after acquiring its exclusive lease. SDK callback drain is
not evidence that every native module stopped.

Audit writer ownership is the host's existing experiment allocation. Each load
or restart uses `host-<boot>/revision-<revision>/attempt-<attempt>` below that
allocation and reports the actual path in the run summary. Existing run
evidence is never truncated or automatically deleted. Each node/attempt has
one shared `audit.max_bytes` quota (default/host ceiling 256 MiB), including
128 KiB reserved metadata. Records, health, steps and clock evidence share it.
Queues, record size and loss/write-error counters are published in health and
the summary; any lost evidence marks `meta.complete=false` and fails clean
completion. There are at most 32 attempts per host boot. Retention across boots
belongs to the existing experiment/archive owner; restarting does not erase
older evidence or prove a global deployment disk quota.

Station IO has its own command/mission service. Its fixed 16-record handoff
reports `queued` only after local publication returns; this says nothing about
device action completion. Command records are 64 bytes and mission records
240 bytes. Expired unpublished handoffs are discarded; caller abort alone
does not prove the module observed cancellation before publication. Its
activation generates a fresh service incarnation without mutable process
environment. Deactivation closes admission, rejects unpublished handoffs and
waits for its SDK endpoint and handler release to finish; it does not close
the shared process Runtime.

Its seven STATE inputs have fixed native payloads (24–96 bytes). Draining each
port conflates into one fixed owned buffer; the final sample determines validity,
and no borrowed Host payload survives the next ABI read. Uplink owns parsed
values and bounded strings, waits for the earliest actual send deadline, and
copies only due channels. The six output ceilings total 45 JSON publications
per second per robot; these telemetry rates do not measure algorithm or bulk
data throughput. Fresh values retain their existing periodic publication rules.

Heartbeat `source_samples` counts updates accepted for each channel; paired
input updates both pose and velocity. `throttled` counts a channel generation
overwritten before selection for encoding (including controller selection by
heartbeat). It does not count worker iterations or transport drops.
`publish_success` and `publish_failure` count local Zenoh `put().wait()` Ok/Err
results. BestEffort/Drop can return Ok without transport admission; remote
delivery, receiver consumption and transport loss remain unknown to this
plugin and require evidence from their actual owners.

The eight-second startup receive timeout bounds only the readiness receipt.
The subsequent native join retains its real worker ownership and has no proven
eight-second total completion bound; a local blackhole handshake measurement
took about ten seconds to return. A failed or abandoned native operation is
not resource recovery. Resource and latency measurements must use the actual
Host/module/transport artifacts and separate startup loading from steady state.

ABI minor 3 appends an optional scoped getter for the official XRPC C runtime
table. Station activation requires that injected owner and binds through
`ForeignRuntime`; it creates no module-specific RPC Runtime. The binary resolves
one startup policy for its root Runtime, control endpoint and module factories.
Module caps can only tighten that baseline. Each exported factory pins the
actual module library until its native callback and handler release complete.
SDK callback release and native module destruction remain separate gates:
failed native deactivation retains the Slot, instance and DLL until process
exit. A foreign callback carries its original absolute deadline; the current
C request ABI supplies no immediate caller-abort token. An unknown receipt
cannot establish that publication was skipped or justify replay.

SDK ABI/package publication, Focal toolchain delivery,
physical device, sustained resource measurements and live station acceptance
remain separately verifiable release requirements.
