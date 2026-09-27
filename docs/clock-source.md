# Simulator clock source

The host supports one external simulator time authority per experiment. The same
source and executor serve colocated modules and separate robot containers. The
experiment selects its ROS graph, simulator world instance and common epoch;
these are not station-wide singletons.

A host manifest opts in explicitly:

```toml
[clock_source]
kind = "ros1_sim"
plugin = "ros_io"
topic = "/clock"
expected_publisher = "/gazebo"
world_instance_id = "experiment-world-generation"
startup_timeout_wall_ms = 10000
stale_after_wall_ms = 250
max_advance_ns = 100000000
poll_wall_ms = 5
queue_capacity = 256
```

`session.epoch_ns` must be a positive shared simulator timestamp in the future
when host startup finishes. The host does not derive a local simulator epoch.
A host that misses the epoch fails before activation. Its selected plugin must
be SHA256 pinned and configure an explicit `node_name`. Every source field is
required; unknown keys are errors. `[clock]` wall probes cannot be combined with
`[clock_source]`. Existing manifests without a source retain the wall path.
The fixed deployment renderer profiles admit wall input; simulator profile
admission is separate from this host manifest capability.

The source is the `xgc_rt_clock_source_v1` service exported by the pinned ROS edge
library. Its C contract is [xgc_clock_source.h](../abi/include/xgc_clock_source.h).
The host owns its polling thread and library lifetime. Source initialization
requires `/use_sim_time=true` without modifying it and shares ROS initialization
with ordinary `ros_io`; its callback queue is independent of domain activation.
Timestamps remain exact simulator nanoseconds, including valid time zero. Sensor
timestamps are never rewritten as Unix time.

The host accepts one publisher matching the frozen resolved caller ID. It rejects
backward time, multiple or different publishers, malformed observations, queue
loss and excess advancement. Intermediate samples are checked before coalescing.
The advance limit also applies relative to the last host-observed committed time,
so rapidly replaying smaller queued jumps cannot conceal a large catch-up jump.

An unchanged timestamp does not execute another domain step, even with dirty
inputs or `wake_ms`. After `stale_after_wall_ms` without advancement, the output
gate closes and input backlog is discarded. Both a paused simulator and an absent
clock produce this suspended state; it is not proof of a healthy pause. Host and
source heartbeats continue on steady time. A bounded positive advance from the
same authority resumes ordinary processing without fabricating sensor freshness,
mission permission or controller readiness. Pending ROS actuator/service outputs
are discarded; a service call already issued cannot be recalled.

Stop and native-call supervision use steady time. Source faults latch, close the
output gate and terminate the Session with a failure result. They require a new
Session rather than silently clamping time or restarting the source in place.
Native polls have a wall bound plus a 250 ms scheduling allowance; a poll that
fails to return faults the Session. An unresponsive native worker is abandoned
with its library retained until the call returns.

ROS `/clock` carries no world-generation field. `world_instance_id` is frozen
provider/Session provenance, not a generation read from that message. The
experiment workflow must create a new Session after a world/provider restart,
including a restart that preserves the caller ID and monotonically increasing
time. Each node records the source identity and common epoch in its audit.
Merging different clock domains, source identities or epochs is rejected. Unknown
clock bounds remain explicitly unknown; simulator timestamp differences do not
measure wall-time network latency.

Startup timeout accepts 1–300000 wall ms; poll 1–50 wall ms; stale timeout
2×poll–60000 wall ms; maximum advance 1–10000000000 ns; queue capacity 1–65536.
Names are absolute resolved ROS names of at most 255 bytes. World identity is
trimmed, nonempty, at most 256 bytes, and excludes ASCII control characters.

A shared-master functional test may use an unimpaired network. Neighbor-radio
experiments must route clock and plant traffic over the physics network while
applying impairment only to neighbor transport. ROS master selection alone does
not determine TCPROS routing: advertised publisher addresses and actual routes
must also match the experiment's physics path.
