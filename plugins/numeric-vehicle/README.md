# Numeric vehicle

`libnumeric_vehicle.so` is an actuator-free world-ENU vehicle model with ideal PVA tracking for onboard planning and wireless HIL. It does not open ROS, MAVLink, FCU or hardware interfaces, synthesize IMU, or depend on an estimator/controller. It is a vehicle model, not a physical flight controller or a safe fallback.

Configuration comes from the selected scene slot:

```toml
initial_position = [0.0, 0.0, 1.2]
initial_velocity = [0.0, 0.0, 0.0]
```

The plugin has four required ports:

| Port | Direction / QoS | Schema |
|---|---|---|
| command | in / event | xgc.command/1, 64 bytes |
| position_target | in / control | xgc.position_target/1, 104 bytes |
| paired_state | out / state | xgc.dmpc.paired_state/1, 96 bytes |
| controller_state | out / state | xgc.controller_status/1, 56 bytes |

`controller_state` reports this numerical vehicle's command/reference state for the HIL planner. It is not physical FCU telemetry. Bind command and planning input origins explicitly in the fixed host graph. There are no actuator/service output ports.

The initial state is `Configured`. `prepare` changes it to `Ready`; `custom1` or `start` from Ready/Hold permits tracking. `hold` or `hover` revokes tracking, clears queued segments and sets acceleration to zero while retaining position and velocity. `stop` terminates model advancement and retains the final p/v; a stopped model requires a new Session. These commands are real inputs, not configuration booleans. No automatic Ready/Custom1 is emitted at activation.

Each planner PositionTarget supplies a world-ENU PVA segment valid on `[stamp, stamp+0.1s)`. The legacy DMPC producer's stamp is its stage-1 activation, `trigger+0.1s`. All position, velocity and acceleration axes must be enabled, FORCE must be absent and frame must be 1 under the existing ROS/MAVROS ENU wire convention. Nonfinite values, wrong frames and invalid payloads are rejected. Reference p/v and yaw never reset the vehicle's independent p/v.

The reference is lifted from its effective timestamp, with `tau = time - stamp`:

```
p_ref = p0 + tau*v0 + 0.5*tau*tau*a0
v_ref = v0 + tau*a0
p_dot = v
v_dot = a0 - 2*w*(v-v_ref) - w*w*(p-p_ref), w = 4 rad/s
```

This fixed, critically damped inner-loop approximation is integrated exactly over each constant-acceleration segment. Position and velocity remain independent model states; receipt never resets them to the reference. It models ideal tracking for planning/network HIL, not PX4, the deployed SMC, actuator limits or aerodynamic dynamics. Their validation uses the full controller/simulator deployment.

A future segment is queued and cannot affect motion before its stamp. A late segment which has not expired becomes the tracking reference only at the current processing time and runs to its original end; it never changes past motion. A fully expired segment is rejected. Equal effective times with identical bytes are idempotent; conflicting content enters Fault. The bounded pending queue holds at most 64 future segments.

When a segment expires without a successor, acceleration becomes zero and the model reports Hold/reference unavailable. Velocity is retained and the model coasts; this is neither point hover nor a safety guarantee. If tracking is still commanded, a later valid segment can resume numerical application. An explicit Hold/Stop cannot be overridden by an arriving plan. Mission admission and recovery remain the planner/rounds owners' responsibility.

Paired pose/twist carry the same current Session stamp. Orientation is identity, because this model has no attitude dynamics. Clock pauses cause no integration; backward time is rejected. Segment receipt logs include effective time, host arrival time, actual processing time/age and rejection reason. There is no separate event ledger.

The numerical HIL composition uses the host's existing `on_dirty` trigger and a 10 ms wake timer. Commands and PVA wake the model immediately; otherwise it advances and publishes at 100 Hz. Exact segment integration preserves the reference timing without 1 kHz state traffic. The host period and step budget are unchanged.

Build and test with `plugins/numeric-vehicle/test.sh` in the runtime build environment. The suite builds the actual cdylib, then loads it through the actual host alongside a controlled C producer/observer using the production C headers. The C compilation checks wire sizes and offsets; the observer checks numerical outputs and command transitions. Passing is not evidence of full DMPC, physical FS150 deployment or an eight-robot experiment.
