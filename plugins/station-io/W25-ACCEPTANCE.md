# Station command tests

`station-io-cmd` sends one request and waits up to two seconds across connection,
write and reply. `queued` means local publication, not vehicle motion. Explicit
rejection is distinct from a missing or invalid reply: the latter reports an
unknown outcome and does not retry. Vehicle state comes from telemetry.

The listener expires incomplete requests and replies without blocking the host
step. Binding preserves a live endpoint and reclaims a stale socket only after
connection refusal and an unchanged socket inode. Deactivation removes its own
socket, preserving any replacement at the pathname.

Run the socket tests:

```sh
cargo test --offline -j 2 -p station-io --lib socket:: -- --nocapture --test-threads=1
```

Build the CLI and plugins, then test the real host, Zenoh fields, command/mission
consumers, disabled ports and host stepping:

```sh
bash plugins/station-io/test.sh
```

These tests use private local endpoints and do not command physical actuators.
