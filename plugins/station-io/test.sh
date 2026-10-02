#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$root"
: "${LIGHTWEIGHT_VEHICLE_ELF:?set LIGHTWEIGHT_VEHICLE_ELF to the real lightweight-vehicle 0.3.0 ELF}"
: "${CTL_PX4_ELF:?set CTL_PX4_ELF to the real ctl-px4 0.2.0 ELF and provide its library closure}"
# Validate the actual loaded descriptors before building or invoking the chain.
# No replacement plant/controller and no legacy FCU schema are admitted.
python3 - "$LIGHTWEIGHT_VEHICLE_ELF" "$CTL_PX4_ELF" <<'PY'
import ctypes as c
import os
import resource
import signal
import sys
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
signal.alarm(10)
class Port(c.Structure):
    _fields_ = [('name', c.c_char_p), ('direction', c.c_int), ('schema', c.c_char_p), ('qos', c.c_int)]
class Descriptor(c.Structure):
    _fields_ = [('abi', c.c_uint32), ('count', c.c_uint32), ('name', c.c_char_p), ('version', c.c_char_p),
                ('ports', c.POINTER(Port)), ('vtbl', c.c_void_p)]
for path, name, version, count, required in [
    (sys.argv[1], 'lightweight-vehicle', '0.3.0', 64,
     {'setpoint': (2, 'xgc.position_target/1', 0), 'attitude_command': (2, 'xgc.attitude_target/2', 0),
      'fcu_request': (2, 'xgc.fcu_request/2', 2),
      'fcu_result': (3, 'xgc.fcu_result/1', 2), 'fcu_extended_state': (3, 'xgc.fcu_extended_state/1', 1),
      'provider_request': (2, 'xgc.sim_provider_request/1', 2), 'provider_result': (3, 'xgc.sim_provider_result/1', 2)}),
    (sys.argv[2], 'ctl-px4', '0.2.0', None,
     {'setpoint': (1, 'xgc.position_target/1', 0), 'attitude_command': (3, 'xgc.attitude_target/2', 0),
      'fcu_request_full': (3, 'xgc.fcu_request/2', 2)}),
]:
    if not os.path.isabs(path) or not os.path.isfile(path):
        sys.exit('station-io test requires an absolute installed ELF path: ' + path)
    library = c.CDLL(path)
    getter = library.xgc_rt_plugin_v1
    getter.argtypes, getter.restype = [], c.POINTER(Descriptor)
    pointer = getter()
    if not pointer:
        sys.exit('null plugin descriptor: ' + path)
    d = pointer.contents
    if d.abi != 1 or d.name != name.encode() or d.version != version.encode() or not 1 <= d.count <= 64 or (count is not None and d.count != count):
        sys.exit('station-io test requires ' + name + ' ' + version + ' with the current ABI/port layout: ' + path)
    ports = {d.ports[i].name.decode(): (d.ports[i].direction, d.ports[i].schema.decode(), d.ports[i].qos) for i in range(d.count)}
    for port, contract in required.items():
        if ports.get(port) != contract:
            sys.exit(name + ': required port contract differs: ' + port)
PY
export CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-2}"
profile="${STATION_TEST_PROFILE:-dev}"
case "$profile" in dev) directory=debug;; release) directory=release;; *) echo 'STATION_TEST_PROFILE must be dev or release' >&2; exit 2;; esac
cargo build --offline --locked --profile "$profile" -p station-io
export STATION_IO_CMD="${CARGO_TARGET_DIR:-$root/target}/$directory/station-io-cmd"
export STATION_IO_ELF="${CARGO_TARGET_DIR:-$root/target}/$directory/libstation_io.so"
cargo test --offline --locked --profile "$profile" -p station-io "$@" -- --nocapture --test-threads=1
