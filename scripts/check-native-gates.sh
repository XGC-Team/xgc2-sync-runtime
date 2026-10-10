#!/usr/bin/env bash
# Replay/equivalence gates with owning C++/ROS dependencies. Missing prerequisites or
# an environment-skipped Rust test are failures, not successful validation.
# Source ROS and the freshly built controller workspace first; see
# docs/validation/native-20260926/README.md. Run tests serially: helpers share
# plugin output paths. Never rebuild those .so files while a host loads them.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
: "${ROS_PREFIX:?set ROS_PREFIX to a sourced ROS Noetic installation}"
: "${XGC2_WS:?set XGC2_WS to the freshly built controller workspace}"
: "${PX4_CONTROLLER_ROOT:?set PX4_CONTROLLER_ROOT to the package source}"
: "${REF_ROOT:?set REF_ROOT to multirotor_reference_trajectory}"
: "${PX4_CORE_LIB_DIR:?set PX4_CORE_LIB_DIR to the freshly built core directory}"
: "${REF_CORE_LIB_DIR:?set REF_CORE_LIB_DIR to the freshly built core directory}"
test -r "$PX4_CORE_LIB_DIR/libpx4_multirotor_controller_core.so"
test -r "$REF_CORE_LIB_DIR/libmultirotor_reference_trajectory_core.so"
px4_replay="$XGC2_WS/devel/lib/px4_multirotor_controller/px4_multirotor_controller_replay_harness"
ref_replay="$XGC2_WS/devel/lib/multirotor_reference_trajectory/multirotor_reference_trajectory_replay_harness"
test -x "$px4_replay"
test -x "$ref_replay"
out="${1:-$root/target/native-gates}"
mkdir -p "$out"
out="$(cd "$out" && pwd)"
mkdir -p "$out/bags" "$out/replays"
# The caller supplies recorded flights here; Runtime no longer builds a ROS bridge
# or launches a live ROS baseline to create these inputs.
for backend in px4_local dfbc nmpc; do
  test -r "$out/bags/px4_flight_$backend.bag"
done

gate() {
  local label="$1" count="$2"
  shift 2
  "$@" 2>&1 | tee "$out/$label.log"
  python3 - "$out/$label.log" "$count" <<'PY'
import pathlib, re, sys
text = pathlib.Path(sys.argv[1]).read_text()
if re.search(r"\bskipped(?::| )", text):
    raise SystemExit("environment-skipped test: this gate did not execute fully")
counts = re.findall(r"test result: ok\. (\d+) passed; 0 failed", text)
if sum(map(int, counts)) != int(sys.argv[2]):
    raise SystemExit("unexpected executed-test count: " + repr(counts))
PY
}

gate native-equivalence 2 cargo test -p xgc-rt-host \
  --test est_rigid_state --test ctl_dfbc -- --nocapture --test-threads=1

python3 "$REF_ROOT/test/replay/make_reference_stream.py" "$out/replays/reference.stream"
"$ref_replay" "$out/replays/reference.stream" "$out/replays/reference.expected.txt"
gate reference-replay 1 env REF_REPLAY_STREAM="$out/replays/reference.stream" \
  REF_REPLAY_REF="$out/replays/reference.expected.txt" \
  cargo test -p xgc-rt-host --test ref_trajectory_replay -- --nocapture --test-threads=1
for backend in px4_local dfbc nmpc; do
  python3 "$PX4_CONTROLLER_ROOT/test/replay/bag_to_stream.py" \
    "$out/bags/px4_flight_$backend.bag" "$out/replays/$backend.stream"
  "$px4_replay" "$out/replays/$backend.stream" "$out/replays/$backend.expected.txt" \
    "$backend" reference_analytic_type=3
  gate "$backend-replay" 1 env PX4_REPLAY_BACKEND="$backend" PX4_REPLAY_REFERENCE_TYPE=3 \
    PX4_REPLAY_STREAM="$out/replays/$backend.stream" PX4_REPLAY_REF="$out/replays/$backend.expected.txt" \
    cargo test -p xgc-rt-host --test ctl_px4_replay -- --nocapture --test-threads=1
done
sha256sum "$out"/*.log "$out"/replays/* > "$out/SHA256SUMS"
echo "6 native equivalence/replay tests executed; no environment skips. Recorded software inputs only."
