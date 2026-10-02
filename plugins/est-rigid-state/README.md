# Rigid-state adapter ownership

The native adapter and replay tests are owned and built by
[`estimator_vrpn_px4_rotor_state`](../../../../ros1/perception/estimator/rigid-state/estimator_vrpn_px4_rotor_state/docs/native_adapter.md).

Consume its installed `libest_rigid_state.so` and the single
`libestimator_vrpn_px4_rotor_state_core.so` (plus the shared state-machine
dependency). The adapter exports the unchanged `xgc_rt_plugin_v1` entry point.
There is no estimator implementation or duplicate core compilation in this
runtime directory. Owner CMake exports `RigidStateEstimator::Core`; the native
adapter itself consumes the installed `XgcRuntime::SDK` target.

The owner test target `rigid_state_reference_replay` provides the direct-core
reference for runtime Host integration tests. Configure with
`RIGID_STATE_BUILD_ROS=OFF`, `RIGID_STATE_BUILD_NATIVE=ON`, and
`RIGID_STATE_BUILD_TESTING=ON` to build both adapter and replay tests.
