# XGC2 Sync Runtime: design for review

**Status: design freeze.** No code changes since `e14de41`. The full design is `/tmp/claude-zenoh-sync/plan.md`, sections A–E.

## Base parts
- **Aggregator:** one process that loads the `.so` modules one manifest lists; usually one per robot.
- **Modules:** one thread each. They hand data to each other in memory: latest-value outputs as double buffers with a version, every-sample inputs as a queue + notify, and dirty flags. No pub/sub or sockets inside the process.
- **Clock:** one Session clock for every module; rounds are E0 + k·P, computed locally.
- **FSMs:** the aggregator's own FSM, a lifecycle FSM per module, and each module's domain FSMs, all running at once.
- **Watchdog:** a slow step marks the module Degraded; a hung module is abandoned and restarted.
- **ROS:** only the `ros_io` module talks ROS (ordinary subscribe/publish), copying topics into and out of module inputs and outputs. Modules never touch ROS.
- **Zenoh:** only between processes (robot↔robot, robot↔station), on the radio network, stamped and audited.

The built code still runs all modules on one thread and passes same-process data through a loopback transport. Fixing that is the first slice after GO.

## Migration (the main work)
- **Pattern:** keep each module's ROS-free core, replace its ROS input and output code with module I/O, and let `ros_io` carry the topics. Each wrap must match the original on recorded inputs before it is used anywhere else.
- **Done:** the hover-thrust estimator and the DFBC controller, both bit-identical to the originals.
- **Next, the TRO flight chain:** rigid-state ESKF, PX4 controller, reference trajectory, and the DMPC planner. The planner first stays a ROS node behind `ros_io`, with its plans exchanged over Zenoh and local rounds replacing the central sync trigger; later it is wrapped once `IDmpcOptimizer` is ROS-free.
- **Then:** UGV controllers, reset safety and measured state.
- **Stays ROS:** third-party code (SLAM, detection, TARE/CMU, drivers, simulators). The full register is in plan section D.

## Deployment
The same modules run at every step; only the manifest and Session change, one thing per step: replay → sim loop → containers over Zenoh → real companion computer → partial onboard → full onboard. Each step leaves behind logs, module step records, the link audit and `ros_io` recordings.

## Decisions (defaults proposed)
- **D2:** create `XGC-Team/xgc2-sync-runtime`.
- **D7:** control links are latest-wins best-effort.
- **D8:** watchdog as above.
- **Others:** D1, D3, D5 and D6 as in the plan. D9 (a thread per module) is locked.
