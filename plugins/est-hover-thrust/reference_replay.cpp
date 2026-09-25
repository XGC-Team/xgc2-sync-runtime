// Reference for the equality test: feeds a recorded input sequence straight
// into HoverThrustEstimatorRuntime, the way the ROS HoverThrustInputProducer
// does per topic, with the runtime clock at each sample stamp, and prints
// every PUBLISH_ESTIMATE output the way HoverThrustOutputConsumer does
// (drive the output model to the event time, then snapshot). No ports, no
// host, no plugin code.
//
// stdin:  one sample per line: "<port> <stamp> <value> <ignore>" (shortest round-trip decimal)
//         port 0 imu accel z, 1 attitude-target thrust, 2 pose z
// stdout: one line per estimate:
//         "<stamp> <hover> <raw> <thr2acc> <last_estimate_stamp> <state> <flags> <sample_used>" (%.17g, exact round trip)

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hte = hover_thrust_estimator;

int main() {
  hte::HoverThrustEstimatorRuntime runtime;
  hte::HoverThrustEstimatorConfig config;  // defaults = the plugin's defaults
  runtime.setConfig(config);
  hte::HoverThrustEstimatorRuntime::Input input;
  unsigned port = 0, ignore = 0;
  double stamp = 0, value = 0;
  while (std::scanf("%u %la %la %u", &port, &stamp, &value, &ignore) == 4) {
    hte::HoverThrustSample* s = port == 0 ? &input.imu_acc_z : port == 1 ? &input.normalized_thrust : &input.altitude;
    const uint32_t event = port == 0 ? hte::event_type::INPUT_IMU_UPDATED
                           : port == 1 ? hte::event_type::INPUT_THRUST_UPDATED
                                       : hte::event_type::INPUT_ALTITUDE_UPDATED;
    if (port == 1) input.thrust_ignored = ignore != 0;
    s->value = value;
    s->period_sec = s->received && std::isfinite(s->stamp_sec) && std::isfinite(stamp) ? stamp - s->stamp_sec : 0.0;
    s->stamp_sec = stamp;
    s->received = true;
    s->finite = std::isfinite(value);
    (void)runtime.postInputEvent(state_machine::Event(event, state_machine::EventTimestamp{stamp}), input);
    runtime.update(stamp);
    for (const auto& ev : runtime.getStateMachine().currentOutputEvents()) {
      if (ev.id != hte::output_event_type::PUBLISH_ESTIMATE) continue;
      const double out_stamp = std::isfinite(ev.timestamp) && ev.timestamp > 0.0 ? ev.timestamp : stamp;
      runtime.outputModel().driveTowardTarget(out_stamp);
      const auto o = runtime.refreshOutputSnapshot();
      std::printf("%.17g %.17g %.17g %.17g %.17g %u %u %u\n", out_stamp, o.hover_thrust, o.raw_hover_thrust,
                  o.thrust_to_acceleration, o.last_estimate_stamp_sec, o.state, o.flags, o.sample_used ? 1u : 0u);
    }
  }
  return 0;
}
