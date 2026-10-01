// ROS-free arithmetic of ros_io's per-step ROS service.
//
// A step services the plugin's ROS callback queue once without blocking, then
// keeps servicing it until the slice budget is spent: until the round's
// publish deadline less 1 ms and, with `slice_ms` > 0, for at most `slice_ms`
// after the step read the Session time. The remaining budget is waited in
// condition-variable waits of at most 1 ms (CallbackQueue::callAvailable).
//
// A wait shorter than the kernel's timer slack (50 us for an ordinary
// thread) cannot end on time: the thread sleeps for about the slack and wakes
// again, which costs a context switch and a few microseconds of CPU and
// overshoots the slice. Such a remainder therefore ends the service. With
// `slice_ms` = 0.001 a step is exactly the one non-blocking pass that the
// lightweight plant deployment asks for.
#pragma once

#include <algorithm>
#include <cstdint>

namespace xgc_ros_slice {

// Linux's default timer slack for a normal (non-realtime) thread.
inline constexpr int64_t kMinWaitNs = 50000;
// Longest single wait, so a stop or shutdown is noticed.
inline constexpr int64_t kMaxWaitNs = 1000000;

// Steady nanoseconds this step may spend servicing ROS after its
// non-blocking pass. `session_now` and `deadline` are Session nanoseconds.
inline int64_t budget_ns(int64_t session_now, int64_t deadline, double slice_ms) {
  int64_t until = deadline - 1000000;
  if (slice_ms > 0.0) until = std::min<int64_t>(until, session_now + static_cast<int64_t>(slice_ms * 1e6));
  return std::max<int64_t>(0, until - session_now);
}

// How long the next callAvailable may wait, `elapsed` steady nanoseconds into
// the budget; 0 ends the service.
inline int64_t next_wait_ns(int64_t budget, int64_t elapsed) {
  const int64_t left = budget - elapsed;
  return left < kMinWaitNs ? 0 : std::min(left, kMaxWaitNs);
}

}  // namespace xgc_ros_slice
