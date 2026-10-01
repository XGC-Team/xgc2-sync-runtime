// ROS-free checks of ros_io's per-step ROS service arithmetic (ros_slice.hpp).
#include "ros_slice.hpp"

#include <cstdint>
#include <iostream>
#include <string>

namespace {

int g_fails = 0;

void expect(bool ok, const std::string& message) {
  if (ok) return;
  std::cerr << "FAIL " << message << "\n";
  ++g_fails;
}

using xgc_ros_slice::budget_ns;
using xgc_ros_slice::kMaxWaitNs;
using xgc_ros_slice::kMinWaitNs;
using xgc_ros_slice::next_wait_ns;

constexpr int64_t kMs = 1000000;

// Waits the service loop makes for a budget when every wait lasts exactly as
// long as asked (no callback arrives) and the loop itself takes `overhead`.
int waits_for(int64_t budget, int64_t overhead, int64_t* serviced) {
  int64_t elapsed = 0;
  int waits = 0;
  for (int64_t wait; (wait = next_wait_ns(budget, elapsed)) != 0; ++waits) elapsed += wait + overhead;
  *serviced = elapsed;
  return waits;
}

void test_budget() {
  const int64_t now = 5 * 1000 * kMs, deadline = now + 10 * kMs;
  expect(budget_ns(now, deadline, 0.001) == 1000, "slice_ms 0.001 is a 1 us budget");
  expect(budget_ns(now, deadline, 2.0) == 2 * kMs, "slice_ms 2 is a 2 ms budget");
  expect(budget_ns(now, deadline, 0.0) == 9 * kMs, "no slice: until the deadline less 1 ms");
  expect(budget_ns(now, deadline, 50.0) == 9 * kMs, "a slice never passes the deadline less 1 ms");
  expect(budget_ns(deadline, deadline, 0.0) == 0, "past the deadline less 1 ms: no budget");
}

void test_lightweight_plant_slice_is_one_nonblocking_pass() {
  // Core's lightweight plant edges: slice_ms = 0.001 on a 10 ms round.
  const int64_t now = 0, deadline = 10 * kMs;
  const int64_t budget = budget_ns(now, deadline, 0.001);
  int64_t serviced = 0;
  expect(waits_for(budget, 0, &serviced) == 0, "a 1 us slice never waits on the ROS queue");
  expect(next_wait_ns(budget, 0) == 0, "first wait of a 1 us slice is none");
}

void test_long_slices_are_serviced_up_to_the_slack() {
  int64_t serviced = 0;
  // DMPC edges: slice_ms 2 -> two 1 ms waits, ending exactly at the budget.
  expect(waits_for(2 * kMs, 0, &serviced) == 2 && serviced == 2 * kMs, "2 ms slice: two 1 ms waits");
  // With loop overhead the last remainder below the slack is not waited.
  expect(waits_for(2 * kMs, 10000, &serviced) == 2, "2 ms slice with 10 us overhead per wait: two waits");
  expect(serviced <= 2 * kMs + 2 * 10000, "never waits past the budget");
  expect(next_wait_ns(2 * kMs, 2 * kMs - kMinWaitNs) == kMinWaitNs, "a remainder of exactly the slack is waited");
  expect(next_wait_ns(2 * kMs, 2 * kMs - kMinWaitNs + 1) == 0, "a remainder below the slack is not");
  // No slice: until the deadline less 1 ms in 1 ms waits.
  expect(waits_for(budget_ns(0, 10 * kMs, 0.0), 0, &serviced) == 9 && serviced == 9 * kMs, "no slice: nine 1 ms waits");
}

void test_wait_bounds() {
  for (int64_t budget = 0; budget <= 3 * kMs; budget += 7919) {
    for (int64_t elapsed = 0; elapsed <= budget + 100000; elapsed += 6151) {
      const int64_t wait = next_wait_ns(budget, elapsed), left = budget - elapsed;
      if (wait < 0 || wait > kMaxWaitNs || (wait > 0 && wait > left) || ((wait == 0) != (left < kMinWaitNs))) {
        expect(false, "wait bounds at budget " + std::to_string(budget) + " elapsed " + std::to_string(elapsed));
        return;
      }
    }
  }
}

}  // namespace

int main() {
  test_budget();
  test_lightweight_plant_slice_is_one_nonblocking_pass();
  test_long_slices_are_serviced_up_to_the_slack();
  test_wait_bounds();
  if (g_fails) {
    std::cerr << g_fails << " failure(s)\n";
    return 1;
  }
  std::cout << "ros_slice: all checks passed\n";
  return 0;
}
