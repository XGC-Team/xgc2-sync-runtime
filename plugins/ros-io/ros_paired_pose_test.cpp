#include "ros_paired_pose.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

namespace {

int g_fails = 0;

void expect(bool ok, const std::string& message) {
  if (ok) return;
  std::cerr << "FAIL " << message << "\n";
  ++g_fails;
}

bool same(double actual, double wanted) { return std::memcmp(&actual, &wanted, sizeof(double)) == 0; }

void test_same_local_header_ignores_vrpn() {
  constexpr double kStamp = 12.25;
  xgc_pose_v1 cache{};
  bool have_pose = note_pair_pose(true, true, &cache, kStamp, 1.5, -2.5, 0.25, 0.9, 0.1, 0.2, 0.3);
  expect(have_pose, "local pose accepted");
  bool vrpn = note_pair_pose(true, false, &cache, 99.0, 8, 8, 8, 0, 0, 0, 1);
  expect(!vrpn, "VRPN pose did not replace local pose");
  expect(same(cache.stamp, kStamp) && same(cache.position[0], 1.5) && same(cache.position[1], -2.5),
         "cache stayed on the local sample");
  expect(!same_raw_header(have_pose, true, cache.stamp, 99.0), "different VRPN stamp is not a pair");
  expect(!same_raw_header(have_pose, false, cache.stamp, kStamp), "missing velocity is not a pair");
  expect(same_raw_header(have_pose, true, cache.stamp, kStamp), "same local header is a pair");
  xgc_dmpc_paired_state_v1 paired{};
  fill_paired(cache, kStamp, 0.4, -0.2, 0.0, &paired);
  expect(same(paired.pose_stamp_sec, kStamp) && same(paired.twist_stamp_sec, kStamp), "pair stamps are the raw header");
  expect(same(paired.position[0], 1.5) && same(paired.position[1], -2.5) && same(paired.position[2], 0.25),
         "pair position is local");
  expect(same(paired.orientation_xyzw[0], 0.1) && same(paired.orientation_xyzw[3], 0.9), "pair orientation is xyzw");
  expect(same(paired.linear_velocity[0], 0.4) && same(paired.linear_velocity[1], -0.2), "pair velocity is local");
  std::cout << "local stamp " << paired.pose_stamp_sec << " p " << paired.position[0] << " " << paired.position[1]
            << " " << paired.position[2] << " v " << paired.linear_velocity[0] << " " << paired.linear_velocity[1]
            << "\n";
}

void test_vrpn_compat_when_local_pose_disabled() {
  xgc_pose_v1 cache{};
  expect(!note_pair_pose(false, true, &cache, 3.0, 1, 1, 1, 1, 0, 0, 0), "local sample is idle without the port");
  expect(note_pair_pose(false, false, &cache, 3.0, 4, 5, 6, 1, 0, 0, 0), "VRPN pose remains the compat source");
  expect(same_raw_header(true, true, cache.stamp, 3.0) && same(cache.position[0], 4), "compat pair uses VRPN");
  expect(!same_raw_header(true, true, cache.stamp, 3.5), "compat path still rejects a different stamp");
  xgc_pose_v1 zero{};
  expect(note_pair_pose(false, false, &zero, 0.0, 1, 0, 0, 1, 0, 0, 0), "zero header is kept");
  expect(same(zero.stamp, 0.0), "zero header was not rewritten");
}

}  // namespace

int main() {
  test_same_local_header_ignores_vrpn();
  test_vrpn_compat_when_local_pose_disabled();
  if (g_fails != 0) {
    std::cerr << g_fails << " assertion(s) failed\n";
    return 1;
  }
  std::cout << "paired-pose assertions passed\n";
  return 0;
}
