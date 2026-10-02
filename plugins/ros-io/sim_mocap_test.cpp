#include "sim_mocap.hpp"

#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>

namespace {

xgc_pose_v1 truth_at(double stamp) {
  xgc_pose_v1 truth{};
  truth.stamp = stamp;
  truth.position[0] = 1.25;
  truth.position[1] = -2.0;
  truth.position[2] = 0.5;
  truth.q_wxyz[0] = 0.5;
  truth.q_wxyz[1] = -0.5;
  truth.q_wxyz[2] = 0.5;
  truth.q_wxyz[3] = 0.5;
  return truth;
}

bool same_bytes(const xgc_pose_v1& a, const xgc_pose_v1& b) {
  // The wire schema contains exactly eight doubles and no padding.
  static_assert(sizeof(xgc_pose_v1) == 8 * sizeof(double));
  return std::memcmp(&a, &b, sizeof a) == 0;
}

void check_unchanged_metadata(const xgc_pose_v1& truth,
                              const xgc_pose_v1& measurement) {
  assert(std::memcmp(&truth.stamp, &measurement.stamp, sizeof truth.stamp) == 0);
  assert(std::memcmp(truth.q_wxyz, measurement.q_wxyz, sizeof truth.q_wxyz) == 0);
}

template <typename F>
void expect_invalid_argument(F operation) {
  bool rejected = false;
  try {
    operation();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  assert(rejected);
}

void check_sampling_identity() {
  SimMocapMeasurement noisy({0.001, 0.002, 0.003}, 12345);
  SimMocapMeasurement control({0.001, 0.002, 0.003}, 12345);
  xgc_pose_v1 truth = truth_at(0.0);
  const xgc_pose_v1 original = truth;
  xgc_pose_v1 first{}, expected{};
  assert(noisy.sample(truth, &first));
  assert(control.sample(truth, &expected));
  assert(same_bytes(truth, original));
  assert(same_bytes(first, expected));
  check_unchanged_metadata(truth, first);
  assert(!same_bytes(first, truth));

  // A source stamp identifies one observation, even if its payload is resent
  // with different values. No extra noise or a refreshed timestamp is allowed.
  truth.position[0] = 99.0;
  truth.q_wxyz[1] = 0.0;
  const xgc_pose_v1 repeated_input = truth;
  for (int repeat = 0; repeat < 10; ++repeat) {
    xgc_pose_v1 repeated{};
    assert(noisy.sample(truth, &repeated));
    assert(same_bytes(repeated, first));
    assert(same_bytes(truth, repeated_input));
  }

  truth = truth_at(1.0);
  assert(noisy.sample(truth, &first));
  assert(control.sample(truth, &expected));
  assert(same_bytes(first, expected));  // repeats did not consume RNG state

  const xgc_pose_v1 late = truth_at(0.5);
  xgc_pose_v1 untouched = truth_at(900.0);
  const xgc_pose_v1 sentinel = untouched;
  assert(!noisy.sample(late, &untouched));
  assert(same_bytes(untouched, sentinel));
  xgc_pose_v1 repeated{};
  assert(noisy.sample(truth, &repeated));
  assert(same_bytes(repeated, first));  // late input did not replace the cache

  truth = truth_at(2.0);
  assert(noisy.sample(truth, &first));
  assert(control.sample(truth, &expected));
  assert(same_bytes(first, expected));  // late input did not consume RNG state

  // ros_io uses an in-place *local copy* of the input wire payload.
  SimMocapMeasurement in_place({0.001, 0.002, 0.003}, 42);
  SimMocapMeasurement separate({0.001, 0.002, 0.003}, 42);
  const xgc_pose_v1 plant_truth = truth_at(5.0);
  xgc_pose_v1 local = plant_truth;
  assert(in_place.sample(local, &local));
  assert(separate.sample(plant_truth, &expected));
  assert(same_bytes(local, expected));
  assert(same_bytes(plant_truth, truth_at(5.0)));
  assert(in_place.sample(local, &local));
  assert(same_bytes(local, expected));
}

void check_zero_noise() {
  SimMocapMeasurement disabled({0.0, 0.0, 0.0}, 17);
  SimMocapMeasurement one_axis({0.0, 0.002, 0.0}, 17);
  bool noisy_axis_changed = false;
  for (int sample = 0; sample < 100; ++sample) {
    xgc_pose_v1 truth = truth_at(sample * 0.01);
    truth.position[0] += sample * 0.1;
    const xgc_pose_v1 original = truth;
    xgc_pose_v1 measurement{};
    assert(disabled.sample(truth, &measurement));
    assert(same_bytes(measurement, truth));
    assert(one_axis.sample(truth, &measurement));
    assert(measurement.position[0] == truth.position[0]);
    assert(measurement.position[2] == truth.position[2]);
    noisy_axis_changed |= measurement.position[1] != truth.position[1];
    check_unchanged_metadata(truth, measurement);
    assert(same_bytes(truth, original));
  }
  assert(noisy_axis_changed);
}

void check_invalid_parameters_and_stamps() {
  const std::array<double, 4> invalid{
      -0.001, std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()};
  for (size_t axis = 0; axis < 3; ++axis) {
    for (double value : invalid) {
      std::array<double, 3> sigma{0.001, 0.002, 0.003};
      sigma[axis] = value;
      expect_invalid_argument([&] { SimMocapMeasurement rejected(sigma, 0); });
    }
  }

  SimMocapMeasurement noisy({0.001, 0.002, 0.003}, 7654);
  SimMocapMeasurement control({0.001, 0.002, 0.003}, 7654);
  for (int phase = 0; phase < 2; ++phase) {
    // Rejection must also work before the first valid source sample.
    for (double stamp : invalid) {
      xgc_pose_v1 truth = truth_at(stamp);
      const xgc_pose_v1 original = truth;
      xgc_pose_v1 output = truth_at(800.0);
      const xgc_pose_v1 sentinel = output;
      expect_invalid_argument([&] { noisy.sample(truth, &output); });
      assert(same_bytes(truth, original));
      assert(same_bytes(output, sentinel));
    }
    const xgc_pose_v1 truth = truth_at(phase);
    xgc_pose_v1 output{}, expected{};
    assert(noisy.sample(truth, &output));
    assert(control.sample(truth, &expected));
    assert(same_bytes(output, expected));  // rejection did not consume RNG state
  }
}

void check_stationary_gaussian_statistics() {
  constexpr int count = 60000;
  const std::array<double, 3> sigma{0.001, 0.002, 0.005};
  SimMocapMeasurement noisy(sigma, 20261002);
  SimMocapMeasurement replay(sigma, 20261002);
  SimMocapMeasurement other_robot(sigma, 314159);
  std::array<double, 3> sum{}, squares{}, fourth{}, other_sum{}, other_squares{};
  std::array<double, 3> cross_axis{}, cross_seed{}, lag_product{}, previous{};
  std::array<int, 3> outside_two_sigma{};
  int distinct_samples = 0;
  xgc_pose_v1 previous_measurement{};

  for (int sample = 0; sample < count; ++sample) {
    // The pose is stationary. Only the source sample time advances.
    const xgc_pose_v1 truth = truth_at(sample * 0.01);
    const xgc_pose_v1 original = truth;
    xgc_pose_v1 measurement{}, repeated{}, independent{};
    assert(noisy.sample(truth, &measurement));
    assert(replay.sample(truth, &repeated));
    assert(other_robot.sample(truth, &independent));
    assert(same_bytes(truth, original));
    assert(same_bytes(measurement, repeated));
    check_unchanged_metadata(truth, measurement);
    check_unchanged_metadata(truth, independent);
    if (sample > 0 && std::memcmp(measurement.position, previous_measurement.position,
                                  sizeof measurement.position) != 0)
      ++distinct_samples;

    std::array<double, 3> z{};
    for (size_t axis = 0; axis < 3; ++axis) {
      z[axis] = (measurement.position[axis] - truth.position[axis]) / sigma[axis];
      const double other = (independent.position[axis] - truth.position[axis]) / sigma[axis];
      sum[axis] += z[axis];
      squares[axis] += z[axis] * z[axis];
      fourth[axis] += z[axis] * z[axis] * z[axis] * z[axis];
      outside_two_sigma[axis] += std::abs(z[axis]) > 2.0;
      other_sum[axis] += other;
      other_squares[axis] += other * other;
      cross_seed[axis] += z[axis] * other;
      if (sample > 0) lag_product[axis] += z[axis] * previous[axis];
    }
    for (size_t axis = 0; axis < 3; ++axis)
      cross_axis[axis] += z[axis] * z[(axis + 1) % 3];
    previous = z;
    previous_measurement = measurement;
  }
  assert(distinct_samples == count - 1);

  // Fixed seeds and wide sampling-error margins avoid expecting one library's
  // exact normal_distribution sequence. Moments and tails reject, for example,
  // a uniform dither with the same variance or a constant per-robot offset.
  for (size_t axis = 0; axis < 3; ++axis) {
    const double mean = sum[axis] / count;
    const double variance = squares[axis] / count - mean * mean;
    const double other_mean = other_sum[axis] / count;
    const double other_variance = other_squares[axis] / count - other_mean * other_mean;
    const double seed_correlation = (cross_seed[axis] / count - mean * other_mean) /
                                    std::sqrt(variance * other_variance);
    const double tail_fraction = static_cast<double>(outside_two_sigma[axis]) / count;
    assert(std::abs(mean) < 0.03);
    assert(std::abs(variance - 1.0) < 0.05);
    assert(std::abs(fourth[axis] / count - 3.0) < 0.3);
    assert(tail_fraction > 0.039 && tail_fraction < 0.052);
    assert(std::abs(cross_axis[axis] / count - mean * sum[(axis + 1) % 3] / count) < 0.03);
    assert(std::abs(lag_product[axis] / (count - 1) - mean * mean) < 0.03);
    assert(std::abs(seed_correlation) < 0.03);
  }
}

}  // namespace

int main() {
  check_sampling_identity();
  check_zero_noise();
  check_invalid_parameters_and_stamps();
  check_stationary_gaussian_statistics();
  std::cout << "sim mocap sampling, reproducibility, and Gaussian statistics passed\n";
}
