#pragma once

#include "xgc_schemas_v1.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>

// A simulated mocap measurement, sampled at the ROS publication boundary.
// The input remains plant truth. Each source sample gets one noise draw;
// repeating that sample repeats the measurement, including its noise and stamp.
// No timer, health-detector feedback, or synthetic motion is involved.
class SimMocapMeasurement {
public:
  SimMocapMeasurement(std::array<double, 3> position_stddev_m, uint32_t seed)
      : position_stddev_m_(position_stddev_m), rng_(seed) {
    for (double sigma : position_stddev_m_)
      if (!std::isfinite(sigma) || sigma < 0.0)
        throw std::invalid_argument("sim mocap position standard deviations must be finite and nonnegative");
  }

  bool sample(const xgc_pose_v1& truth, xgc_pose_v1* measurement) {
    if (!std::isfinite(truth.stamp) || truth.stamp < 0.0)
      throw std::invalid_argument("sim mocap requires a finite nonnegative source stamp");
    // A late sample cannot create a new observation of an already sampled past.
    if (have_sample_ && truth.stamp < last_.stamp) return false;
    if (!have_sample_ || truth.stamp > last_.stamp) {
      last_ = truth;
      for (size_t axis = 0; axis < position_stddev_m_.size(); ++axis)
        if (position_stddev_m_[axis] > 0.0)
          last_.position[axis] += position_stddev_m_[axis] * normal_(rng_);
      have_sample_ = true;
    }
    *measurement = last_;
    return true;
  }

private:
  std::array<double, 3> position_stddev_m_;
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  xgc_pose_v1 last_{};
  bool have_sample_{false};
};
