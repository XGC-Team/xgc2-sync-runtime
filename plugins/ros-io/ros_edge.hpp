#pragma once
// Shared by the clock-source service and ordinary ros_io. One ROS client per
// process; the clock service and the domain edge keep separate callback queues.
#include <atomic>
#include <mutex>
#include <string>

#include <ros/ros.h>

#include "xgc_clock_source.h"

namespace xgc_ros_edge {

inline constexpr int kGateUnclaimed = -1;

inline std::atomic<int>& output_gate() {
  static std::atomic<int> gate{kGateUnclaimed};
  return gate;
}

inline std::atomic<int>& suppress_backlog() {
  static std::atomic<int> flag{0};
  return flag;
}

inline bool output_allowed() {
  const int gate = output_gate().load(std::memory_order_acquire);
  return gate == kGateUnclaimed || gate == static_cast<int>(XGC_CLOCK_GATE_OPEN);
}

inline bool take_suppress_backlog() { return suppress_backlog().exchange(0, std::memory_order_acq_rel) == 1; }

inline void set_output_gate(uint32_t gate) {
  if (gate != XGC_CLOCK_GATE_OPEN) suppress_backlog().store(1, std::memory_order_release);
  output_gate().store(static_cast<int>(gate), std::memory_order_release);
}

struct RosInit {
  bool ok{false};
  const char* error{""};
};

inline std::mutex& ros_init_mu() {
  static std::mutex mu;
  return mu;
}

inline std::string& frozen_node_name() {
  static std::string name;
  return name;
}

inline std::string bare_node_name(std::string name) {
  if (!name.empty() && name.front() == '/') name.erase(name.begin());
  return name;
}

// Initialize ROS at most once. A later caller must repeat the same bare node
// name; the master and ROS_IP stay those of the process environment.
inline RosInit ensure_ros(const std::string& requested_bare) {
  if (requested_bare.empty() || requested_bare.front() == '/' || requested_bare.find('/') != std::string::npos) {
    return {false, "node_name must be one bare ROS name"};
  }
  std::lock_guard<std::mutex> lock(ros_init_mu());
  try {
    if (!ros::isInitialized()) {
      ros::M_string remappings;
      ros::init(remappings, requested_bare, ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
      frozen_node_name() = requested_bare;
      return {true, ""};
    }
  } catch (...) {
    return {false, "ros::init failed"};
  }
  std::string current = frozen_node_name();
  if (current.empty()) {
    current = bare_node_name(ros::this_node::getName());
    frozen_node_name() = current;
  }
  if (current != requested_bare) return {false, "ROS node name does not match the frozen node"};
  return {true, ""};
}

}  // namespace xgc_ros_edge
