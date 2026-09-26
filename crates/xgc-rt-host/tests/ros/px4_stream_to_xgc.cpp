// Test tool: convert the PX4 controller's replay stream (ROS-serialized
// messages, from xgc2-multirotor-controller test/replay/bag_to_stream.py)
// into ctl-px4 port payloads, the way ros_io does for live topics.
//
// Usage: px4_stream_to_xgc IN.stream OUT.xgcstream
//
// OUT: magic "XGCPX4S1", then records u64 receive-time ns, u32 ctl-px4 port
// index, u32 length, payload (an xgc schema struct). The receive time
// becomes the sample's envelope t_produce.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/serialization.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include "xgc_schemas_v1.h"

namespace {

template <typename M>
M decode(std::vector<uint8_t>& d) {
  M m;
  ros::serialization::IStream s(d.data(), static_cast<uint32_t>(d.size()));
  ros::serialization::deserialize(s, m);
  return m;
}

void vec3(double out[3], double x, double y, double z) { out[0] = x; out[1] = y; out[2] = z; }

xgc_pose_v1 pose(const geometry_msgs::PoseStamped& m) {
  xgc_pose_v1 p{};
  p.stamp = m.header.stamp.toSec();
  vec3(p.position, m.pose.position.x, m.pose.position.y, m.pose.position.z);
  p.q_wxyz[0] = m.pose.orientation.w; p.q_wxyz[1] = m.pose.orientation.x;
  p.q_wxyz[2] = m.pose.orientation.y; p.q_wxyz[3] = m.pose.orientation.z;
  return p;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  std::ifstream in(argv[1], std::ios::binary);
  char magic[8];
  if (!in.read(magic, 8) || std::memcmp(magic, "PMCRPLY1", 8) != 0) throw std::runtime_error("not a replay stream");
  std::ofstream out(argv[2], std::ios::binary);
  out.write("XGCPX4S1", 8);
  auto emit = [&](uint64_t t, uint32_t port, const void* p, uint32_t n) {
    out.write(reinterpret_cast<const char*>(&t), 8);
    out.write(reinterpret_cast<const char*>(&port), 4);
    out.write(reinterpret_cast<const char*>(&n), 4);
    out.write(static_cast<const char*>(p), n);
  };
  long count = 0;
  for (;;) {
    uint64_t t = 0; uint8_t kind = 0; uint32_t len = 0;
    if (!in.read(reinterpret_cast<char*>(&t), 8)) break;
    in.read(reinterpret_cast<char*>(&kind), 1);
    in.read(reinterpret_cast<char*>(&len), 4);
    std::vector<uint8_t> d(len);
    in.read(reinterpret_cast<char*>(d.data()), len);
    switch (kind) {
      case 1: {  // -> estimate (port 0)
        const auto m = decode<rigid_state_estimator_msgs::RigidStateEstimate>(d);
        xgc_rigid_state_estimate_v1 e{};
        e.stamp = m.header.stamp.toSec();
        vec3(e.position, m.position.x, m.position.y, m.position.z);
        vec3(e.velocity, m.velocity.x, m.velocity.y, m.velocity.z);
        e.q_wxyz[0] = m.orientation.w; e.q_wxyz[1] = m.orientation.x; e.q_wxyz[2] = m.orientation.y; e.q_wxyz[3] = m.orientation.z;
        vec3(e.angular_velocity, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z);
        vec3(e.linear_acceleration, m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z);
        vec3(e.gravity, m.gravity.x, m.gravity.y, m.gravity.z);
        vec3(e.accel_bias, m.accel_bias.x, m.accel_bias.y, m.accel_bias.z);
        e.last_fused_pose_stamp_sec = m.last_fused_pose_stamp_sec;
        e.vrpn_innovation_window_chi_square = m.vrpn_innovation_window_chi_square;
        e.last_pose_position_innovation_norm_m = m.last_pose_position_innovation_norm_m;
        e.last_pose_orientation_innovation_norm_rad = m.last_pose_orientation_innovation_norm_rad;
        e.last_pose_mahalanobis_distance = m.last_pose_mahalanobis_distance;
        e.innovation_position_gate_m = m.innovation_position_gate_m;
        e.innovation_orientation_gate_rad = m.innovation_orientation_gate_rad;
        e.pose_nis_gate = m.pose_nis_gate;
        e.last_imu_sample_stamp_sec = m.last_imu_sample_stamp_sec;
        e.last_vrpn_pose_stamp_sec = m.last_vrpn_pose_stamp_sec;
        e.filter_inertial_stamp_sec = m.filter_inertial_stamp_sec;
        e.filter_pose_stamp_sec = m.filter_pose_stamp_sec;
        e.flags = m.flags;
        e.vrpn_consecutive_rejects = m.vrpn_consecutive_rejects;
        e.vrpn_consecutive_accepts = m.vrpn_consecutive_accepts;
        e.estimator_state = m.estimator_state;
        e.vrpn_observation_state = m.vrpn_observation_state;
        e.filter_health = m.filter_health;
        e.last_pose_reject_reason = m.last_pose_reject_reason;
        e.last_pose_accepted = m.last_pose_accepted ? 1 : 0;
        emit(t, 0, &e, sizeof e);
        break;
      }
      case 2: { const auto p = pose(decode<geometry_msgs::PoseStamped>(d)); emit(t, 1, &p, sizeof p); break; }
      case 3: {
        const auto m = decode<geometry_msgs::TwistStamped>(d);
        xgc_twist_v1 v{};
        v.stamp = m.header.stamp.toSec();
        vec3(v.linear, m.twist.linear.x, m.twist.linear.y, m.twist.linear.z);
        vec3(v.angular, m.twist.angular.x, m.twist.angular.y, m.twist.angular.z);
        emit(t, 2, &v, sizeof v);
        break;
      }
      case 4: {
        const auto m = decode<sensor_msgs::Imu>(d);
        xgc_imu_v1 i{};
        i.stamp = m.header.stamp.toSec();
        vec3(i.accel, m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z);
        vec3(i.gyro, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z);
        emit(t, 3, &i, sizeof i);
        break;
      }
      case 5: {
        const auto m = decode<mavros_msgs::State>(d);
        xgc_fcu_state_v1 s{};
        s.stamp = m.header.stamp.toSec();
        s.connected = m.connected; s.armed = m.armed; s.guided = m.guided; s.manual_input = m.manual_input;
        s.system_status = m.system_status;
        std::strncpy(s.mode, m.mode.c_str(), sizeof s.mode - 1);
        emit(t, 4, &s, sizeof s);
        break;
      }
      case 6: {
        const auto m = decode<sensor_msgs::BatteryState>(d);
        const xgc_battery_v1 b{m.header.stamp.toSec(), m.voltage, m.percentage};
        emit(t, 5, &b, sizeof b);
        break;
      }
      case 7: { const auto p = pose(decode<geometry_msgs::PoseStamped>(d)); emit(t, 6, &p, sizeof p); break; }
      case 8: {
        const auto m = decode<std_msgs::String>(d);
        xgc_command_v1 c{};
        std::strncpy(c.text, m.data.c_str(), sizeof c.text - 1);
        emit(t, 7, &c, sizeof c);
        break;
      }
      default:
        throw std::runtime_error("unknown record kind");
    }
    ++count;
  }
  std::printf("%ld records\n", count);
  return 0;
}
