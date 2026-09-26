// Encode and decode the xgc.ref.* schemas (xgc_schemas_v1.h): a fixed head
// followed by its variable parts.
//
// The functions are templates over the message type, because the
// multirotor_reference_trajectory_msgs messages (in ros_io) and the ROS-free
// runtime's plain types (reference_types.h, in ref-trajectory) have the same
// field names. Times travel as their exact sec/nsec, so a round trip changes
// no bit. frame_id is cut to 31 bytes. A header type without seq or frame_id
// (the PX4 controller's plain reference types carry only the stamp) writes
// them as zero / empty and ignores them on read.

#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "xgc_schemas_v1.h"

namespace xgc_ref_wire {

namespace detail {

struct Out {
  std::vector<uint8_t> bytes;
  void raw(const void* p, size_t n) {
    const auto* b = static_cast<const uint8_t*>(p);
    bytes.insert(bytes.end(), b, b + n);
  }
  void f64(double v) { raw(&v, sizeof v); }
  void f64s(const std::vector<double>& v) {
    if (!v.empty()) raw(v.data(), 8 * v.size());
  }
};

struct In {
  const uint8_t* p;
  size_t left;
  bool raw(void* out, size_t n) {
    if (n > left) return false;
    std::memcpy(out, p, n);
    p += n;
    left -= n;
    return true;
  }
  bool f64(double& v) { return raw(&v, sizeof v); }
  bool f64s(std::vector<double>& v, uint32_t n) {
    v.resize(n);
    return n == 0 || raw(v.data(), 8 * static_cast<size_t>(n));
  }
};

template <class T, class = void>
struct has_seq : std::false_type {};
template <class T>
struct has_seq<T, std::void_t<decltype(std::declval<T&>().seq)>> : std::true_type {};
template <class T, class = void>
struct has_frame_id : std::false_type {};
template <class T>
struct has_frame_id<T, std::void_t<decltype(std::declval<T&>().frame_id)>> : std::true_type {};

template <class H>
void put_header(xgc_ref_header_v1& o, const H& h) {
  if constexpr (has_seq<H>::value) o.seq = h.seq;
  o.stamp_sec = h.stamp.sec;
  o.stamp_nsec = h.stamp.nsec;
  if constexpr (has_frame_id<H>::value) {
    const size_t n = std::min(h.frame_id.size(), sizeof o.frame_id - 1);
    std::memcpy(o.frame_id, h.frame_id.data(), n);
    o.frame_id[n] = '\0';
  }
}

template <class H>
void get_header(const xgc_ref_header_v1& i, H& h) {
  if constexpr (has_seq<H>::value) h.seq = i.seq;
  h.stamp.sec = i.stamp_sec;
  h.stamp.nsec = i.stamp_nsec;
  if constexpr (has_frame_id<H>::value) h.frame_id.assign(i.frame_id, strnlen(i.frame_id, sizeof i.frame_id));
}

template <class V>
void put3(double* o, const V& v) {
  o[0] = v.x;
  o[1] = v.y;
  o[2] = v.z;
}

template <class V>
void get3(const double* i, V& v) {
  v.x = i[0];
  v.y = i[1];
  v.z = i[2];
}

template <class Head>
bool head(In& in, Head& h) {
  return in.raw(&h, sizeof h);
}

}  // namespace detail

template <class M>
std::vector<uint8_t> encode_analytic(const M& m) {
  xgc_ref_analytic_v1 h{};
  detail::put_header(h.header, m.header);
  h.request_id = m.request_id;
  h.trajectory_id = m.trajectory_id;
  h.revision = m.revision;
  h.flags = m.flags;
  h.start_sec = m.start_time.sec;
  h.start_nsec = m.start_time.nsec;
  h.analytic_type = m.analytic_type;
  h.params_len = static_cast<uint32_t>(m.params.size());
  h.duration = m.duration;
  detail::put3(h.origin_position, m.origin.position);
  h.origin_q_xyzw[0] = m.origin.orientation.x;
  h.origin_q_xyzw[1] = m.origin.orientation.y;
  h.origin_q_xyzw[2] = m.origin.orientation.z;
  h.origin_q_xyzw[3] = m.origin.orientation.w;
  detail::Out out;
  out.raw(&h, sizeof h);
  out.f64s(m.params);
  return std::move(out.bytes);
}

template <class M>
bool decode_analytic(const uint8_t* data, size_t len, M& m) {
  detail::In in{data, len};
  xgc_ref_analytic_v1 h;
  if (!detail::head(in, h)) return false;
  detail::get_header(h.header, m.header);
  m.request_id = h.request_id;
  m.trajectory_id = h.trajectory_id;
  m.revision = h.revision;
  m.flags = h.flags;
  m.start_time.sec = h.start_sec;
  m.start_time.nsec = h.start_nsec;
  m.analytic_type = h.analytic_type;
  m.duration = h.duration;
  detail::get3(h.origin_position, m.origin.position);
  m.origin.orientation.x = h.origin_q_xyzw[0];
  m.origin.orientation.y = h.origin_q_xyzw[1];
  m.origin.orientation.z = h.origin_q_xyzw[2];
  m.origin.orientation.w = h.origin_q_xyzw[3];
  return in.f64s(m.params, h.params_len) && in.left == 0;
}

template <class M>
std::vector<uint8_t> encode_sampled(const M& m) {
  xgc_ref_sampled_v1 h{};
  detail::put_header(h.header, m.header);
  h.trajectory_id = m.trajectory_id;
  h.revision = m.revision;
  h.flags = m.flags;
  h.points_len = static_cast<uint32_t>(m.points.size());
  h.start_sec = m.start_time.sec;
  h.start_nsec = m.start_time.nsec;
  h.sample_dt = m.sample_dt;
  detail::Out out;
  out.raw(&h, sizeof h);
  for (const auto& p : m.points) {
    xgc_ref_flat_point_v1 q{};
    q.t_from_start = p.t_from_start;
    detail::put3(q.position, p.position);
    detail::put3(q.velocity, p.velocity);
    detail::put3(q.acceleration, p.acceleration);
    detail::put3(q.jerk, p.jerk);
    detail::put3(q.snap, p.snap);
    q.yaw = p.yaw;
    q.yaw_rate = p.yaw_rate;
    q.yaw_accel = p.yaw_accel;
    out.raw(&q, sizeof q);
  }
  return std::move(out.bytes);
}

template <class M>
bool decode_sampled(const uint8_t* data, size_t len, M& m) {
  detail::In in{data, len};
  xgc_ref_sampled_v1 h;
  if (!detail::head(in, h)) return false;
  detail::get_header(h.header, m.header);
  m.trajectory_id = h.trajectory_id;
  m.revision = h.revision;
  m.flags = h.flags;
  m.start_time.sec = h.start_sec;
  m.start_time.nsec = h.start_nsec;
  m.sample_dt = h.sample_dt;
  if (in.left != static_cast<size_t>(h.points_len) * sizeof(xgc_ref_flat_point_v1)) return false;
  m.points.resize(h.points_len);
  for (auto& p : m.points) {
    xgc_ref_flat_point_v1 q;
    in.raw(&q, sizeof q);
    p.t_from_start = q.t_from_start;
    detail::get3(q.position, p.position);
    detail::get3(q.velocity, p.velocity);
    detail::get3(q.acceleration, p.acceleration);
    detail::get3(q.jerk, p.jerk);
    detail::get3(q.snap, p.snap);
    p.yaw = q.yaw;
    p.yaw_rate = q.yaw_rate;
    p.yaw_accel = q.yaw_accel;
  }
  return true;
}

template <class M>
std::vector<uint8_t> encode_waypoint_request(const M& m) {
  xgc_ref_waypoint_request_v1 h{};
  detail::put_header(h.header, m.header);
  h.request_id = m.request_id;
  h.trajectory_id = m.trajectory_id;
  h.revision = m.revision;
  h.flags = m.flags;
  h.waypoints_len = static_cast<uint32_t>(m.waypoints.size());
  h.constraint_types_len = static_cast<uint32_t>(m.constraint_types.size());
  h.region_size_len = static_cast<uint32_t>(m.region_size.size());
  h.segment_times_len = static_cast<uint32_t>(m.segment_times.size());
  detail::put3(h.start_velocity, m.start_velocity);
  detail::put3(h.start_acceleration, m.start_acceleration);
  detail::put3(h.end_velocity, m.end_velocity);
  detail::put3(h.end_acceleration, m.end_acceleration);
  h.desired_speed = m.desired_speed;
  h.time_weight = m.time_weight;
  h.max_body_rate = m.max_body_rate;
  h.max_tilt = m.max_tilt;
  h.min_thrust = m.min_thrust;
  h.max_thrust = m.max_thrust;
  h.max_iterations = m.max_iterations;
  h.objective = m.objective;
  h.rel_cost_tol = m.rel_cost_tol;
  h.max_velocity = m.max_velocity;
  h.max_acceleration = m.max_acceleration;
  h.max_jerk = m.max_jerk;
  h.max_snap = m.max_snap;
  detail::Out out;
  out.raw(&h, sizeof h);
  for (const auto& w : m.waypoints) {
    out.f64(w.position.x);
    out.f64(w.position.y);
    out.f64(w.position.z);
    out.f64(w.orientation.x);
    out.f64(w.orientation.y);
    out.f64(w.orientation.z);
    out.f64(w.orientation.w);
  }
  std::vector<uint8_t> types(m.constraint_types.begin(), m.constraint_types.end());
  types.resize((types.size() + 7) / 8 * 8, 0);
  out.raw(types.data(), types.size());
  for (const auto& r : m.region_size) {
    out.f64(r.x);
    out.f64(r.y);
    out.f64(r.z);
  }
  out.f64s(m.segment_times);
  return std::move(out.bytes);
}

template <class M>
bool decode_waypoint_request(const uint8_t* data, size_t len, M& m) {
  detail::In in{data, len};
  xgc_ref_waypoint_request_v1 h;
  if (!detail::head(in, h)) return false;
  detail::get_header(h.header, m.header);
  m.request_id = h.request_id;
  m.trajectory_id = h.trajectory_id;
  m.revision = h.revision;
  m.flags = h.flags;
  detail::get3(h.start_velocity, m.start_velocity);
  detail::get3(h.start_acceleration, m.start_acceleration);
  detail::get3(h.end_velocity, m.end_velocity);
  detail::get3(h.end_acceleration, m.end_acceleration);
  m.desired_speed = h.desired_speed;
  m.time_weight = h.time_weight;
  m.max_body_rate = h.max_body_rate;
  m.max_tilt = h.max_tilt;
  m.min_thrust = h.min_thrust;
  m.max_thrust = h.max_thrust;
  m.max_iterations = h.max_iterations;
  m.objective = h.objective;
  m.rel_cost_tol = h.rel_cost_tol;
  m.max_velocity = h.max_velocity;
  m.max_acceleration = h.max_acceleration;
  m.max_jerk = h.max_jerk;
  m.max_snap = h.max_snap;
  const size_t types_padded = (static_cast<size_t>(h.constraint_types_len) + 7) / 8 * 8;
  const size_t expect = 56 * static_cast<size_t>(h.waypoints_len) + types_padded +
                        24 * static_cast<size_t>(h.region_size_len) + 8 * static_cast<size_t>(h.segment_times_len);
  if (in.left != expect) return false;
  m.waypoints.resize(h.waypoints_len);
  for (auto& w : m.waypoints) {
    in.f64(w.position.x);
    in.f64(w.position.y);
    in.f64(w.position.z);
    in.f64(w.orientation.x);
    in.f64(w.orientation.y);
    in.f64(w.orientation.z);
    in.f64(w.orientation.w);
  }
  std::vector<uint8_t> types(types_padded);
  in.raw(types.data(), types.size());
  m.constraint_types.assign(types.begin(), types.begin() + h.constraint_types_len);
  m.region_size.resize(h.region_size_len);
  for (auto& r : m.region_size) {
    in.f64(r.x);
    in.f64(r.y);
    in.f64(r.z);
  }
  return in.f64s(m.segment_times, h.segment_times_len);
}

template <class M>
std::vector<uint8_t> encode_polynomial(const M& m) {
  xgc_ref_polynomial_v1 h{};
  detail::put_header(h.header, m.header);
  h.trajectory_id = m.trajectory_id;
  h.revision = m.revision;
  h.flags = m.flags;
  h.order = m.order;
  h.start_sec = m.start_time.sec;
  h.start_nsec = m.start_time.nsec;
  h.duration = m.duration;
  h.segment_durations_len = static_cast<uint32_t>(m.segment_durations.size());
  h.coeff_x_len = static_cast<uint32_t>(m.coeff_x.size());
  h.coeff_y_len = static_cast<uint32_t>(m.coeff_y.size());
  h.coeff_z_len = static_cast<uint32_t>(m.coeff_z.size());
  h.coeff_yaw_len = static_cast<uint32_t>(m.coeff_yaw.size());
  detail::Out out;
  out.raw(&h, sizeof h);
  out.f64s(m.segment_durations);
  out.f64s(m.coeff_x);
  out.f64s(m.coeff_y);
  out.f64s(m.coeff_z);
  out.f64s(m.coeff_yaw);
  return std::move(out.bytes);
}

template <class M>
bool decode_polynomial(const uint8_t* data, size_t len, M& m) {
  detail::In in{data, len};
  xgc_ref_polynomial_v1 h;
  if (!detail::head(in, h)) return false;
  detail::get_header(h.header, m.header);
  m.trajectory_id = h.trajectory_id;
  m.revision = h.revision;
  m.flags = h.flags;
  m.order = h.order;
  m.start_time.sec = h.start_sec;
  m.start_time.nsec = h.start_nsec;
  m.duration = h.duration;
  return in.f64s(m.segment_durations, h.segment_durations_len) && in.f64s(m.coeff_x, h.coeff_x_len) &&
         in.f64s(m.coeff_y, h.coeff_y_len) && in.f64s(m.coeff_z, h.coeff_z_len) &&
         in.f64s(m.coeff_yaw, h.coeff_yaw_len) && in.left == 0;
}

template <class M>
xgc_ref_status_v1 encode_status(const M& m) {
  xgc_ref_status_v1 h{};
  detail::put_header(h.header, m.header);
  h.state = m.state;
  h.active_type = m.active_type;
  h.flags = m.flags;
  h.active_trajectory_id = m.active_trajectory_id;
  h.active_revision = m.active_revision;
  return h;
}

template <class M>
bool decode_status(const uint8_t* data, size_t len, M& m) {
  xgc_ref_status_v1 h;
  if (len != sizeof h) return false;
  std::memcpy(&h, data, sizeof h);
  detail::get_header(h.header, m.header);
  m.state = h.state;
  m.active_type = h.active_type;
  m.flags = h.flags;
  m.active_trajectory_id = h.active_trajectory_id;
  m.active_revision = h.active_revision;
  return true;
}

}  // namespace xgc_ref_wire
