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
