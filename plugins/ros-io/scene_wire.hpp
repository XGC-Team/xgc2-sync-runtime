// xgc2_geometry_msgs/SceneSnapshot and SceneState as the payloads
// "xgc.scene.snapshot/1" and "xgc.scene.state/1" (abi/include/xgc_schemas_v1.h):
// every field except ScenePart.color, the stamp as header.stamp.toSec().
// Templates over the message types, so this header needs only the generated
// message headers of its user.
#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "xgc_schemas_v1.h"

namespace xgc_scene_wire {

class Writer {
 public:
  template <class T>
  void pod(const T& v) {
    const auto* p = reinterpret_cast<const uint8_t*>(&v);
    out.insert(out.end(), p, p + sizeof(T));
  }
  void u32(size_t v) { pod(static_cast<uint32_t>(v)); }
  void str(const std::string& s) {
    u32(s.size());
    out.insert(out.end(), s.begin(), s.end());
  }
  template <class Vec>
  void vec3(const Vec& v) {
    pod(static_cast<double>(v.x));
    pod(static_cast<double>(v.y));
    pod(static_cast<double>(v.z));
  }
  template <class Pose>
  void pose(const Pose& p) {
    vec3(p.position);
    pod(static_cast<double>(p.orientation.x));
    pod(static_cast<double>(p.orientation.y));
    pod(static_cast<double>(p.orientation.z));
    pod(static_cast<double>(p.orientation.w));
  }
  std::vector<uint8_t> out;
};

template <class Snapshot>
std::vector<uint8_t> encode_snapshot(const Snapshot& m) {
  Writer w;
  xgc_scene_snapshot_v1 head{};
  head.stamp = m.header.stamp.toSec();
  head.revision = m.revision;
  head.obstacle_count = static_cast<uint32_t>(m.obstacles.size());
  w.pod(head);
  w.str(m.header.frame_id);
  w.str(m.scene_id);
  w.str(m.epoch);
  for (const auto& o : m.obstacles) {
    w.str(o.id);
    w.str(o.name);
    w.pose(o.pose);
    w.u32(o.dynamic ? 1 : 0);
    w.str(o.motion_type);
    w.u32(o.parts.size());
    for (const auto& p : o.parts) {
      w.str(p.id);
      w.pose(p.pose);
      w.str(p.geometry.type);
      w.vec3(p.geometry.size);
      w.pod(static_cast<double>(p.geometry.radius));
      w.pod(static_cast<double>(p.geometry.height));
      w.u32(p.geometry.vertices.size());
      for (const auto& v : p.geometry.vertices) w.vec3(v);
      w.u32(p.geometry.triangles.size());
      for (uint32_t t : p.geometry.triangles) w.pod(t);
    }
  }
  return std::move(w.out);
}

template <class State>
std::vector<uint8_t> encode_state(const State& m) {
  Writer w;
  xgc_scene_state_v1 head{};
  head.stamp = m.header.stamp.toSec();
  head.scene_time = m.scene_time;
  head.revision = m.revision;
  head.playing = m.playing ? 1u : 0u;
  head.obstacle_count = static_cast<uint32_t>(m.obstacles.size());
  w.pod(head);
  w.str(m.header.frame_id);
  w.str(m.epoch);
  for (const auto& o : m.obstacles) {
    w.str(o.id);
    w.pose(o.pose);
    w.vec3(o.twist.linear);
    w.vec3(o.twist.angular);
  }
  return std::move(w.out);
}

}  // namespace xgc_scene_wire
