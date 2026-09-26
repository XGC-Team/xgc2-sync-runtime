// ros_io's shared-scene payloads (plugins/ros-io/scene_wire.hpp) decode with
// the planner's codec (formation_generator core/scene_wire.h, which plan-dmpc
// uses) to exactly the message's fields: every string, count and double, bit
// for bit. Exit status 0 when they all match.

#include <cstdio>
#include <cstring>
#include <string>

#include <xgc2_geometry_msgs/SceneSnapshot.h>
#include <xgc2_geometry_msgs/SceneState.h>

#include "formation_generator/core/scene_wire.h"
#include "scene_wire.hpp"

namespace {

int failures = 0;
void check(bool ok, const std::string& what) {
  if (!ok && ++failures <= 20) std::printf("FAIL: %s\n", what.c_str());
}
bool same(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }
template <class A, class B>
bool same3(const A& a, const B& b) {
  return same(a.x, b.x) && same(a.y, b.y) && same(a.z, b.z);
}
template <class A, class B>
bool samePose(const A& a, const B& b) {
  return same3(a.position, b.position) && same(a.orientation.x, b.orientation.x) &&
         same(a.orientation.y, b.orientation.y) && same(a.orientation.z, b.orientation.z) &&
         same(a.orientation.w, b.orientation.w);
}
geometry_msgs::Pose pose(double x, double y, double z, double qz, double qw) {
  geometry_msgs::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.z = qz;
  p.orientation.w = qw;
  return p;
}

}  // namespace

int main() {
  xgc2_geometry_msgs::SceneSnapshot snap;
  snap.header.stamp.sec = 1790000000;
  snap.header.stamp.nsec = 123456789;
  snap.header.frame_id = "world";
  snap.scene_id = "mixed_circle";
  snap.epoch = "5f0e8c2a-epoch";
  snap.revision = 42;
  const char* types[] = {"box", "sphere", "cylinder", "capsule", "convex"};
  for (int i = 0; i < 5; ++i) {
    xgc2_geometry_msgs::SceneObstacle o;
    o.id = "obstacle_" + std::to_string(i);
    o.name = "Obstacle " + std::to_string(i);
    o.pose = pose(0.1 * i - 3.3, 1.0 / 3.0, 1.1, 0.3826834323650898, 0.9238795325112867);
    o.dynamic = i == 1;
    o.motion_type = o.dynamic ? "constant_twist" : "hold";
    for (int k = 0; k <= i % 2; ++k) {
      xgc2_geometry_msgs::ScenePart part;
      part.id = "part" + std::to_string(k);
      part.pose = pose(0.2 * k, -0.1, 0.05, 0.0, 1.0);
      part.geometry.type = types[i];
      part.geometry.size.x = 0.7;
      part.geometry.size.y = 0.3 + k;
      part.geometry.size.z = 2.2;
      part.geometry.radius = 0.35;
      part.geometry.height = 1.9;
      if (i == 4) {
        for (int v = 0; v < 4; ++v) {
          geometry_msgs::Point p;
          p.x = v == 0 ? 1.0 : -0.5;
          p.y = v == 1 ? 2.0 : -0.25;
          p.z = v == 2 ? 3.0 : -1.0 / 7.0;
          part.geometry.vertices.push_back(p);
        }
        part.geometry.triangles = {0, 1, 2, 0, 2, 3, 0, 3, 1, 1, 3, 2};
      }
      part.color.r = 1.0f;
      o.parts.push_back(part);
    }
    snap.obstacles.push_back(o);
  }
  xgc2_geometry_msgs::SceneState state;
  state.header = snap.header;
  state.header.stamp.nsec = 987654321;
  state.epoch = snap.epoch;
  state.revision = snap.revision;
  state.playing = true;
  state.scene_time = 12.345;
  for (const auto& o : snap.obstacles) {
    xgc2_geometry_msgs::SceneObstacleState s;
    s.id = o.id;
    s.pose = o.pose;
    s.twist.linear.x = o.dynamic ? 0.4 : 0.0;
    s.twist.linear.y = o.dynamic ? -0.1 : 0.0;
    state.obstacles.push_back(s);
  }

  namespace fg = formation_generator_dmpc;
  const auto snap_bytes = xgc_scene_wire::encode_snapshot(snap);
  const auto state_bytes = xgc_scene_wire::encode_state(state);
  fg::SceneSnapshotData d;
  fg::SceneStateData ds;
  check(fg::decodeSceneSnapshot(snap_bytes.data(), snap_bytes.size(), d), "snapshot decodes");
  check(fg::decodeSceneState(state_bytes.data(), state_bytes.size(), ds), "state decodes");
  check(fg::encodeSceneSnapshot(d) == snap_bytes, "snapshot: same bytes from both encoders");
  check(fg::encodeSceneState(ds) == state_bytes, "state: same bytes from both encoders");

  check(same(d.stamp, snap.header.stamp.toSec()) && d.frame_id == snap.header.frame_id &&
            d.scene_id == snap.scene_id && d.epoch == snap.epoch && d.revision == snap.revision &&
            d.obstacles.size() == snap.obstacles.size(),
        "snapshot head");
  for (size_t i = 0; i < d.obstacles.size() && i < snap.obstacles.size(); ++i) {
    const auto& a = d.obstacles[i];
    const auto& b = snap.obstacles[i];
    check(a.id == b.id && a.name == b.name && samePose(a.pose, b.pose) && a.dynamic == b.dynamic &&
              a.motion_type == b.motion_type && a.parts.size() == b.parts.size(),
          "obstacle " + b.id);
    for (size_t k = 0; k < a.parts.size() && k < b.parts.size(); ++k) {
      const auto& p = a.parts[k];
      const auto& q = b.parts[k];
      bool vertices = p.geometry.vertices.size() == q.geometry.vertices.size();
      for (size_t v = 0; vertices && v < p.geometry.vertices.size(); ++v)
        vertices = same3(p.geometry.vertices[v], q.geometry.vertices[v]);
      check(p.id == q.id && samePose(p.pose, q.pose) && p.geometry.type == q.geometry.type &&
                same3(p.geometry.size, q.geometry.size) && same(p.geometry.radius, q.geometry.radius) &&
                same(p.geometry.height, q.geometry.height) && vertices &&
                p.geometry.triangles == q.geometry.triangles,
            "part " + b.id + "/" + q.id);
    }
  }
  check(same(ds.stamp, state.header.stamp.toSec()) && ds.frame_id == state.header.frame_id &&
            ds.epoch == state.epoch && ds.revision == state.revision && ds.playing == state.playing &&
            same(ds.scene_time, state.scene_time) && ds.obstacles.size() == state.obstacles.size(),
        "state head");
  for (size_t i = 0; i < ds.obstacles.size() && i < state.obstacles.size(); ++i) {
    const auto& a = ds.obstacles[i];
    const auto& b = state.obstacles[i];
    check(a.id == b.id && samePose(a.pose, b.pose) && same3(a.twist.linear, b.twist.linear) &&
              same3(a.twist.angular, b.twist.angular),
          "state " + b.id);
  }
  std::printf("scene_wire_check: %zu + %zu bytes, %d failures\n", snap_bytes.size(), state_bytes.size(), failures);
  return failures == 0 ? 0 : 1;
}
