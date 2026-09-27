#include "ros_dmpc_edge.hpp"

#include "xgc_schemas_v1.h"

#include <cmath>
#include <cstddef>
#include <cstring>

namespace {

static_assert(sizeof(xgc_dmpc_paired_state_v1) == 96, "paired state stays 96");
static_assert(sizeof(xgc_dmpc_controller_status_v1) == 56, "controller status stays 56");
static_assert(sizeof(xgc_dmpc_timeline_ack_v1) == 56, "timeline ack stays 56");
static_assert(sizeof(xgc_position_target_v1) == 104, "position target stays 104");
static_assert(sizeof(xgc_dmpc_scene_vertex_v1) == kSceneWireVertexBytes, "scene vertex stays 24");
static_assert(sizeof(xgc_dmpc_scene_header_v1) == kSceneWireHeaderBytes, "scene header is 144");
static_assert(offsetof(xgc_dmpc_scene_header_v1, revision) == 16, "scene header revision");
static_assert(offsetof(xgc_dmpc_scene_header_v1, scene_id) == 24, "scene header scene_id");
static_assert(offsetof(xgc_dmpc_scene_header_v1, frame) == 56, "scene header frame");
static_assert(offsetof(xgc_dmpc_scene_header_v1, epoch) == kSceneWireEpochOffset, "scene header epoch");
static_assert(sizeof(xgc_dmpc_scene_obstacle_v1) == kSceneWireObstacleBytes, "scene obstacle is 240");
static_assert(offsetof(xgc_dmpc_scene_obstacle_v1, angular) == 200, "scene obstacle angular");
static_assert(offsetof(xgc_dmpc_scene_obstacle_v1, motion_type) == kSceneWireMotionOffset, "scene obstacle motion");
static_assert(sizeof(xgc_dmpc_scene_part_v1) == kSceneWirePartBytes, "scene part is 120");
static_assert(offsetof(xgc_dmpc_scene_part_v1, vertex_begin) == 56, "scene part vertex_begin");
static_assert(offsetof(xgc_dmpc_scene_part_v1, vertex_count) == 60, "scene part vertex_count");
static_assert(offsetof(xgc_dmpc_scene_part_v1, position) == kSceneWirePartPoseOffset, "scene part position");
static_assert(offsetof(xgc_dmpc_scene_part_v1, orientation_xyzw) == 88, "scene part orientation");

bool put_text(char* dest, size_t cap, const std::string& src) {
  if (src.empty() || src.size() >= cap) return false;
  std::memcpy(dest, src.data(), src.size());
  dest[src.size()] = '\0';
  return true;
}

bool finite3(double x, double y, double z) { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }

bool orientation_usable(double x, double y, double z, double w) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(w)) return false;
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  return std::isfinite(norm) && norm >= 1e-9;
}

int geometry_type(const std::string& type) {
  if (type == "cylinder") return 0;
  if (type == "box") return 1;
  if (type == "capsule") return 2;
  if (type == "sphere") return 3;
  if (type == "convex") return 4;
  return -1;
}

void append(std::vector<uint8_t>* blob, const void* data, size_t size) {
  const auto* bytes = static_cast<const uint8_t*>(data);
  blob->insert(blob->end(), bytes, bytes + size);
}

}  // namespace

bool xgc_dmpc_pack_scene_blob(const xgc2_geometry_msgs::SceneSnapshot& snapshot,
                              const xgc2_geometry_msgs::SceneState& state, std::vector<uint8_t>* blob,
                              std::string* error) {
  if (snapshot.header.frame_id != "world" || snapshot.epoch.empty() || state.epoch != snapshot.epoch ||
      state.revision != snapshot.revision || state.header.frame_id != snapshot.header.frame_id) {
    *error = "scene snapshot/state frame, epoch, or revision does not match";
    return false;
  }
  if (snapshot.epoch.size() >= kSceneWireEpochBytes) {
    *error = "scene epoch does not fit";
    return false;
  }
  xgc_dmpc_scene_header_v1 header{};
  header.schema = 1;
  header.stamp_sec = state.header.stamp.toSec();
  header.revision = snapshot.revision;
  if (!put_text(header.scene_id, sizeof header.scene_id, snapshot.scene_id)) {
    *error = "scene_id does not fit";
    return false;
  }
  if (!put_text(header.frame, sizeof header.frame, snapshot.header.frame_id)) {
    *error = "scene frame does not fit";
    return false;
  }
  std::memcpy(header.epoch, snapshot.epoch.data(), snapshot.epoch.size());
  header.epoch[snapshot.epoch.size()] = '\0';

  std::vector<xgc_dmpc_scene_obstacle_v1> obstacles;
  std::vector<xgc_dmpc_scene_part_v1> parts;
  std::vector<xgc_dmpc_scene_vertex_v1> vertices;
  for (const auto& obstacle : snapshot.obstacles) {
    const xgc2_geometry_msgs::SceneObstacleState* motion = nullptr;
    for (const auto& item : state.obstacles) {
      if (item.id != obstacle.id) continue;
      if (motion != nullptr) {
        *error = "duplicate scene state id";
        return false;
      }
      motion = &item;
    }
    if (motion == nullptr || obstacle.parts.empty()) {
      *error = "scene obstacle has no matching state or no parts";
      return false;
    }
    if (obstacle.motion_type.size() >= kSceneWireMotionBytes) {
      *error = "scene obstacle motion_type does not fit";
      return false;
    }
    xgc_dmpc_scene_obstacle_v1 body{};
    if (!put_text(body.id, sizeof body.id, obstacle.id) || !put_text(body.name, sizeof body.name, obstacle.name)) {
      *error = "scene obstacle id or name does not fit";
      return false;
    }
    body.dynamic = obstacle.dynamic ? 1u : 0u;
    body.position[0] = motion->pose.position.x;
    body.position[1] = motion->pose.position.y;
    body.position[2] = motion->pose.position.z;
    body.orientation_xyzw[0] = motion->pose.orientation.x;
    body.orientation_xyzw[1] = motion->pose.orientation.y;
    body.orientation_xyzw[2] = motion->pose.orientation.z;
    body.orientation_xyzw[3] = motion->pose.orientation.w;
    body.linear[0] = motion->twist.linear.x;
    body.linear[1] = motion->twist.linear.y;
    body.linear[2] = motion->twist.linear.z;
    body.angular[0] = motion->twist.angular.x;
    body.angular[1] = motion->twist.angular.y;
    body.angular[2] = motion->twist.angular.z;
    if (!finite3(body.position[0], body.position[1], body.position[2]) ||
        !finite3(body.linear[0], body.linear[1], body.linear[2]) ||
        !finite3(body.angular[0], body.angular[1], body.angular[2]) ||
        !orientation_usable(body.orientation_xyzw[0], body.orientation_xyzw[1], body.orientation_xyzw[2],
                            body.orientation_xyzw[3])) {
      *error = !orientation_usable(body.orientation_xyzw[0], body.orientation_xyzw[1], body.orientation_xyzw[2],
                                   body.orientation_xyzw[3])
                   ? "invalid scene obstacle orientation"
                   : "nonfinite scene pose or twist";
      return false;
    }
    if (!obstacle.motion_type.empty()) {
      std::memcpy(body.motion_type, obstacle.motion_type.data(), obstacle.motion_type.size());
    }
    body.motion_type[obstacle.motion_type.size()] = '\0';
    const uint32_t obstacle_index = static_cast<uint32_t>(obstacles.size());
    obstacles.push_back(body);
    for (const auto& part : obstacle.parts) {
      const int type = geometry_type(part.geometry.type);
      xgc_dmpc_scene_part_v1 packed{};
      packed.obstacle_index = obstacle_index;
      if (type < 0) {
        *error = "scene part type is not usable";
        return false;
      }
      if (!put_text(packed.part_id, sizeof packed.part_id, part.id)) {
        *error = "scene part id does not fit";
        return false;
      }
      packed.geometry_type = static_cast<uint32_t>(type);
      if (type == 4) {
        if (part.geometry.vertices.size() < 4) {
          *error = "convex part has fewer than four vertices";
          return false;
        }
        packed.vertex_begin = static_cast<uint32_t>(vertices.size());
        packed.vertex_count = static_cast<uint32_t>(part.geometry.vertices.size());
        for (const auto& vertex : part.geometry.vertices) {
          if (!finite3(vertex.x, vertex.y, vertex.z)) {
            *error = "nonfinite convex vertex";
            return false;
          }
          vertices.push_back(xgc_dmpc_scene_vertex_v1{{vertex.x, vertex.y, vertex.z}});
        }
      } else if (type == 1) {
        packed.param[0] = part.geometry.size.x;
        packed.param[1] = part.geometry.size.y;
        packed.param[2] = part.geometry.size.z;
      } else if (type == 3) {
        packed.param[0] = part.geometry.radius;
      } else {
        packed.param[0] = part.geometry.radius;
        packed.param[1] = part.geometry.height;
      }
      const double px = part.pose.position.x;
      const double py = part.pose.position.y;
      const double pz = part.pose.position.z;
      const double qx = part.pose.orientation.x;
      const double qy = part.pose.orientation.y;
      const double qz = part.pose.orientation.z;
      const double qw = part.pose.orientation.w;
      if (!finite3(px, py, pz)) {
        *error = "nonfinite scene part position";
        return false;
      }
      if (!orientation_usable(qx, qy, qz, qw)) {
        *error = "invalid scene part orientation";
        return false;
      }
      packed.position[0] = px;
      packed.position[1] = py;
      packed.position[2] = pz;
      packed.orientation_xyzw[0] = qx;
      packed.orientation_xyzw[1] = qy;
      packed.orientation_xyzw[2] = qz;
      packed.orientation_xyzw[3] = qw;
      parts.push_back(packed);
    }
  }
  header.obstacle_count = static_cast<uint32_t>(obstacles.size());
  header.part_count = static_cast<uint32_t>(parts.size());
  header.vertex_count = static_cast<uint32_t>(vertices.size());
  std::vector<uint8_t> packed;
  append(&packed, &header, sizeof header);
  if (!obstacles.empty()) append(&packed, obstacles.data(), obstacles.size() * sizeof(xgc_dmpc_scene_obstacle_v1));
  if (!parts.empty()) append(&packed, parts.data(), parts.size() * sizeof(xgc_dmpc_scene_part_v1));
  if (!vertices.empty()) append(&packed, vertices.data(), vertices.size() * sizeof(xgc_dmpc_scene_vertex_v1));
  const size_t need = kSceneWireHeaderBytes + obstacles.size() * kSceneWireObstacleBytes +
                      parts.size() * kSceneWirePartBytes + vertices.size() * sizeof(xgc_dmpc_scene_vertex_v1);
  if (packed.size() != need) {
    *error = "scene snapshot length does not match its header counts";
    return false;
  }
  *blob = std::move(packed);
  return true;
}
