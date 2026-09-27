#include "ros_dmpc_edge.hpp"

#include "xgc_schemas_v1.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr char kEpoch[] = "11111111-2222-4333-8444-555555555555";
constexpr char kArchUuid[] = "fdf4eaa7-325f-4248-9fb3-bc5f565a0af5";
constexpr double kArchX = -1.091443422204702;
constexpr double kArchY = -2.7660944910242002;
constexpr double kArchQz = -0.35144306293389693;
constexpr double kArchQw = -0.9362092573328044;

int g_fails = 0;

void expect(bool ok, const std::string& message) {
  if (ok) return;
  std::cerr << "FAIL " << message << "\n";
  ++g_fails;
}

bool same_double(double actual, double wanted) {
  return std::memcmp(&actual, &wanted, sizeof(double)) == 0;
}

void read_bytes(const std::vector<uint8_t>& blob, size_t offset, void* dest, size_t size) {
  std::memcpy(dest, blob.data() + offset, size);
}

std::string read_c_string(const std::vector<uint8_t>& blob, size_t offset, size_t cap) {
  std::string raw(cap, '\0');
  read_bytes(blob, offset, raw.data(), cap);
  const size_t end = raw.find('\0');
  expect(end != std::string::npos, "missing NUL at offset " + std::to_string(offset));
  if (end == std::string::npos) return raw;
  for (size_t index = end; index < cap; ++index) {
    expect(raw[index] == '\0', "tail byte is not zero at offset " + std::to_string(offset + index));
  }
  return raw.substr(0, end);
}

xgc2_geometry_msgs::ScenePart make_part(const std::string& id, const std::string& type, double x, double y, double z,
                                        double qx, double qy, double qz, double qw) {
  xgc2_geometry_msgs::ScenePart part;
  part.id = id;
  part.pose.position.x = x;
  part.pose.position.y = y;
  part.pose.position.z = z;
  part.pose.orientation.x = qx;
  part.pose.orientation.y = qy;
  part.pose.orientation.z = qz;
  part.pose.orientation.w = qw;
  part.geometry.type = type;
  return part;
}

void expect_pose(const std::vector<uint8_t>& blob, size_t part_offset, const std::string& id, double x, double y,
                 double z, double qx, double qy, double qz, double qw) {
  const size_t pose = part_offset + 64;
  double values[7];
  read_bytes(blob, pose, values, sizeof values);
  expect(same_double(values[0], x) && same_double(values[1], y) && same_double(values[2], z),
         id + " local position");
  expect(same_double(values[3], qx) && same_double(values[4], qy) && same_double(values[5], qz) &&
             same_double(values[6], qw),
         id + " local orientation");
  std::cout << id << " xyz " << values[0] << " " << values[1] << " " << values[2] << " xyzw " << values[3] << " "
            << values[4] << " " << values[5] << " " << values[6] << "\n";
}

void test_arch() {
  expect(std::strlen(kEpoch) == 36, "scene epoch is a uuid4");
  expect(std::strlen(kEpoch) >= sizeof(xgc_dmpc_scene_header_v1::scene_id), "uuid epoch does not fit in scene_id");
  expect(std::strlen(kArchUuid) == 36, "knot Arch id is a uuid4");
  expect(std::strlen(kArchUuid) < sizeof(xgc_dmpc_scene_obstacle_v1::id), "Arch uuid fits without truncation");
  expect(kSceneWireHeaderBytes == 144 && kSceneWireObstacleBytes == 240 && kSceneWirePartBytes == 120 &&
             kSceneWireVertexBytes == 24 && kSceneWireEpochOffset == 72 && kSceneWireMotionOffset == 224 &&
             kSceneWirePartPoseOffset == 64,
         "wire constants drifted from the approved sizes");
  expect(sizeof(xgc_dmpc_paired_state_v1) == 96 && sizeof(xgc_dmpc_controller_status_v1) == 56 &&
             sizeof(xgc_position_target_v1) == 104 && sizeof(xgc_dmpc_timeline_ack_v1) == 56,
         "paired, controller, position target, or ack size changed");

  xgc2_geometry_msgs::SceneSnapshot snapshot;
  snapshot.header.frame_id = "world";
  snapshot.scene_id = "knot_fs150";
  snapshot.epoch = kEpoch;
  snapshot.revision = 42;

  xgc2_geometry_msgs::SceneObstacle arch;
  arch.id = kArchUuid;
  arch.name = "Arch";
  arch.dynamic = false;
  arch.motion_type = "hold";
  arch.pose.position.x = 9;
  arch.pose.position.y = 9;
  arch.pose.position.z = 9;
  arch.pose.orientation.z = 1;
  auto left = make_part("left", "box", -1, 0, 1, 0, 0, 0, 1);
  left.geometry.size.x = 0.4;
  left.geometry.size.y = 0.6;
  left.geometry.size.z = 2;
  auto right = make_part("right", "box", 1, 0, 1, 0, 0, 0, 1);
  right.geometry.size.x = 0.4;
  right.geometry.size.y = 0.6;
  right.geometry.size.z = 2;
  auto lintel = make_part("lintel", "box", 0, 0, 2.2, 0, 0, 0, 1);
  lintel.geometry.size.x = 2.4;
  lintel.geometry.size.y = 0.6;
  lintel.geometry.size.z = 0.4;
  arch.parts = {left, right, lintel};

  xgc2_geometry_msgs::SceneObstacle mover;
  mover.id = "mover";
  mover.name = "arch-traffic";
  mover.dynamic = true;
  mover.motion_type = "constant_twist";
  auto slab = make_part("slab", "box", 0.4, -0.2, 0.8, 0, 0, 0, 2);
  slab.geometry.size.x = 0.5;
  slab.geometry.size.y = 0.4;
  slab.geometry.size.z = 0.3;
  mover.parts = {slab};
  snapshot.obstacles = {arch, mover};

  xgc2_geometry_msgs::SceneState state;
  state.header.frame_id = "world";
  state.header.stamp = ros::Time(1000, 200000000);
  state.epoch = kEpoch;
  state.revision = 42;
  xgc2_geometry_msgs::SceneObstacleState arch_state;
  arch_state.id = kArchUuid;
  arch_state.pose.position.x = kArchX;
  arch_state.pose.position.y = kArchY;
  arch_state.pose.orientation.z = kArchQz;
  arch_state.pose.orientation.w = kArchQw;
  xgc2_geometry_msgs::SceneObstacleState mover_state;
  mover_state.id = "mover";
  mover_state.pose.position.x = 8;
  mover_state.pose.position.y = -2;
  mover_state.pose.position.z = 1;
  mover_state.pose.orientation.w = 1;
  mover_state.twist.linear.x = 0.3;
  mover_state.twist.linear.y = -0.1;
  state.obstacles = {arch_state, mover_state};

  std::vector<uint8_t> blob{0xAB};
  std::string error = "unset";
  expect(xgc_dmpc_pack_scene_blob(snapshot, state, &blob, &error), error);
  constexpr size_t kHeader = 144;
  constexpr size_t kObstacle = 240;
  constexpr size_t kPart = 120;
  const size_t need = kHeader + 2 * kObstacle + 4 * kPart;
  expect(blob.size() == need, "blob length " + std::to_string(blob.size()));
  std::cout << "arch blob " << blob.size() << "\n";
  if (blob.size() != need) return;

  uint32_t counts[4];
  read_bytes(blob, 0, counts, sizeof counts);
  expect(counts[0] == 1 && counts[1] == 2 && counts[2] == 4 && counts[3] == 0, "header counts");
  uint64_t revision = 0;
  read_bytes(blob, 16, &revision, sizeof revision);
  expect(revision == 42, "revision");
  double stamp = 0;
  read_bytes(blob, 136, &stamp, sizeof stamp);
  expect(same_double(stamp, state.header.stamp.toSec()), "source state timestamp");
  expect(read_c_string(blob, 24, 32) == "knot_fs150", "scene_id");
  expect(read_c_string(blob, 24, 32) != kEpoch, "scene_id is not the epoch");
  expect(read_c_string(blob, 56, 16) == "world", "frame");
  expect(read_c_string(blob, 72, 64) == kEpoch, "epoch");
  std::cout << "epoch " << read_c_string(blob, 72, 64) << "\n";
  std::cout << "scene_id " << read_c_string(blob, 24, 32) << "\n";

  auto obstacle_at = [&](size_t index) { return kHeader + index * kObstacle; };
  expect(read_c_string(blob, obstacle_at(0), 64) == kArchUuid, "arch id");
  expect(read_c_string(blob, obstacle_at(0) + 224, 16) == "hold", "arch motion");
  expect(read_c_string(blob, obstacle_at(1) + 224, 16) == "constant_twist", "mover motion");
  uint32_t dynamic = 99;
  read_bytes(blob, obstacle_at(0) + 112, &dynamic, sizeof dynamic);
  expect(dynamic == 0, "arch dynamic flag");
  read_bytes(blob, obstacle_at(1) + 112, &dynamic, sizeof dynamic);
  expect(dynamic == 1, "mover dynamic flag");
  double body[3];
  read_bytes(blob, obstacle_at(0) + 120, body, sizeof body);
  expect(same_double(body[0], kArchX) && same_double(body[1], kArchY) && same_double(body[2], 0), "arch state position");
  expect(!(same_double(body[0], 9) && same_double(body[1], 9)), "arch body is not the definition pose");
  double body_quat[4];
  read_bytes(blob, obstacle_at(0) + 144, body_quat, sizeof body_quat);
  expect(same_double(body_quat[0], 0) && same_double(body_quat[1], 0) && same_double(body_quat[2], kArchQz) &&
             same_double(body_quat[3], kArchQw),
         "arch state orientation");
  read_bytes(blob, obstacle_at(1) + 176, body, sizeof body);
  expect(same_double(body[0], 0.3) && same_double(body[1], -0.1) && same_double(body[2], 0), "mover linear twist");
  std::cout << "motion arch " << read_c_string(blob, obstacle_at(0) + 224, 16) << "\n";
  std::cout << "motion mover " << read_c_string(blob, obstacle_at(1) + 224, 16) << "\n";

  auto part_at = [&](size_t index) { return kHeader + 2 * kObstacle + index * kPart; };
  expect(read_c_string(blob, part_at(0) + 8, 16) == "left", "left id");
  expect(read_c_string(blob, part_at(2) + 8, 16) == "lintel", "lintel id");
  expect(read_c_string(blob, part_at(3) + 8, 16) == "slab", "slab id");
  expect_pose(blob, part_at(0), "left", -1, 0, 1, 0, 0, 0, 1);
  expect_pose(blob, part_at(1), "right", 1, 0, 1, 0, 0, 0, 1);
  expect_pose(blob, part_at(2), "lintel", 0, 0, 2.2, 0, 0, 0, 1);
  expect_pose(blob, part_at(3), "slab", 0.4, -0.2, 0.8, 0, 0, 0, 2);

  uint32_t geometry = 0;
  read_bytes(blob, part_at(2) + 4, &geometry, sizeof geometry);
  expect(geometry == 1, "lintel is a box");
  double param[4];
  read_bytes(blob, part_at(2) + 24, param, sizeof param);
  expect(same_double(param[0], 2.4) && same_double(param[1], 0.6) && same_double(param[2], 0.4), "lintel size");
  char arch_id[64];
  read_bytes(blob, obstacle_at(0), arch_id, sizeof arch_id);
  expect(std::strcmp(arch_id, kArchUuid) == 0, "packed arch id preserves the source uuid");
}

void expect_rejected(const xgc2_geometry_msgs::SceneSnapshot& snapshot, const xgc2_geometry_msgs::SceneState& state,
                     const std::string& token) {
  std::vector<uint8_t> blob{0xAB};
  std::string error;
  expect(!xgc_dmpc_pack_scene_blob(snapshot, state, &blob, &error), "pack succeeded for " + token);
  expect(error.find(token) != std::string::npos, "error [" + error + "] missing " + token);
  expect(blob.size() == 1 && blob[0] == 0xAB, "rejected pack replaced the previous blob");
  std::cout << "rejected " << token << " -> " << error << "\n";
}

xgc2_geometry_msgs::SceneState matching_state(const xgc2_geometry_msgs::SceneSnapshot& snapshot) {
  xgc2_geometry_msgs::SceneState state;
  state.header.frame_id = snapshot.header.frame_id;
  state.epoch = snapshot.epoch;
  state.revision = snapshot.revision;
  for (const auto& obstacle : snapshot.obstacles) {
    xgc2_geometry_msgs::SceneObstacleState item;
    item.id = obstacle.id;
    item.pose.orientation.w = 1;
    state.obstacles.push_back(item);
  }
  return state;
}

xgc2_geometry_msgs::SceneSnapshot one_obstacle(const std::string& motion, bool dynamic) {
  xgc2_geometry_msgs::SceneSnapshot snapshot;
  snapshot.header.frame_id = "world";
  snapshot.scene_id = "arch-scene";
  snapshot.epoch = kEpoch;
  snapshot.revision = 7;
  xgc2_geometry_msgs::SceneObstacle obstacle;
  obstacle.id = "arch";
  obstacle.name = "compound-arch";
  obstacle.dynamic = dynamic;
  obstacle.motion_type = motion;
  obstacle.parts = {make_part("left", "box", -1.5, 0, 1, 0, 0, 0, 1)};
  snapshot.obstacles = {obstacle};
  return snapshot;
}

void expect_motion(const xgc2_geometry_msgs::SceneSnapshot& snapshot, const std::string& motion, uint32_t dynamic) {
  std::vector<uint8_t> blob;
  std::string error;
  expect(xgc_dmpc_pack_scene_blob(snapshot, matching_state(snapshot), &blob, &error), error);
  if (blob.size() < 144 + 240) {
    expect(false, "motion blob is shorter than one obstacle");
    return;
  }
  expect(read_c_string(blob, 144 + 224, 16) == motion, "motion changed to " + read_c_string(blob, 144 + 224, 16));
  uint32_t flag = 99;
  read_bytes(blob, 144 + 112, &flag, sizeof flag);
  expect(flag == dynamic, "dynamic flag changed");
  std::cout << "kept motion " << motion << " dynamic " << dynamic << "\n";
}

void test_rejections() {
  auto dynamic_hold = one_obstacle("hold", true);
  expect_motion(dynamic_hold, "hold", 1);
  auto static_hold = one_obstacle("hold", false);
  expect_motion(static_hold, "hold", 0);
  auto too_long = one_obstacle(std::string(16, 'm'), false);
  expect_rejected(too_long, matching_state(too_long), "does not fit");
  auto epoch = one_obstacle("hold", false);
  epoch.epoch = std::string(64, 'e');
  auto epoch_state = matching_state(epoch);
  expect_rejected(epoch, epoch_state, "does not fit");
  auto zero = one_obstacle("hold", false);
  zero.obstacles[0].parts[0].pose.orientation.w = 0;
  expect_rejected(zero, matching_state(zero), "orientation");
  auto nonfinite = one_obstacle("hold", false);
  nonfinite.obstacles[0].parts[0].pose.position.x = std::nan("");
  expect_rejected(nonfinite, matching_state(nonfinite), "nonfinite");
  auto mismatch = one_obstacle("static", false);
  auto mismatched = matching_state(mismatch);
  mismatched.epoch = "other-epoch";
  expect_rejected(mismatch, mismatched, "epoch");

  auto packed_static = one_obstacle("static", false);
  std::vector<uint8_t> blob;
  std::string error;
  expect(xgc_dmpc_pack_scene_blob(packed_static, matching_state(packed_static), &blob, &error), error);
  if (blob.size() >= 144 + 240) {
    expect(read_c_string(blob, 144 + 224, 16) == "static", "static motion preserved");
  } else {
    expect(false, "static blob is shorter than one obstacle");
  }
  auto packed_empty = one_obstacle("", false);
  expect(xgc_dmpc_pack_scene_blob(packed_empty, matching_state(packed_empty), &blob, &error), error);
  char motion[16];
  std::memset(motion, 1, sizeof motion);
  if (blob.size() >= 144 + 240) read_bytes(blob, 144 + 224, motion, sizeof motion);
  const char zeros[16] = {};
  expect(blob.size() >= 144 + 240 && std::memcmp(motion, zeros, sizeof motion) == 0, "empty static motion stays empty");

  auto arch_uuid = one_obstacle("hold", false);
  arch_uuid.obstacles[0].id = kArchUuid;
  arch_uuid.obstacles[0].name = "Arch";
  expect(xgc_dmpc_pack_scene_blob(arch_uuid, matching_state(arch_uuid), &blob, &error), error);
  expect(read_c_string(blob, 144, 64) == kArchUuid, "source UUID survived packing");
  auto too_long_id = arch_uuid;
  too_long_id.obstacles[0].id = std::string(64, 'i');
  expect_rejected(too_long_id, matching_state(too_long_id), "does not fit");
  auto long_name = one_obstacle("hold", false);
  long_name.obstacles[0].name = std::string(48, 'n');
  expect_rejected(long_name, matching_state(long_name), "does not fit");
  auto fitted_name = one_obstacle("hold", false);
  fitted_name.obstacles[0].name = std::string(37, 'n');
  expect(xgc_dmpc_pack_scene_blob(fitted_name, matching_state(fitted_name), &blob, &error), error);
  expect(blob.size() >= 144 + 240 && read_c_string(blob, 144 + 64, 48) == std::string(37, 'n'), "37-character name was truncated");
  auto missing_scene = one_obstacle("hold", false);
  missing_scene.scene_id.clear();
  expect_rejected(missing_scene, matching_state(missing_scene), "scene_id");
  auto cube = one_obstacle("hold", false);
  cube.obstacles[0].parts[0].geometry.type = "cube";
  expect_rejected(cube, matching_state(cube), "type");
  for (const char* type : {"cylinder", "box", "capsule", "sphere", "convex"}) {
    auto known = one_obstacle("hold", false);
    known.obstacles[0].parts[0].geometry.type = type;
    if (std::strcmp(type, "convex") == 0) {
      for (int index = 0; index < 4; ++index) {
        geometry_msgs::Point point;
        point.x = index;
        known.obstacles[0].parts[0].geometry.vertices.push_back(point);
      }
    }
    std::string known_error;
    std::vector<uint8_t> known_blob;
    expect(xgc_dmpc_pack_scene_blob(known, matching_state(known), &known_blob, &known_error),
           std::string(type) + " " + known_error);
  }
}

}  // namespace

int main() {
  test_arch();
  test_rejections();
  if (g_fails != 0) {
    std::cerr << g_fails << " assertion(s) failed\n";
    return 1;
  }
  std::cout << "scene-wire assertions passed\n";
  return 0;
}
