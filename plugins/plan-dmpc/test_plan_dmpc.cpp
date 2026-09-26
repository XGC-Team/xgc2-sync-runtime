#include "plan_dmpc.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <dlfcn.h>

#include <yaml-cpp/yaml.h>

#include "xgc_rt.h"

namespace {
xgc_dmpc_planner_config_v1 comprehensive() {
  xgc_dmpc_planner_config_v1 config{};
  std::strcpy(config.algorithm, "legacy");
  config.chain_n = 3;
  config.state_dim = 9;
  config.horizon = 40;
  config.sampling_time = 0.1;
  config.self_id = 1;
  config.fleet_count = 8;
  std::strcpy(config.scene_id, "dmpc-uav8_comprehensive");
  config.timeline_authority = 4;
  return config;
}

int fail(const char* what) {
  std::cerr << "FAIL " << what << '\n';
  return 1;
}

bool put_text(char* dest, size_t cap, const std::string& src) {
  if (src.size() >= cap) return false;
  std::memset(dest, 0, cap);
  if (!src.empty()) std::memcpy(dest, src.data(), src.size());
  return true;
}

void write_xyzw(double out[4], const YAML::Node& orientation) {
  out[0] = orientation[0].as<double>();
  out[1] = orientation[1].as<double>();
  out[2] = orientation[2].as<double>();
  out[3] = orientation[3].as<double>();
}

void write_xyz(double out[3], const YAML::Node& position) {
  out[0] = position[0].as<double>();
  out[1] = position[1].as<double>();
  out[2] = position[2].as<double>();
}

std::vector<uint8_t> pack_scene(const xgc_dmpc_scene_header_v1& header,
                                const std::vector<xgc_dmpc_scene_obstacle_v1>& obstacles,
                                const std::vector<xgc_dmpc_scene_part_v1>& parts,
                                const std::vector<xgc_dmpc_scene_vertex_v1>& vertices) {
  std::vector<uint8_t> blob(sizeof header + obstacles.size() * sizeof(xgc_dmpc_scene_obstacle_v1) +
                            parts.size() * sizeof(xgc_dmpc_scene_part_v1) +
                            vertices.size() * sizeof(xgc_dmpc_scene_vertex_v1));
  size_t at = 0;
  std::memcpy(blob.data() + at, &header, sizeof header);
  at += sizeof header;
  if (!obstacles.empty()) {
    std::memcpy(blob.data() + at, obstacles.data(), obstacles.size() * sizeof(xgc_dmpc_scene_obstacle_v1));
    at += obstacles.size() * sizeof(xgc_dmpc_scene_obstacle_v1);
  }
  if (!parts.empty()) {
    std::memcpy(blob.data() + at, parts.data(), parts.size() * sizeof(xgc_dmpc_scene_part_v1));
    at += parts.size() * sizeof(xgc_dmpc_scene_part_v1);
  }
  if (!vertices.empty()) std::memcpy(blob.data() + at, vertices.data(), vertices.size() * sizeof(xgc_dmpc_scene_vertex_v1));
  return blob;
}
}  // namespace

int main() {
  xgc_dmpc_planner_config_v1 bad = comprehensive();
  std::strcpy(bad.algorithm, "exploration");
  if (xgc_plan_dmpc::PlanDmpc(bad).configure_error().find("unsupported algorithm") == std::string::npos) {
    return fail("exploration was accepted");
  }
  bad = comprehensive();
  bad.horizon = 20;
  if (xgc_plan_dmpc::PlanDmpc(bad).configure_error().find("unsupported dimensions") == std::string::npos) {
    return fail("N20 was accepted");
  }

  xgc_plan_dmpc::PlanDmpc planner(comprehensive());
  if (!planner.configure_error().empty()) return fail(planner.configure_error().c_str());

  xgc_dmpc_paired_state_v1 state{};
  state.pose_stamp_sec = 12.0;
  state.twist_stamp_sec = 10.0;
  state.position[2] = 1.0;
  if (planner.push_state(state, 10.0).empty()) return fail("future pose stamp was accepted");
  state.pose_stamp_sec = 10.0;
  state.twist_stamp_sec = 10.1;
  if (planner.push_state(state, 10.2).empty()) return fail("unequal stamps were accepted");
  state.twist_stamp_sec = 10.0;
  if (planner.push_state(state, 12.0).empty()) return fail("stale paired state was accepted");
  if (!planner.push_state(state, 10.2).empty()) return fail("fresh paired state was rejected");

  xgc_dmpc_controller_status_v1 idle{};
  std::strcpy(idle.state, "Hover");
  idle.stamp_sec = 12.0;
  if (planner.push_controller(idle, 10.2).empty()) return fail("future controller stamp was accepted");
  idle.stamp_sec = 8.0;
  if (planner.push_controller(idle, 10.2).empty()) return fail("stale controller stamp was accepted");
  idle.stamp_sec = 10.0;
  if (!planner.push_controller(idle, 10.2).empty()) return fail("fresh controller status was rejected");

  xgc_dmpc_scene_ids_v1 snapshot{};
  xgc_dmpc_scene_ids_v1 scene_state{};
  std::strcpy(snapshot.scene_id, "dmpc-uav8_comprehensive");
  std::strcpy(scene_state.scene_id, "dmpc-uav8_comprehensive");
  snapshot.revision = scene_state.revision = 3;
  if (planner.push_scene(snapshot, scene_state).empty()) return fail("id-only scene was accepted");

  xgc_dmpc_mission_commit_v1 commit{};
  commit.request.schema = 1;
  commit.request.kind = 1;
  commit.request.revision = 1;
  commit.request.command_id = 9;
  commit.request.rolling = 1;
  commit.request.effective_round = 20;
  commit.round_k = 20;
  commit.mission_ns = 0;
  std::strcpy(commit.request.session_id, "tro-comprehensive");
  std::strcpy(commit.request.origin, "workflow");
  uint8_t digest[32] = {1};
  if (!planner.push_commit(commit, digest).empty()) return fail("start commit was rejected");
  commit.mission_ns = 1;
  digest[0] = 2;
  if (planner.push_commit(commit, digest).empty()) return fail("conflicting mission time was accepted");
  digest[0] = 1;
  commit.mission_ns = 0;
  if (!planner.push_commit(commit, digest).empty()) return fail("identical revision was not idempotent");

  if (!planner.push_neighbor(2).empty()) return fail("first neighbor was rejected");
  if (planner.push_neighbor(2).empty()) return fail("duplicate neighbor was accepted");
  for (int id = 3; id <= 8; ++id) {
    if (!planner.push_neighbor(static_cast<uint32_t>(id)).empty()) return fail("roster neighbor rejected");
  }
  if (planner.push_neighbor(2).empty()) return fail("eighth copy was accepted");

  xgc_dmpc_scene_heartbeat_v1 beat{100.0};
  if (!planner.push_heartbeat(beat).empty()) return fail("heartbeat receipt was rejected");
  auto held = planner.step(10.2, 10.2, 100.2);
  auto stale = planner.step(10.2, 10.2, 101.0);
  if (std::strstr(held.status.reject_reason, "scene geometry") == nullptr) {
    return fail("fresh heartbeat was treated as a loaded scene or a substitute leader");
  }
  if (std::strstr(stale.status.reject_reason, "heartbeat") == nullptr) {
    return fail("heartbeat age was not recomputed");
  }
  if (held.status.solver_called || stale.status.solver_called) return fail("solver ran without a leader factory");
  if (std::strcmp(held.status.lifecycle, "OPTIMIZING_HOLD") == 0) {
    return fail("lifecycle reached optimizing without a loaded scene");
  }
  if (held.timeline.mission_ns != 0 || stale.timeline.mission_ns != 0) {
    return fail("mission time advanced from local steps");
  }
  if (held.timeline.applied_revision != 1) return fail("start revision was not applied from the commit");
  commit.request.kind = 6;
  commit.request.revision = 2;
  commit.request.predecessor_revision = 1;
  std::memcpy(commit.request.predecessor_digest, digest, 32);
  commit.request.command_id = 10;
  commit.request.rolling = 1;
  commit.request.pattern_id = 5;
  commit.request.effective_round = 25;
  commit.request.anchor_ns = 5 * 100000000LL;
  commit.round_k = 25;
  commit.mission_ns = commit.request.anchor_ns;
  digest[0] = 3;
  if (!planner.push_commit(commit, digest).empty()) return fail("pattern commit was rejected");
  auto pattern = planner.step(10.2, 10.2, 100.2);
  if (pattern.timeline.applied_revision != 2 || pattern.timeline.fault != 0 || !std::isfinite(planner.pattern_offset().x())) {
    return fail("pattern 5 was not applied from the installed factory");
  }
  if (pattern.timeline.mission_ns != 5 * 100000000LL) return fail("pattern commit did not keep the rolling phase");
  auto ack = planner.acknowledge(commit.request, digest, 4);
  if (ack.accepted != 1 || sizeof(ack) != 56 || planner.step(10.2, 10.2, 100.2).timeline.applied_revision != 2) {
    return fail("ack changed applied phase or rejected the frozen envelope origin");
  }
  std::memset(commit.request.origin, 0, sizeof commit.request.origin);
  std::strcpy(commit.request.origin, "spoof");
  if (planner.acknowledge(commit.request, digest, 9).accepted != 0) {
    return fail("payload origin string was trusted");
  }
  xgc_dmpc_mission_commit_v1 rewind = commit;
  rewind.request.revision = 3;
  rewind.request.predecessor_revision = 2;
  std::memcpy(rewind.request.predecessor_digest, digest, 32);
  rewind.request.kind = 5;
  rewind.request.rolling = 1;
  rewind.request.pattern_id = 0;
  rewind.request.goal_xyz[0] = 15.0;
  rewind.request.effective_round = 30;
  rewind.request.anchor_ns = 0;
  rewind.round_k = 30;
  rewind.mission_ns = 0;
  uint8_t rewind_digest[32] = {4};
  if (planner.push_commit(rewind, rewind_digest).empty()) return fail("rewinding anchor was accepted");
  rewind.request.anchor_ns = 10 * 100000000LL;
  rewind.mission_ns = rewind.request.anchor_ns;
  if (!planner.push_commit(rewind, rewind_digest).empty()) return fail("continuous rolling anchor was rejected");
  rewind.request.rolling = 0;
  rewind.mission_ns = rewind.request.anchor_ns;
  rewind_digest[0] = 5;
  if (!planner.push_commit(rewind, rewind_digest).empty()) return fail("held same-revision commit was rejected");
  auto frozen = planner.step(10.2, 10.2, 100.2);
  if (frozen.timeline.mission_ns != rewind.request.anchor_ns || frozen.timeline.committed_revision != 3) {
    return fail("planner did not keep the held mission");
  }

  YAML::Node document = YAML::LoadFile(
      "/home/lxk/Paper/academic/ros1_ws/src/planner/formation_generator/config/scenarios/"
      "dmpc_comprehensive/scene/scene.yaml");
  std::vector<xgc_dmpc_scene_obstacle_v1> obstacles(77);
  std::vector<xgc_dmpc_scene_part_v1> parts(77);
  std::vector<xgc_dmpc_scene_vertex_v1> vertices;
  const auto& nodes = document["obstacles"];
  if (nodes.size() != 77) return fail("scene file no longer has 77 obstacles");
  for (int index = 0; index < 77; ++index) {
    const auto& obstacle = nodes[index];
    if (!put_text(obstacles[index].id, sizeof obstacles[index].id, obstacle["id"].as<std::string>()) ||
        !put_text(obstacles[index].name, sizeof obstacles[index].name, obstacle["name"].as<std::string>())) {
      return fail("comprehensive obstacle id or name does not fit");
    }
    const auto& position = obstacle["pose"]["position"];
    obstacles[index].position[0] = position[0].as<double>();
    obstacles[index].position[1] = position[1].as<double>();
    obstacles[index].position[2] = position[2].as<double>();
    const auto& orientation = obstacle["pose"]["orientation"];
    obstacles[index].orientation_xyzw[0] = orientation[0].as<double>();
    obstacles[index].orientation_xyzw[1] = orientation[1].as<double>();
    obstacles[index].orientation_xyzw[2] = orientation[2].as<double>();
    obstacles[index].orientation_xyzw[3] = orientation[3].as<double>();
    const std::string motion = obstacle["motion"]["type"].as<std::string>();
    if (!put_text(obstacles[index].motion_type, sizeof obstacles[index].motion_type, motion)) {
      return fail("comprehensive motion_type does not fit");
    }
    const auto& geometry = obstacle["parts"][0]["geometry"];
    const std::string type = geometry["type"].as<std::string>();
    parts[index].obstacle_index = static_cast<uint32_t>(index);
    if (!put_text(parts[index].part_id, sizeof parts[index].part_id, obstacle["parts"][0]["id"].as<std::string>())) {
      return fail("comprehensive part id does not fit");
    }
    parts[index].orientation_xyzw[3] = 1.0;
    if (type == "cylinder" || type == "capsule") {
      parts[index].geometry_type = type == "cylinder" ? 0u : 2u;
      parts[index].param[0] = geometry["radius"].as<double>();
      parts[index].param[1] = geometry["height"].as<double>();
    } else if (type == "box") {
      parts[index].geometry_type = 1;
      parts[index].param[0] = geometry["size"][0].as<double>();
      parts[index].param[1] = geometry["size"][1].as<double>();
      parts[index].param[2] = geometry["size"][2].as<double>();
    } else if (type == "sphere") {
      parts[index].geometry_type = 3;
      parts[index].param[0] = geometry["radius"].as<double>();
    } else if (type == "convex") {
      parts[index].geometry_type = 4;
      parts[index].vertex_begin = static_cast<uint32_t>(vertices.size());
      for (const auto& vertex : geometry["vertices"]) {
        xgc_dmpc_scene_vertex_v1 value{};
        value.xyz[0] = vertex[0].as<double>();
        value.xyz[1] = vertex[1].as<double>();
        value.xyz[2] = vertex[2].as<double>();
        vertices.push_back(value);
      }
      parts[index].vertex_count = static_cast<uint32_t>(vertices.size()) - parts[index].vertex_begin;
    } else {
      return fail("unexpected geometry");
    }
  }
  xgc_dmpc_scene_header_v1 header{};
  header.schema = 1;
  header.obstacle_count = 77;
  header.part_count = 77;
  header.vertex_count = static_cast<uint32_t>(vertices.size());
  header.revision = 3;
  std::strcpy(header.scene_id, "dmpc-uav8_comprehensive");
  std::strcpy(header.frame, "world");
  const char* epoch = "6db3ae1c-97b9-4019-800b-3b21ca818b99";
  if (std::strlen(epoch) != 36 || !put_text(header.epoch, sizeof header.epoch, epoch)) {
    return fail("snapshot epoch does not fit");
  }
  if (!put_text(obstacles[1].motion_type, sizeof obstacles[1].motion_type, "static")) {
    return fail("static motion_type does not fit");
  }
  std::memset(obstacles[2].motion_type, 0, sizeof obstacles[2].motion_type);
  auto dynamic_hold = obstacles;
  dynamic_hold[0].dynamic = 1;
  if (!put_text(dynamic_hold[0].motion_type, sizeof dynamic_hold[0].motion_type, "hold")) {
    return fail("hold motion_type does not fit");
  }
  const std::string rejected = planner.push_scene_wire(header, dynamic_hold.data(), parts.data(), vertices.data());
  if (rejected.find("constant_twist") == std::string::npos || rejected.find("hold") == std::string::npos ||
      planner.static_obstacle_count() != 0) {
    return fail("dynamic hold was rewritten or accepted");
  }
  const std::string loaded = planner.push_scene_wire(header, obstacles.data(), parts.data(), vertices.data());
  if (!loaded.empty()) return fail(loaded.c_str());
  if (planner.static_obstacle_count() != 77 || planner.leader_rows() != 6 || planner.leader_cols() != 41) {
    return fail("scene or leader factory did not enter the planner");
  }
  if (planner.scene_epoch() != epoch || planner.scene_epoch() == header.scene_id) {
    return fail("admitted epoch was replaced by scene_id");
  }
  const auto comprehensive_blob = pack_scene(header, obstacles, parts, vertices);
  formation_generator_dmpc::PlainSceneDefinitionView decoded;
  formation_generator_dmpc::PlainSceneDynamicState decoded_state;
  bool have_state = true;
  const std::string decode_error =
      planner.decode_scene_blob(comprehensive_blob.data(), comprehensive_blob.size(), &decoded, &decoded_state, &have_state);
  if (!decode_error.empty() || have_state || decoded.epoch != epoch || decoded.definition.epoch != epoch ||
      decoded.definition.obstacles.size() != 77 ||
      decoded.definition.obstacles[0].motion_type != "hold" ||
      decoded.definition.obstacles[1].motion_type != "static" ||
      !decoded.definition.obstacles[2].motion_type.empty() ||
      decoded.definition.obstacles[0].parts.at(0).pose.orientation.w() != 1.0 ||
      decoded.definition.obstacles[0].parts.at(0).pose.position.norm() != 0.0) {
    return fail("comprehensive blob dropped epoch, motion_type, or identity part pose");
  }
  xgc_dmpc_scene_header_v1 wrapped_header{};
  wrapped_header.schema = 1;
  wrapped_header.obstacle_count = 1;
  wrapped_header.part_count = 1;
  wrapped_header.vertex_count = 4;
  wrapped_header.revision = 1;
  if (!put_text(wrapped_header.scene_id, sizeof wrapped_header.scene_id, "wrap") ||
      !put_text(wrapped_header.frame, sizeof wrapped_header.frame, "world") ||
      !put_text(wrapped_header.epoch, sizeof wrapped_header.epoch, epoch)) {
    return fail("wrapped header does not fit");
  }
  xgc_dmpc_scene_obstacle_v1 wrapped_body{};
  wrapped_body.orientation_xyzw[3] = 1.0;
  xgc_dmpc_scene_part_v1 wrapped_part{};
  wrapped_part.geometry_type = 4;
  wrapped_part.vertex_begin = 1;
  wrapped_part.vertex_count = 0xFFFFFFFFu;
  wrapped_part.orientation_xyzw[3] = 1.0;
  if (!put_text(wrapped_body.id, sizeof wrapped_body.id, "wrap") ||
      !put_text(wrapped_body.name, sizeof wrapped_body.name, "wrap") ||
      !put_text(wrapped_body.motion_type, sizeof wrapped_body.motion_type, "hold") ||
      !put_text(wrapped_part.part_id, sizeof wrapped_part.part_id, "hull")) {
    return fail("wrapped obstacle text does not fit");
  }
  if (wrapped_part.vertex_begin + wrapped_part.vertex_count > wrapped_header.vertex_count) {
    return fail("convex counterexample no longer wraps the old sum");
  }
  const auto wrapped_blob = pack_scene(wrapped_header, {wrapped_body}, {wrapped_part},
                                       std::vector<xgc_dmpc_scene_vertex_v1>(4));
  formation_generator_dmpc::PlainSceneDefinitionView wrapped_view;
  formation_generator_dmpc::PlainSceneDynamicState wrapped_state;
  bool wrapped_dynamic = false;
  const std::string wrapped_error = planner.decode_scene_blob(
      wrapped_blob.data(), wrapped_blob.size(), &wrapped_view, &wrapped_state, &wrapped_dynamic);
  if (wrapped_error.find("vertices") == std::string::npos) {
    return fail("wrapped convex vertex range was accepted");
  }
  bool saw_solve = false;
  for (int attempt = 0; attempt < 6 && !saw_solve; ++attempt) {
    const auto trace = planner.step(10.2, 10.2, 100.2);
    if (!trace.status.solver_called) continue;
    saw_solve = true;
    if (!trace.status.solver_ok) {
      if (trace.have_position_target || !trace.own_plan.empty()) {
        return fail("unsolved round published a position target");
      }
    } else if (!trace.have_position_target || trace.own_plan.empty()) {
      return fail("solved round did not publish position target and own plan");
    }
  }
  if (!saw_solve) return fail("real solve was not called");
  YAML::Node arch_document = YAML::LoadFile(
      "/home/lxk/Paper/academic/ros1_ws/src/planner/formation_generator/config/scenarios/knot_fs150/scene.yaml");
  YAML::Node arch;
  const auto arch_obstacles = arch_document["obstacles"];
  for (std::size_t index = 0; index < arch_obstacles.size(); ++index) {
    if (arch_obstacles[index]["name"].as<std::string>() == "Arch") arch = arch_obstacles[index];
  }
  if (!arch || !arch["id"]) return fail("knot scene has no Arch");
  const std::string arch_source_id = arch["id"].as<std::string>();
  xgc_dmpc_scene_obstacle_v1 id_probe{};
  if (arch_source_id.size() < sizeof id_probe.id || put_text(id_probe.id, sizeof id_probe.id, arch_source_id)) {
    return fail("uuid arch id was truncated or unexpectedly fit in id[16]");
  }
  xgc_dmpc_scene_header_v1 arch_header{};
  arch_header.schema = 1;
  arch_header.obstacle_count = 1;
  arch_header.part_count = static_cast<uint32_t>(arch["parts"].size());
  arch_header.revision = 9;
  if (arch_header.part_count != 3 || !put_text(arch_header.scene_id, sizeof arch_header.scene_id, "knot_fs150") ||
      !put_text(arch_header.frame, sizeof arch_header.frame, "world") ||
      !put_text(arch_header.epoch, sizeof arch_header.epoch, epoch)) {
    return fail("arch header does not match the source compound");
  }
  xgc_dmpc_scene_obstacle_v1 arch_body{};
  if (!put_text(arch_body.id, sizeof arch_body.id, "arch") ||
      !put_text(arch_body.name, sizeof arch_body.name, arch["name"].as<std::string>()) ||
      !put_text(arch_body.motion_type, sizeof arch_body.motion_type, arch["motion"]["type"].as<std::string>()) ||
      std::strncmp(arch_body.id, arch_source_id.c_str(), sizeof arch_body.id - 1) == 0) {
    return fail("arch id was truncated into the wire slot");
  }
  write_xyz(arch_body.position, arch["pose"]["position"]);
  write_xyzw(arch_body.orientation_xyzw, arch["pose"]["orientation"]);
  std::vector<xgc_dmpc_scene_part_v1> arch_parts(arch_header.part_count);
  for (uint32_t index = 0; index < arch_header.part_count; ++index) {
    const auto& part = arch["parts"][index];
    arch_parts[index].obstacle_index = 0;
    arch_parts[index].geometry_type = 1;
    if (part["geometry"]["type"].as<std::string>() != "box" ||
        !put_text(arch_parts[index].part_id, sizeof arch_parts[index].part_id, part["id"].as<std::string>())) {
      return fail("arch part is not the source box");
    }
    write_xyz(arch_parts[index].position, part["pose"]["position"]);
    write_xyzw(arch_parts[index].orientation_xyzw, part["pose"]["orientation"]);
    write_xyz(arch_parts[index].param, part["geometry"]["size"]);
  }
  const auto arch_blob = pack_scene(arch_header, {arch_body}, arch_parts, {});
  formation_generator_dmpc::PlainSceneDefinitionView arch_view;
  formation_generator_dmpc::PlainSceneDynamicState arch_state;
  bool arch_dynamic = true;
  const std::string arch_error =
      planner.decode_scene_blob(arch_blob.data(), arch_blob.size(), &arch_view, &arch_state, &arch_dynamic);
  if (!arch_error.empty() || arch_dynamic || arch_view.epoch != epoch || arch_view.epoch == arch_header.scene_id ||
      arch_view.definition.obstacles.size() != 1 || arch_view.definition.obstacles[0].motion_type != "hold" ||
      arch_view.definition.obstacles[0].parts.size() != 3) {
    return fail("arch blob dropped epoch, hold, or local parts");
  }
  const auto adapted = formation_generator_dmpc::PlainSceneAdapter().convert(arch_view, nullptr);
  const Eigen::Quaterniond parent(arch_body.orientation_xyzw[3], arch_body.orientation_xyzw[0],
                                  arch_body.orientation_xyzw[1], arch_body.orientation_xyzw[2]);
  const Eigen::Quaterniond parent_rotation = parent.normalized();
  const Eigen::Vector3d parent_position(arch_body.position[0], arch_body.position[1], arch_body.position[2]);
  if (adapted.statics.size() != 3) return fail("arch did not yield three compound bodies");
  for (const auto& part : arch_view.definition.obstacles[0].parts) {
    const Eigen::Vector3d world = parent_position + parent_rotation * part.pose.position;
    const Eigen::Quaterniond orientation = parent_rotation * part.pose.orientation.normalized();
    const convex_geometry::BodyInstance* body = nullptr;
    for (const auto& candidate : adapted.statics) {
      if (candidate.name == "arch/" + part.id) body = &candidate;
    }
    if (body == nullptr) return fail("arch part was not placed in the compound");
    const Eigen::Vector3d got(body->pose.position.x, body->pose.position.y, body->pose.position.z);
    const Eigen::Quaterniond got_q(body->pose.orientation.w, body->pose.orientation.x, body->pose.orientation.y,
                                   body->pose.orientation.z);
    if ((got - world).norm() > 1e-9 || std::abs(got_q.normalized().dot(orientation.normalized())) < 1.0 - 1e-9 ||
        (got - parent_position).norm() < 1e-6) {
      return fail("rotated translated arch part was left on the obstacle origin");
    }
  }
  header.obstacle_count = 0;
  if (planner.push_scene_wire(header, obstacles.data(), parts.data(), vertices.data()).empty()) {
    return fail("id-only scene became ready");
  }
  using Entry = const xgc_plugin_descriptor* (*)();
  auto* entry = reinterpret_cast<Entry>(dlsym(RTLD_DEFAULT, "xgc_rt_plugin_v1"));
  if (entry == nullptr || entry() == nullptr || std::strcmp(entry()->name, "plan-dmpc") != 0 || entry()->port_count != 11) {
    return fail("xgc_rt_plugin_v1 descriptor was not exported");
  }
  bool stale_ack = false;
  bool position_target = false;
  bool own_plan = false;
  bool sync_trigger = false;
  bool planner_status = false;
  for (uint32_t index = 0; index < entry()->port_count; ++index) {
    const char* schema = entry()->ports[index].schema_id;
    const char* name = entry()->ports[index].name;
    if (std::strcmp(schema, "xgc.dmpc.mission_ack/1") == 0) stale_ack = true;
    if (std::strcmp(name, "position_target") == 0 && std::strcmp(schema, "xgc.position_target/1") == 0 &&
        entry()->ports[index].dir == XGC_PORT_OUT) {
      position_target = true;
    }
    if (std::strcmp(name, "own_plan") == 0 && std::strcmp(schema, "xgc.dmpc.assumed_trajectory/1") == 0 &&
        entry()->ports[index].dir == XGC_PORT_OUT) {
      own_plan = true;
    }
    if (std::strcmp(name, "sync_trigger") == 0 && std::strcmp(schema, "xgc.dmpc.sync_trigger/1") == 0 &&
        entry()->ports[index].dir == XGC_PORT_IN) {
      sync_trigger = true;
    }
    if (std::strcmp(name, "planner_status") == 0 && std::strcmp(schema, "xgc.dmpc.planner_status/1") == 0) {
      planner_status = true;
    }
  }
  if (stale_ack || !position_target || !own_plan || !sync_trigger || !planner_status) {
    return fail("descriptor is missing the vehicle position target or own plan");
  }
  std::cout << "lifecycle=" << held.status.lifecycle << " reject=" << held.status.reject_reason << '\n';
  std::cout << "obstacles=" << planner.static_obstacle_count() << " ports=" << entry()->port_count << '\n';
  return 0;
}
