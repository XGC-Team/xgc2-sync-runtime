#include "plan_dmpc.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "formation_generator/params/rosparam_yaml.h"

namespace xgc_plan_dmpc {
namespace {
constexpr double kStaleSec = 1.0;
constexpr double kHeartbeatTimeoutSec = 0.5;

bool finite3(const double v[3]) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool canonical_text(const char* text, size_t size) {
  size_t index = 0;
  while (index < size && text[index] != '\0') ++index;
  while (index < size) {
    if (text[index] != '\0') return false;
    ++index;
  }
  return true;
}

bool take_text(const char* text, size_t size, std::string* out) {
  size_t index = 0;
  while (index < size && text[index] != '\0') ++index;
  if (index == size) return false;
  *out = std::string(text, index);
  return true;
}

std::string stamp_error(double stamp, double now, const char* what) {
  if (!std::isfinite(stamp) || !std::isfinite(now) || stamp <= 0.0) {
    return std::string(what) + " stamp is missing or not finite";
  }
  if (stamp > now) return std::string(what) + " stamp is in the future";
  if (now - stamp > kStaleSec) return std::string(what) + " sample is stale";
  return {};
}
}  // namespace

namespace {
std::string mismatch(const char* name) {
  return std::string("plugin config does not match the loaded manifest: ") + name;
}
}  // namespace

std::unique_ptr<PlanDmpc> PlanDmpc::open(const PlanDmpcOpen& request, std::string* error) {
  auto fail = [&](const std::string& text) {
    if (error != nullptr) *error = text;
    return std::unique_ptr<PlanDmpc>();
  };
  try {
    if (request.manifest_path.empty()) return fail("manifest path is required");
    if (request.scene_id.empty()) return fail("scene_id is required");
    if (request.self_id < 1) return fail("self_id is required");
    const auto params = formation_generator_dmpc::privateParamsFromManifest(
        formation_generator_dmpc::loadParamManifest(request.manifest_path));
    auto agent = std::make_unique<formation_generator_dmpc::DmpcAgent>(params);
    const auto& loaded = agent->configuration();
    const int horizon = agent->referenceInfo().horizon - 1;
    if (!loaded.params.hasParam("num_uavs")) return fail("manifest is missing num_uavs");
    if (!loaded.params.hasParam("uav_id")) return fail("manifest is missing uav_id");
    if (static_cast<int>(loaded.uav_id) != request.self_id) {
      return fail("self_id does not match the manifest uav_id");
    }
    if (request.algorithm && *request.algorithm != loaded.algorithm) return fail(mismatch("algorithm"));
    if (request.chain_n && *request.chain_n != loaded.chain_n) return fail(mismatch("chain_n"));
    if (request.state_dim && *request.state_dim != loaded.assumed_state_dim) return fail(mismatch("state_dim"));
    if (request.horizon && *request.horizon != horizon) return fail(mismatch("horizon"));
    if (request.fleet_count && *request.fleet_count != loaded.num_uavs) return fail(mismatch("fleet_count"));
    if (request.sampling_time && *request.sampling_time != loaded.mpc_params.sampling_time) {
      return fail(mismatch("sampling_time"));
    }
    if (horizon <= 0 || !std::isfinite(loaded.mpc_params.sampling_time) ||
        loaded.mpc_params.sampling_time <= 0.0 || loaded.num_uavs < 1 || loaded.uav_id < 1 ||
        loaded.uav_id > loaded.num_uavs) {
      return fail("loaded roster or horizon is not usable");
    }
    xgc_dmpc_planner_config_v1 fields{};
    std::snprintf(fields.algorithm, sizeof fields.algorithm, "%s", loaded.algorithm.c_str());
    if (request.scene_id.size() >= sizeof fields.scene_id) return fail("scene_id does not fit");
    std::snprintf(fields.scene_id, sizeof fields.scene_id, "%s", request.scene_id.c_str());
    fields.chain_n = loaded.chain_n;
    fields.state_dim = loaded.assumed_state_dim;
    fields.horizon = horizon;
    fields.sampling_time = loaded.mpc_params.sampling_time;
    fields.self_id = static_cast<int32_t>(loaded.uav_id);
    fields.fleet_count = loaded.num_uavs;
    fields.timeline_authority = request.timeline_authority;
    return std::unique_ptr<PlanDmpc>(new PlanDmpc(fields, std::move(agent)));
  } catch (const std::exception& ex) {
    return fail(ex.what());
  }
}

PlanDmpc::PlanDmpc(const xgc_dmpc_planner_config_v1& config,
                   std::unique_ptr<formation_generator_dmpc::DmpcAgent> agent)
    : config_(config), agent_(std::move(agent)) {}

int PlanDmpc::leader_rows() const {
  return agent_ ? agent_->referenceInfo().rows : 0;
}

int PlanDmpc::leader_cols() const {
  return agent_ ? agent_->referenceInfo().columns : 0;
}

std::string PlanDmpc::push_state(const xgc_dmpc_paired_state_v1& state, double now_sec) {
  if (state.pose_stamp_sec != state.twist_stamp_sec) {
    state_ok_ = false;
    return "pose and twist stamps differ";
  }
  const std::string stamp = stamp_error(state.pose_stamp_sec, now_sec, "paired state");
  if (!stamp.empty() || !finite3(state.position) || !finite3(state.linear_velocity)) {
    state_ok_ = false;
    return stamp.empty() ? "paired state is nonfinite" : stamp;
  }
  state_ok_ = true;
  state_stamp_sec_ = state.pose_stamp_sec;
  measured_ << state.position[0], state.position[1], state.position[2], state.linear_velocity[0],
      state.linear_velocity[1], state.linear_velocity[2];
  agent_->observeState(measured_, Eigen::Vector4d(state.orientation_xyzw[0], state.orientation_xyzw[1],
                                               state.orientation_xyzw[2], state.orientation_xyzw[3]), state_stamp_sec_);
  return {};
}

std::string PlanDmpc::push_controller(const xgc_dmpc_controller_status_v1& status, double now_sec) {
  const std::string stamp = stamp_error(status.stamp_sec, now_sec, "controller");
  if (!stamp.empty()) {
    agent_->observeController("", now_sec);
    return stamp;
  }
  const std::string state(status.state, strnlen(status.state, sizeof status.state));
  agent_->observeController(state, status.stamp_sec);
  return {};
}

std::string PlanDmpc::push_scene(const xgc_dmpc_scene_ids_v1& snapshot, const xgc_dmpc_scene_ids_v1& state) {
  if (std::strcmp(snapshot.scene_id, config_.scene_id) != 0 ||
      std::strcmp(state.scene_id, config_.scene_id) != 0 || snapshot.revision == 0 ||
      snapshot.revision != state.revision) {
    return "scene snapshot/state identity does not match";
  }
  // Identity alone does not supply the scene geometry.
  return "scene geometry is not loaded; refusing id-only acceptance";
}

xgc_dmpc_mission_ack_v1 PlanDmpc::acknowledge(const xgc_dmpc_mission_timeline_v1& request,
                                                     const uint8_t digest[32],
                                                     uint16_t envelope_origin) const {
  xgc_dmpc_mission_ack_v1 ack{};
  ack.revision = request.revision;
  ack.command_id = request.command_id;
  if (digest != nullptr) std::memcpy(ack.digest, digest, 32);
  if (envelope_origin != config_.timeline_authority) return ack;
  ack.accepted = 1;
  return ack;
}

std::string PlanDmpc::push_scene_definition(const formation_generator_dmpc::PlainSceneDefinitionView& snapshot,
                                            const formation_generator_dmpc::PlainSceneDynamicState* state) {
  namespace fg = formation_generator_dmpc;
  geometry_ready_ = false;
  scene_epoch_.clear();
  // One scene converter in the academic core. The native wire supplies its
  // ROS-free scene data; the planner owns geometry admission and local crop.
  auto pose = [](const fg::PlainScenePose& value) {
    convex_geometry::PlainPose out;
    out.position = {value.position.x(), value.position.y(), value.position.z()};
    out.orientation = {value.orientation.x(), value.orientation.y(), value.orientation.z(), value.orientation.w()};
    return out;
  };
  fg::SceneSnapshotData data;
  data.scene_id = config_.scene_id;
  data.epoch = snapshot.epoch;
  data.frame_id = snapshot.frame_id;
  data.revision = snapshot.revision;
  data.stamp = snapshot.definition.stamp_sec;
  for (const auto& obstacle : snapshot.definition.obstacles) {
    fg::SceneObstacleData entry;
    entry.id = obstacle.id;
    entry.dynamic = obstacle.dynamic;
    entry.motion_type = obstacle.motion_type;
    entry.pose = pose(obstacle.pose);
    for (const auto& part : obstacle.parts) {
      fg::ScenePartData item;
      item.id = part.id;
      item.pose = pose(part.pose);
      item.geometry.type = part.type;
      item.geometry.size = {part.size.x(), part.size.y(), part.size.z()};
      item.geometry.radius = part.radius;
      item.geometry.height = part.height;
      for (const auto& vertex : part.vertices) {
        item.geometry.vertices.push_back({vertex.x(), vertex.y(), vertex.z()});
      }
      entry.parts.push_back(std::move(item));
    }
    data.obstacles.push_back(std::move(entry));
  }
  agent_->receiveSceneSnapshot(data);
  if (state) {
    fg::SceneStateData dynamic;
    dynamic.epoch = state->epoch;
    dynamic.frame_id = state->frame_id;
    dynamic.revision = state->revision;
    dynamic.stamp = state->stamp_sec;
    for (const auto& obstacle : state->obstacles) {
      fg::SceneObstacleStateData entry;
      entry.id = obstacle.id;
      entry.pose = pose(obstacle.pose);
      entry.twist.linear = {obstacle.twist.linear.x(), obstacle.twist.linear.y(), obstacle.twist.linear.z()};
      entry.twist.angular = {obstacle.twist.angular.x(), obstacle.twist.angular.y(), obstacle.twist.angular.z()};
      dynamic.obstacles.push_back(std::move(entry));
    }
    agent_->receiveSceneState(dynamic, state->stamp_sec);
  }
  if (!agent_->sceneValid()) return agent_->sceneError().empty() ? "scene awaits matching state" : agent_->sceneError();
  geometry_ready_ = true;
  scene_epoch_ = snapshot.epoch;
  return {};
}

std::string PlanDmpc::decode_records(const xgc_dmpc_scene_header_v1& header,
                                     const xgc_dmpc_scene_obstacle_v1* obstacles,
                                     const xgc_dmpc_scene_part_v1* parts,
                                     const xgc_dmpc_scene_vertex_v1* vertices,
                                     formation_generator_dmpc::PlainSceneDefinitionView* view,
                                     formation_generator_dmpc::PlainSceneDynamicState* state,
                                     bool* have_state) const {
  static_assert(offsetof(xgc_dmpc_scene_header_v1, epoch) == 72, "epoch offset");
  static_assert(sizeof(xgc_dmpc_scene_header_v1) == 144, "scene header");
  static_assert(offsetof(xgc_dmpc_scene_obstacle_v1, motion_type) == 224, "motion_type offset");
  static_assert(sizeof(xgc_dmpc_scene_obstacle_v1) == 240, "scene obstacle");
  static_assert(offsetof(xgc_dmpc_scene_part_v1, vertex_begin) == 56, "vertex_begin");
  static_assert(offsetof(xgc_dmpc_scene_part_v1, vertex_count) == 60, "vertex_count");
  static_assert(offsetof(xgc_dmpc_scene_part_v1, position) == 64, "part pose offset");
  static_assert(sizeof(xgc_dmpc_scene_part_v1) == 120, "scene part");
  static_assert(sizeof(xgc_dmpc_scene_vertex_v1) == 24, "scene vertex");
  static_assert(sizeof(xgc_dmpc_paired_state_v1) == 96, "paired");
  static_assert(sizeof(xgc_dmpc_controller_status_v1) == 56, "controller");
  static_assert(sizeof(xgc_position_target_v1) == 104, "position target");
  static_assert(sizeof(xgc_dmpc_timeline_ack_v1) == 56, "timeline ack");
  if (view == nullptr || state == nullptr || have_state == nullptr) return "scene decode output is missing";
  *have_state = false;
  *view = {};
  *state = {};
  std::string epoch;
  std::string frame;
  if (header.schema != 1 || !take_text(header.frame, sizeof header.frame, &frame) || frame != "world" ||
      !take_text(header.epoch, sizeof header.epoch, &epoch) ||
      (header.obstacle_count > 0 && obstacles == nullptr) || (header.part_count > 0 && parts == nullptr) ||
      (header.vertex_count > 0 && vertices == nullptr)) {
    return "scene snapshot frame or epoch is not usable";
  }
  view->epoch = epoch;
  view->frame_id = frame;
  view->revision = header.revision;
  view->definition.epoch = epoch;
  view->definition.frame_id = frame;
  view->definition.stamp_sec = header.stamp_sec;
  bool any_dynamic = false;
  for (uint32_t index = 0; index < header.obstacle_count; ++index) {
    formation_generator_dmpc::PlainSceneObstacle obstacle;
    const auto& body = obstacles[index];
    if (!take_text(body.id, sizeof body.id, &obstacle.id) ||
        !take_text(body.motion_type, sizeof body.motion_type, &obstacle.motion_type)) {
      return "scene obstacle id or motion_type does not fit";
    }
    obstacle.dynamic = body.dynamic != 0;
    if (obstacle.dynamic) any_dynamic = true;
    obstacle.pose.position = Eigen::Vector3d(body.position[0], body.position[1], body.position[2]);
    obstacle.pose.orientation = Eigen::Quaterniond(body.orientation_xyzw[3], body.orientation_xyzw[0],
                                                   body.orientation_xyzw[1], body.orientation_xyzw[2]);
    for (uint32_t part_index = 0; part_index < header.part_count; ++part_index) {
      if (parts[part_index].obstacle_index != index) continue;
      formation_generator_dmpc::PlainScenePart part;
      const auto& packed = parts[part_index];
      if (!take_text(packed.part_id, sizeof packed.part_id, &part.id)) return "scene part id does not fit";
      part.pose.position = Eigen::Vector3d(packed.position[0], packed.position[1], packed.position[2]);
      part.pose.orientation = Eigen::Quaterniond(packed.orientation_xyzw[3], packed.orientation_xyzw[0],
                                                 packed.orientation_xyzw[1], packed.orientation_xyzw[2]);
      const auto& param = packed.param;
      switch (packed.geometry_type) {
        case 0: part.type = "cylinder"; part.radius = param[0]; part.height = param[1]; break;
        case 1: part.type = "box"; part.size = Eigen::Vector3d(param[0], param[1], param[2]); break;
        case 2: part.type = "capsule"; part.radius = param[0]; part.height = param[1]; break;
        case 3: part.type = "sphere"; part.radius = param[0]; break;
        case 4:
          part.type = "convex";
          if (vertices == nullptr || packed.vertex_begin > header.vertex_count ||
              packed.vertex_count > header.vertex_count - packed.vertex_begin) {
            return "convex part vertices are missing";
          }
          for (uint32_t vertex = 0; vertex < packed.vertex_count; ++vertex) {
            const auto& xyz = vertices[packed.vertex_begin + vertex].xyz;
            part.vertices.emplace_back(xyz[0], xyz[1], xyz[2]);
          }
          break;
        default: return "scene part type is not usable";
      }
      obstacle.parts.push_back(std::move(part));
    }
    view->definition.obstacles.push_back(std::move(obstacle));
  }
  if (!any_dynamic) return {};
  state->epoch = view->epoch;
  state->frame_id = view->frame_id;
  state->revision = view->revision;
  state->stamp_sec = header.stamp_sec;
  for (uint32_t index = 0; index < header.obstacle_count; ++index) {
    formation_generator_dmpc::PlainSceneObstacleState item;
    if (!take_text(obstacles[index].id, sizeof obstacles[index].id, &item.id)) {
      return "scene obstacle id or motion_type does not fit";
    }
    item.pose.position = Eigen::Vector3d(obstacles[index].position[0], obstacles[index].position[1],
                                         obstacles[index].position[2]);
    item.pose.orientation = Eigen::Quaterniond(obstacles[index].orientation_xyzw[3],
                                               obstacles[index].orientation_xyzw[0],
                                               obstacles[index].orientation_xyzw[1],
                                               obstacles[index].orientation_xyzw[2]);
    item.twist.linear = Eigen::Vector3d(obstacles[index].linear[0], obstacles[index].linear[1],
                                        obstacles[index].linear[2]);
    item.twist.angular = Eigen::Vector3d(obstacles[index].angular[0], obstacles[index].angular[1],
                                         obstacles[index].angular[2]);
    state->obstacles.push_back(std::move(item));
  }
  *have_state = true;
  return {};
}

std::string PlanDmpc::push_scene_wire(const xgc_dmpc_scene_header_v1& header,
                                      const xgc_dmpc_scene_obstacle_v1* obstacles,
                                      const xgc_dmpc_scene_part_v1* parts,
                                      const xgc_dmpc_scene_vertex_v1* vertices) {
  std::string scene_id;
  if (header.schema != 1 || std::strcmp(header.frame, "world") != 0 ||
      !take_text(header.scene_id, sizeof header.scene_id, &scene_id) || scene_id != config_.scene_id ||
      (header.obstacle_count > 0 && obstacles == nullptr) || (header.part_count > 0 && parts == nullptr) ||
      (header.vertex_count > 0 && vertices == nullptr)) {
    geometry_ready_ = false;
    scene_epoch_.clear();
    return "scene snapshot identity or frame is not usable";
  }
  formation_generator_dmpc::PlainSceneDefinitionView view;
  formation_generator_dmpc::PlainSceneDynamicState state;
  bool have_state = false;
  const std::string decoded = decode_records(header, obstacles, parts, vertices, &view, &state, &have_state);
  if (!decoded.empty()) {
    geometry_ready_ = false;
    scene_epoch_.clear();
    return decoded;
  }
  return push_scene_definition(view, have_state ? &state : nullptr);
}

std::string PlanDmpc::push_heartbeat(const xgc_dmpc_scene_heartbeat_v1& beat) {
  if (!std::isfinite(beat.received_wall_sec)) return "scene heartbeat is not finite";
  heartbeat_wall_sec_ = beat.received_wall_sec;
  return {};
}

std::string PlanDmpc::push_commit(const xgc_dmpc_mission_commit_v1& commit, const uint8_t digest[32]) {
  static_assert(sizeof(xgc_dmpc_mission_timeline_v1) == 240, "timeline request size");
  static_assert(sizeof(xgc_dmpc_mission_commit_v1) == 256, "timeline commit size");
  static_assert(sizeof(xgc_dmpc_timeline_status_v1) == 160, "timeline status size");
  static_assert(sizeof(xgc_dmpc_mission_ack_v1) == 56, "timeline ack size");
  static_assert(offsetof(xgc_dmpc_mission_ack_v1, command_id) == 40, "ack command_id");
  static_assert(offsetof(xgc_dmpc_mission_ack_v1, accepted) == 48, "ack accepted");
  static_assert(offsetof(xgc_dmpc_mission_commit_v1, mission_ns) == 248, "mission_ns offset");
  const auto& request = commit.request;
  if (request.schema != 1 || request.kind < 1 || request.kind > 6 || request.revision == 0 ||
      request.command_id == 0 || request.session_id[0] == '\0' ||
      !canonical_text(request.session_id, sizeof(request.session_id)) ||
      !canonical_text(request.origin, sizeof(request.origin))) {
    return "timeline request fields are incomplete or not canonical";
  }
  if (request.kind != 5 && !(request.goal_xyz[0] == 0.0 && request.goal_xyz[1] == 0.0 &&
                             request.goal_xyz[2] == 0.0)) {
    return "unused goal bytes are not canonical zero";
  }
  if (request.kind == 5 && !finite3(request.goal_xyz)) return "goal is nonfinite";
  if (request.kind != 6 && request.pattern_id != 0) return "unused pattern id is not canonical zero";
  const int64_t period = std::llround(config_.sampling_time * 1e9);
  int64_t expected = request.anchor_ns;
  if (request.rolling == 1) {
    if (commit.round_k < request.effective_round) return "timeline effective round is still in the future";
    const uint64_t steps = commit.round_k - request.effective_round;
    if (steps > static_cast<uint64_t>(INT64_MAX / period)) return "timeline mission time overflow";
    const int64_t delta = static_cast<int64_t>(steps) * period;
    if (expected > INT64_MAX - delta) return "timeline mission time overflow";
    expected += delta;
  } else if (commit.mission_ns != request.anchor_ns) {
    return "held timeline mission time is not its anchor";
  }
  if (commit.mission_ns != expected) return "timeline mission time does not match the common formula";
  if (commit_ok_ && request.revision == commit_.request.revision &&
      std::memcmp(digest, commit_digest_, 32) == 0) {
    if (commit.round_k < commit_.round_k) return "timeline round went backwards";
    commit_ = commit;
    return {};
  }
  // Same revision, but the rounds owner has frozen this beat: rolling cleared
  // and mission_ns held at the anchor. This is not a new command.
  if (commit_ok_ && request.revision == commit_.request.revision &&
      commit_.request.rolling == 1 && request.rolling == 0 &&
      commit.mission_ns == request.anchor_ns) {
    commit_ = commit;
    std::memcpy(commit_digest_, digest, 32);
    return {};
  }
  if (commit_ok_ && request.revision == commit_.request.revision) return "timeline digest conflict";
  const uint64_t expected_predecessor = commit_ok_ ? commit_.request.revision : 0;
  if (request.predecessor_revision != expected_predecessor) return "timeline predecessor skipped";
  if (request.predecessor_revision != 0 && std::memcmp(request.predecessor_digest, commit_digest_, 32) != 0) {
    return "timeline predecessor digest mismatch";
  }
  if (request.rolling > 1) return "timeline rolling flag is not 0 or 1";
  xgc_dmpc_mission_timeline_v1 previous{};
  if (commit_ok_) previous = commit_.request;
  auto phase_at = [&](uint64_t round, int64_t* out) -> const char* {
    if (previous.rolling != 1 || round <= previous.effective_round) {
      *out = previous.anchor_ns;
      return nullptr;
    }
    const uint64_t steps = round - previous.effective_round;
    if (steps > static_cast<uint64_t>(INT64_MAX / period)) return "timeline mission time overflow";
    const int64_t delta = static_cast<int64_t>(steps) * period;
    if (previous.anchor_ns > INT64_MAX - delta) return "timeline mission time overflow";
    *out = previous.anchor_ns + delta;
    return nullptr;
  };
  if (request.kind == 1 || request.kind == 2) {
    if (previous.rolling != 0 || request.rolling != 1 || request.anchor_ns != previous.anchor_ns) {
      return "anchor does not continue the timeline";
    }
  } else if (request.kind == 4) {
    if (request.rolling != 0 || request.anchor_ns != 0) return "anchor does not continue the timeline";
  } else if (request.kind == 3 || request.kind == 5 || request.kind == 6) {
    if ((request.kind == 5 || request.kind == 6) &&
        (previous.rolling != 1 || request.rolling != 1 ||
         request.effective_round <= previous.effective_round)) {
      return "anchor does not continue the timeline";
    }
    if (request.kind == 3 && request.rolling != 0) return "anchor does not continue the timeline";
    int64_t projected = 0;
    if (const char* overflow = phase_at(request.effective_round, &projected)) return overflow;
    if (request.anchor_ns != projected) return "anchor does not continue the timeline";
  }
  commit_ = commit;
  std::memcpy(commit_digest_, digest, 32);
  commit_ok_ = true;
  return {};
}

StepTrace PlanDmpc::step(double trigger_sec, double now_sec, double now_wall_sec) {
  StepTrace out;
  out.status.qp_status = -1;
  out.status.qp_iterations = -1;
  out.status.stamp_sec = trigger_sec + config_.sampling_time;
  const bool receipt_valid = heartbeat_wall_sec_.has_value() && std::isfinite(now_wall_sec);
  const double age = receipt_valid ? now_wall_sec - *heartbeat_wall_sec_ : 1.0e9;
  const bool heartbeat_fresh = receipt_valid && age >= 0.0 && age <= kHeartbeatTimeoutSec;
  const char* reason = "";
  if (!state_ok_ || now_sec - state_stamp_sec_ > kStaleSec) reason = "paired state is not current";
  if (!heartbeat_fresh) reason = "scene heartbeat is not current";
  else if (formation_generator_dmpc::sceneHoldRequired(true, geometry_ready_, heartbeat_fresh, age,
                                                      kHeartbeatTimeoutSec)) {
    reason = "scene geometry is not loaded";
  }
  std::strncpy(out.status.reject_reason, reason, sizeof(out.status.reject_reason) - 1);
  if (commit_ok_) {
    out.timeline.committed_revision = commit_.request.revision;
    std::memcpy(out.timeline.committed_digest, commit_digest_, 32);
    out.timeline.effective_round = commit_.request.effective_round;
    out.timeline.mission_ns = commit_.mission_ns;
    out.timeline.applied_revision = applied_revision_;
    out.timeline.applied_mission_ns = applied_mission_ns_;
    // Commits are drained before a planner trigger. If more than one beat
    // arrived, the effective beat may already have passed; apply it once at
    // the current mission phase and report that actual application time.
    const bool due = commit_.round_k >= commit_.request.effective_round &&
                     applied_revision_ != commit_.request.revision;
    if (due && commit_.request.kind == 6) {
      try {
        if (!agent_->switchPattern(static_cast<uint8_t>(commit_.request.pattern_id))) {
          throw std::invalid_argument("pattern id is not in the loaded formation_patterns");
        }
        const auto pattern = formation_patterns::FormationPatternBase::getCurrentPattern();
        pattern_offset_ =
            pattern->computeOffset(static_cast<double>(commit_.mission_ns) * 1e-9, config_.self_id).position;
        applied_revision_ = commit_.request.revision;
        applied_mission_ns_ = commit_.mission_ns;
        out.timeline.applied_revision = applied_revision_;
        out.timeline.applied_mission_ns = applied_mission_ns_;
      } catch (const std::exception& error) {
        out.timeline.held = 1;
        out.timeline.fault = 1;
        std::strncpy(out.timeline.reason, error.what(), sizeof(out.timeline.reason) - 1);
      }
    } else if (due && commit_.request.kind == 5) {
      if (queued_revision_ != commit_.request.revision) {
        formation_generator_dmpc::QueuedGoal goal;
        goal.x = commit_.request.goal_xyz[0];
        goal.y = commit_.request.goal_xyz[1];
        replace_goal(goal);
        queued_revision_ = commit_.request.revision;
      }
    } else if (due) {
      applied_revision_ = commit_.request.revision;
      applied_mission_ns_ = commit_.mission_ns;
      out.timeline.applied_revision = applied_revision_;
      out.timeline.applied_mission_ns = applied_mission_ns_;
    }
  }
  const double mission_sec = commit_ok_ ? static_cast<double>(commit_.mission_ns) * 1e-9 : 0.0;
  run_round(trigger_sec, now_sec, mission_sec, out);
  if (agent_) std::strncpy(out.status.lifecycle, agent_->lifecycle().stateName(), sizeof(out.status.lifecycle) - 1);
  return out;
}

void PlanDmpc::replace_goal(const formation_generator_dmpc::QueuedGoal& goal) {
  while (!goals_.empty()) goals_.pop();
  goals_.enqueue(goal);
}

std::string PlanDmpc::decode_scene_blob(const uint8_t* bytes, size_t size,
                                      formation_generator_dmpc::PlainSceneDefinitionView* view,
                                      formation_generator_dmpc::PlainSceneDynamicState* state,
                                      bool* have_state) const {
  if (bytes == nullptr || size < sizeof(xgc_dmpc_scene_header_v1)) {
    return "scene snapshot is shorter than its header";
  }
  xgc_dmpc_scene_header_v1 header{};
  std::memcpy(&header, bytes, sizeof header);
  const size_t need = sizeof header + header.obstacle_count * sizeof(xgc_dmpc_scene_obstacle_v1) +
                      header.part_count * sizeof(xgc_dmpc_scene_part_v1) +
                      header.vertex_count * sizeof(xgc_dmpc_scene_vertex_v1);
  if (size != need) return "scene snapshot length does not match its header counts";
  const auto* obstacles = reinterpret_cast<const xgc_dmpc_scene_obstacle_v1*>(bytes + sizeof header);
  const auto* parts = reinterpret_cast<const xgc_dmpc_scene_part_v1*>(
      bytes + sizeof header + header.obstacle_count * sizeof(xgc_dmpc_scene_obstacle_v1));
  const auto* vertices = reinterpret_cast<const xgc_dmpc_scene_vertex_v1*>(
      bytes + sizeof header + header.obstacle_count * sizeof(xgc_dmpc_scene_obstacle_v1) +
      header.part_count * sizeof(xgc_dmpc_scene_part_v1));
  return decode_records(header, header.obstacle_count ? obstacles : nullptr, header.part_count ? parts : nullptr,
                        header.vertex_count ? vertices : nullptr, view, state, have_state);
}

std::string PlanDmpc::push_scene_blob(const uint8_t* bytes, size_t size) {
  if (bytes == nullptr || size < sizeof(xgc_dmpc_scene_header_v1)) {
    geometry_ready_ = false;
    scene_epoch_.clear();
    return "scene snapshot is shorter than its header";
  }
  xgc_dmpc_scene_header_v1 header{};
  std::memcpy(&header, bytes, sizeof header);
  const size_t need = sizeof header + header.obstacle_count * sizeof(xgc_dmpc_scene_obstacle_v1) +
                      header.part_count * sizeof(xgc_dmpc_scene_part_v1) +
                      header.vertex_count * sizeof(xgc_dmpc_scene_vertex_v1);
  if (size != need) {
    geometry_ready_ = false;
    scene_epoch_.clear();
    return "scene snapshot length does not match its header counts";
  }
  const auto* obstacles = reinterpret_cast<const xgc_dmpc_scene_obstacle_v1*>(bytes + sizeof header);
  const auto* parts = reinterpret_cast<const xgc_dmpc_scene_part_v1*>(
      bytes + sizeof header + header.obstacle_count * sizeof(xgc_dmpc_scene_obstacle_v1));
  const auto* vertices = reinterpret_cast<const xgc_dmpc_scene_vertex_v1*>(
      bytes + sizeof header + header.obstacle_count * sizeof(xgc_dmpc_scene_obstacle_v1) +
      header.part_count * sizeof(xgc_dmpc_scene_part_v1));
  return push_scene_wire(header, header.obstacle_count ? obstacles : nullptr,
                         header.part_count ? parts : nullptr, header.vertex_count ? vertices : nullptr);
}

std::string PlanDmpc::push_neighbor_plan(const uint8_t* bytes, size_t size) {
  formation_generator_dmpc::PlanMessage message;
  if (!formation_generator_dmpc::decodePlanPayload(bytes, size, message)) {
    return "neighbor plan payload is not an assumed trajectory";
  }
  if (message.uav_id == config_.self_id || message.uav_id < 1 || message.uav_id > config_.fleet_count) {
    return "neighbor identity is outside the loaded roster";
  }
  if (message.num_timesteps < static_cast<uint32_t>(config_.horizon + 1)) {
    return "neighbor trajectory is shorter than the loaded horizon";
  }
  if (!agent_->receiveNeighborPlan(message)) return "neighbor plan was not accepted";
  return {};
}

std::string PlanDmpc::push_neighbor_position(const xgc_dmpc_measured_position_v1& position) {
  if (position.uav_id == static_cast<uint32_t>(config_.self_id) || position.uav_id < 1 ||
      position.uav_id > static_cast<uint32_t>(config_.fleet_count) ||
      !std::isfinite(position.stamp_sec) || !finite3(position.position)) {
    return "neighbor position is not a finite roster measurement";
  }
  const auto found = neighbor_positions_.find(position.uav_id);
  if (found == neighbor_positions_.end() || position.stamp_sec > found->second.stamp_sec) {
    neighbor_positions_[position.uav_id] = position;
  }
  return {};
}

void PlanDmpc::run_round(double trigger_sec, double now_sec, double mission_sec, StepTrace& out) {
  if (!agent_ || !state_ok_ || now_sec - state_stamp_sec_ > kStaleSec) return;
  if (!geometry_ready_ || !heartbeat_wall_sec_.has_value() ||
      out.status.reject_reason[0] != '\0') {
    formation_generator_dmpc::DmpcTickOutput hold;
    double activation_time = trigger_sec;
    if (!formation_generator_dmpc::rosTimePlus(trigger_sec, config_.sampling_time, activation_time)) return;
    agent_->sceneHold(activation_time, hold);
    out.have_position_target = hold.has_setpoint;
    out.have_planar_target = hold.has_planar;
    std::memcpy(&out.position_target, &hold.setpoint, sizeof out.position_target);
    std::memcpy(&out.planar_target, &hold.planar, sizeof out.planar_target);
    return;
  }
  agent_->observeSceneHeartbeat(trigger_sec);
  formation_generator_dmpc::QueuedGoal pending;
  const auto take = goals_.peek(true, trigger_sec,
                                trigger_sec == 0.0, mission_sec, pending);
  goal_waiting_ = take != formation_generator_dmpc::GoalTake::Ready;
  formation_generator_dmpc::DmpcGoal goal;
  if (take == formation_generator_dmpc::GoalTake::Ready) {
    goal.x = pending.x;
    goal.y = pending.y;
    for (const auto& [id, position] : neighbor_positions_) {
      if (now_sec - position.stamp_sec <= kStaleSec) {
        goal.neighbor_positions[static_cast<uint8_t>(id)] =
            Eigen::Vector3d(position.position[0], position.position[1], position.position[2]);
      }
    }
  }
  const auto result = agent_->tick({commit_.round_k, mission_sec,
                                    commit_ok_ && commit_.request.rolling == 1, trigger_sec},
                                   take == formation_generator_dmpc::GoalTake::Ready ? &goal : nullptr);
  if (result.goal_applied) {
    goals_.pop();
    applied_revision_ = queued_revision_;
    applied_mission_ns_ = commit_.mission_ns;
    out.timeline.applied_revision = applied_revision_;
    out.timeline.applied_mission_ns = applied_mission_ns_;
    if (!result.goal_failure.empty()) {
      std::strncpy(out.status.reject_reason, result.goal_failure.c_str(), sizeof(out.status.reject_reason) - 1);
    }
  }
  out.status.solver_called = result.solver_called;
  out.status.solver_ok = result.solved;
  out.status.qp_status = result.qp_status;
  out.status.qp_iterations = result.qp_iterations;
  out.status.tracking = agent_->lifecycle().isRolling() && result.solved;
  if (result.solver_called && !result.solved) {
    std::strncpy(out.status.reject_reason, agent_->lastFailure().c_str(), sizeof(out.status.reject_reason) - 1);
  }
  if (result.has_setpoint) {
    std::memcpy(&out.position_target, &result.setpoint, sizeof out.position_target);
    out.have_position_target = true;
  }
  if (result.has_planar) {
    std::memcpy(&out.planar_target, &result.planar, sizeof out.planar_target);
    out.have_planar_target = true;
  }
  if (result.has_plan) out.own_plan = formation_generator_dmpc::encodePlanPayload(result.plan);
}

StepTrace PlanDmpc::pass_through_hold(double now_sec) {
  StepTrace out;
  if (!agent_) return out;
  const auto result = agent_->passThroughHold(now_sec);
  if (result.has_setpoint) {
    std::memcpy(&out.position_target, &result.setpoint, sizeof out.position_target);
    out.have_position_target = true;
  }
  return out;
}

}  // namespace xgc_plan_dmpc
