#include "plan_dmpc.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "formation_generator/core/dmpc_round.h"
#include "formation_generator/core/position_target_output.h"
#include "formation_generator/lifecycle/goal_apply.h"
#include "formation_generator/mpc/core/mpc_problem.h"
#include "formation_generator/mpc/dmpc/dmpc_algorithm_config.h"
#include "formation_generator/mpc/dmpc/dmpc_constraint_assembler.h"

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

PlanDmpc::PlanDmpc(const xgc_dmpc_planner_config_v1& config) : config_(config) {
  if (std::strcmp(config.algorithm, "legacy") != 0) {
    configure_error_ = "unsupported algorithm; Comprehensive admits legacy only";
  } else if (config.chain_n != 3 || config.state_dim != 9 || config.horizon != 40 ||
             config.sampling_time != 0.1 || config.fleet_count != 8 || config.self_id < 1 ||
             config.self_id > 8 || std::strcmp(config.scene_id, "dmpc-uav8_comprehensive") != 0) {
    configure_error_ = "unsupported dimensions; Comprehensive legacy is n3 state 9 N40 T.1 fleet 8";
  }
  lifecycle_.setHooks({}, {}, [this](std::string& fault) { return initialize_optimizer(fault); });
  lifecycle_.start();
  if (configure_error_.empty()) {
    formation_generator_dmpc::LeaderReferenceSpec spec;
    spec.leader_speed = 0.5;
    leader_ = formation_generator_dmpc::makeLeaderReference(spec, config_.sampling_time, config_.horizon + 1);
    leader_->initialize(Eigen::Vector3d(-15.0, -15.0, 2.0));
  }
}

int PlanDmpc::leader_rows() const {
  return leader_ ? static_cast<int>(leader_->getPredictedTrajectory().rows()) : 0;
}

int PlanDmpc::leader_cols() const {
  return leader_ ? static_cast<int>(leader_->getPredictedTrajectory().cols()) : 0;
}

std::string PlanDmpc::push_state(const xgc_dmpc_paired_state_v1& state, double now_sec) {
  if (!configure_error_.empty()) return configure_error_;
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
  return {};
}

std::string PlanDmpc::push_controller(const xgc_dmpc_controller_status_v1& status, double now_sec) {
  if (!configure_error_.empty()) return configure_error_;
  const std::string stamp = stamp_error(status.stamp_sec, now_sec, "controller");
  if (!stamp.empty()) {
    controller_ok_ = false;
    return stamp;
  }
  std::strncpy(controller_state_, status.state, sizeof(controller_state_) - 1);
  controller_state_[sizeof(controller_state_) - 1] = '\0';
  controller_ok_ = true;
  controller_stamp_sec_ = status.stamp_sec;
  return {};
}

std::string PlanDmpc::push_scene(const xgc_dmpc_scene_ids_v1& snapshot, const xgc_dmpc_scene_ids_v1& state) {
  if (!configure_error_.empty()) return configure_error_;
  scene_ids_match_ = false;
  if (std::strcmp(snapshot.scene_id, config_.scene_id) != 0 ||
      std::strcmp(state.scene_id, config_.scene_id) != 0 || snapshot.revision == 0 ||
      snapshot.revision != state.revision) {
    return "scene snapshot/state identity does not match";
  }
  // Identity match is not a loaded scene. ScenarioInput / SharedSceneAdapter
  // are not in the core yet, so no obstacle is installed.
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
  if (!configure_error_.empty()) return configure_error_;
  geometry_ready_ = false;
  scene_ids_match_ = false;
  scene_epoch_.clear();
  try {
    const formation_generator_dmpc::AdaptedScene adapted = scene_adapter_.convert(snapshot, state);
    formation_generator_dmpc::acceptScenePolicy(adapted, true, false, false);
    if (adapted.statics.size() != 77 || !adapted.movers.empty()) {
      return "comprehensive scene did not yield 77 held obstacles";
    }
    scene_.setGeometryTemplates(adapted.templates);
    scene_.updateStaticObstacles(adapted.statics, config_.self_id, 77);
    templates_ = adapted.templates;
    statics_ = adapted.statics;
  } catch (const std::exception& error) {
    return error.what();
  }
  if (scene_.staticObstacles().size() != 77) return "planner scene did not keep 77 obstacles";
  geometry_ready_ = true;
  scene_ids_match_ = true;
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
  static_assert(sizeof(xgc_dmpc_scene_header_v1) == 136, "scene header");
  static_assert(offsetof(xgc_dmpc_scene_obstacle_v1, motion_type) == 176, "motion_type offset");
  static_assert(sizeof(xgc_dmpc_scene_obstacle_v1) == 192, "scene obstacle");
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
      !take_text(header.epoch, sizeof header.epoch, &epoch) || header.part_count == 0 ||
      obstacles == nullptr || parts == nullptr || (header.vertex_count > 0 && vertices == nullptr)) {
    return "scene snapshot frame or epoch is not usable";
  }
  view->epoch = epoch;
  view->frame_id = frame;
  view->revision = header.revision;
  view->definition.epoch = epoch;
  view->definition.frame_id = frame;
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
  if (header.schema != 1 || std::strcmp(header.frame, "world") != 0 || header.scene_id[0] == '\0' ||
      header.obstacle_count != 77 || header.part_count == 0 || obstacles == nullptr || parts == nullptr ||
      (header.vertex_count > 0 && vertices == nullptr)) {
    geometry_ready_ = false;
    scene_epoch_.clear();
    return "comprehensive scene is not the 77-obstacle world document";
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
  if (!configure_error_.empty()) return configure_error_;
  if (!std::isfinite(beat.received_wall_sec)) return "scene heartbeat is not finite";
  heartbeat_wall_sec_ = beat.received_wall_sec;
  return {};
}

std::string PlanDmpc::push_commit(const xgc_dmpc_mission_commit_v1& commit, const uint8_t digest[32]) {
  if (!configure_error_.empty()) return configure_error_;
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
  if (request.kind == 6 && request.pattern_id != 0 && request.pattern_id != 4 &&
      request.pattern_id != 5 && request.pattern_id != 6) {
    return "unsupported formation pattern";
  }
  if (commit_ok_ && request.revision == commit_.request.revision &&
      std::memcmp(digest, commit_digest_, 32) == 0) {
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
  const int64_t period = 100000000;
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
  int64_t expected = request.anchor_ns;
  if (request.rolling == 1) {
    if (commit.round_k < request.effective_round) return "timeline effective round is still in the future";
    const uint64_t steps = commit.round_k - request.effective_round;
    if (steps > static_cast<uint64_t>(INT64_MAX / period)) return "timeline mission time overflow";
    expected += static_cast<int64_t>(steps) * period;
  } else if (commit.mission_ns != request.anchor_ns) {
    return "held timeline mission time is not its anchor";
  }
  if (commit.mission_ns != expected) return "timeline mission time does not match the common formula";
  commit_ = commit;
  std::memcpy(commit_digest_, digest, 32);
  commit_ok_ = true;
  return {};
}

std::string PlanDmpc::push_neighbor(uint32_t uav_id) {
  if (!configure_error_.empty()) return configure_error_;
  if (uav_id == static_cast<uint32_t>(config_.self_id) || uav_id < 1 || uav_id > 8) {
    return "neighbor identity is not another Comprehensive agent";
  }
  if (neighbors_.count(uav_id) != 0) return "duplicate neighbor plan";
  if (neighbors_.size() >= 7) return "neighbor cache is full";
  neighbors_.emplace(uav_id, true);
  return {};
}

void PlanDmpc::refresh(double now_sec, double now_wall_sec) {
  using formation_generator_dmpc::obstacleInfoReady;
  using formation_generator_dmpc::sceneHoldRequired;
  const bool receipt_valid = heartbeat_wall_sec_.has_value() && std::isfinite(now_wall_sec);
  const double age = receipt_valid ? now_wall_sec - *heartbeat_wall_sec_ : 1.0e9;
  const bool heartbeat_fresh = receipt_valid && age >= 0.0 && age <= kHeartbeatTimeoutSec;
  formation_generator_dmpc::PlannerLifecycleFacts facts;
  facts.obstacle_info_ready = obstacleInfoReady(false, false, true, geometry_ready_, geometry_ready_,
                                                heartbeat_fresh && scene_ids_match_);
  facts.first_state_received = state_ok_;
  facts.vehicle_control_state = controller_ok_ ? controller_state_ : "";
  facts.vehicle_stamp_valid = controller_ok_;
  facts.vehicle_state_age_sec =
      controller_ok_ && std::isfinite(now_sec) ? now_sec - controller_stamp_sec_ : 1.0e9;
  facts.neighbors_ready = neighbors_.size() == 7;
  facts.last_published_position_valid = false;
  facts.swarm_mission_clock = false;
  lifecycle_.setFacts(facts);
  (void)sceneHoldRequired(true, geometry_ready_, heartbeat_fresh, age, kHeartbeatTimeoutSec);
}

StepTrace PlanDmpc::step(double trigger_sec, double now_sec, double now_wall_sec) {
  StepTrace out;
  out.status.qp_status = -1;
  out.status.qp_iterations = -1;
  out.status.stamp_sec = trigger_sec + config_.sampling_time;
  if (!configure_error_.empty()) {
    std::strncpy(out.status.reject_reason, configure_error_.c_str(), sizeof(out.status.reject_reason) - 1);
  } else {
    refresh(now_sec, now_wall_sec);
    lifecycle_.tick(now_sec);
    const bool receipt_valid = heartbeat_wall_sec_.has_value() && std::isfinite(now_wall_sec);
    const double age = receipt_valid ? now_wall_sec - *heartbeat_wall_sec_ : 1.0e9;
    const bool heartbeat_fresh = receipt_valid && age >= 0.0 && age <= kHeartbeatTimeoutSec;
    const char* reason = optimizer_ ? "" : "legacy optimizer is not initialized";
    if (!heartbeat_fresh) reason = "scene heartbeat is not current";
    else if (formation_generator_dmpc::sceneHoldRequired(true, geometry_ready_, heartbeat_fresh, age,
                                                        kHeartbeatTimeoutSec)) {
      reason = "scene geometry is not loaded";
    }
    if (leader_ && commit_ok_) leader_->advanceTo(static_cast<double>(commit_.mission_ns) * 1e-9);
    std::strncpy(out.status.reject_reason, reason, sizeof(out.status.reject_reason) - 1);
  }
  std::strncpy(out.status.lifecycle, lifecycle_.stateName(), sizeof(out.status.lifecycle) - 1);
  if (commit_ok_) {
    out.timeline.committed_revision = commit_.request.revision;
    std::memcpy(out.timeline.committed_digest, commit_digest_, 32);
    out.timeline.effective_round = commit_.request.effective_round;
    out.timeline.mission_ns = commit_.mission_ns;
    out.timeline.applied_revision = applied_revision_;
    out.timeline.applied_mission_ns = applied_mission_ns_;
    const bool due = commit_.round_k == commit_.request.effective_round &&
                     applied_revision_ != commit_.request.revision;
    if (due && commit_.request.kind == 6) {
      try {
        pattern_ = formation_generator_dmpc::makeComprehensivePattern(
            static_cast<int>(commit_.request.pattern_id), config_.fleet_count);
        pattern_offset_ = pattern_->computeOffset(static_cast<double>(commit_.mission_ns) * 1e-9, config_.self_id).position;
        formation_patterns::FormationPatternBase::initialize(config_.fleet_count);
        formation_patterns::FormationPatternBase::registerPattern(
            static_cast<int>(commit_.request.pattern_id), pattern_);
        formation_patterns::FormationPatternBase::switchPattern(
            static_cast<int>(commit_.request.pattern_id), {});
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
  run_round(trigger_sec, mission_sec, out);
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
  const std::string identity = push_neighbor(message.uav_id);
  if (!identity.empty() && identity != "duplicate neighbor plan") return identity;
  StateTrajectory trajectory;
  if (!formation_generator_dmpc::assumedTrajectoryToStateTrajectory(message, config_.state_dim,
                                                                   config_.sampling_time, trajectory)) {
    return formation_generator_dmpc::describe(
        formation_generator_dmpc::checkAssumedTrajectory(message, config_.state_dim));
  }
  neighbor_plans_[message.uav_id] = std::move(trajectory);
  return {};
}

bool PlanDmpc::initialize_optimizer(std::string& fault) {
  if (!geometry_ready_ || statics_.size() != 77) {
    fault = "scene geometry is not loaded";
    return false;
  }
  try {
    constexpr int kNeighbors = 7;
    const int intermediate = kBoundaryConstraintCount + static_cast<int>(statics_.size()) + kNeighbors;
    DmpcAlgorithmConfig algorithm;
    algorithm.num_static_obstacles = static_cast<int>(statics_.size());
    algorithm.num_constraint_neighbors = kNeighbors;
    algorithm.distributed_reference_weight = 0.4;
    algorithm.boundary_x_min = -20.0;
    algorithm.boundary_x_max = 20.0;
    algorithm.boundary_y_min = -20.0;
    algorithm.boundary_y_max = 20.0;
    algorithm.boundary_z_min = 0.0;
    algorithm.boundary_z_max = 6.0;
    algorithm.vo_horizon = 10.0;
    algorithm.gjk_min_distance = 0.0001;
    algorithm.constraint_min_distance = 0.02;
    algorithm.gjk_tolerance = 0.01;
    algorithm.gjk_max_iterations = 30;
    algorithm.qp_solver_iter_max = 50;
    algorithm.control_period_ms = 100.0;
    algorithm.separation_query_mode = SeparationQueryMode::kGuidedAxisGjk;
    MPCProblem problem;
    problem.setHorizon(config_.horizon)
        .setSamplingTime(config_.sampling_time)
        .setBoxConstraintsEnabled(true)
        .setGeneralConstraintDimensions(intermediate, 2 * intermediate)
        .setWeights(4.0, 1.0, 3.0, 3.0, 2.0, 5.0, 1.0, 2.0, 1.0, 3.0, 3.0, 100.0)
        .setPositionConstraints(-20.0, 20.0, -0.5, 10.0, 10.0, 0.0)
        .setVelocityConstraints(2.0, 5.0, 0.0)
        .setAccelerationConstraints(4.0, 3.0, 2.0, 0.0)
        .initialize();
    optimizer_ = std::make_unique<AcadosDmpcOptimizer>(std::move(problem), algorithm, config_.self_id);
    optimizer_->configureConvexFeasibleRegionSnapshots(false, {});
    optimizer_->setGeometryTemplates(templates_);
    convex_geometry::BodyInstance self_body;
    self_body.id = 0;
    self_body.name = "default_uav_geometry";
    self_body.geometry_type = "sphere";
    self_body.pose.orientation.w = 1.0;
    self_body.scale.x = self_body.scale.y = self_body.scale.z = 0.15;
    self_body.is_static = true;
    optimizer_->setUavGeometries(self_body, {});
    optimizer_->updateStaticObstacles(statics_);
  } catch (const std::exception& error) {
    optimizer_.reset();
    fault = error.what();
    return false;
  }
  return true;
}

void PlanDmpc::run_round(double trigger_sec, double mission_sec, StepTrace& out) {
  using formation_generator_dmpc::GoalApplyRequest;
  using formation_generator_dmpc::GoalBootstrapInitializer;
  using formation_generator_dmpc::GoalSeedStatus;
  using formation_generator_dmpc::GoalTake;
  const bool hold = std::strstr(out.status.reject_reason, "scene geometry") != nullptr ||
                    std::strstr(out.status.reject_reason, "heartbeat") != nullptr;
  auto publish_hold = [&]() {
    if (!state_ok_) return;
    formation_generator_dmpc::PositionTargetPayload payload;
    formation_generator_dmpc::writeSceneHoldPositionTarget(
        trigger_sec + config_.sampling_time, measured_.head<3>(), 0.0, payload);
    static_assert(sizeof payload == sizeof(out.position_target), "position target layout");
    std::memcpy(&out.position_target, &payload, sizeof payload);
    out.have_position_target = true;
    out.status.tracking = 0;
  };
  if (!optimizer_ || !leader_ || hold) {
    if (hold) publish_hold();
    return;
  }
  formation_generator_dmpc::QueuedGoal pending;
  const GoalTake take =
      goals_.peek(true, trigger_sec, trigger_sec == 0.0, mission_sec, pending);
  goal_waiting_ = take != GoalTake::Ready;
  if (take == GoalTake::Ready) {
    leader_->advanceTo(mission_sec);
    GoalApplyRequest request;
    request.measured_state = measured_;
    request.takeoff_altitude = 2.0;
    request.local_takeoff_altitude = 1.0;
    request.optimizing = lifecycle_.isRolling();
    request.goal_x = pending.x;
    request.goal_y = pending.y;
    auto preview = formation_generator_dmpc::previewAppliedGoal(*leader_, request);
    leader_ = std::move(preview.trajectory);
    relative_state_ = preview.relative_state;
    leader_->getPredictedTrajectory();
    optimizer_->setLeaderReferenceTrajectory(leader_->getPredictedTrajectory());
    formation_generator_dmpc::GoalBootstrapInput seed;
    seed.uav_id = config_.self_id;
    seed.num_uavs = config_.fleet_count;
    seed.now_sec = mission_sec;
    seed.acceleration = acceleration_;
    seed.mpc_params.horizon = config_.horizon;
    seed.mpc_params.sampling_time = config_.sampling_time;
    seed.mpc_params.separation_query_mode = "guided_axis_gjk";
    seed.mpc_params.gjk_min_distance = 0.0001;
    seed.mpc_params.constraint_min_distance = 0.02;
    seed.mpc_params.gjk_tolerance = 0.01;
    seed.mpc_params.gjk_max_iterations = 30;
    seed.geometry_templates = templates_;
    seed.static_bodies_available = true;
    seed.static_obstacles = statics_;
    seed.num_static_obstacles = static_cast<int>(statics_.size());
    convex_geometry::BodyInstance self_body;
    self_body.geometry_type = "sphere";
    self_body.scale.x = self_body.scale.y = self_body.scale.z = 0.15;
    self_body.pose.orientation.w = 1.0;
    seed.default_uav_geometry = self_body;
    const auto bootstrap = GoalBootstrapInitializer().build(seed, preview.relative_state, preview.leader_position);
    const auto committed = formation_generator_dmpc::commitGoalSeed(*optimizer_, bootstrap);
    goals_.pop();
    if (committed.status != GoalSeedStatus::Seeded && !committed.reason.empty()) {
      std::strncpy(out.status.reject_reason, committed.reason.c_str(), sizeof(out.status.reject_reason) - 1);
    }
  }
  if (lifecycle_.isRolling()) leader_->advanceTo(mission_sec);
  formation_generator_dmpc::DmpcRoundPreparation round;
  round.mission_time = mission_sec;
  round.advance_reference_clock = lifecycle_.isRolling();
  round.leader_trajectory = &leader_->getPredictedTrajectory();
  round.roll_local_state = lifecycle_.isRolling();
  round.uav_id = config_.self_id;
  round.num_uavs = config_.fleet_count;
  formation_generator_dmpc::prepareDmpcRound(*optimizer_, round);
  std::vector<StateTrajectory> neighbors;
  neighbors.reserve(neighbor_plans_.size());
  for (const auto& item : neighbor_plans_) neighbors.push_back(item.second);
  out.status.solver_called = 1;
  const bool solved = formation_generator_dmpc::solveDmpcRound(
      *optimizer_, relative_state_, acceleration_, std::move(neighbors), mission_sec);
  const auto diagnostics = optimizer_->getLastSolverDiagnostics();
  out.status.qp_status = diagnostics.qp_status;
  out.status.qp_iterations = diagnostics.qp_iterations;
  out.status.solver_ok = solved ? 1 : 0;
  if (!solved) {
    const std::string& reason = optimizer_->getLastFailureReason();
    if (!reason.empty()) {
      std::strncpy(out.status.reject_reason, reason.c_str(), sizeof(out.status.reject_reason) - 1);
    }
    return;
  }
  bool states_ok = false;
  bool controls_ok = false;
  const auto& states = optimizer_->getOptimalStates(states_ok);
  const auto& controls = optimizer_->getOptimalControls(controls_ok);
  formation_generator_dmpc::PositionTargetPayload payload;
  if (formation_generator_dmpc::writeCommandPositionTarget(
          states, states_ok, controls, controls_ok, true, config_.chain_n, leader_->getCurrentPosition(),
          leader_->getCurrentVelocity(), leader_->getPredictedTrajectory(), lifecycle_.isRolling(),
          trigger_sec + config_.sampling_time, 0.0, payload)) {
    std::memcpy(&out.position_target, &payload, sizeof payload);
    out.have_position_target = true;
    out.status.tracking = lifecycle_.isRolling() ? 1 : 0;
  }
  bool predicted_ok = false;
  const auto& predicted = optimizer_->getPredictedStates(predicted_ok);
  formation_generator_dmpc::PlanMessage plan;
  formation_generator_dmpc::planFromPredictedStates(predicted, config_.horizon, predicted_ok, plan);
  plan.stamp = trigger_sec + config_.sampling_time;
  plan.uav_id = static_cast<uint8_t>(config_.self_id);
  if (predicted_ok) out.own_plan = formation_generator_dmpc::encodePlanPayload(plan);
  if (lifecycle_.isRolling()) {
    formation_generator_dmpc::advanceDmpcRound(*optimizer_, relative_state_, acceleration_);
  }
}

}  // namespace xgc_plan_dmpc
