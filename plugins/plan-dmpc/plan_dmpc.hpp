#pragma once

#include "xgc_dmpc_planner_v1.h"
#include "xgc_schemas_v1.h"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "formation_generator/core/plan_wire.h"
#include "formation_generator/dmpc_scheduler/dmpc_configuration.h"
#include "formation_generator/dmpc_scheduler/pattern_manager.h"
#include "formation_generator/lifecycle/goal_queue.h"
#include "formation_generator/lifecycle/leader_reference_factory.h"
#include "formation_generator/lifecycle/plain_scene_adapter.h"
#include "formation_generator/lifecycle/planner_lifecycle.h"
#include "formation_generator/lifecycle/scene_admission.h"
#include "formation_generator/mpc/idmpc_optimizer.h"
#include "formation_generator/scene/dmpc_scene.h"
#include "reference_trajectory/leader_reference.h"

namespace xgc_plan_dmpc {

struct StepTrace {
  xgc_dmpc_planner_status_v1 status{};
  xgc_dmpc_timeline_status_v1 timeline{};
  bool have_position_target = false;
  xgc_position_target_v1 position_target{};
  std::vector<uint8_t> own_plan;
};

// Flat configure text names an existing ParamManifest. Dimensions in
// xgc_dmpc_planner_config_v1 are copied from that load. The struct layout
// is unchanged.
struct PlanDmpcOpen {
  std::string manifest_path;
  int self_id = 0;
  uint16_t timeline_authority = 0;
  std::string scene_id;
  std::optional<std::string> algorithm;
  std::optional<int> chain_n;
  std::optional<int> state_dim;
  std::optional<int> horizon;
  std::optional<int> fleet_count;
  std::optional<double> sampling_time;
};

// Runtime entry over the installed core. Configuration, patterns and the
// leader reference come from the existing loader and factories.
class PlanDmpc {
 public:
  static std::unique_ptr<PlanDmpc> open(const PlanDmpcOpen& request, std::string* error);

  const xgc_dmpc_planner_config_v1& config() const { return config_; }
  const formation_generator_dmpc::DmpcConfiguration& loaded() const { return loaded_; }
  std::string configure_error() const { return configure_error_; }
  std::string push_state(const xgc_dmpc_paired_state_v1& state, double now_sec);
  std::string push_controller(const xgc_dmpc_controller_status_v1& status, double now_sec);
  std::string push_scene(const xgc_dmpc_scene_ids_v1& snapshot, const xgc_dmpc_scene_ids_v1& state);
  std::string push_scene_definition(const formation_generator_dmpc::PlainSceneDefinitionView& snapshot,
                                    const formation_generator_dmpc::PlainSceneDynamicState* state);
  std::string push_scene_wire(const xgc_dmpc_scene_header_v1& header,
                              const xgc_dmpc_scene_obstacle_v1* obstacles,
                              const xgc_dmpc_scene_part_v1* parts,
                              const xgc_dmpc_scene_vertex_v1* vertices);
  std::string push_scene_blob(const uint8_t* bytes, size_t size);
  std::string decode_scene_blob(const uint8_t* bytes, size_t size,
                                formation_generator_dmpc::PlainSceneDefinitionView* view,
                                formation_generator_dmpc::PlainSceneDynamicState* state,
                                bool* have_state) const;
  std::string push_neighbor_plan(const uint8_t* bytes, size_t size);
  std::string push_neighbor_position(const xgc_dmpc_measured_position_v1& position);
  size_t static_obstacle_count() const { return scene_.staticObstacles().size(); }
  const std::vector<convex_geometry::BodyInstance>& static_bodies() const { return statics_; }
  const std::string& scene_epoch() const { return scene_epoch_; }
  int leader_rows() const;
  int leader_cols() const;
  Eigen::Vector3d pattern_offset() const { return pattern_offset_; }
  bool goal_waiting() const { return goal_waiting_; }
  std::string push_heartbeat(const xgc_dmpc_scene_heartbeat_v1& beat);
  xgc_dmpc_mission_ack_v1 acknowledge(const xgc_dmpc_mission_timeline_v1& request,
                                      const uint8_t digest[32], uint16_t envelope_origin) const;
  std::string push_commit(const xgc_dmpc_mission_commit_v1& commit, const uint8_t digest[32]);
  StepTrace step(double trigger_sec, double now_sec, double now_wall_sec);

 private:
  PlanDmpc(const xgc_dmpc_planner_config_v1& config,
           formation_generator_dmpc::DmpcConfiguration loaded);
  void refresh(double now_sec, double now_wall_sec);
  bool initialize_optimizer(std::string& fault);
  std::string decode_records(const xgc_dmpc_scene_header_v1& header,
                             const xgc_dmpc_scene_obstacle_v1* obstacles,
                             const xgc_dmpc_scene_part_v1* parts,
                             const xgc_dmpc_scene_vertex_v1* vertices,
                             formation_generator_dmpc::PlainSceneDefinitionView* view,
                             formation_generator_dmpc::PlainSceneDynamicState* state,
                             bool* have_state) const;
  void replace_goal(const formation_generator_dmpc::QueuedGoal& goal);
  bool neighbors_ready(double now_sec) const;
  void write_own_plan(double valid_sec, StepTrace& out) const;
  void run_round(double trigger_sec, double now_sec, double mission_sec, StepTrace& out);
  formation_generator_dmpc::PlannerLifecycle lifecycle_;
  xgc_dmpc_planner_config_v1 config_{};
  formation_generator_dmpc::DmpcConfiguration loaded_{};
  std::unique_ptr<formation_generator_dmpc::PatternManager> patterns_;
  std::string configure_error_;
  bool state_ok_ = false;
  double state_stamp_sec_ = 0.0;
  bool controller_ok_ = false;
  double controller_stamp_sec_ = 0.0;
  char controller_state_[48] = {};
  bool scene_ids_match_ = false;
  bool geometry_ready_ = false;
  std::string scene_epoch_;
  Eigen::Matrix<double, 6, 1> measured_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> relative_state_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();
  std::map<std::string, convex_geometry::BodyTemplate> templates_;
  std::vector<convex_geometry::BodyInstance> statics_;
  std::map<uint32_t, StateTrajectory> neighbor_plans_;
  std::map<uint32_t, xgc_dmpc_measured_position_v1> neighbor_positions_;
  std::unique_ptr<IDmpcOptimizer> optimizer_;
  DmpcScene scene_;
  formation_generator_dmpc::PlainSceneAdapter scene_adapter_;
  formation_generator_dmpc::GoalQueue goals_;
  std::unique_ptr<reference_trajectory::LeaderReference> leader_;
  std::shared_ptr<formation_patterns::FormationPatternBase> pattern_;
  Eigen::Vector3d pattern_offset_ = Eigen::Vector3d::Zero();
  bool goal_waiting_ = false;
  std::optional<double> heartbeat_wall_sec_;
  xgc_dmpc_mission_commit_v1 commit_{};
  uint8_t commit_digest_[32] = {};
  bool commit_ok_ = false;
  uint64_t applied_revision_ = 0;
  int64_t applied_mission_ns_ = 0;
  uint64_t queued_revision_ = 0;
};

}  // namespace xgc_plan_dmpc
