#pragma once
// One SceneSnapshot+SceneState sample on scene_snapshot.
// The wire record appends epoch, motion_type, and part-local pose onto the
// existing prefix: header 144, obstacle 240, part 120, vertex 24.
#include "xgc_dmpc_planner_v1.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

inline constexpr std::size_t kSceneWireHeaderBytes = 144;
inline constexpr std::size_t kSceneWireEpochOffset = 72;
inline constexpr std::size_t kSceneWireEpochBytes = 64;
inline constexpr std::size_t kSceneWireObstacleBytes = 240;
inline constexpr std::size_t kSceneWireMotionOffset = 224;
inline constexpr std::size_t kSceneWireMotionBytes = 16;
inline constexpr std::size_t kSceneWirePartBytes = 120;
inline constexpr std::size_t kSceneWirePartPoseOffset = 64;
inline constexpr std::size_t kSceneWirePartPoseDoubles = 7;
inline constexpr std::size_t kSceneWireVertexBytes = 24;

#include <xgc2_geometry_msgs/SceneSnapshot.h>
#include <xgc2_geometry_msgs/SceneState.h>

bool xgc_dmpc_pack_scene_blob(const xgc2_geometry_msgs::SceneSnapshot& snapshot,
                              const xgc2_geometry_msgs::SceneState& state, std::vector<uint8_t>* blob,
                              std::string* error);
