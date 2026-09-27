#include "plan_dmpc.hpp"

#include <cmath>
#include <map>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <dlfcn.h>

#include <yaml-cpp/yaml.h>

#include "xgc_rt.h"

namespace {
std::string formation_root;
std::string testdata(const std::string& name) { return std::string(PLAN_DMPC_TESTDATA) + "/" + name; }
std::string scenario_path(const std::string& relative) { return formation_root + "/" + relative; }

std::string comprehensive_manifest() {
  const std::string path = "/tmp/plan-dmpc-comprehensive-manifest.yaml";
  std::ofstream out(path);
  out << "namespace: /uav1/mpc\n"
      << "node_namespace: /uav1\n"
      << "loads:\n"
      << "  - {file: " << scenario_path("config/scenarios/dmpc_comprehensive/scene/scenario.yaml") << "}\n"
      << "  - {file: " << scenario_path("config/scenarios/dmpc_comprehensive/scene/formation_patterns.yaml")
      << ", param: formation_patterns}\n"
      << "params:\n  uav_id: 1\n  num_uavs: 8\n";
  out.close();
  return path;
}

xgc_plan_dmpc::PlanDmpcOpen comprehensive_request() {
  xgc_plan_dmpc::PlanDmpcOpen request;
  request.manifest_path = comprehensive_manifest();
  request.self_id = 1;
  request.timeline_authority = 4;
  request.scene_id = "dmpc-uav8_comprehensive";
  return request;
}

bool near(double got, double expect) { return std::isfinite(got) && std::fabs(got - expect) <= 1e-9; }

bool put_text(char* dest, size_t cap, const std::string& src);

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

int compare_comprehensive(const xgc_plan_dmpc::PlanDmpc& planner) {
  const YAML::Node scenario = YAML::LoadFile(
      scenario_path("config/scenarios/dmpc_comprehensive/scene/scenario.yaml"));
  const auto& loaded = planner.loaded();
  const auto& mpc = loaded.mpc_params;
  struct Item {
    const char* name;
    double previous;
    double after;
    double yaml;
  };
  const Item items[] = {
      {"horizon", 40, static_cast<double>(mpc.horizon), scenario["horizon"].as<double>()},
      {"sampling_time", 0.1, mpc.sampling_time, scenario["sampling_time"].as<double>()},
      {"fleet_count", 8, static_cast<double>(planner.config().fleet_count), 8},
      {"chain_n", 3, static_cast<double>(planner.config().chain_n), 3},
      {"state_dim", 9, static_cast<double>(planner.config().state_dim), 9},
      {"q_pos", 4, mpc.q_pos, scenario["q_pos"].as<double>()},
      {"q_vel", 1, mpc.q_vel, scenario["q_vel"].as<double>()},
      {"r_acc_xy", 3, mpc.r_acc_xy, scenario["r_acc_xy"].as<double>()},
      {"r_acc_z", 3, mpc.r_acc_z, scenario["r_acc_z"].as<double>()},
      {"w_jerk_xy", 2, mpc.w_jerk_xy, scenario["w_jerk_xy"].as<double>()},
      {"w_jerk_z", 5, mpc.w_jerk_z, scenario["w_jerk_z"].as<double>()},
      {"omega_1", 0.4, mpc.omega_1, scenario["omega_1"].as<double>()},
      {"base_jerk", 100, mpc.base_jerk, scenario["base_jerk"].as<double>()},
      {"fence_x_min", -20, mpc.boundary_x_min, scenario["fence_x_min"].as<double>()},
      {"fence_z_max", 6, mpc.boundary_z_max, scenario["fence_z_max"].as<double>()},
      {"box_pos_z_min", -0.5, mpc.pos_z_min, scenario["box_pos_z_min"].as<double>()},
      {"vo_horizon", 10, mpc.vo_horizon, scenario["vo_horizon"].as<double>()},
      {"leader_speed", 0.5, loaded.params.param("leader_speed", 0.0), scenario["leader_speed"].as<double>()},
      {"takeoff_altitude", 2, loaded.takeoff_altitude, scenario["takeoff_altitude"].as<double>()},
      {"local_takeoff_altitude", 1, loaded.local_takeoff_altitude, scenario["takeoff_altitude"].as<double>()},
  };
  for (const auto& item : items) {
    std::cout << item.name << " previous=" << item.previous << " loaded=" << item.after
              << " scenario=" << item.yaml << '\n';
    if (!near(item.after, item.yaml)) return fail(item.name);
    if (std::strcmp(item.name, "local_takeoff_altitude") == 0) {
      if (near(item.after, item.previous)) return fail("local takeoff stayed at the old literal 1.0");
    } else if (!near(item.after, item.previous)) {
      return fail(item.name);
    }
  }
  if (loaded.default_uav_geometry.scale.x != 0.15) return fail("default geometry scale drifted");
  std::cout << "local_takeoff_altitude now follows scenario takeoff_altitude 2.0; the plugin used to set 1.0\n";
  return 0;
}

int admit_two_static_scene() {
  xgc_plan_dmpc::PlanDmpcOpen request;
  request.manifest_path = testdata("two_static/manifest.yaml");
  request.self_id = 1;
  request.timeline_authority = 4;
  request.scene_id = "two-static";
  std::string error;
  auto opened = xgc_plan_dmpc::PlanDmpc::open(request, &error);
  if (!opened) return fail(error.c_str());
  if (opened->config().horizon != 20 || opened->config().fleet_count != 3 ||
      opened->loaded().mpc_params.q_pos != 6.0 || opened->leader_cols() != 21) {
    return fail("two-static manifest did not supply its own horizon, fleet, or weight");
  }
  xgc_dmpc_scene_header_v1 header{};
  header.schema = 1;
  header.obstacle_count = 2;
  header.part_count = 2;
  header.revision = 1;
  if (!put_text(header.scene_id, sizeof header.scene_id, "two-static") ||
      !put_text(header.frame, sizeof header.frame, "world") ||
      !put_text(header.epoch, sizeof header.epoch, "6db3ae1c-97b9-4019-800b-3b21ca818b99")) {
    return fail("two-static header does not fit");
  }
  std::vector<xgc_dmpc_scene_obstacle_v1> obstacles(2);
  std::vector<xgc_dmpc_scene_part_v1> parts(2);
  for (int index = 0; index < 2; ++index) {
    if (!put_text(obstacles[index].id, sizeof obstacles[index].id, index == 0 ? "a" : "b") ||
        !put_text(obstacles[index].name, sizeof obstacles[index].name, "post") ||
        !put_text(obstacles[index].motion_type, sizeof obstacles[index].motion_type, "hold") ||
        !put_text(parts[index].part_id, sizeof parts[index].part_id, "body")) {
      return fail("two-static text does not fit");
    }
    obstacles[index].position[0] = index;
    obstacles[index].orientation_xyzw[3] = 1.0;
    parts[index].obstacle_index = static_cast<uint32_t>(index);
    parts[index].geometry_type = 0;
    parts[index].param[0] = 0.2;
    parts[index].param[1] = 1.0;
    parts[index].orientation_xyzw[3] = 1.0;
  }
  const std::string loaded = opened->push_scene_wire(header, obstacles.data(), parts.data(), nullptr);
  if (!loaded.empty() || opened->static_obstacle_count() != 2) {
    std::cerr << loaded << '\n';
    return fail("two-static scene was not admitted");
  }
  std::cout << "two-static obstacles=" << opened->static_obstacle_count()
            << " horizon=" << opened->config().horizon << " fleet=" << opened->config().fleet_count << '\n';
  return 0;
}

// Exercise the actual plugin ABI: measured peer telemetry follows planner
// triggers rather than the host's 1 kHz step frequency.
int measured_position_rate(const xgc_plugin_descriptor* descriptor) {
  struct Host {
    std::map<uint32_t, std::vector<uint8_t>> pending;
    uint32_t paired = 0, trigger = 0, output = 0;
    unsigned sent = 0;
    xgc_dmpc_measured_position_v1 last{};
  } host;
  for (uint32_t i = 0; i < descriptor->port_count; ++i) {
    const std::string name = descriptor->ports[i].name;
    if (name == "paired_state") host.paired = i;
    if (name == "sync_trigger") host.trigger = i;
    if (name == "own_position") host.output = i;
  }
  xgc_host_api api{};
  api.abi_version = XGC_RT_ABI_VERSION;
  api.abi_minor = XGC_RT_ABI_MINOR;
  api.host = &host;
  api.next = [](void* raw, uint32_t port, xgc_sample_view* view) {
    auto& h = *static_cast<Host*>(raw);
    const auto found = h.pending.find(port);
    if (found == h.pending.end() || found->second.empty()) return XGC_ERR_AGAIN;
    static thread_local std::vector<uint8_t> sample;
    sample.swap(found->second);
    h.pending.erase(found);
    *view = {};
    view->data = sample.data();
    view->len = static_cast<uint32_t>(sample.size());
    return XGC_OK;
  };
  api.publish = [](void* raw, uint32_t port, uint64_t, const uint8_t* data, uint32_t len) {
    auto& h = *static_cast<Host*>(raw);
    if (port == h.output) {
      if (len != sizeof h.last) return XGC_ERR;
      std::memcpy(&h.last, data, len);
      ++h.sent;
    }
    return XGC_OK;
  };
  const auto* v = descriptor->vtbl;
  std::unique_ptr<void, void (*)(void*)> plugin(v->create(&api), v->destroy);
  const std::string config = "self_id = 1\ntimeline_authority = 4\nscene_id = \"dmpc-uav8_comprehensive\"\nmanifest = \"" + comprehensive_manifest() + "\"\n";
  if (v->configure(plugin.get(), config.c_str()) != XGC_OK || v->activate(plugin.get()) != XGC_OK)
    return fail("measurement plugin configuration");
  auto feed = [&](uint32_t port, const auto& sample) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(&sample);
    host.pending[port] = std::vector<uint8_t>(bytes, bytes + sizeof sample);
  };
  xgc_dmpc_paired_state_v1 paired{};
  paired.pose_stamp_sec = paired.twist_stamp_sec = 10.0;
  paired.position[0] = 1.25;
  paired.position[1] = -3.5;
  paired.position[2] = 2.0;
  feed(host.paired, paired);
  xgc_step_ctx ctx{};
  ctx.now = 10000000000LL;
  if (v->step(plugin.get(), &ctx) != XGC_OK || host.sent != 0) return fail("measurement before trigger");
  xgc_dmpc_sync_trigger_v1 trigger{};
  trigger.sequence_id = 1;
  trigger.trigger_time = 10.0;
  feed(host.trigger, trigger);
  if (v->step(plugin.get(), &ctx) != XGC_OK || host.sent != 1) return fail("measurement trigger");
  for (int tick = 1; tick <= 99; ++tick) {
    ctx.now += 1000000;
    if (v->step(plugin.get(), &ctx) != XGC_OK) return fail("measurement idle step");
  }
  feed(host.trigger, trigger);  // Duplicate trigger must not transmit twice.
  if (v->step(plugin.get(), &ctx) != XGC_OK || host.sent != 1) return fail("measurement exceeds planner rate");
  trigger.sequence_id = 2;
  trigger.trigger_time = 10.1;
  feed(host.trigger, trigger);
  if (v->step(plugin.get(), &ctx) != XGC_OK || host.sent != 2 || host.last.uav_id != 1 ||
      host.last.stamp_sec != 10.0 || std::memcmp(host.last.position, paired.position, sizeof paired.position))
    return fail("measurement changed actual pose or header time");
  v->deactivate(plugin.get());
  return 0;
}

int main(int argc, char** argv) {
  if (argc != 2 || argv[1][0] == '\0') {
    std::cerr << "usage: test_plan_dmpc FORMATION_GENERATOR_ROOT\n";
    return 2;
  }
  formation_root = argv[1];
  std::string error;
  auto exploration = comprehensive_request();
  exploration.manifest_path = testdata("exploration/manifest.yaml");
  exploration.scene_id = "two-static";
  if (xgc_plan_dmpc::PlanDmpc::open(exploration, &error) ||
      error.find("unsupported algorithm") == std::string::npos) {
    return fail("exploration was accepted");
  }
  auto mismatch = comprehensive_request();
  mismatch.horizon = 20;
  if (xgc_plan_dmpc::PlanDmpc::open(mismatch, &error) || error.find("horizon") == std::string::npos) {
    return fail("a flat horizon that disagrees with the manifest was accepted");
  }

  auto opened = xgc_plan_dmpc::PlanDmpc::open(comprehensive_request(), &error);
  if (!opened) return fail(error.c_str());
  xgc_plan_dmpc::PlanDmpc& planner = *opened;
  if (compare_comprehensive(planner) != 0) return 1;

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
  commit.round_k = 21;
  commit.mission_ns = 100000001;
  if (planner.push_commit(commit, digest).empty()) return fail("same command accepted an invalid round time");
  commit.mission_ns = 100000000;
  if (!planner.push_commit(commit, digest).empty() ||
      planner.step(10.2, 10.2, 100.2).timeline.mission_ns != 100000000) {
    return fail("same command did not advance its per-round mission time");
  }

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
  // A host step can drain more than one commit before processing a trigger.
  // Missing the effective beat must not lose the already committed command.
  ++commit.round_k;
  commit.mission_ns += 100000000;
  if (!planner.push_commit(commit, digest).empty()) return fail("next pattern beat was rejected");
  auto pattern = planner.step(10.2, 10.2, 100.2);
  if (pattern.timeline.applied_revision != 2 || pattern.timeline.fault != 0 || !std::isfinite(planner.pattern_offset().x())) {
    return fail("pattern 5 was not applied from the installed factory");
  }
  if (pattern.timeline.mission_ns != 6 * 100000000LL ||
      pattern.timeline.applied_mission_ns != 6 * 100000000LL) {
    return fail("late pattern did not report the actual application phase");
  }
  ++commit.round_k;
  commit.mission_ns += 100000000;
  if (!planner.push_commit(commit, digest).empty()) return fail("repeated pattern beat was rejected");
  if (planner.step(10.2, 10.2, 100.2).timeline.applied_mission_ns != 6 * 100000000LL) {
    return fail("repeated beat reapplied the pattern");
  }
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
  ++rewind.round_k;
  rewind.mission_ns += 100000000;
  if (!planner.push_commit(rewind, rewind_digest).empty()) return fail("next goal beat was rejected");
  rewind.request.rolling = 0;
  rewind.mission_ns = rewind.request.anchor_ns;
  rewind_digest[0] = 5;
  if (!planner.push_commit(rewind, rewind_digest).empty()) return fail("held same-revision commit was rejected");
  auto frozen = planner.step(10.2, 10.2, 100.2);
  if (frozen.timeline.mission_ns != rewind.request.anchor_ns || frozen.timeline.committed_revision != 3) {
    return fail("planner did not keep the held mission");
  }

  YAML::Node document = YAML::LoadFile(
      scenario_path("config/scenarios/dmpc_comprehensive/scene/scene.yaml"));
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
  std::vector<uint8_t> initial_plan;
  for (int attempt = 0; attempt < 6; ++attempt) {
    const auto trace = planner.step(10.2, 10.2, 100.2);
    if (trace.status.solver_called) return fail("solver ran before any neighbor plan arrived");
    if (!trace.own_plan.empty()) initial_plan = trace.own_plan;
  }
  if (initial_plan.empty()) return fail("startup did not broadcast a seeded trajectory while waiting for neighbors");
  formation_generator_dmpc::PlanMessage neighbor;
  if (!formation_generator_dmpc::decodePlanPayload(initial_plan.data(), initial_plan.size(), neighbor)) {
    return fail("initial seeded trajectory is not a usable wire plan");
  }
  auto feed_neighbor = [&](const formation_generator_dmpc::PlanMessage& plan) {
    const auto wire = formation_generator_dmpc::encodePlanPayload(plan);
    return planner.push_neighbor_plan(wire.data(), wire.size());
  };
  if (feed_neighbor(neighbor).empty()) return fail("own trajectory was accepted as a neighbor");
  neighbor.uav_id = 9;
  if (feed_neighbor(neighbor).empty()) return fail("out of roster neighbor was accepted");
  neighbor.uav_id = 2;
  neighbor.valid = false;
  if (feed_neighbor(neighbor).empty()) return fail("invalid trajectory counted as a ready neighbor");
  neighbor.valid = true;
  for (int id = 2; id <= 8; ++id) {
    neighbor.uav_id = id;
    for (uint32_t k = 0; k < neighbor.num_timesteps; ++k) {
      neighbor.states[k * neighbor.num_states] += 2.0;
    }
    if (!feed_neighbor(neighbor).empty()) return fail("valid neighbor trajectory was rejected");
    xgc_dmpc_measured_position_v1 position{};
    position.uav_id = static_cast<uint32_t>(id);
    position.stamp_sec = 10.2;
    position.position[0] = 2.0 * id;
    position.position[1] = -16.0;
    position.position[2] = 2.0;
    if (!planner.push_neighbor_position(position).empty()) return fail("measured neighbor position was rejected");
  }
  bool saw_solve = false;
  for (int attempt = 0; attempt < 6 && !saw_solve; ++attempt) {
    const auto trace = planner.step(10.2, 10.2, 100.2);
    if (!trace.status.solver_called) continue;
    saw_solve = true;
    if (trace.timeline.applied_revision != 3 ||
        trace.timeline.applied_mission_ns != rewind.mission_ns || planner.goal_waiting()) {
      return fail("consumed goal did not report its applied revision and mission time");
    }
    if (std::strstr(trace.status.reject_reason, "missing cached pose") != nullptr) {
      return fail("goal bootstrap did not receive the measured neighbor positions");
    }
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
      scenario_path("config/scenarios/knot_fs150/scene.yaml"));
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
  header.part_count = 0;
  header.vertex_count = 0;
  const std::string empty_world = planner.push_scene_wire(header, nullptr, nullptr, nullptr);
  if (!empty_world.empty() || planner.static_obstacle_count() != 0 || planner.scene_epoch() != epoch) {
    std::cerr << empty_world << '\n';
    return fail("empty world was not a loaded scene");
  }
  using Entry = const xgc_plugin_descriptor* (*)();
  auto* entry = reinterpret_cast<Entry>(dlsym(RTLD_DEFAULT, "xgc_rt_plugin_v1"));
  if (entry == nullptr || entry() == nullptr || std::strcmp(entry()->name, "plan-dmpc") != 0 || entry()->port_count != 13) {
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
  if (admit_two_static_scene() != 0 || measured_position_rate(entry()) != 0) return 1;
  std::cout << "lifecycle=" << held.status.lifecycle << " reject=" << held.status.reject_reason << '\n';
  std::cout << "obstacles=" << planner.static_obstacle_count() << " ports=" << entry()->port_count << '\n';
  return 0;
}
