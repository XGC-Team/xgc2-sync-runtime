// Real Noetic /clock adapter test. A private roscore is started by
// run-clock-test.sh. This process does not call the station.
#include "xgc_clock_source.h"
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/String.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <string>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace {

int fail(const std::string& message) {
  std::cerr << "ros-clock-test: " << message << "\n";
  return 1;
}

int publish_main(const char* node_name) {
  ros::init(ros::M_string{}, node_name, ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 16, false);
  const ros::WallTime begin = ros::WallTime::now();
  while (pub.getNumSubscribers() == 0 && ros::ok()) {
    ros::spinOnce();
    ros::WallDuration(0.02).sleep();
    if ((ros::WallTime::now() - begin).toSec() > 5.0) return 3;
  }
  std::cout << "ready\n" << std::flush;
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line == "quit") return 0;
    int sec = 0;
    int nsec = 0;
    if (std::sscanf(line.c_str(), "%d %d", &sec, &nsec) != 2) return 2;
    rosgraph_msgs::Clock message;
    message.clock.sec = static_cast<uint32_t>(sec);
    message.clock.nsec = static_cast<uint32_t>(nsec);
    pub.publish(message);
    ros::spinOnce();
    std::cout << "published\n" << std::flush;
  }
  return 0;
}

struct Loaded {
  void* so{nullptr};
  const xgc_clock_source_vtbl_v1* clock{nullptr};
  const xgc_plugin_vtbl* plugin{nullptr};
  void* clock_self{nullptr};
  void* plugin_self{nullptr};
};

int load(Loaded* loaded, const char* path) {
  loaded->so = dlopen(path, RTLD_NOW);
  if (loaded->so == nullptr) return fail(dlerror());
  auto* clock_entry = reinterpret_cast<xgc_clock_source_entry_v1>(dlsym(loaded->so, XGC_CLOCK_SOURCE_ENTRY));
  auto* plugin_entry = reinterpret_cast<const xgc_plugin_descriptor* (*)()>(dlsym(loaded->so, "xgc_rt_plugin_v1"));
  if (clock_entry == nullptr || plugin_entry == nullptr) return fail("plugin exports are missing");
  const xgc_clock_source_descriptor_v1* clock = clock_entry();
  const xgc_plugin_descriptor* plugin = plugin_entry();
  if (clock == nullptr || clock->abi_version != XGC_CLOCK_SOURCE_ABI_VERSION || clock->reserved != 0 || clock->vtbl == nullptr) {
    return fail("clock descriptor is not v1");
  }
  loaded->clock = clock->vtbl;
  loaded->plugin = plugin->vtbl;
  loaded->clock_self = loaded->clock->create();
  if (loaded->clock_self == nullptr) return fail("clock create failed");
  return 0;
}

struct Host {
  xgc_host_api api{};
  std::vector<uint8_t> sample;
  bool pending{false};
  int pending_port{-1};
  int64_t now{1000000000};
  int command_inputs{0};
};

xgc_status host_publish(void* host, uint32_t port, uint64_t, const uint8_t*, uint32_t) {
  if (port == 13) ++static_cast<Host*>(host)->command_inputs;
  return XGC_OK;
}
xgc_status host_next(void* host, uint32_t port, xgc_sample_view* out) {
  auto* self = static_cast<Host*>(host);
  if (!self->pending || port != static_cast<uint32_t>(self->pending_port)) return XGC_ERR_AGAIN;
  self->pending = false;
  out->data = self->sample.data();
  out->len = static_cast<uint32_t>(self->sample.size());
  return XGC_OK;
}
int64_t host_now(void* host) { return static_cast<Host*>(host)->now; }
void host_log(void*, xgc_log_level, const char*) {}
void host_degrade(void*, const char*) {}
void host_recover(void*) {}
uint32_t host_origins(void*, uint32_t, uint16_t*, uint32_t) { return 0; }
uint16_t host_node(void*) { return 1; }

bool wait_message(bool* seen, int milliseconds) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
  while (std::chrono::steady_clock::now() < deadline) {
    ros::spinOnce();
    if (*seen) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return *seen;
}

bool send_stamp(FILE* to_child, FILE* from_child, int sec, int nsec) {
  std::fprintf(to_child, "%d %d\n", sec, nsec);
  std::fflush(to_child);
  char line[64];
  return std::fgets(line, sizeof line, from_child) != nullptr && std::string(line) == "published\n";
}

int32_t poll_for(Loaded* loaded, xgc_clock_observation_v1* observation, int32_t want, int attempts) {
  for (int i = 0; i < attempts; ++i) {
    const int32_t code = loaded->clock->poll(loaded->clock_self, 100000000ULL, observation);
    if (code == want || code == XGC_CLOCK_ERROR) return code;
  }
  return loaded->clock->poll(loaded->clock_self, 20000000ULL, observation);
}

// The network thread receives a complete burst while the source callback
// queue is deliberately not polled. A hidden intermediate reset must never
// become an accepted monotonic final sample.
int overflow_main(const char* executable, const char* library, int capacity, bool one_extra = false) {
  int to_child[2], from_child[2];
  if (pipe(to_child) != 0 || pipe(from_child) != 0) return fail("burst pipe failed");
  const pid_t child = fork();
  if (child < 0) return fail("burst fork failed");
  if (child == 0) {
    dup2(to_child[0], STDIN_FILENO); dup2(from_child[1], STDOUT_FILENO);
    close(to_child[1]); close(from_child[0]);
    execl(executable, executable, "--publish", "gazebo", nullptr);
    return 2;
  }
  close(to_child[0]); close(from_child[1]);
  FILE* cmd = fdopen(to_child[1], "w");
  FILE* ack = fdopen(from_child[0], "r");
  ros::init(ros::M_string{}, "xgc_ros_io", ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
  ros::param::set("/use_sim_time", true);
  Loaded loaded;
  if (load(&loaded, library)) return 2;
  const std::string config = "node_name = \"xgc_ros_io\"\n"
      "topic = \"/clock\"\nexpected_publisher = \"/gazebo\"\nqueue_capacity = " + std::to_string(capacity) + "\n";
  xgc_clock_observation_v1 observation{};
  if (loaded.clock->start(loaded.clock_self, config.c_str(), &observation) != XGC_CLOCK_OK) return fail(observation.error);
  char ready[64];
  if (!std::fgets(ready, sizeof ready, ack)) return fail("burst publisher missing handshake");
  for (int attempt = 0; attempt < 50 && observation.publisher_count != 1; ++attempt) {
    loaded.clock->poll(loaded.clock_self, 20000000ULL, &observation);
  }
  if (!send_stamp(cmd, ack, 10, 0)) return fail("burst baseline publish failed");
  const int baseline = poll_for(&loaded, &observation, XGC_CLOCK_OK, 30);
  if (baseline != XGC_CLOCK_OK) return fail("burst baseline missing code=" + std::to_string(baseline) +
      " pubs=" + std::to_string(observation.publisher_count) + " error=" + observation.error);
  // Exactly the admitted capacity must still preserve every sample.
  for (int i = 1; i <= capacity; ++i) {
    if (!send_stamp(cmd, ack, 10, i)) return fail("capacity boundary publish failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  observation = {};
  const int boundary = loaded.clock->poll(loaded.clock_self, 20000000ULL, &observation);
  if (boundary != XGC_CLOCK_OK || observation.time_ns != 10000000000LL + capacity ||
      observation.coalesced != static_cast<uint32_t>(capacity - 1) || observation.dropped != 0) {
    return fail("exact capacity rejected or lost samples: code=" + std::to_string(boundary) + " error=" + observation.error);
  }
  std::cout << "capacity-boundary capacity=" << capacity << " code=" << boundary << " coalesced=" << observation.coalesced << " dropped=" << observation.dropped << "\n";
  const int burst_count = one_extra ? capacity + 1 : capacity * 3 + 4;
  for (int i = 0; i < burst_count; ++i) {
    const int second = i == (one_extra ? 0 : 1) ? 1 : 11 + i;
    if (!send_stamp(cmd, ack, second, 0)) return fail("burst publish failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  observation = {};
  const int code = loaded.clock->poll(loaded.clock_self, 20000000ULL, &observation);
  std::cout << "overflow capacity=" << capacity << " sent=" << burst_count << " code=" << code
      << " time_ns=" << observation.time_ns << " coalesced=" << observation.coalesced
      << " dropped=" << observation.dropped << " error=" << observation.error << "\n";
  if (code == XGC_CLOCK_ERROR && loaded.clock->poll(loaded.clock_self, 0, &observation) != XGC_CLOCK_ERROR) {
    return fail("overflow fault did not remain latched");
  }
  std::fprintf(cmd, "quit\n"); std::fflush(cmd);
  loaded.clock->stop(loaded.clock_self); loaded.clock->destroy(loaded.clock_self);
  dlclose(loaded.so); std::fclose(cmd); std::fclose(ack);
  int status = 0; waitpid(child, &status, 0);
  return code == XGC_CLOCK_ERROR ? 0 : fail("subscription overflow hid the intermediate reset");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc >= 2 && std::string(argv[1]) == "--publish") return publish_main(argc >= 3 ? argv[2] : "gazebo");
  if (argc == 4 && std::string(argv[1]) == "--overflow") return overflow_main(argv[0], argv[3], std::stoi(argv[2]));
  if (argc == 4 && std::string(argv[1]) == "--overflow-one") return overflow_main(argv[0], argv[3], std::stoi(argv[2]), true);
  if (argc != 2) return fail("usage: ros_clock_source_test LIBROS | --overflow CAPACITY LIBROS");

  int to_child[2];
  int from_child[2];
  if (pipe(to_child) != 0 || pipe(from_child) != 0) return fail("pipe failed");
  const pid_t publisher = fork();
  if (publisher < 0) return fail("fork failed");
  if (publisher == 0) {
    dup2(to_child[0], STDIN_FILENO);
    dup2(from_child[1], STDOUT_FILENO);
    close(to_child[1]);
    close(from_child[0]);
    execl(argv[0], argv[0], "--publish", "gazebo", nullptr);
    return 2;
  }
  close(to_child[0]);
  close(from_child[1]);
  FILE* cmd = fdopen(to_child[1], "w");
  FILE* ack = fdopen(from_child[0], "r");

  ros::init(argc, argv, "xgc_ros_io", ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
  ros::param::set("/use_sim_time", false);
  Loaded loaded;
  if (const int rc = load(&loaded, argv[1])) return rc;
  xgc_clock_observation_v1 observation{};
  const char* config =
      "node_name = \"xgc_ros_io\"\n"
      "topic = \"/clock\"\n"
      "expected_publisher = \"/gazebo\"\n"
      "queue_capacity = 8\n";
  if (loaded.clock->start(loaded.clock_self, config, &observation) != XGC_CLOCK_ERROR) {
    return fail("/use_sim_time false was accepted");
  }
  ros::param::set("/use_sim_time", true);
  std::memset(&observation, 0, sizeof observation);
  if (loaded.clock->start(loaded.clock_self, config, &observation) != XGC_CLOCK_OK) {
    return fail(std::string("start failed: ") + observation.error);
  }
  if (observation.sequence != 0) return fail("start OK was treated as a clock sample");
  if (loaded.clock->poll(loaded.clock_self, 30000000ULL, &observation) != XGC_CLOCK_AGAIN) {
    return fail("missing clock was reported as a sample");
  }
  char ready[64];
  if (std::fgets(ready, sizeof ready, ack) == nullptr || std::string(ready) != "ready\n") {
    return fail("clock publisher did not connect");
  }
  if (!send_stamp(cmd, ack, 0, 0)) return fail("publisher did not accept the zero stamp");
  std::memset(&observation, 0, sizeof observation);
  const int32_t zero_code = poll_for(&loaded, &observation, XGC_CLOCK_OK, 30);
  if (zero_code != XGC_CLOCK_OK || observation.time_ns != 0 || observation.sequence == 0 ||
      std::string(observation.publisher) != "/gazebo" || observation.publisher_count != 1) {
    return fail("zero stamp code=" + std::to_string(zero_code) + " time=" + std::to_string(observation.time_ns) +
                " seq=" + std::to_string(observation.sequence) + " pubs=" + std::to_string(observation.publisher_count) +
                " publisher=[" + observation.publisher + "] error=[" + observation.error + "]");
  }
  const uint64_t zero_sequence = observation.sequence;
  std::memset(&observation, 0, sizeof observation);
  if (loaded.clock->poll(loaded.clock_self, 20000000ULL, &observation) != XGC_CLOCK_AGAIN) {
    return fail("a second poll without a new stamp reused the zero sample");
  }

  if (!send_stamp(cmd, ack, 1, 500000000)) return fail("publisher did not accept 1.5 s");
  std::memset(&observation, 0, sizeof observation);
  if (poll_for(&loaded, &observation, XGC_CLOCK_OK, 30) != XGC_CLOCK_OK || observation.time_ns != 1500000000LL ||
      observation.sequence <= zero_sequence) {
    return fail("1.5 s simulation stamp was not preserved");
  }
  if (!send_stamp(cmd, ack, 1, 0)) return fail("publisher did not accept the backward stamp");
  std::memset(&observation, 0, sizeof observation);
  if (poll_for(&loaded, &observation, XGC_CLOCK_ERROR, 30) != XGC_CLOCK_ERROR) {
    return fail("backward clock was accepted");
  }

  Host host;
  host.api.abi_version = XGC_RT_ABI_VERSION;
  host.api.abi_minor = XGC_RT_ABI_MINOR;
  host.api.host = &host;
  host.api.publish = host_publish;
  host.api.next = host_next;
  host.api.now = host_now;
  host.api.log = host_log;
  host.api.request_degrade = host_degrade;
  host.api.request_recover = host_recover;
  host.api.port_origins = host_origins;
  host.api.node_id = host_node;
  loaded.plugin_self = loaded.plugin->create(&host.api);
  const char* plugin_config =
      "setpoint_topic = \"/clock_test/setpoint\"\n"
      "command_topic = \"/clock_test/command\"\n"
      "node_name = \"xgc_ros_io\"\n";
  if (loaded.plugin->configure(loaded.plugin_self, plugin_config) != XGC_OK) return fail("ros_io configure failed");
  if (loaded.plugin->activate(loaded.plugin_self) != XGC_OK) return fail("ros_io activate failed");

  ros::NodeHandle nh;
  ros::Publisher command_pub = nh.advertise<std_msgs::String>("/clock_test/command", 1);
  bool published = false;
  ros::Subscriber sub = nh.subscribe<mavros_msgs::PositionTarget>(
      "/clock_test/setpoint", 1, [&](const mavros_msgs::PositionTarget::ConstPtr&) { published = true; });
  std_msgs::String command;
  command.data = "alive";
  for (int i = 0; i < 20 && command_pub.getNumSubscribers() == 0; ++i) {
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  command_pub.publish(command);
  xgc_step_ctx ctx{};
  ctx.now = host.now;
  ctx.deadline = host.now + 200000000LL;
  if (loaded.plugin->step(loaded.plugin_self, &ctx) != XGC_OK) return fail("input step failed");
  if (host.command_inputs < 1) return fail("ordinary ros_io input queue did not deliver");

  xgc_position_target_v1 sample{};
  sample.position[0] = 1.0;
  host.sample.assign(reinterpret_cast<uint8_t*>(&sample), reinterpret_cast<uint8_t*>(&sample) + sizeof sample);
  host.pending = true;
  host.pending_port = 15;
  ctx.deadline = host.now + 30000000LL;
  if (loaded.plugin->step(loaded.plugin_self, &ctx) != XGC_OK) return fail("closed-gate step failed");
  if (wait_message(&published, 300)) return fail("closed gate published a setpoint");

  ctx.deadline = host.now + 40000000LL;
  const auto started = std::chrono::steady_clock::now();
  if (loaded.plugin->step(loaded.plugin_self, &ctx) != XGC_OK) return fail("steady step failed");
  if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started).count() > 500) {
    return fail("paused session deadline extended the callback slice");
  }

  if (loaded.clock->set_gate(loaded.clock_self, XGC_CLOCK_GATE_OPEN) != XGC_CLOCK_OK) return fail("set_gate open failed");
  published = false;
  host.pending = true;
  ctx.deadline = host.now + 30000000LL;
  if (loaded.plugin->step(loaded.plugin_self, &ctx) != XGC_OK) return fail("open transition step failed");
  if (wait_message(&published, 200)) return fail("the first open step replayed a suspended output");
  for (int i = 0; i < 30 && sub.getNumPublishers() == 0; ++i) {
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  published = false;
  host.pending = true;
  if (loaded.plugin->step(loaded.plugin_self, &ctx) != XGC_OK) return fail("second open step failed");
  if (!wait_message(&published, 1000)) return fail("open gate did not publish the current setpoint");

  if (loaded.clock->set_gate(loaded.clock_self, XGC_CLOCK_GATE_FAULT) != XGC_CLOCK_OK) return fail("set_gate fault failed");
  published = false;
  host.pending = true;
  if (loaded.plugin->step(loaded.plugin_self, &ctx) != XGC_OK) return fail("fault-gate step failed");
  if (wait_message(&published, 300)) return fail("fault gate published a setpoint");
  if (loaded.clock->set_gate(loaded.clock_self, 9) != XGC_CLOCK_ERROR) return fail("invalid gate was accepted");

  std::fprintf(cmd, "quit\n");
  std::fflush(cmd);
  loaded.clock->stop(loaded.clock_self);
  loaded.clock->destroy(loaded.clock_self);
  loaded.plugin->deactivate(loaded.plugin_self);
  loaded.plugin->destroy(loaded.plugin_self);
  dlclose(loaded.so);
  std::cerr << "ros-clock-test: ok\n";
  return 0;
}
