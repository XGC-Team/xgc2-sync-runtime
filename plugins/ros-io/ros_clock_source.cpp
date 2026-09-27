// Clock-source service exported from libros_io.so. The host polls it on one
// thread before domain activation. It does not stamp messages in Unix time
// and does not spin the ordinary ros_io callback queue.
#include "ros_edge.hpp"

#include <ros/callback_queue.h>
#include <ros/message_event.h>
#include <ros/subscription_queue.h>
#include <rosgraph_msgs/Clock.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <string>

namespace {

struct ClockSample {
  int64_t time_ns{0};
  std::string publisher;
};

struct ParsedConfig {
  std::string node_name;
  std::string topic;
  std::string expected_publisher;
  int queue_capacity{0};
};

void copy_text(char* dest, size_t cap, const std::string& text) {
  const size_t n = std::min(text.size(), cap - 1);
  std::memcpy(dest, text.data(), n);
  dest[n] = '\0';
}

bool parse_config(const char* toml, ParsedConfig* out, std::string* error) {
  if (toml == nullptr) {
    *error = "clock source config is missing";
    return false;
  }
  bool seen_node = false, seen_topic = false, seen_publisher = false, seen_queue = false;
  std::string text(toml);
  size_t line_start = 0;
  while (line_start <= text.size()) {
    size_t line_end = text.find('\n', line_start);
    if (line_end == std::string::npos) line_end = text.size();
    std::string line = text.substr(line_start, line_end - line_start);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    line_start = line_end + 1;
    const auto first = line.find_first_not_of(" \t");
    if (first == std::string::npos || line[first] == '#') {
      if (line_end == text.size()) break;
      continue;
    }
    line = line.substr(first);
    const auto eq = line.find('=');
    if (eq == std::string::npos) {
      *error = "clock source config line has no value";
      return false;
    }
    std::string key = line.substr(0, eq);
    std::string value = line.substr(eq + 1);
    const auto key_end = key.find_last_not_of(" \t");
    const auto value_begin = value.find_first_not_of(" \t");
    if (key_end == std::string::npos || value_begin == std::string::npos) {
      *error = "clock source config line is incomplete";
      return false;
    }
    key = key.substr(0, key_end + 1);
    value = value.substr(value_begin);
    const auto value_end = value.find_last_not_of(" \t");
    value = value.substr(0, value_end + 1);
    auto take_string = [&](bool* seen, std::string* dest) {
      if (*seen || value.size() < 2 || value.front() != '"' || value.back() != '"') return false;
      *seen = true;
      *dest = value.substr(1, value.size() - 2);
      return true;
    };
    if (key == "node_name") {
      if (!take_string(&seen_node, &out->node_name)) {
        *error = "node_name must be one quoted string";
        return false;
      }
    } else if (key == "topic") {
      if (!take_string(&seen_topic, &out->topic)) {
        *error = "topic must be one quoted string";
        return false;
      }
    } else if (key == "expected_publisher") {
      if (!take_string(&seen_publisher, &out->expected_publisher)) {
        *error = "expected_publisher must be one quoted string";
        return false;
      }
    } else if (key == "queue_capacity") {
      if (seen_queue || value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
        *error = "queue_capacity must be one positive integer";
        return false;
      }
      seen_queue = true;
      try {
        out->queue_capacity = std::stoi(value);
      } catch (...) {
        *error = "queue_capacity is not a valid integer";
        return false;
      }
    } else {
      *error = "clock source config has an unknown key";
      return false;
    }
    if (line_end == text.size()) break;
  }
  if (!seen_node || !seen_topic || !seen_publisher || !seen_queue || out->node_name.empty() || out->topic.empty() ||
      out->topic.front() != '/' || out->expected_publisher.empty() || out->queue_capacity < 1 ||
      out->queue_capacity > 65536) {
    *error = "clock source config is missing a required key or has an invalid value";
    return false;
  }
  return true;
}

bool stamp_to_ns(uint32_t sec, uint32_t nsec, int64_t* out, std::string* error) {
  if (nsec >= 1000000000u) {
    *error = "clock stamp is not a valid ROS time";
    return false;
  }
  if (static_cast<uint64_t>(sec) > static_cast<uint64_t>(std::numeric_limits<int64_t>::max() / 1000000000LL)) {
    *error = "clock stamp overflows nanoseconds";
    return false;
  }
  *out = static_cast<int64_t>(sec) * 1000000000LL + static_cast<int64_t>(nsec);
  return true;
}

// Noetic calls addCallback after a non-dropping SubscriptionQueue::push,
// under Subscription::callbacks_mutex_. Reserve one extra ROS queue slot and
// latch when that slot fills: the NEXT full push could silently replace an
// older message without scheduling another callback. Check at ingress, not
// just in on_clock, so simultaneous polling cannot conceal that boundary.
class ClockCallbackQueue final : public ros::CallbackQueue {
 public:
  std::atomic<bool> overflow{false};
  void addCallback(const ros::CallbackInterfacePtr& callback, uint64_t owner = 0) override {
    auto* subscription = dynamic_cast<ros::SubscriptionQueue*>(callback.get());
    if (subscription == nullptr || subscription->full()) overflow.store(true, std::memory_order_release);
    ros::CallbackQueue::addCallback(callback, owner);
  }
};

struct ClockSource {
  bool started{false};
  ParsedConfig config;
  ClockCallbackQueue queue;
  std::unique_ptr<ros::NodeHandle> nh;
  ros::Subscriber sub;
  std::deque<ClockSample> samples;
  uint64_t dropped{0};
  bool drop_fault{false};
  bool have_committed{false};
  int64_t committed_ns{0};
  uint64_t sequence{0};

  void on_clock(const ros::MessageEvent<rosgraph_msgs::Clock const>& event) {
    try {
      if (queue.overflow.load(std::memory_order_acquire) || drop_fault || samples.size() >= static_cast<size_t>(config.queue_capacity)) {
        ++dropped;
        drop_fault = true;
        return;
      }
      const rosgraph_msgs::Clock::ConstPtr msg = event.getConstMessage();
      if (!msg) return;
      int64_t stamp = 0;
      std::string error;
      if (!stamp_to_ns(msg->clock.sec, msg->clock.nsec, &stamp, &error)) {
        samples.push_back(ClockSample{std::numeric_limits<int64_t>::min(), error});
        return;
      }
      samples.push_back(ClockSample{stamp, event.getPublisherName()});
    } catch (...) {
      drop_fault = true;
      ++dropped;
    }
  }

  void counts(xgc_clock_observation_v1* out) const {
    out->publisher_count = sub ? sub.getNumPublishers() : 0;
    out->dropped = dropped;
    out->coalesced = 0;
    out->time_ns = 0;
    out->sequence = 0;
    out->publisher[0] = '\0';
  }
};

void fail(xgc_clock_observation_v1* out, const std::string& error) {
  if (out == nullptr) return;
  copy_text(out->error, XGC_CLOCK_TEXT_CAP, error);
}

int32_t clock_start(void* self, const char* config_toml, xgc_clock_observation_v1* out) {
  try {
    auto* source = static_cast<ClockSource*>(self);
    if (source == nullptr || out == nullptr) return XGC_CLOCK_ERROR;
    if (source->started) {
      fail(out, "clock source is already started");
      return XGC_CLOCK_ERROR;
    }
    std::string error;
    if (!parse_config(config_toml, &source->config, &error)) {
      fail(out, error);
      return XGC_CLOCK_ERROR;
    }
    const xgc_ros_edge::RosInit init = xgc_ros_edge::ensure_ros(source->config.node_name);
    if (!init.ok) {
      fail(out, init.error);
      return XGC_CLOCK_ERROR;
    }
    source->nh = std::make_unique<ros::NodeHandle>();
    source->nh->setCallbackQueue(&source->queue);
    bool use_sim_time = false;
    if (!ros::param::get("/use_sim_time", use_sim_time) || !use_sim_time) {
      source->nh.reset();
      fail(out, "/use_sim_time is not true");
      return XGC_CLOCK_ERROR;
    }
    source->sub = source->nh->subscribe(source->config.topic, static_cast<uint32_t>(source->config.queue_capacity) + 1u,
                                        &ClockSource::on_clock, source);
    xgc_ros_edge::set_output_gate(XGC_CLOCK_GATE_CLOSED);
    source->started = true;
    source->counts(out);
    return XGC_CLOCK_OK;
  } catch (...) {
    fail(out, "clock source start failed");
    return XGC_CLOCK_ERROR;
  }
}

int32_t clock_poll(void* self, uint64_t timeout_wall_ns, xgc_clock_observation_v1* out) {
  try {
    auto* source = static_cast<ClockSource*>(self);
    if (source == nullptr || out == nullptr || !source->started) return XGC_CLOCK_ERROR;
    const double seconds = static_cast<double>(timeout_wall_ns) / 1000000000.0;
    source->queue.callAvailable(ros::WallDuration(seconds));
    source->counts(out);
    if (source->drop_fault || source->queue.overflow.load(std::memory_order_acquire)) {
      fail(out, "clock queue capacity exceeded; observations may have been dropped");
      return XGC_CLOCK_ERROR;
    }
    if (out->publisher_count > 1) {
      fail(out, "clock topic has multiple publishers");
      return XGC_CLOCK_ERROR;
    }
    if (source->samples.empty()) return XGC_CLOCK_AGAIN;
    if (out->publisher_count != 1) {
      source->samples.clear();
      fail(out, "clock publisher is not connected");
      return XGC_CLOCK_ERROR;
    }
    std::deque<ClockSample> batch;
    batch.swap(source->samples);
    int64_t previous = source->have_committed ? source->committed_ns : batch.front().time_ns;
    bool have_previous = source->have_committed;
    for (const ClockSample& sample : batch) {
      if (sample.time_ns == std::numeric_limits<int64_t>::min()) {
        fail(out, sample.publisher.empty() ? "clock stamp is not a valid ROS time" : sample.publisher);
        return XGC_CLOCK_ERROR;
      }
      if (sample.publisher != source->config.expected_publisher) {
        fail(out, "clock publisher does not match the frozen caller");
        return XGC_CLOCK_ERROR;
      }
      if (have_previous && sample.time_ns < previous) {
        fail(out, "clock moved backward");
        return XGC_CLOCK_ERROR;
      }
      previous = sample.time_ns;
      have_previous = true;
    }
    if (source->queue.overflow.load(std::memory_order_acquire)) {
      fail(out, "clock queue capacity exceeded during poll");
      return XGC_CLOCK_ERROR;
    }
    const ClockSample& newest = batch.back();
    source->have_committed = true;
    source->committed_ns = newest.time_ns;
    ++source->sequence;
    out->time_ns = newest.time_ns;
    out->sequence = source->sequence;
    out->coalesced = static_cast<uint32_t>(batch.size() - 1);
    copy_text(out->publisher, XGC_CLOCK_TEXT_CAP, newest.publisher);
    return XGC_CLOCK_OK;
  } catch (...) {
    fail(out, "clock source poll failed");
    return XGC_CLOCK_ERROR;
  }
}

int32_t clock_set_gate(void* self, uint32_t gate) {
  try {
    auto* source = static_cast<ClockSource*>(self);
    if (source == nullptr || !source->started) return XGC_CLOCK_ERROR;
    if (gate != XGC_CLOCK_GATE_CLOSED && gate != XGC_CLOCK_GATE_OPEN && gate != XGC_CLOCK_GATE_FAULT) {
      return XGC_CLOCK_ERROR;
    }
    xgc_ros_edge::set_output_gate(gate);
    return XGC_CLOCK_OK;
  } catch (...) {
    return XGC_CLOCK_ERROR;
  }
}

void clock_stop(void* self) {
  try {
    auto* source = static_cast<ClockSource*>(self);
    if (source == nullptr) return;
    xgc_ros_edge::set_output_gate(XGC_CLOCK_GATE_CLOSED);
    source->sub.shutdown();
    source->samples.clear();
    source->nh.reset();
    source->started = false;
  } catch (...) {
  }
}

void clock_destroy(void* self) {
  clock_stop(self);
  delete static_cast<ClockSource*>(self);
}

void* clock_create() {
  try {
    return new ClockSource{};
  } catch (...) {
    return nullptr;
  }
}

const xgc_clock_source_vtbl_v1 kClockVtbl = {
    clock_create, clock_start, clock_poll, clock_set_gate, clock_stop, clock_destroy,
};

const xgc_clock_source_descriptor_v1 kClockDescriptor = {XGC_CLOCK_SOURCE_ABI_VERSION, 0u, &kClockVtbl};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc_clock_source_descriptor_v1* xgc_rt_clock_source_v1(void) {
  return &kClockDescriptor;
}
