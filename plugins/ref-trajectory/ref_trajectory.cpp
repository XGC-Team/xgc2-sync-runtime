// ref-trajectory: the multirotor reference trajectory generator as an
// aggregator module.
//
// Runs multirotor_reference_trajectory_core, the ROS-free runtime of
// xgc2-multirotor-controller's multirotor_reference_trajectory (analytic
// curves, sampled references, and the state machine
// that activates them), behind module I/O. What the ROS node's input
// producer and output consumer did becomes ports; ros_io does the ROS side.
//
//   in  analytic         xgc.ref.analytic/1          (request/analytic)
//   in  sampled          xgc.ref.sampled/1           (request/sampled)
//   in  reset            xgc.ref.reset/1             (reset)
//   in  clock            xgc.clock/1                 (optional; replay: advance time)
//   out status           xgc.ref.status/1            (status)
//   out active_analytic  xgc.ref.analytic/1          (active/analytic)
//   out active_sampled   xgc.ref.sampled/1           (active/sampled)
//   out trace            xgc.text/1                  (optional; replay trace)
//
// Every request's receive time is its envelope t_produce.
//
// time_source:
//   "session" (default)  one runtime update per round at Session time (the
//                        node ran at 100 Hz: use 10 ms rounds).
//   "input"              replay: updates every 10 ms of input time from the
//                        first request; an update runs once every request up
//                        to it has arrived (a later one, or the clock port,
//                        proves it). This reproduces the runtime's replay
//                        harness (multirotor_reference_trajectory
//                        test/replay/reference_replay_harness.cpp) exactly;
//                        with trace = true the trace port carries its lines.
//
// Config: time_source, trace, status_rate,
// active_publish_rate, validation_sample_dt, trajectory_timeout,
// min_lead_time, max_velocity, max_acceleration, max_jerk, max_snap,
// min_specific_thrust (defaults: config/multirotor_reference_trajectory.yaml).

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <exception>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include "flat_config.hpp"
#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"
#include "reference_wire.hpp"
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

namespace {

namespace mrt = multirotor_reference_trajectory;
namespace ref = multirotor_reference_trajectory::reference;
namespace sm = state_machine;
namespace trajectory = xgc2_math::trajectory;

enum Port : uint32_t {
  kAnalytic, kSampled, kReset, kClock,
  kStatus, kActiveAnalytic, kActiveSampled, kTrace, kPortCount
};

struct Input {
  uint64_t t_ns;
  uint32_t port;
  std::vector<uint8_t> data;
};

uint64_t bits(double v) {
  uint64_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
}

// The harness's text, written into a string.
struct Text {
  std::string s;
  void f(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) s.append(buf, std::min<size_t>(static_cast<size_t>(n), sizeof buf - 1));
  }
  void d(double v) { f(" %016" PRIx64, bits(v)); }
  void ds(const std::vector<double>& v) {
    f(" [%zu", v.size());
    for (double x : v) d(x);
    s += ']';
  }
  void t(const mrt::Time& v) { f(" %u.%09u", v.sec, v.nsec); }
  template <class V>
  void v3(const V& v) {
    d(v.x); d(v.y); d(v.z);
  }
  void header(const ref::Header& h) {
    f(" hdr %u", h.seq);
    t(h.stamp);
    f(" %s", h.frame_id.empty() ? "-" : h.frame_id.c_str());
  }
  void status(const ref::ReferenceStatus& m) {
    s += " status";
    header(m.header);
    f(" st %u fl %u id %u rev %u type %u", m.state, m.flags, m.active_trajectory_id, m.active_revision,
      m.active_type);
  }
  void analytic(const ref::AnalyticReference& m) {
    s += " analytic";
    header(m.header);
    f(" req %u id %u rev %u type %u fl %u", m.request_id, m.trajectory_id, m.revision, m.analytic_type, m.flags);
    t(m.start_time);
    d(m.duration);
    v3(m.origin.position);
    d(m.origin.orientation.x); d(m.origin.orientation.y); d(m.origin.orientation.z); d(m.origin.orientation.w);
    ds(m.params);
  }
  void sampled(const ref::SampledReference& m) {
    s += " sampled";
    header(m.header);
    f(" id %u rev %u fl %u", m.trajectory_id, m.revision, m.flags);
    t(m.start_time);
    d(m.sample_dt);
    f(" [%zu", m.points.size());
    for (const auto& p : m.points) {
      d(p.t_from_start);
      v3(p.position);
      v3(p.velocity); v3(p.acceleration); v3(p.jerk); v3(p.snap);
      d(p.yaw); d(p.yaw_rate); d(p.yaw_accel);
    }
    s += ']';
  }
};

struct RefTrajectory {
  const xgc_host_api* host{nullptr};
  mrt::ReferenceTrajectoryRuntime runtime;
  mrt::ReferenceTrajectoryConfig config;
  bool input_time{false};
  bool trace{false};

  // Clock state.
  bool started{false};
  double t0{0.0};
  uint64_t k{0};
  // Replay state.
  std::vector<Input> pending;
  uint64_t latest_ns{0};
  double clock_limit{-std::numeric_limits<double>::infinity()};

  uint8_t last_state{0xFF};
  uint32_t last_flags{0xFFFFFFFFu};
  std::tuple<int, uint32_t, uint32_t, uint64_t> last_active{-1, 0, 0, 0};
  std::string domain{"idle"};
  Text text;
  bool publish_failed{false};

  RefTrajectory() {
    config.limits.min_specific_thrust = 0.1;
  }

  void log(xgc_log_level level, const std::string& m) const { host->log(host->host, level, m.c_str()); }

  void publish(uint32_t port, uint64_t round, const std::vector<uint8_t>& bytes) {
    if (host->publish(host->host, port, round, bytes.data(), static_cast<uint32_t>(bytes.size())) != XGC_OK) {
      publish_failed = true;
    }
  }

  void flushTrace(uint64_t round) {
    if (trace && !text.s.empty()) {
      publish(kTrace, round, std::vector<uint8_t>(text.s.begin(), text.s.end()));
    }
    text.s.clear();
  }

  // ReferenceInputProducer::post, at the request's receive time.
  void post(uint32_t id, const char* source, double now) {
    sm::Event event(id, sm::EventTimestamp{now});
    event.source = source;
    const auto status = runtime.postEvent(std::move(event));
    if (trace) text.f("  post %u %s %s\n", id, source, status.ok() ? "ok" : status.message.c_str());
  }

  // One request, exactly as the node's input producer callback for it.
  void apply(const Input& in) {
    const double now = mrt::Time().fromNSec(in.t_ns).toSec();
    const uint32_t record_kind = in.port == kAnalytic ? 1U : in.port == kSampled ? 3U : 4U;
    if (trace) text.f("%" PRIu64 " in %u\n", k, record_kind);
    bool accepted = false;
    uint32_t event = 0;
    const char* source = "";
    switch (in.port) {
      case kAnalytic: {
        ref::AnalyticReference m;
        accepted = xgc_ref_wire::decode_analytic(in.data.data(), in.data.size(), m) && runtime.acceptAnalytic(m);
        event = mrt::event_type::ANALYTIC_RECEIVED;
        source = "analytic_reference";
        break;
      }
      case kSampled: {
        ref::SampledReference m;
        accepted = xgc_ref_wire::decode_sampled(in.data.data(), in.data.size(), m) && runtime.acceptSampled(m);
        event = mrt::event_type::SAMPLED_RECEIVED;
        source = "sampled_reference";
        break;
      }
      case kReset:
        runtime.reset();
        post(mrt::event_type::RESET_REQUESTED, "reset", now);
        return;
      default:
        return;
    }
    if (accepted) {
      post(event, source, now);
    } else if (trace) {
      text.f("  rejected\n");
    }
  }

  // The active evaluator, sampled as the harness does.
  void writeEvaluator() {
    const auto* evaluator = runtime.evaluator();
    text.f("%" PRIu64 " evaluator", k);
    if (evaluator == nullptr) {
      text.f(" none\n");
      return;
    }
    text.f(" type %d", static_cast<int>(evaluator->type()));
    text.d(evaluator->duration());
    text.f(" fl %u\n", evaluator->flags());
    const double span = std::isfinite(evaluator->duration()) ? std::min(evaluator->duration(), 20.0) : 20.0;
    for (int i = 0; static_cast<double>(i) * 0.05 <= span + 1e-9; ++i) {
      const double at = static_cast<double>(i) * 0.05;
      trajectory::FlatOutput3 f;
      const bool ok = evaluator->evaluate(at, f);
      text.f("  %d %d", i, ok ? 1 : 0);
      for (const auto* v : {&f.position, &f.velocity, &f.acceleration, &f.jerk, &f.snap}) {
        text.d(v->x()); text.d(v->y()); text.d(v->z());
      }
      text.d(f.yaw); text.d(f.yaw_rate); text.d(f.yaw_accel);
      text.f(" fl %u\n", f.flags);
    }
  }

  // One main-loop iteration at time t (ReferenceTrajectoryNode::run).
  void update(double t, uint64_t round) {
    const double now = mrt::Time(t).toSec();
    runtime.update(now);
    for (const auto& e : runtime.stateMachine().currentOutputEvents()) {
      if (trace) {
        text.f("%" PRIu64 " ev %u ts %016" PRIx64 " seq %" PRIu64 " cat %d src %s", k, static_cast<unsigned>(e.id),
               bits(e.timestamp), e.sequence, static_cast<int>(e.category), e.source.c_str());
      }
      // ReferenceOutputConsumer::handle
      if (e.id == mrt::output_event_type::PUBLISH_STATUS) {
        const auto status = runtime.makeStatus(e.timestamp > 0.0 ? e.timestamp : now);
        const xgc_ref_status_v1 wire = xgc_ref_wire::encode_status(status);
        const auto* b = reinterpret_cast<const uint8_t*>(&wire);
        publish(kStatus, round, std::vector<uint8_t>(b, b + sizeof wire));
        if (trace) text.status(status);
      } else if (e.id == mrt::output_event_type::PUBLISH_ACTIVE_ANALYTIC) {
        publish(kActiveAnalytic, round, xgc_ref_wire::encode_analytic(runtime.activeAnalyticMessage()));
        if (trace) text.analytic(runtime.activeAnalyticMessage());

      } else if (e.id == mrt::output_event_type::PUBLISH_ACTIVE_SAMPLED) {
        publish(kActiveSampled, round, xgc_ref_wire::encode_sampled(runtime.activeSampledMessage()));
        if (trace) text.sampled(runtime.activeSampledMessage());
      }
      if (trace) text.s += '\n';
    }
    if (runtime.currentState() != last_state) {
      if (trace) text.f("%" PRIu64 " state %u\n", k, runtime.currentState());
      last_state = runtime.currentState();
      static const char* const kNames[] = {"", "SelfCheck", "Ready", "", "Active"};
      domain = last_state < 5 ? kNames[last_state] : "unknown";
    }
    if (runtime.flags() != last_flags) {
      if (trace) text.f("%" PRIu64 " flags %u\n", k, runtime.flags());
      last_flags = runtime.flags();
    }
    mrt::Time start;
    if (runtime.activeType() == trajectory::TrajectoryModelType::kAnalytic) {
      start = runtime.activeAnalyticMessage().start_time;
    } else if (runtime.activeType() == trajectory::TrajectoryModelType::kSampled) {
      start = runtime.activeSampledMessage().start_time;

    }
    const std::tuple<int, uint32_t, uint32_t, uint64_t> active{static_cast<int>(runtime.activeType()),
                                                               runtime.activeTrajectoryId(), runtime.activeRevision(),
                                                               start.toNSec()};
    if (active != last_active) {
      if (trace) writeEvaluator();
      last_active = active;
    }
    flushTrace(round);
    ++k;
  }

  void drain(std::vector<Input>& into) {
    xgc_sample_view v;
    for (uint32_t port : {kAnalytic, kSampled, kReset, kClock}) {
      while (host->next(host->host, port, &v) == XGC_OK) {
        if (port == kClock) {
          xgc_clock_v1 c;
          if (v.len == sizeof c) {
            std::memcpy(&c, v.data, sizeof c);
            clock_limit = std::max(clock_limit, c.seconds);
          }
          continue;
        }
        into.push_back(Input{static_cast<uint64_t>(v.t_produce), port, std::vector<uint8_t>(v.data, v.data + v.len)});
      }
    }
    std::stable_sort(into.begin(), into.end(), [](const Input& a, const Input& b) { return a.t_ns < b.t_ns; });
  }

  xgc_status step(const xgc_step_ctx* ctx) {
    if (input_time) {
      drain(pending);
      if (!pending.empty()) {
        latest_ns = std::max(latest_ns, pending.back().t_ns);
        if (!started) {
          t0 = std::floor(pending.front().t_ns * 1e-9 * 100.0) / 100.0;
          started = true;
        }
      }
      if (!started) return XGC_OK;
      size_t next = 0;
      for (;;) {
        const double t = t0 + static_cast<double>(k) * 0.01;
        // Safe once every request up to t has arrived: a later one has, or
        // the clock says the stream is done up to t.
        if (!(latest_ns * 1e-9 > t) && !(t <= clock_limit)) break;
        while (next < pending.size() && pending[next].t_ns * 1e-9 <= t) apply(pending[next++]);
        update(t, ctx->round);
      }
      pending.erase(pending.begin(), pending.begin() + static_cast<std::ptrdiff_t>(next));
    } else {
      std::vector<Input> batch;
      drain(batch);
      for (const auto& in : batch) apply(in);
      if (ctx->round_advanced != 0) update(static_cast<double>(ctx->now) * 1e-9, ctx->round);
      flushTrace(ctx->round);
    }
    if (publish_failed) {
      log(XGC_LOG_ERROR, "ref-trajectory: an output publish failed");
      return XGC_ERR;
    }
    return XGC_OK;
  }
};

template <typename F>
xgc_status guarded(const xgc_host_api* host, const char* where, F&& f) {
  try {
    return f();
  } catch (const std::exception& e) {
    host->log(host->host, XGC_LOG_ERROR, (std::string(where) + ": " + e.what()).c_str());
  } catch (...) {
    host->log(host->host, XGC_LOG_ERROR, (std::string(where) + ": unknown exception").c_str());
  }
  return XGC_ERR;
}

void* create(const xgc_host_api* host) {
  try {
    auto* self = new RefTrajectory();
    self->host = host;
    return self;
  } catch (...) {
    return nullptr;
  }
}

xgc_status configure(void* p, const char* config) {
  auto* self = static_cast<RefTrajectory*>(p);
  return guarded(self->host, "configure", [&] {
    namespace cfg = xgc_rt_config;
    const std::string t = config ? config : "";
    mrt::ReferenceTrajectoryConfig& c = self->config;
    const std::string source = cfg::text_or(t, "time_source", "session");
    const bool ok = cfg::boolean(t, "trace", &self->trace) &&
                    cfg::number(t, "status_rate", &c.status_rate_hz) &&
                    cfg::number(t, "active_publish_rate", &c.active_publish_rate_hz) &&
                    cfg::number(t, "validation_sample_dt", &c.validation_sample_dt) &&
                    cfg::number(t, "trajectory_timeout", &c.trajectory_timeout) &&
                    cfg::number(t, "min_lead_time", &c.min_lead_time) &&
                    cfg::number(t, "max_velocity", &c.limits.max_velocity) &&
                    cfg::number(t, "max_acceleration", &c.limits.max_acceleration) &&
                    cfg::number(t, "max_jerk", &c.limits.max_jerk) && cfg::number(t, "max_snap", &c.limits.max_snap) &&
                    cfg::number(t, "min_specific_thrust", &c.limits.min_specific_thrust) &&
                    (source == "session" || source == "input");
    if (!ok) {
      self->log(XGC_LOG_ERROR, "invalid ref-trajectory config");
      return XGC_ERR;
    }
    self->input_time = source == "input";
    self->runtime.setConfig(c);
    return XGC_OK;
  });
}

xgc_status activate(void*) { return XGC_OK; }

xgc_status step(void* p, const xgc_step_ctx* ctx) {
  auto* self = static_cast<RefTrajectory*>(p);
  return guarded(self->host, "step", [&] { return self->step(ctx); });
}

xgc_status deactivate(void*) { return XGC_OK; }

void destroy(void* p) { delete static_cast<RefTrajectory*>(p); }

const char* domain_state(void* p) { return static_cast<RefTrajectory*>(p)->domain.c_str(); }

const xgc_port_decl kPorts[kPortCount] = {
    {"analytic", XGC_PORT_IN_OPTIONAL, "xgc.ref.analytic/1", XGC_QOS_EVENT},
    {"sampled", XGC_PORT_IN_OPTIONAL, "xgc.ref.sampled/1", XGC_QOS_EVENT},
    {"reset", XGC_PORT_IN_OPTIONAL, "xgc.ref.reset/1", XGC_QOS_EVENT},
    {"clock", XGC_PORT_IN_OPTIONAL, "xgc.clock/1", XGC_QOS_EVENT},
    {"status", XGC_PORT_OUT, "xgc.ref.status/1", XGC_QOS_STATE},
    {"active_analytic", XGC_PORT_OUT_OPTIONAL, "xgc.ref.analytic/1", XGC_QOS_STATE},
    {"active_sampled", XGC_PORT_OUT_OPTIONAL, "xgc.ref.sampled/1", XGC_QOS_STATE},
    {"trace", XGC_PORT_OUT_OPTIONAL, "xgc.text/1", XGC_QOS_BULK},
};

const xgc_plugin_vtbl kVtbl = {create, configure, activate, step, deactivate, destroy, domain_state};

const xgc_plugin_descriptor kDescriptor = {XGC_RT_ABI_VERSION, kPortCount, "ref-trajectory", "0.1.0", kPorts, &kVtbl};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {
  return &kDescriptor;
}
