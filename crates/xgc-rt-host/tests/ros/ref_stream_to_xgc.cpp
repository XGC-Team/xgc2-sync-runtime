// Test tool: convert the reference trajectory replay stream (ROS-serialized
// requests, from xgc2-multirotor-controller
// multirotor_reference_trajectory/test/replay/make_reference_stream.py)
// into ref-trajectory port payloads, the way ros_io does for live topics.
//
// Usage: ref_stream_to_xgc IN.stream OUT.xgcstream
//
// OUT: magic "XGCREFS1", then records u64 receive-time ns, u32 ref-trajectory
// port index, u32 length, payload (an xgc.ref.* schema). The receive time
// becomes the sample's envelope t_produce.

#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <vector>

#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>
#include <multirotor_reference_trajectory_msgs/WaypointReferenceRequest.h>
#include <ros/serialization.h>

#include "reference_wire.hpp"

namespace msgs = multirotor_reference_trajectory_msgs;

namespace {

template <typename M>
M decode(const std::vector<uint8_t>& d) {
  M m;
  ros::serialization::IStream s(const_cast<uint8_t*>(d.data()), static_cast<uint32_t>(d.size()));
  ros::serialization::deserialize(s, m);
  return m;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: ref_stream_to_xgc IN.stream OUT.xgcstream\n");
    return 2;
  }
  std::ifstream in(argv[1], std::ios::binary);
  std::ofstream out(argv[2], std::ios::binary);
  char magic[8];
  if (!in.read(magic, 8) || std::memcmp(magic, "MRTRPLY1", 8) != 0) throw std::runtime_error("not a reference stream");
  out.write("XGCREFS1", 8);
  auto emit = [&](uint64_t t, uint32_t port, const std::vector<uint8_t>& p) {
    const uint32_t n = static_cast<uint32_t>(p.size());
    out.write(reinterpret_cast<const char*>(&t), 8);
    out.write(reinterpret_cast<const char*>(&port), 4);
    out.write(reinterpret_cast<const char*>(&n), 4);
    out.write(reinterpret_cast<const char*>(p.data()), n);
  };
  long count = 0;
  for (;;) {
    uint64_t t = 0; uint8_t kind = 0; uint32_t len = 0;
    if (!in.read(reinterpret_cast<char*>(&t), 8)) break;
    in.read(reinterpret_cast<char*>(&kind), 1);
    in.read(reinterpret_cast<char*>(&len), 4);
    std::vector<uint8_t> d(len);
    if (len > 0 && !in.read(reinterpret_cast<char*>(d.data()), len)) throw std::runtime_error("truncated stream");
    switch (kind) {
      case 1: emit(t, 0, xgc_ref_wire::encode_analytic(decode<msgs::AnalyticReference>(d))); break;
      case 2: emit(t, 1, xgc_ref_wire::encode_waypoint_request(decode<msgs::WaypointReferenceRequest>(d))); break;
      case 3: emit(t, 2, xgc_ref_wire::encode_sampled(decode<msgs::SampledReference>(d))); break;
      case 4: {
        const xgc_ref_reset_v1 r{};
        const auto* b = reinterpret_cast<const uint8_t*>(&r);
        emit(t, 3, std::vector<uint8_t>(b, b + sizeof r));
        break;
      }
      default:
        throw std::runtime_error("unknown record kind");
    }
    ++count;
  }
  std::printf("%ld records\n", count);
  return 0;
}
