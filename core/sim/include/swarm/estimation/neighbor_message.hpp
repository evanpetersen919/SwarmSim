#pragma once

// Estimator headers may include only this directory, sensor_data.hpp, swarm/format/byte_io.hpp, and
// vector types. They must never include vehicle truth. Enforced by
// scripts/check_core_invariants.sh.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "swarm/vec3.hpp"

namespace swarm {

// What a drone tells its neighbors about itself: its own estimate, never ground truth. The sender's
// identity comes from the message envelope, not the payload.
struct NeighborReport {
  std::uint64_t send_tick = 0;  // the tick at which the sender made the estimate
  Vec3 position;                // meters, world frame
  Vec3 velocity;                // m/s, world frame
  double position_sigma = 0.0;  // the sender's per-axis position uncertainty, meters
};

inline constexpr std::uint8_t kNeighborReportVersion = 1;
inline constexpr std::size_t kNeighborReportBytes = 1 + 8 + 6 * 8 + 8;  // 65

// Layout, little-endian: u8 version | u64 send_tick | f64 position[3] | f64 velocity[3] |
// f64 position_sigma. Throws std::invalid_argument if a value is not finite or the sigma is
// negative.
std::vector<std::uint8_t> encode_neighbor_report(const NeighborReport& report);

enum class DecodeStatus {
  kOk,
  kBadSize,
  kBadVersion,
  kInvalidValue,  // a value is not finite, or the sigma is negative
};

DecodeStatus decode_neighbor_report(const std::uint8_t* data, std::size_t size,
                                    NeighborReport& out);

}  // namespace swarm
