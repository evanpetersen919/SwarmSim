#pragma once

#include <cstdint>

namespace swarm {

// RNG stream ids. All streams share the run seed and differ by id, so turning one subsystem on
// or off does not shift the draws of another. Add new ids here, never inline.
inline constexpr std::uint64_t kStreamChannelLoss = 0x100;
inline constexpr std::uint64_t kStreamChannelJitter = 0x101;
inline constexpr std::uint64_t kStreamChannelJam = 0x103;
inline constexpr std::uint64_t kStreamScene = 0x104;
inline constexpr std::uint64_t kStreamStart = 0x105;     // where scenario drones start
inline constexpr std::uint64_t kStreamWind = 0x106;      // the gust field
inline constexpr std::uint64_t kStreamMapError = 0x107;  // the random error of a prior map

// Per-agent streams: agent i uses base + i. Each base has room for kMaxAgents ids, which keeps
// the ranges from colliding; Simulation rejects more agents than that.
inline constexpr std::uint64_t kStreamAgentBase = 0x1000;
inline constexpr std::uint64_t kStreamImuBase = 0x2000;
inline constexpr std::uint64_t kStreamLidarBase = 0x3000;
inline constexpr std::uint64_t kStreamFixBase = 0x4000;
inline constexpr std::uint64_t kStreamAltimeterBase = 0x5000;
inline constexpr std::uint64_t kStreamForwardBase = 0x6000;
inline constexpr std::uint32_t kMaxAgents = 4096;

}  // namespace swarm
