#pragma once

#include <cstdint>

namespace swarm {

// RNG stream ids. All streams share the run seed and differ by id, so turning one subsystem on
// or off does not shift the draws of another. Add new ids here, never inline.
inline constexpr std::uint64_t kStreamChannelLoss = 0x100;
inline constexpr std::uint64_t kStreamChannelJitter = 0x101;
inline constexpr std::uint64_t kStreamAgentBase = 0x1000;  // agent i uses kStreamAgentBase + i

}  // namespace swarm
