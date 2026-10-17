#pragma once

#include <cstdint>
#include <vector>

namespace swarm {

// A unit of communication between two endpoints. `deliver_tick` and `seq` are set by the
// channel when the message is sent; callers fill in sender, receiver, and payload.
struct Message {
  std::uint64_t deliver_tick = 0;
  std::uint32_t sender = 0;
  std::uint32_t receiver = 0;
  std::uint64_t seq = 0;
  std::vector<std::uint8_t> payload;
};

}  // namespace swarm
