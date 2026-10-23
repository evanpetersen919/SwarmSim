#pragma once

#include <cstdint>
#include <vector>

namespace swarm {

// What a message is for, and so how it travels. Beacons are small periodic state reports that may
// be lost; commands must arrive, once, in order; hazard reports are lost like beacons. The numbers
// are the wire values.
enum class MessageType : std::uint8_t {
  kBeacon = 1,
  kCommand = 2,
  // What an aircraft has seen of the ground ahead (ADR 0016): lossy like a beacon.
  kHazard = 3,
};

// A unit of communication between two endpoints. `send_tick`, `deliver_tick`, and `seq` are set by
// the channel when the message is sent; callers fill in sender, receiver, type, and payload.
struct Message {
  MessageType type = MessageType::kBeacon;
  std::uint64_t send_tick = 0;
  std::uint64_t deliver_tick = 0;
  std::uint32_t sender = 0;
  std::uint32_t receiver = 0;
  std::uint64_t seq = 0;
  std::vector<std::uint8_t> payload;
};

}  // namespace swarm
