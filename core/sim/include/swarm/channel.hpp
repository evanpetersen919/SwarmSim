#pragma once

#include <cstdint>
#include <vector>

#include "swarm/message.hpp"

namespace swarm {

struct ChannelStats {
  std::uint64_t sent = 0;
  std::uint64_t delivered = 0;
  std::uint64_t dropped_loss = 0;
  std::uint64_t dropped_rate = 0;
};

// Message transport between endpoints (drones). Threading contract:
//   send() and deliver() are called only from the single-threaded barrier step, in a fixed order.
//   take_inbox() may be called concurrently, provided each thread uses a different receiver.
class IChannel {
 public:
  virtual ~IChannel() = default;

  // Accepts a message sent during tick `now_tick`. Assigns seq and deliver_tick, and may drop it.
  virtual void send(std::uint64_t now_tick, Message msg) = 0;

  // Moves every message due at or before `tick` into the receivers' inboxes.
  virtual void deliver(std::uint64_t tick) = 0;

  // Removes and returns the messages waiting for `receiver`, in delivery order.
  virtual std::vector<Message> take_inbox(std::uint32_t receiver) = 0;

  virtual const ChannelStats& stats() const = 0;
};

}  // namespace swarm
