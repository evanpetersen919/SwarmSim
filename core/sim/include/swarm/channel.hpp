#pragma once

#include <cstdint>
#include <vector>

#include "swarm/message.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

struct ChannelStats {
  std::uint64_t sent = 0;
  std::uint64_t delivered = 0;
  std::uint64_t dropped_loss = 0;
  std::uint64_t dropped_rate = 0;
  std::uint64_t dropped_jam = 0;
};

// What the channel decided for one message, recorded when an event sink is set.
enum class MessageOutcome : std::uint8_t {
  kScheduled = 1,    // will be delivered at deliver_tick
  kDroppedLoss = 2,  // the loss model dropped it
  kDroppedRate = 3,  // the per-sender rate limit dropped it
  kDroppedJam = 4,   // a jammer dropped it
};

struct ChannelEvent {
  MessageOutcome outcome = MessageOutcome::kScheduled;
  std::uint32_t sender = 0;
  std::uint32_t receiver = 0;
  std::uint64_t send_tick = 0;
  std::uint64_t deliver_tick = 0;  // 0 for a dropped message
};

// Message transport between endpoints (drones). Threading contract:
//   send() and deliver() are called only from the single-threaded barrier step, in a fixed order.
//   take_inbox() may be called concurrently, provided each thread uses a different receiver.
class IChannel {
 public:
  virtual ~IChannel() = default;

  // Accepts a message sent during tick `now_tick`. Assigns seq and deliver_tick, and may drop it.
  virtual void send(std::uint64_t now_tick, Message msg) = 0;

  // Ground-truth endpoint positions, indexed by endpoint id, used to decide which links a jammer
  // covers. Called from the barrier step before the tick's messages are sent. Channels without
  // jammers ignore them.
  virtual void set_positions(const std::vector<Vec3>& positions) = 0;

  // Moves every message due at or before `tick` into the receivers' inboxes.
  virtual void deliver(std::uint64_t tick) = 0;

  // Removes and returns the messages waiting for `receiver`, in delivery order.
  virtual std::vector<Message> take_inbox(std::uint32_t receiver) = 0;

  virtual const ChannelStats& stats() const = 0;

  // The number of endpoints (agents) the channel connects.
  virtual std::uint32_t endpoint_count() const = 0;

  // When set, send() appends one event per message, in send order. The vector must outlive the
  // channel or be cleared with set_event_sink(nullptr). With no sink the channel behaves exactly
  // as without one.
  virtual void set_event_sink(std::vector<ChannelEvent>* sink) = 0;
};

}  // namespace swarm
