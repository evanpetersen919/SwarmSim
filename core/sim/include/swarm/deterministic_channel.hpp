#pragma once

#include <cstdint>
#include <vector>

#include "swarm/channel.hpp"
#include "swarm/channel_model.hpp"
#include "swarm/delivery_queue.hpp"

namespace swarm {

// In-process channel whose behavior depends only on its configuration, its seed, and the order
// of calls. This is the channel used for benchmarks. What happens to each message is decided by a
// ChannelModel; this class holds the messages until their delivery tick.
class DeterministicChannel : public IChannel {
 public:
  DeterministicChannel(std::uint32_t num_endpoints, ChannelConfig config);

  void send(std::uint64_t now_tick, Message msg) override;
  void set_positions(const std::vector<Vec3>& positions) override {
    model_.set_positions(positions);
  }
  void deliver(std::uint64_t tick) override;
  std::vector<Message> take_inbox(std::uint32_t receiver) override;
  const ChannelStats& stats() const override { return stats_; }
  std::uint32_t endpoint_count() const override { return model_.endpoint_count(); }

  // When set, send() appends one event per message. The vector must outlive the channel or be
  // cleared with set_event_sink(nullptr). Events are in send order, which is fixed, so they are
  // identical for every thread count. With no sink the channel behaves exactly as before.
  void set_event_sink(std::vector<ChannelEvent>* sink) override { sink_ = sink; }

 private:
  void record(MessageOutcome outcome, const Message& msg);

  ChannelModel model_;
  DeliveryQueue queue_;
  std::vector<std::uint64_t> next_seq_;
  // Sized once at construction and never resized, so take_inbox() is safe from different
  // threads for different receivers.
  std::vector<std::vector<Message>> inboxes_;
  ChannelStats stats_;
  std::vector<ChannelEvent>* sink_ = nullptr;
};

}  // namespace swarm
