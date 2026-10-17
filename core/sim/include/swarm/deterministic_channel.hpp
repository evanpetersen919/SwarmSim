#pragma once

#include <cstdint>
#include <vector>

#include "swarm/channel.hpp"
#include "swarm/delivery_queue.hpp"
#include "swarm/rng.hpp"

namespace swarm {

enum class LossModel {
  kNone,
  kIndependent,  // each message is dropped with probability `loss`
};

struct ChannelConfig {
  // Ticks between send and earliest delivery. Must be at least 1, so no agent can observe
  // another agent's step within the same tick.
  std::uint32_t latency_ticks = 1;
  std::uint64_t seed = 0;
  LossModel loss_model = LossModel::kNone;
  // Mean probability that a message is dropped, in [0, 1]. Ignored when loss_model is kNone.
  double loss = 0.0;
};

// In-process channel whose behavior depends only on its configuration, its seed, and the order
// of calls. This is the channel used for benchmarks.
class DeterministicChannel : public IChannel {
 public:
  DeterministicChannel(std::uint32_t num_endpoints, ChannelConfig config);

  void send(std::uint64_t now_tick, Message msg) override;
  void deliver(std::uint64_t tick) override;
  std::vector<Message> take_inbox(std::uint32_t receiver) override;
  const ChannelStats& stats() const override { return stats_; }

 private:
  std::uint32_t num_endpoints_;
  ChannelConfig config_;
  Rng loss_rng_;
  DeliveryQueue queue_;
  std::vector<std::uint64_t> next_seq_;
  // Sized once at construction and never resized, so take_inbox() is safe from different
  // threads for different receivers.
  std::vector<std::vector<Message>> inboxes_;
  ChannelStats stats_;
};

}  // namespace swarm
