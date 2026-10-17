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
  // Two-state chain per directed link: the bad state drops everything, the good state nothing.
  // With mean loss p and mean burst length L, P(bad to good) = 1/L and
  // P(good to bad) = p / (L * (1 - p)), which gives a stationary bad-state probability of p.
  kBurst,
};

struct ChannelConfig {
  // Ticks between send and earliest delivery. Must be at least 1, so no agent can observe
  // another agent's step within the same tick.
  std::uint32_t latency_ticks = 1;
  std::uint64_t seed = 0;
  LossModel loss_model = LossModel::kNone;
  // Mean probability that a message is dropped, in [0, 1]. Ignored when loss_model is kNone.
  double loss = 0.0;
  // Mean length, in messages on one link, of a run of consecutive drops. Used by kBurst only.
  // Must be at least 1, and at least loss / (1 - loss) when loss < 1. Below that bound the
  // good-to-bad probability would exceed 1, so the requested mean loss cannot be reached.
  double mean_burst = 1.0;
  // Each delivered message gets an extra delay drawn uniformly from [0, jitter_ticks].
  std::uint32_t jitter_ticks = 0;
  // Most messages one sender may send in a single tick; the rest are dropped. 0 means no limit.
  std::uint32_t max_msgs_per_sender_per_tick = 0;
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
  bool should_drop(std::uint32_t sender, std::uint32_t receiver);

  std::uint32_t num_endpoints_;
  ChannelConfig config_;
  Rng loss_rng_;
  Rng jitter_rng_;
  std::uint64_t rate_tick_ = 0;
  bool rate_tick_valid_ = false;
  std::vector<std::uint32_t> sent_this_tick_;  // per sender, for the rate limit
  double p_good_to_bad_ = 0.0;
  double p_bad_to_good_ = 1.0;
  std::vector<std::uint8_t> link_bad_;  // index sender * num_endpoints + receiver
  DeliveryQueue queue_;
  std::vector<std::uint64_t> next_seq_;
  // Sized once at construction and never resized, so take_inbox() is safe from different
  // threads for different receivers.
  std::vector<std::vector<Message>> inboxes_;
  ChannelStats stats_;
};

}  // namespace swarm
