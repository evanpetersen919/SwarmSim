#pragma once

#include <cstdint>
#include <vector>

#include "swarm/jammer.hpp"
#include "swarm/message.hpp"
#include "swarm/rng.hpp"
#include "swarm/vec3.hpp"

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
  // Jammers, checked in order after the base loss model. A message dropped by one jammer is not
  // offered to later ones. Evaluated at send time with the endpoints' positions at that tick.
  std::vector<Jammer> jammers;
};

// What happens to a message on a channel, and when: the rate limit, the loss model, the burst
// state, the jammers, and the latency and jitter, with their seeded random streams. It decides;
// it does not hold messages. The deterministic channel and the socket channel both use it, so the
// socket channel cannot drift from the simulated one.
//
// Messages are decided one at a time, in the order they are given, and the outcome depends only on
// the configuration, the seed, and that order.
//
// Reliable messages (MessageType::kCommand) model a transport that retransmits: the rate limit,
// the loss model, and the jammers do not drop them, and they are delivered after exactly the base
// latency. They do not touch the rate counters or draw from any random stream, so a run's
// beacons are the same with or without them. The model does not represent retransmission delay.
class ChannelModel {
 public:
  enum class Outcome : std::uint8_t {
    kScheduled,
    kDroppedLoss,
    kDroppedRate,
    kDroppedJam,
  };

  struct Decision {
    Outcome outcome = Outcome::kScheduled;
    std::uint64_t deliver_tick = 0;  // only for kScheduled
  };

  // Throws std::invalid_argument for an invalid configuration.
  ChannelModel(std::uint32_t num_endpoints, ChannelConfig config);

  // Ground-truth endpoint positions, indexed by endpoint id, used by the jammers. Must be set
  // every tick before messages are decided if the configuration has jammers.
  void set_positions(const std::vector<Vec3>& positions) { positions_ = positions; }

  // Throws std::out_of_range for an endpoint id outside the model, and std::logic_error if there
  // are jammers and the positions were not set.
  Decision decide(std::uint64_t now_tick, const Message& msg);

  std::uint32_t endpoint_count() const { return num_endpoints_; }

 private:
  bool should_drop(std::uint32_t sender, std::uint32_t receiver);
  bool jammed(const Message& msg, std::uint64_t now_tick);

  std::uint32_t num_endpoints_;
  ChannelConfig config_;
  Rng loss_rng_;
  Rng jitter_rng_;
  Rng jam_rng_;
  std::vector<Vec3> positions_;
  std::uint64_t rate_tick_ = 0;
  bool rate_tick_valid_ = false;
  std::vector<std::uint32_t> sent_this_tick_;  // per sender, for the rate limit
  double p_good_to_bad_ = 0.0;
  double p_bad_to_good_ = 1.0;
  std::vector<std::uint8_t> link_bad_;  // index sender * num_endpoints + receiver
};

}  // namespace swarm
