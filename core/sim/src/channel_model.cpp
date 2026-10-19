#include "swarm/channel_model.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

#include "swarm/streams.hpp"

namespace swarm {

ChannelModel::ChannelModel(std::uint32_t num_endpoints, ChannelConfig config)
    : num_endpoints_(num_endpoints),
      config_(config),
      loss_rng_(config.seed, kStreamChannelLoss),
      jitter_rng_(config.seed, kStreamChannelJitter),
      jam_rng_(config.seed, kStreamChannelJam),
      sent_this_tick_(num_endpoints, 0) {
  if (config_.latency_ticks < 1) {
    throw std::invalid_argument("DeterministicChannel: latency_ticks must be at least 1");
  }
  for (const Jammer& jammer : config_.jammers) {
    validate(jammer);
  }
  if (config_.jitter_ticks == std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("DeterministicChannel: jitter_ticks is too large");
  }
  if (config_.loss_model != LossModel::kNone && !(config_.loss >= 0.0 && config_.loss <= 1.0)) {
    throw std::invalid_argument("DeterministicChannel: loss must be in [0, 1]");
  }

  if (config_.loss_model == LossModel::kBurst) {
    const double p = config_.loss;
    const double burst = config_.mean_burst;
    if (!(burst >= 1.0)) {
      throw std::invalid_argument("DeterministicChannel: mean_burst must be at least 1");
    }
    if (p >= 1.0) {
      p_good_to_bad_ = 1.0;
      p_bad_to_good_ = 0.0;
    } else {
      p_bad_to_good_ = 1.0 / burst;
      p_good_to_bad_ = p * p_bad_to_good_ / (1.0 - p);
      if (p_good_to_bad_ > 1.0) {
        throw std::invalid_argument(
            "DeterministicChannel: mean_burst too short for the requested loss; need "
            "mean_burst >= loss / (1 - loss)");
      }
    }

    // Start every link in its stationary distribution so early messages are not biased.
    link_bad_.assign(static_cast<std::size_t>(num_endpoints) * num_endpoints, 0);
    for (auto& bad : link_bad_) {
      bad = loss_rng_.bernoulli(p) ? 1 : 0;
    }
  }
}

bool ChannelModel::should_drop(std::uint32_t sender, std::uint32_t receiver) {
  switch (config_.loss_model) {
    case LossModel::kNone:
      return false;
    case LossModel::kIndependent:
      return loss_rng_.bernoulli(config_.loss);
    case LossModel::kBurst: {
      auto& bad = link_bad_[static_cast<std::size_t>(sender) * num_endpoints_ + receiver];
      const double u = loss_rng_.uniform();
      if (bad != 0) {
        if (u < p_bad_to_good_) {
          bad = 0;
        }
      } else if (u < p_good_to_bad_) {
        bad = 1;
      }
      return bad != 0;
    }
  }
  return false;
}

bool ChannelModel::jammed(const Message& msg, std::uint64_t now_tick) {
  if (config_.jammers.empty()) {
    return false;
  }
  if (positions_.size() != num_endpoints_) {
    throw std::logic_error("DeterministicChannel: jammers need set_positions() every tick");
  }
  for (const Jammer& jammer : config_.jammers) {
    if (!jammer_active(jammer, now_tick)) {
      continue;
    }
    bool covered = jammer_covers(jammer, positions_[msg.receiver]);
    if (!covered && jammer.affects == JamAffects::kEither) {
      covered = jammer_covers(jammer, positions_[msg.sender]);
    }
    if (!covered) {
      continue;
    }
    // A jammer with loss 1 draws nothing, so it never moves the jamming stream.
    if (jammer.loss_inside >= 1.0 || jam_rng_.uniform() < jammer.loss_inside) {
      return true;
    }
  }
  return false;
}

ChannelModel::Decision ChannelModel::decide(std::uint64_t now_tick, const Message& msg) {
  if (msg.sender >= num_endpoints_ || msg.receiver >= num_endpoints_) {
    throw std::out_of_range("DeterministicChannel: endpoint id out of range");
  }

  // A reliable message is not dropped, is not counted against the rate limit, and draws no random
  // numbers, so it cannot change what happens to any beacon.
  if (msg.type == MessageType::kCommand) {
    return {Outcome::kScheduled, now_tick + config_.latency_ticks};
  }

  // Rate limit first, and without any random draw, so limited messages never move the loss or
  // jitter streams.
  if (config_.max_msgs_per_sender_per_tick > 0) {
    if (!rate_tick_valid_ || rate_tick_ != now_tick) {
      std::fill(sent_this_tick_.begin(), sent_this_tick_.end(), 0U);
      rate_tick_ = now_tick;
      rate_tick_valid_ = true;
    }
    if (sent_this_tick_[msg.sender] >= config_.max_msgs_per_sender_per_tick) {
      return {Outcome::kDroppedRate, 0};
    }
    ++sent_this_tick_[msg.sender];
  }

  if (should_drop(msg.sender, msg.receiver)) {
    return {Outcome::kDroppedLoss, 0};
  }

  if (jammed(msg, now_tick)) {
    return {Outcome::kDroppedJam, 0};
  }

  // Jitter is drawn only for messages that survive, so its stream position depends on which
  // messages were delivered and on nothing else.
  std::uint64_t delay = config_.latency_ticks;
  if (config_.jitter_ticks > 0) {
    delay += jitter_rng_.below(config_.jitter_ticks + 1U);
  }
  return {Outcome::kScheduled, now_tick + delay};
}

}  // namespace swarm
