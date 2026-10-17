#include "swarm/deterministic_channel.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include "swarm/streams.hpp"

namespace swarm {

DeterministicChannel::DeterministicChannel(std::uint32_t num_endpoints, ChannelConfig config)
    : num_endpoints_(num_endpoints),
      config_(config),
      loss_rng_(config.seed, kStreamChannelLoss),
      jitter_rng_(config.seed, kStreamChannelJitter),
      sent_this_tick_(num_endpoints, 0),
      next_seq_(num_endpoints, 0),
      inboxes_(num_endpoints) {
  if (config_.latency_ticks < 1) {
    throw std::invalid_argument("DeterministicChannel: latency_ticks must be at least 1");
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

bool DeterministicChannel::should_drop(std::uint32_t sender, std::uint32_t receiver) {
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

void DeterministicChannel::send(std::uint64_t now_tick, Message msg) {
  if (msg.sender >= num_endpoints_ || msg.receiver >= num_endpoints_) {
    throw std::out_of_range("DeterministicChannel: endpoint id out of range");
  }
  ++stats_.sent;
  // seq is consumed even if the message is dropped, so it reflects send order.
  msg.seq = next_seq_[msg.sender]++;

  // Rate limit first, and without any random draw, so limited messages never move the loss or
  // jitter streams.
  if (config_.max_msgs_per_sender_per_tick > 0) {
    if (!rate_tick_valid_ || rate_tick_ != now_tick) {
      std::fill(sent_this_tick_.begin(), sent_this_tick_.end(), 0U);
      rate_tick_ = now_tick;
      rate_tick_valid_ = true;
    }
    if (sent_this_tick_[msg.sender] >= config_.max_msgs_per_sender_per_tick) {
      ++stats_.dropped_rate;
      return;
    }
    ++sent_this_tick_[msg.sender];
  }

  if (should_drop(msg.sender, msg.receiver)) {
    ++stats_.dropped_loss;
    return;
  }

  // Jitter is drawn only for messages that survive, so its stream position depends on which
  // messages were delivered and on nothing else.
  std::uint64_t delay = config_.latency_ticks;
  if (config_.jitter_ticks > 0) {
    delay += jitter_rng_.below(config_.jitter_ticks + 1U);
  }
  msg.deliver_tick = now_tick + delay;
  queue_.push(std::move(msg));
}

void DeterministicChannel::deliver(std::uint64_t tick) {
  for (auto& msg : queue_.pop_ready(tick)) {
    ++stats_.delivered;
    inboxes_[msg.receiver].push_back(std::move(msg));
  }
}

std::vector<Message> DeterministicChannel::take_inbox(std::uint32_t receiver) {
  if (receiver >= num_endpoints_) {
    throw std::out_of_range("DeterministicChannel: endpoint id out of range");
  }
  std::vector<Message> out = std::move(inboxes_[receiver]);
  inboxes_[receiver].clear();
  return out;
}

}  // namespace swarm
