#include "swarm/simulation.hpp"

#include <utility>

#include "swarm/format/hash.hpp"
#include "swarm/streams.hpp"

namespace swarm {

namespace {

ChannelConfig channel_config_for(const SimConfig& config) {
  ChannelConfig channel = config.channel;
  channel.seed = config.seed;
  return channel;
}

}  // namespace

Simulation::Simulation(const SimConfig& config, std::vector<std::unique_ptr<IAgent>> agents,
                       std::size_t threads)
    : config_(config),
      clock_(config.dt),
      channel_(static_cast<std::uint32_t>(agents.size()), channel_config_for(config)),
      pool_(threads),
      agents_(std::move(agents)),
      outboxes_(agents_.size()) {
  rngs_.reserve(agents_.size());
  for (std::size_t i = 0; i < agents_.size(); ++i) {
    rngs_.emplace_back(config_.seed, kStreamAgentBase + i);
  }

  Fnv1a64 initial;
  initial.update_u64(config_.seed);
  initial.update_u64(agents_.size());
  hash_ = initial.digest();
}

void Simulation::step() {
  const std::uint64_t tick = clock_.tick();
  const double time = clock_.time();

  channel_.deliver(tick);

  pool_.for_each(agents_.size(), [&](std::size_t i) {
    const auto id = static_cast<std::uint32_t>(i);
    outboxes_[i].clear();
    AgentContext ctx{tick, time, id, rngs_[i], channel_.take_inbox(id), outboxes_[i]};
    agents_[i]->step(ctx);
  });

  for (std::size_t sender = 0; sender < agents_.size(); ++sender) {
    for (auto& msg : outboxes_[sender]) {
      msg.sender = static_cast<std::uint32_t>(sender);
      channel_.send(tick, std::move(msg));
    }
  }

  Fnv1a64 hash;
  hash.update_u64(hash_);
  hash.update_u64(tick);
  for (const auto& agent : agents_) {
    agent->hash_state(hash);
  }
  const ChannelStats& stats = channel_.stats();
  hash.update_u64(stats.sent);
  hash.update_u64(stats.delivered);
  hash.update_u64(stats.dropped_loss);
  hash.update_u64(stats.dropped_rate);
  hash_ = hash.digest();

  clock_.advance();
}

}  // namespace swarm
