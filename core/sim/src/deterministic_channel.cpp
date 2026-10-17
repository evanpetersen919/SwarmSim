#include "swarm/deterministic_channel.hpp"

#include <stdexcept>
#include <utility>

namespace swarm {

DeterministicChannel::DeterministicChannel(std::uint32_t num_endpoints, ChannelConfig config)
    : num_endpoints_(num_endpoints),
      config_(config),
      next_seq_(num_endpoints, 0),
      inboxes_(num_endpoints) {
  if (config_.latency_ticks < 1) {
    throw std::invalid_argument("DeterministicChannel: latency_ticks must be at least 1");
  }
}

void DeterministicChannel::send(std::uint64_t now_tick, Message msg) {
  if (msg.sender >= num_endpoints_ || msg.receiver >= num_endpoints_) {
    throw std::out_of_range("DeterministicChannel: endpoint id out of range");
  }
  ++stats_.sent;
  msg.seq = next_seq_[msg.sender]++;
  msg.deliver_tick = now_tick + config_.latency_ticks;
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
