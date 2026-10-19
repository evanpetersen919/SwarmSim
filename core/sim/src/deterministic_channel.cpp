#include "swarm/deterministic_channel.hpp"

#include <stdexcept>
#include <utility>

namespace swarm {

DeterministicChannel::DeterministicChannel(std::uint32_t num_endpoints, ChannelConfig config)
    : model_(num_endpoints, std::move(config)),
      next_seq_(num_endpoints, 0),
      inboxes_(num_endpoints) {}

void DeterministicChannel::record(MessageOutcome outcome, const Message& msg) {
  if (sink_ != nullptr) {
    const std::uint64_t deliver = outcome == MessageOutcome::kScheduled ? msg.deliver_tick : 0;
    sink_->push_back({outcome, msg.sender, msg.receiver, msg.send_tick, deliver});
  }
}

void DeterministicChannel::send(std::uint64_t now_tick, Message msg) {
  if (msg.sender >= model_.endpoint_count() || msg.receiver >= model_.endpoint_count()) {
    throw std::out_of_range("DeterministicChannel: endpoint id out of range");
  }
  ++stats_.sent;
  // seq is consumed even if the message is dropped, so it reflects send order.
  msg.seq = next_seq_[msg.sender]++;
  msg.send_tick = now_tick;

  const ChannelModel::Decision decision = model_.decide(now_tick, msg);
  switch (decision.outcome) {
    case ChannelModel::Outcome::kDroppedRate:
      ++stats_.dropped_rate;
      record(MessageOutcome::kDroppedRate, msg);
      return;
    case ChannelModel::Outcome::kDroppedLoss:
      ++stats_.dropped_loss;
      record(MessageOutcome::kDroppedLoss, msg);
      return;
    case ChannelModel::Outcome::kDroppedJam:
      ++stats_.dropped_jam;
      record(MessageOutcome::kDroppedJam, msg);
      return;
    case ChannelModel::Outcome::kScheduled:
      break;
  }
  msg.deliver_tick = decision.deliver_tick;
  record(MessageOutcome::kScheduled, msg);
  queue_.push(std::move(msg));
}

void DeterministicChannel::deliver(std::uint64_t tick) {
  for (auto& msg : queue_.pop_ready(tick)) {
    ++stats_.delivered;
    inboxes_[msg.receiver].push_back(std::move(msg));
  }
}

std::vector<Message> DeterministicChannel::take_inbox(std::uint32_t receiver) {
  if (receiver >= model_.endpoint_count()) {
    throw std::out_of_range("DeterministicChannel: endpoint id out of range");
  }
  std::vector<Message> out = std::move(inboxes_[receiver]);
  inboxes_[receiver].clear();
  return out;
}

}  // namespace swarm
