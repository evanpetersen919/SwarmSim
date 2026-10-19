#include "swarm/net/socket_channel.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

namespace swarm::net {

namespace {

std::uint32_t checked_endpoints(std::uint32_t n) {
  if (n == 0 || n > SocketChannel::kMaxEndpoints) {
    throw std::invalid_argument("SocketChannel: the number of endpoints must be between 1 and " +
                                std::to_string(SocketChannel::kMaxEndpoints));
  }
  return n;
}

}  // namespace

SocketChannel::SocketChannel(std::uint32_t num_endpoints, ChannelConfig config,
                             int receive_timeout_ms)
    : model_(checked_endpoints(num_endpoints), std::move(config)),
      udp_(num_endpoints),
      tcp_(num_endpoints),
      receive_timeout_ms_(receive_timeout_ms),
      next_seq_(num_endpoints, 0),
      inboxes_(num_endpoints) {
  if (receive_timeout_ms < 0) {
    throw std::invalid_argument("SocketChannel: the receive timeout must not be negative");
  }
}

void SocketChannel::record(MessageOutcome outcome, const Message& msg) {
  if (sink_ != nullptr) {
    const std::uint64_t deliver = outcome == MessageOutcome::kScheduled ? msg.deliver_tick : 0;
    sink_->push_back({outcome, msg.sender, msg.receiver, msg.send_tick, deliver});
  }
}

void SocketChannel::send(std::uint64_t now_tick, Message msg) {
  if (msg.sender >= model_.endpoint_count() || msg.receiver >= model_.endpoint_count()) {
    throw std::out_of_range("SocketChannel: endpoint id out of range");
  }
  if (msg.payload.size() > kMaxWirePayload) {
    // Refused before anything changes, so a message the wire cannot carry leaves no trace.
    throw std::invalid_argument("SocketChannel: the payload is longer than " +
                                std::to_string(kMaxWirePayload) + " bytes");
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

  // A placeholder with the message's ordering fields goes into the queue; the bytes go on a socket.
  Message placeholder;
  placeholder.type = msg.type;
  placeholder.sender = msg.sender;
  placeholder.receiver = msg.receiver;
  placeholder.seq = msg.seq;
  placeholder.send_tick = msg.send_tick;
  placeholder.deliver_tick = msg.deliver_tick;
  queue_.push(std::move(placeholder));
  pending_.insert({msg.sender, msg.seq});

  if (lose_ && lose_(msg)) {
    return;  // as if the network had lost it
  }
  if (msg.type == MessageType::kCommand) {
    tcp_.send(msg);
  } else {
    udp_.send(msg);
  }
}

void SocketChannel::absorb(std::vector<Message> arrived) {
  for (Message& message : arrived) {
    const Key key{message.sender, message.seq};
    if (pending_.count(key) == 0) {
      ++net_.unexpected;
      continue;
    }
    if (!arrived_.emplace(key, std::move(message)).second) {
      ++net_.duplicates;
    }
  }
}

void SocketChannel::poll_transports(int udp_wait_ms) {
  absorb(udp_.receive(udp_wait_ms));
  absorb(tcp_.receive(0));
}

void SocketChannel::deliver(std::uint64_t tick) {
  poll_transports(0);
  for (Message& slot : queue_.pop_ready(tick)) {
    const Key key{slot.sender, slot.seq};

    // Wait for the bytes if they are not here yet. Almost always they are.
    if (arrived_.find(key) == arrived_.end()) {
      const auto start = std::chrono::steady_clock::now();
      while (arrived_.find(key) == arrived_.end() &&
             std::chrono::steady_clock::now() - start <
                 std::chrono::milliseconds(receive_timeout_ms_)) {
        poll_transports(2);
      }
    }

    pending_.erase(key);
    const auto found = arrived_.find(key);
    if (found == arrived_.end()) {
      ++stats_.dropped_network;
      ++net_.timed_out;
      continue;
    }
    Message message = std::move(found->second);
    arrived_.erase(found);
    message.deliver_tick = slot.deliver_tick;
    ++stats_.delivered;
    inboxes_[message.receiver].push_back(std::move(message));
  }
}

std::vector<Message> SocketChannel::take_inbox(std::uint32_t receiver) {
  if (receiver >= model_.endpoint_count()) {
    throw std::out_of_range("SocketChannel: endpoint id out of range");
  }
  std::vector<Message> out = std::move(inboxes_[receiver]);
  inboxes_[receiver].clear();
  return out;
}

SocketChannel::NetStats SocketChannel::net_stats() const {
  NetStats out = net_;
  out.udp = udp_.stats();
  out.tcp = tcp_.stats();
  return out;
}

}  // namespace swarm::net
