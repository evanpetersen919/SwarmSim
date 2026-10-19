#include "swarm/net/tcp_connections.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace swarm::net {

namespace {

constexpr std::size_t kReadChunk = 64 * 1024;

}  // namespace

TcpConnections::TcpConnections(std::uint32_t count) {
  if (count == 0 || count > kMaxEndpoints) {
    throw std::invalid_argument("TcpConnections: the number of endpoints must be between 1 and " +
                                std::to_string(kMaxEndpoints));
  }
  listeners_.reserve(count);
  addresses_.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    listeners_.push_back(TcpListener::listen_loopback(0));
    addresses_.push_back(listeners_.back().local_address());
  }
}

Address TcpConnections::address(std::uint32_t endpoint) const {
  if (endpoint >= addresses_.size()) {
    throw std::out_of_range("TcpConnections: no such endpoint");
  }
  return addresses_[endpoint];
}

void TcpConnections::check_endpoints(std::uint32_t from, std::uint32_t to) const {
  if (from >= listeners_.size() || to >= listeners_.size()) {
    throw std::out_of_range("TcpConnections: no such endpoint");
  }
}

TcpStream& TcpConnections::outbound(std::uint32_t from, std::uint32_t to) {
  const auto key = std::make_pair(from, to);
  auto it = outbound_.find(key);
  if (it == outbound_.end()) {
    TcpStream stream = TcpStream::connect(addresses_[to]);
    ++stats_.connections_opened;
    it = outbound_.emplace(key, std::move(stream)).first;
    // Accept it at once, so that many connections to one listener never pile up past its backlog.
    pump(0);
  }
  return it->second;
}

void TcpConnections::send(const Message& message) {
  check_endpoints(message.sender, message.receiver);
  const std::vector<std::uint8_t> bytes = encode_message(message);  // may throw invalid_argument
  send_raw(message.sender, message.receiver, bytes.data(), bytes.size());
}

void TcpConnections::send_raw(std::uint32_t from, std::uint32_t to, const std::uint8_t* data,
                              std::size_t size) {
  check_endpoints(from, to);
  for (int attempt = 0; attempt < 2; ++attempt) {
    TcpStream& stream = outbound(from, to);
    if (stream.send_all(data, size) == IoStatus::kOk) {
      ++stats_.sent;
      if (++sends_since_drain_ >= kDrainEvery) {
        sends_since_drain_ = 0;
        pump(0);
      }
      return;
    }
    outbound_.erase(std::make_pair(from, to));  // the peer closed it: open a new one and retry
  }
  throw NetError("TcpConnections: the connection was closed twice in a row", 0);
}

void TcpConnections::close_connection(std::uint32_t from, std::uint32_t to) {
  check_endpoints(from, to);
  outbound_.erase(std::make_pair(from, to));
}

void TcpConnections::read_inbound(Inbound& in) {
  std::vector<std::uint8_t> buffer(kReadChunk);
  for (;;) {
    std::size_t got = 0;
    const IoStatus status = in.stream.receive_some(buffer.data(), buffer.size(), got, 0);
    if (status == IoStatus::kTimeout) {
      return;
    }
    if (status == IoStatus::kClosed) {
      ++stats_.closed;
      in.dead = true;
      return;
    }
    if (in.decoder.feed(buffer.data(), got) != WireStatus::kOk) {
      ++stats_.broken;  // the messages completed before the corruption are still taken below
      in.dead = true;
    }
    for (Message& message : in.decoder.take()) {
      if (message.receiver != in.endpoint) {
        ++stats_.misdelivered;
        continue;
      }
      if (!in.peer) {
        in.peer = message.sender;
      }
      if (message.sender != *in.peer) {
        ++stats_.misattributed;
        continue;
      }
      ++stats_.received;
      arrived_.push_back(std::move(message));
    }
    if (in.dead) {
      return;
    }
  }
}

void TcpConnections::pump(int timeout_ms) {
  int timeout = timeout_ms;
  for (;;) {
    std::vector<const Socket*> watched;
    watched.reserve(listeners_.size() + inbound_.size());
    for (const TcpListener& l : listeners_) {
      watched.push_back(&l);
    }
    for (const Inbound& in : inbound_) {
      watched.push_back(&in.stream);
    }
    const std::vector<std::size_t> ready = wait_readable(watched, timeout);
    if (ready.empty()) {
      break;
    }
    timeout = 0;

    // Streams first, by position, then new connections; accepting appends to inbound_, which
    // must not happen while it is being walked by index of an earlier snapshot.
    std::vector<std::size_t> streams;
    std::vector<std::size_t> listeners;
    for (std::size_t position : ready) {
      (position < listeners_.size() ? listeners : streams).push_back(position);
    }
    for (std::size_t position : streams) {
      read_inbound(inbound_[position - listeners_.size()]);
    }
    for (std::size_t position : listeners) {
      while (std::optional<TcpStream> accepted = listeners_[position].accept(0)) {
        Inbound in;
        in.endpoint = static_cast<std::uint32_t>(position);
        in.stream = std::move(*accepted);
        inbound_.push_back(std::move(in));
      }
    }
    inbound_.erase(
        std::remove_if(inbound_.begin(), inbound_.end(), [](const Inbound& in) { return in.dead; }),
        inbound_.end());
  }
}

std::vector<Message> TcpConnections::receive(int timeout_ms) {
  pump(arrived_.empty() ? timeout_ms : 0);
  std::vector<Message> out = std::move(arrived_);
  arrived_.clear();
  return out;
}

}  // namespace swarm::net
