#include "swarm/net/udp_endpoints.hpp"

#include <stdexcept>
#include <utility>

namespace swarm::net {

namespace {

// Room for the largest message plus one byte, so that a datagram longer than a message is seen as
// truncated and is not mistaken for a valid one.
constexpr std::size_t kReceiveBuffer = kMaxWireMessageBytes + 1;

}  // namespace

UdpEndpoints::UdpEndpoints(std::uint32_t count, int receive_buffer) {
  if (count == 0 || count > kMaxEndpoints) {
    throw std::invalid_argument("UdpEndpoints: the number of endpoints must be between 1 and " +
                                std::to_string(kMaxEndpoints));
  }
  sockets_.reserve(count);
  addresses_.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    sockets_.push_back(UdpSocket::bind_loopback(0, receive_buffer));
    addresses_.push_back(sockets_.back().local_address());
  }
}

Address UdpEndpoints::address(std::uint32_t endpoint) const {
  if (endpoint >= addresses_.size()) {
    throw std::out_of_range("UdpEndpoints: no such endpoint");
  }
  return addresses_[endpoint];
}

void UdpEndpoints::send(const Message& message) {
  if (message.sender >= sockets_.size() || message.receiver >= sockets_.size()) {
    throw std::out_of_range("UdpEndpoints: no such endpoint");
  }
  const std::vector<std::uint8_t> bytes = encode_message(message);  // may throw invalid_argument
  sockets_[message.sender].send_to(addresses_[message.receiver], bytes.data(), bytes.size());
  ++stats_.sent;
  if (++sends_since_drain_ >= kDrainEvery) {
    sends_since_drain_ = 0;
    drain_ready(0);
  }
}

void UdpEndpoints::send_raw(std::uint32_t from, std::uint32_t to, const std::uint8_t* data,
                            std::size_t size) {
  if (from >= sockets_.size() || to >= sockets_.size()) {
    throw std::out_of_range("UdpEndpoints: no such endpoint");
  }
  sockets_[from].send_to(addresses_[to], data, size);
  ++stats_.sent;
}

void UdpEndpoints::drain(std::size_t socket_index) {
  std::vector<std::uint8_t> buffer(kReceiveBuffer);
  for (;;) {
    const UdpSocket::Received r = sockets_[socket_index].receive(buffer.data(), buffer.size(), 0);
    if (r.status != IoStatus::kOk) {
      return;
    }
    Message message;
    if (r.truncated || decode_message(buffer.data(), r.size, message) != WireStatus::kOk) {
      ++stats_.malformed;
      continue;
    }
    if (message.sender >= addresses_.size() || r.from != addresses_[message.sender]) {
      ++stats_.foreign;
      continue;
    }
    if (message.receiver != socket_index) {
      ++stats_.misdelivered;
      continue;
    }
    ++stats_.received;
    arrived_.push_back(std::move(message));
  }
}

void UdpEndpoints::drain_ready(int timeout_ms) {
  std::vector<const Socket*> all;
  all.reserve(sockets_.size());
  for (const UdpSocket& s : sockets_) {
    all.push_back(&s);
  }
  // Wait for the first arrival, then keep going while there is more, so that datagrams that land
  // while the others are being read are not left behind.
  int timeout = timeout_ms;
  for (;;) {
    const std::vector<std::size_t> ready = wait_readable(all, timeout);
    if (ready.empty()) {
      return;
    }
    for (std::size_t index : ready) {
      drain(index);
    }
    timeout = 0;
  }
}

std::vector<Message> UdpEndpoints::receive(int timeout_ms) {
  if (arrived_.empty()) {
    drain_ready(timeout_ms);
  } else {
    drain_ready(0);
  }
  std::vector<Message> out = std::move(arrived_);
  arrived_.clear();
  return out;
}

}  // namespace swarm::net
