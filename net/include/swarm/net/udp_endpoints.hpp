#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "swarm/message.hpp"
#include "swarm/net/socket.hpp"
#include "swarm/net/wire.hpp"

namespace swarm::net {

// A set of endpoints that exchange messages as UDP datagrams over the loopback interface, one
// socket per endpoint, each bound to a port the operating system chose. A message is serialized
// with the wire format and sent from the sender's socket to the receiver's. This is the transport
// for state beacons: it is unreliable by nature, and it does not hide that.
//
// What it checks on arrival, in this order, counting what it throws away:
//   - the datagram decodes as exactly one wire message and was not truncated (else `malformed`);
//   - it came from the address of the endpoint named as its sender (else `foreign`);
//   - it arrived at the socket of the endpoint named as its receiver (else `misdelivered`).
// A bad datagram is dropped and the transport carries on. These checks catch accidents, such as
// another program sending to a port, and are not a defense against an attacker on the machine.
//
// A UDP receiver silently drops datagrams once its queue is full, and the queue is small and holds
// fewer small datagrams than its byte size suggests. So `send` drains the sockets every
// `kDrainEvery` sends into an internal list, and a burst of thousands of messages does not
// overflow anything. `receive` returns that list and what else has arrived.
//
// Not thread-safe: one thread uses it. Sockets are limited by the process's descriptor limit, so
// the number of endpoints is limited to kMaxEndpoints.
class UdpEndpoints {
 public:
  static constexpr std::uint32_t kMaxEndpoints = 512;
  static constexpr std::uint32_t kDrainEvery = 16;

  struct Stats {
    std::uint64_t sent = 0;          // datagrams handed to the operating system
    std::uint64_t received = 0;      // valid messages taken from the sockets
    std::uint64_t malformed = 0;     // did not decode, or were truncated
    std::uint64_t foreign = 0;       // came from an address that is not their sender's
    std::uint64_t misdelivered = 0;  // arrived at a socket that is not their receiver's
  };

  // Throws std::invalid_argument if `count` is zero or above kMaxEndpoints, and NetError if a
  // socket cannot be created.
  explicit UdpEndpoints(std::uint32_t count, int receive_buffer = 4 << 20);

  std::uint32_t count() const { return static_cast<std::uint32_t>(sockets_.size()); }
  Address address(std::uint32_t endpoint) const;

  // Serializes `message` and sends it from `message.sender` to `message.receiver`. Throws
  // std::out_of_range for an endpoint that does not exist and std::invalid_argument for a message
  // the wire format cannot carry.
  void send(const Message& message);

  // Sends the given bytes as one datagram from one endpoint's socket to another's, without
  // looking at them. For tests and fault injection.
  void send_raw(std::uint32_t from, std::uint32_t to, const std::uint8_t* data, std::size_t size);

  // Returns the valid messages that have arrived, including those taken in by `send`, in order of
  // arrival per socket. Waits up to `timeout_ms` for something to arrive if nothing has (0 polls).
  std::vector<Message> receive(int timeout_ms);

  const Stats& stats() const { return stats_; }

 private:
  void drain(std::size_t socket_index);
  void drain_ready(int timeout_ms);

  std::vector<UdpSocket> sockets_;
  std::vector<Address> addresses_;
  std::vector<Message> arrived_;
  std::uint32_t sends_since_drain_ = 0;
  Stats stats_;
};

}  // namespace swarm::net
