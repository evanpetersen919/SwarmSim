#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "swarm/message.hpp"
#include "swarm/net/socket.hpp"
#include "swarm/net/wire.hpp"

namespace swarm::net {

// A set of endpoints that exchange messages over TCP on the loopback interface. Each endpoint
// listens on a port the operating system chose. The first time a message goes from one endpoint
// to another, a connection is opened from the sender to the receiver's listener, and it is reused
// for every later message in that direction, so a ground station that commands eight drones holds
// eight connections and not a mesh. Messages use the wire format, back to back on the stream;
// TCP keeps no write boundaries, so the receiving side reassembles them with a FrameDecoder. This
// is the transport for commands: bytes arrive whole, in order, and once.
//
// What it checks on arrival, counting what it throws away:
//   - the stream decodes (else the connection is corrupt: `broken`, and it is closed);
//   - a message arrived on the listener of the endpoint it is addressed to (else `misdelivered`);
//   - every message on one connection names the same sender, the one named by the first (else
//     `misattributed`).
// A connection the peer closes is counted as `closed` and forgotten; the next send in that
// direction opens a new one. The checks catch accidents, not an attacker.
//
// A sender that writes a lot before anything reads would fill the socket buffers and block. So
// `send` drains the inbound side every `kDrainEvery` sends, as the UDP transport does.
//
// Not thread-safe: one thread uses it.
class TcpConnections {
 public:
  static constexpr std::uint32_t kMaxEndpoints = 512;
  static constexpr std::uint32_t kDrainEvery = 16;

  struct Stats {
    std::uint64_t sent = 0;
    std::uint64_t received = 0;
    std::uint64_t connections_opened = 0;  // by senders
    std::uint64_t closed = 0;              // inbound connections the peer closed
    std::uint64_t broken = 0;  // inbound connections closed because the stream was corrupt
    std::uint64_t misdelivered = 0;
    std::uint64_t misattributed = 0;
  };

  // Throws std::invalid_argument if `count` is zero or above kMaxEndpoints, and NetError if a
  // socket cannot be created.
  explicit TcpConnections(std::uint32_t count);

  std::uint32_t count() const { return static_cast<std::uint32_t>(listeners_.size()); }
  Address address(std::uint32_t endpoint) const;

  // Serializes `message` and writes it on the connection from `message.sender` to
  // `message.receiver`, opening it if there is none. If the connection turns out to be closed, one
  // new connection is opened and the message is written again; if that fails too, NetError is
  // thrown. Throws std::out_of_range for an endpoint that does not exist and
  // std::invalid_argument for a message the wire format cannot carry.
  void send(const Message& message);

  // Writes the given bytes on the connection from one endpoint to another, opening it if needed,
  // without looking at them. For tests and fault injection.
  void send_raw(std::uint32_t from, std::uint32_t to, const std::uint8_t* data, std::size_t size);

  // Closes the sending end of the connection from one endpoint to another, if there is one. For
  // tests and fault injection.
  void close_connection(std::uint32_t from, std::uint32_t to);

  // Returns the messages that have arrived, including those taken in by `send`, in order of
  // arrival per connection. Waits up to `timeout_ms` if nothing has arrived (0 polls).
  std::vector<Message> receive(int timeout_ms);

  const Stats& stats() const { return stats_; }

 private:
  struct Inbound {
    std::uint32_t endpoint = 0;  // the listener it was accepted on
    TcpStream stream;
    FrameDecoder decoder;
    std::optional<std::uint32_t> peer;  // the sender named by the first message
    bool dead = false;
  };

  TcpStream& outbound(std::uint32_t from, std::uint32_t to);
  void pump(int timeout_ms);
  void read_inbound(Inbound& in);
  void check_endpoints(std::uint32_t from, std::uint32_t to) const;

  std::vector<TcpListener> listeners_;
  std::vector<Address> addresses_;
  std::map<std::pair<std::uint32_t, std::uint32_t>, TcpStream> outbound_;
  std::vector<Inbound> inbound_;
  std::vector<Message> arrived_;
  std::uint32_t sends_since_drain_ = 0;
  Stats stats_;
};

}  // namespace swarm::net
