#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "swarm/channel.hpp"
#include "swarm/channel_model.hpp"
#include "swarm/delivery_queue.hpp"
#include "swarm/net/tcp_connections.hpp"
#include "swarm/net/udp_endpoints.hpp"

namespace swarm::net {

// A channel that carries its messages over real UDP and TCP sockets on the loopback interface, and
// still behaves exactly like the simulated one.
//
// What happens to each message is decided by the same ChannelModel the deterministic channel uses:
// whether the rate limit, the loss model, or a jammer drops it, and the tick it is delivered at.
// Dropped messages never touch a socket. A message the model lets through is serialized and sent
// at once: beacons as UDP datagrams, commands over TCP. The tick at which it is delivered is
// still the model's. When the simulation reaches that tick, the channel hands the message to the
// receiver's inbox, in exactly the order the deterministic channel would, waiting (up to a
// timeout) for the bytes to arrive if they have not. The order comes from the same DeliveryQueue
// class, holding a placeholder for each message in flight.
//
// So, when the operating system loses and delays nothing, a run over this channel is bit-identical
// to the same run over the deterministic channel: the sockets validate the wire format, the
// framing, and the transports, and the model keeps the timing. If a message does not arrive within
// the timeout it is counted as a network drop and not delivered. That is real behavior, and it
// makes the run differ from the simulated one, and the verification of its log fail.
//
// send and deliver are called from one thread, the barrier step, as for any channel. take_inbox
// may be called concurrently for different receivers. Wall-clock time is used only for the
// timeout. The number of endpoints is limited to kMaxEndpoints, because each takes two descriptors.
class SocketChannel : public IChannel {
 public:
  static constexpr std::uint32_t kMaxEndpoints = 256;

  struct NetStats {
    std::uint64_t timed_out = 0;   // scheduled messages that never arrived (also dropped_network)
    std::uint64_t duplicates = 0;  // a message that arrived twice
    std::uint64_t unexpected = 0;  // arrived but was not awaited (late, or never sent by us)
    UdpEndpoints::Stats udp;
    TcpConnections::Stats tcp;
  };

  // Throws std::invalid_argument for an invalid configuration or for a number of endpoints that
  // is zero or above kMaxEndpoints, and NetError if a socket cannot be created.
  SocketChannel(std::uint32_t num_endpoints, ChannelConfig config, int receive_timeout_ms = 1000);

  // Throws std::out_of_range for an endpoint that does not exist and std::invalid_argument for a
  // payload longer than kMaxWirePayload; in both cases nothing is changed.
  void send(std::uint64_t now_tick, Message msg) override;
  void set_positions(const std::vector<Vec3>& positions) override {
    model_.set_positions(positions);
  }
  void deliver(std::uint64_t tick) override;
  std::vector<Message> take_inbox(std::uint32_t receiver) override;
  const ChannelStats& stats() const override { return stats_; }
  std::uint32_t endpoint_count() const override { return model_.endpoint_count(); }
  void set_event_sink(std::vector<ChannelEvent>* sink) override { sink_ = sink; }

  NetStats net_stats() const;

  // For tests: when set and it returns true for a message the model scheduled, that message is not
  // put on a socket, which is what the operating system losing it would look like.
  void set_loss_in_transit(std::function<bool(const Message&)> hook) { lose_ = std::move(hook); }

 private:
  using Key = std::pair<std::uint32_t, std::uint64_t>;  // sender and sequence number

  void record(MessageOutcome outcome, const Message& msg);
  void absorb(std::vector<Message> arrived);
  void poll_transports(int udp_wait_ms);

  ChannelModel model_;
  UdpEndpoints udp_;
  TcpConnections tcp_;
  int receive_timeout_ms_;
  std::function<bool(const Message&)> lose_;

  DeliveryQueue queue_;             // placeholders, ordered as the deterministic channel orders
  std::set<Key> pending_;           // scheduled and not yet delivered or given up on
  std::map<Key, Message> arrived_;  // received from a socket, waiting for their tick
  std::vector<std::uint64_t> next_seq_;
  // Sized once at construction and never resized, so take_inbox() is safe from different
  // threads for different receivers.
  std::vector<std::vector<Message>> inboxes_;
  ChannelStats stats_;
  NetStats net_;
  std::vector<ChannelEvent>* sink_ = nullptr;
};

}  // namespace swarm::net
