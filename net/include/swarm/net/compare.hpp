#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "swarm/message.hpp"

namespace swarm::net {

// A comparison of the real transport with the simulated channel, in two parts.
//
// Equivalence. The same traffic, a beacon from each endpoint to its next two neighbors every tick
// and, if enabled, a command now and then, is sent through the deterministic channel and through
// the socket channel, which carries the messages over real UDP and TCP on loopback. At every tick
// the messages delivered to every endpoint are compared field by field, payload included. They
// should be identical, because the socket channel uses the simulated channel's model and only
// carries the bytes.
//
// Cost. How long the real transport itself takes, which the simulation does not model: for UDP
// and for TCP separately, the time from the call that sends a message to the call that returns it
// on the receiving side, measured one message at a time on one thread, and the rate at which a
// batch of messages is sent and received. These depend on the machine and are not asserted by any
// test. They are shown next to the latency the channel model uses, which is a parameter, so the
// two can be compared. On loopback the real transport takes tens to hundreds of microseconds, and
// the model's latency is milliseconds. The comparison validates the transport and its semantics,
// not a radio.
struct CompareConfig {
  std::uint32_t endpoints = 8;       // at least 2
  std::uint64_t ticks = 500;         // ticks of traffic compared
  std::uint32_t payload_bytes = 65;  // at most the wire format allows
  std::uint32_t samples = 2000;      // messages timed one at a time, and sent in the batch
  double loss = 0.1;                 // independent loss in the model, in [0, 1]
  std::uint32_t latency_ticks = 2;   // the model's latency, at least 1
  std::uint32_t jitter_ticks = 1;
  double dt = 0.02;  // seconds per tick, for showing the model's latency
  std::uint64_t seed = 1;
  bool commands = true;  // send a command every 25 ticks, which travels over TCP
  int receive_timeout_ms = 1000;
  // For testing the tool: when set and it returns true for a message, that message is lost in
  // transit, as if the operating system had dropped it.
  std::function<bool(const Message&)> lose_in_transit;
};

struct LatencySummary {
  std::uint32_t samples = 0;
  double p50_us = 0.0;  // nearest-rank percentiles
  double p95_us = 0.0;
  double p99_us = 0.0;
  double max_us = 0.0;
};

struct CompareResult {
  bool identical = false;        // the channels delivered the same messages at every tick
  std::string first_difference;  // empty when identical
  std::uint64_t sent = 0;
  std::uint64_t delivered_simulated = 0;
  std::uint64_t delivered_real = 0;
  std::uint64_t dropped_by_model = 0;  // loss, rate limit, and jamming together
  std::uint64_t dropped_by_network = 0;
  LatencySummary udp;
  LatencySummary tcp;
  double udp_messages_per_second = 0.0;
  double tcp_messages_per_second = 0.0;
};

// Throws std::invalid_argument for an invalid configuration, and NetError if sockets cannot be
// created.
CompareResult run_comparison(const CompareConfig& config);

// A plain-text report: the platform, the traffic, whether the deliveries were identical, the
// measured costs, the model's latency, and what the numbers do and do not show.
std::string format_report(const CompareConfig& config, const CompareResult& result);

}  // namespace swarm::net
