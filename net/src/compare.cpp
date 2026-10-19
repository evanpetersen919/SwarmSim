#include "swarm/net/compare.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <thread>
#include <vector>

#include "swarm/channel_model.hpp"
#include "swarm/deterministic_channel.hpp"
#include "swarm/net/socket_channel.hpp"
#include "swarm/net/tcp_connections.hpp"
#include "swarm/net/udp_endpoints.hpp"
#include "swarm/net/wire.hpp"

namespace swarm::net {

namespace {

using Clock = std::chrono::steady_clock;

void validate(const CompareConfig& c) {
  if (c.endpoints < 2 || c.endpoints > SocketChannel::kMaxEndpoints) {
    throw std::invalid_argument("endpoints must be between 2 and " +
                                std::to_string(SocketChannel::kMaxEndpoints));
  }
  if (c.ticks == 0) {
    throw std::invalid_argument("ticks must be at least 1");
  }
  if (c.payload_bytes > kMaxWirePayload) {
    throw std::invalid_argument("payload must be at most " + std::to_string(kMaxWirePayload) +
                                " bytes");
  }
  if (c.samples == 0) {
    throw std::invalid_argument("samples must be at least 1");
  }
  if (!(c.loss >= 0.0 && c.loss <= 1.0)) {
    throw std::invalid_argument("loss must be in [0, 1]");
  }
  if (c.latency_ticks < 1) {
    throw std::invalid_argument("latency must be at least 1 tick");
  }
  if (!(c.dt > 0.0) || !std::isfinite(c.dt)) {
    throw std::invalid_argument("dt must be positive");
  }
  if (c.receive_timeout_ms < 0) {
    throw std::invalid_argument("the receive timeout must not be negative");
  }
}

bool same(const Message& a, const Message& b) {
  return a.type == b.type && a.sender == b.sender && a.receiver == b.receiver && a.seq == b.seq &&
         a.send_tick == b.send_tick && a.deliver_tick == b.deliver_tick && a.payload == b.payload;
}

Message traffic_message(std::uint32_t from, std::uint32_t to, std::uint64_t tick,
                        std::uint32_t payload_bytes, MessageType type) {
  Message m;
  m.type = type;
  m.sender = from;
  m.receiver = to;
  m.payload.resize(payload_bytes);
  for (std::uint32_t i = 0; i < payload_bytes; ++i) {
    m.payload[i] = static_cast<std::uint8_t>((tick * 31 + from * 7 + to * 3 + i) & 0xFF);
  }
  return m;
}

// Nearest-rank percentile of a sorted list, with the rank computed in integers.
double percentile(const std::vector<double>& sorted, unsigned percent) {
  const std::size_t n = sorted.size();
  const std::size_t rank = (static_cast<std::size_t>(percent) * n + 99) / 100;
  return sorted[std::max<std::size_t>(rank, 1) - 1];
}

LatencySummary summarize(std::vector<double> microseconds) {
  LatencySummary out;
  out.samples = static_cast<std::uint32_t>(microseconds.size());
  if (microseconds.empty()) {
    return out;
  }
  std::sort(microseconds.begin(), microseconds.end());
  out.p50_us = percentile(microseconds, 50);
  out.p95_us = percentile(microseconds, 95);
  out.p99_us = percentile(microseconds, 99);
  out.max_us = microseconds.back();
  return out;
}

double micros_since(Clock::time_point start) {
  return std::chrono::duration<double, std::micro>(Clock::now() - start).count();
}

// Times one message at a time from endpoint 0 to endpoint 1: from before the call that sends it to
// the return of the call that receives it. The first message is a warm-up and is not counted, so
// that opening a connection or the first system call is not in the numbers.
template <typename Transport>
LatencySummary time_transport(Transport& transport, const CompareConfig& c, MessageType type) {
  std::vector<double> samples;
  samples.reserve(c.samples);
  for (std::uint32_t i = 0; i <= c.samples; ++i) {
    const Message m = traffic_message(0, 1, i, c.payload_bytes, type);
    const auto start = Clock::now();
    transport.send(m);
    std::vector<Message> got;
    while (got.empty()) {
      got = transport.receive(c.receive_timeout_ms);
      if (Clock::now() - start > std::chrono::milliseconds(c.receive_timeout_ms)) {
        break;
      }
    }
    if (!got.empty() && i > 0) {
      samples.push_back(micros_since(start));
    }
  }
  return summarize(std::move(samples));
}

// Sends `samples` messages in a row and receives them all; returns messages per second.
template <typename Transport>
double time_batch(Transport& transport, const CompareConfig& c, MessageType type) {
  const auto start = Clock::now();
  for (std::uint32_t i = 0; i < c.samples; ++i) {
    transport.send(traffic_message(0, 1, i, c.payload_bytes, type));
  }
  std::size_t received = 0;
  while (received < c.samples &&
         Clock::now() - start < std::chrono::milliseconds(c.receive_timeout_ms) * 10) {
    received += transport.receive(50).size();
  }
  const double seconds = std::chrono::duration<double>(Clock::now() - start).count();
  return seconds > 0.0 ? static_cast<double>(received) / seconds : 0.0;
}

}  // namespace

CompareResult run_comparison(const CompareConfig& config) {
  validate(config);
  CompareResult result;

  // Equivalence: the same traffic through both channels, compared at every tick.
  ChannelConfig channel_config;
  channel_config.seed = config.seed;
  channel_config.latency_ticks = config.latency_ticks;
  channel_config.jitter_ticks = config.jitter_ticks;
  if (config.loss > 0.0) {
    channel_config.loss_model = LossModel::kIndependent;
    channel_config.loss = config.loss;
  }
  DeterministicChannel simulated(config.endpoints, channel_config);
  SocketChannel real(config.endpoints, channel_config, config.receive_timeout_ms);
  real.set_loss_in_transit(config.lose_in_transit);

  result.identical = true;
  for (std::uint64_t tick = 0; tick < config.ticks && result.identical; ++tick) {
    for (std::uint32_t e = 0; e < config.endpoints; ++e) {
      for (std::uint32_t hop = 1; hop <= 2 && hop < config.endpoints; ++hop) {
        const Message m = traffic_message(e, (e + hop) % config.endpoints, tick,
                                          config.payload_bytes, MessageType::kBeacon);
        simulated.send(tick, m);
        real.send(tick, m);
      }
    }
    if (config.commands && tick % 25 == 0) {
      const Message c = traffic_message(0, 1, tick, config.payload_bytes, MessageType::kCommand);
      simulated.send(tick, c);
      real.send(tick, c);
    }
    simulated.deliver(tick);
    real.deliver(tick);
    for (std::uint32_t e = 0; e < config.endpoints && result.identical; ++e) {
      const std::vector<Message> a = simulated.take_inbox(e);
      const std::vector<Message> b = real.take_inbox(e);
      if (a.size() != b.size()) {
        result.identical = false;
        result.first_difference = "tick " + std::to_string(tick) + ", endpoint " +
                                  std::to_string(e) + ": the simulated channel delivered " +
                                  std::to_string(a.size()) + " messages and the real one " +
                                  std::to_string(b.size());
        break;
      }
      for (std::size_t i = 0; i < a.size(); ++i) {
        if (!same(a[i], b[i])) {
          result.identical = false;
          result.first_difference = "tick " + std::to_string(tick) + ", endpoint " +
                                    std::to_string(e) + ": message " + std::to_string(i) +
                                    " differs";
          break;
        }
      }
    }
  }
  result.sent = real.stats().sent;
  result.delivered_simulated = simulated.stats().delivered;
  result.delivered_real = real.stats().delivered;
  result.dropped_by_model =
      real.stats().dropped_loss + real.stats().dropped_rate + real.stats().dropped_jam;
  result.dropped_by_network = real.stats().dropped_network;

  // Cost: the transports on their own.
  {
    UdpEndpoints udp(2);
    result.udp = time_transport(udp, config, MessageType::kBeacon);
    UdpEndpoints udp_batch(2);
    result.udp_messages_per_second = time_batch(udp_batch, config, MessageType::kBeacon);
  }
  {
    TcpConnections tcp(2);
    result.tcp = time_transport(tcp, config, MessageType::kCommand);
    TcpConnections tcp_batch(2);
    result.tcp_messages_per_second = time_batch(tcp_batch, config, MessageType::kCommand);
  }
  return result;
}

namespace {

const char* platform_name() {
#if defined(_WIN32)
  return "Windows";
#elif defined(__linux__)
  return "Linux";
#elif defined(__APPLE__)
  return "macOS";
#else
  return "an unrecognized platform";
#endif
}

const char* compiler_name() {
#if defined(_MSC_VER) && !defined(__clang__)
  return "MSVC";
#elif defined(__clang__)
  return "Clang";
#elif defined(__GNUC__)
  return "GCC";
#else
  return "an unrecognized compiler";
#endif
}

std::string fixed(double value, int digits) {
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%.*f", digits, value);
  return buf;
}

std::string latency_line(const char* name, const LatencySummary& s) {
  return std::string("  ") + name + "  p50 " + fixed(s.p50_us, 1) + "  p95 " + fixed(s.p95_us, 1) +
         "  p99 " + fixed(s.p99_us, 1) + "  max " + fixed(s.max_us, 1) + "  (" +
         std::to_string(s.samples) + " samples)\n";
}

}  // namespace

std::string format_report(const CompareConfig& c, const CompareResult& r) {
  std::string out;
  out += "swarm_netcmp: real sockets versus the simulated channel\n";
  out += std::string("platform: ") + platform_name() + ", " + compiler_name() + ", " +
         std::to_string(std::thread::hardware_concurrency()) + " logical cores\n";
  out += "traffic: " + std::to_string(c.endpoints) + " endpoints, " + std::to_string(c.ticks) +
         " ticks, " + std::to_string(c.payload_bytes) + "-byte payloads, loss " + fixed(c.loss, 2) +
         ", seed " + std::to_string(c.seed) + (c.commands ? ", with commands" : ", no commands") +
         "\n\n";

  out += std::string("identical deliveries: ") + (r.identical ? "yes" : "no") + "\n";
  out += "  sent " + std::to_string(r.sent) + ", delivered " + std::to_string(r.delivered_real) +
         " over sockets and " + std::to_string(r.delivered_simulated) +
         " simulated, dropped by the model " + std::to_string(r.dropped_by_model) +
         ", lost by the network " + std::to_string(r.dropped_by_network) + "\n";
  if (!r.identical) {
    out += "  first difference: " + r.first_difference + "\n";
  }

  out +=
      "\nreal transport, one message at a time, microseconds from the call that sends to the\n"
      "call that returns the message, on one thread:\n";
  out += latency_line("UDP", r.udp);
  out += latency_line("TCP", r.tcp);
  out += "\nreal transport, a batch of " + std::to_string(c.samples) +
         " messages sent and then received:\n";
  out += "  UDP " + fixed(r.udp_messages_per_second, 0) + " messages per second\n";
  out += "  TCP " + fixed(r.tcp_messages_per_second, 0) + " messages per second\n";

  const double low = static_cast<double>(c.latency_ticks) * c.dt * 1000.0;
  const double high = static_cast<double>(c.latency_ticks + c.jitter_ticks) * c.dt * 1000.0;
  out += "\nsimulated channel model latency: " + std::to_string(c.latency_ticks) + " to " +
         std::to_string(c.latency_ticks + c.jitter_ticks) + " ticks, " + fixed(low, 1) + " to " +
         fixed(high, 1) + " ms at dt = " + fixed(c.dt, 3) + " s\n";

  out +=
      "\nreading: the sockets carry the messages the model lets through, and the model decides\n"
      "when they arrive, so identical deliveries show that the wire format, the framing, and the\n"
      "transports preserve the simulation exactly. The measured times are what the real\n"
      "transport costs on this machine over loopback. They are not the latency of a radio, and\n"
      "the model's latency is a parameter, not a measurement. Timings vary between machines and\n"
      "runs.\n";
  return out;
}

}  // namespace swarm::net
