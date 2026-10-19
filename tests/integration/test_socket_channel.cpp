#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "swarm/channel_model.hpp"
#include "swarm/demo_agent.hpp"
#include "swarm/deterministic_channel.hpp"
#include "swarm/format/hash.hpp"
#include "swarm/net/socket_channel.hpp"
#include "swarm/simulation.hpp"
#include "test_scale.hpp"

using swarm::ChannelEvent;
using swarm::Message;
using swarm::MessageType;
using swarm::net::SocketChannel;

namespace {

// The harsh configuration of the channel-model tests: burst loss, jitter, a rate limit, a pulsed
// local jammer, and a global jammer that also affects senders.
swarm::SimConfig harsh() {
  swarm::SimConfig c;
  c.seed = 17;
  c.channel.latency_ticks = 2;
  c.channel.jitter_ticks = 3;
  c.channel.loss_model = swarm::LossModel::kBurst;
  c.channel.loss = 0.2;
  c.channel.mean_burst = 4.0;
  c.channel.max_msgs_per_sender_per_tick = 1;

  swarm::Jammer pulsed;
  pulsed.center = {40.0, 0.0, 0.0};
  pulsed.radius = 25.0;
  pulsed.mode = swarm::JamMode::kPulsed;
  pulsed.start_tick = 50;
  pulsed.end_tick = 350;
  pulsed.pulse_period_ticks = 20;
  pulsed.pulse_duty = 0.5;
  pulsed.loss_inside = 1.0;
  swarm::Jammer global;
  global.radius = std::numeric_limits<double>::infinity();
  global.start_tick = 100;
  global.end_tick = 140;
  global.loss_inside = 0.5;
  global.affects = swarm::JamAffects::kEither;
  c.channel.jammers = {pulsed, global};
  return c;
}

std::vector<std::unique_ptr<swarm::IAgent>> demo(std::uint32_t n) {
  std::vector<std::unique_ptr<swarm::IAgent>> agents;
  for (std::uint32_t i = 0; i < n; ++i) {
    agents.push_back(std::make_unique<swarm::DemoAgent>(i, n));
  }
  return agents;
}

struct Outcome {
  std::vector<std::uint64_t> hashes;
  std::vector<ChannelEvent> events;
  swarm::ChannelStats stats;
  SocketChannel::NetStats net;
};

Outcome run(bool sockets, std::uint64_t ticks, std::size_t threads) {
  const swarm::SimConfig config = harsh();
  std::unique_ptr<swarm::Simulation> sim;
  SocketChannel* channel = nullptr;
  if (sockets) {
    swarm::ChannelConfig c = config.channel;
    c.seed = config.seed;
    auto owned = std::make_unique<SocketChannel>(8, c, 3000);
    channel = owned.get();
    sim = std::make_unique<swarm::Simulation>(config, demo(8), threads, std::move(owned));
  } else {
    sim = std::make_unique<swarm::Simulation>(config, demo(8), threads);
  }
  Outcome out;
  sim->set_event_sink(&out.events);
  for (std::uint64_t t = 0; t < ticks; ++t) {
    sim->step();
    out.hashes.push_back(sim->state_hash());
  }
  out.stats = sim->channel().stats();
  if (channel != nullptr) {
    out.net = channel->net_stats();
  }
  return out;
}

bool same_event(const ChannelEvent& a, const ChannelEvent& b) {
  return a.outcome == b.outcome && a.sender == b.sender && a.receiver == b.receiver &&
         a.send_tick == b.send_tick && a.deliver_tick == b.deliver_tick;
}

bool same_message(const Message& a, const Message& b) {
  return a.type == b.type && a.sender == b.sender && a.receiver == b.receiver && a.seq == b.seq &&
         a.send_tick == b.send_tick && a.deliver_tick == b.deliver_tick && a.payload == b.payload;
}

Message beacon(std::uint32_t from, std::uint32_t to, std::uint8_t a, std::uint8_t b) {
  Message m;
  m.sender = from;
  m.receiver = to;
  m.payload = {a, b};
  return m;
}

}  // namespace

TEST(SocketChannel, TheHarshRunGivesTheFingerprintOfTheDeterministicChannel) {
  // The same constants as the channel-model test, recorded from the deterministic channel before
  // its model was extracted: the real sockets reproduce the simulated channel exactly.
  const Outcome o = run(true, 400, 3);
  EXPECT_EQ(o.hashes.back(), 13965888112468217684ULL);
  swarm::Fnv1a64 h;
  for (const ChannelEvent& e : o.events) {
    h.update_u64(static_cast<std::uint64_t>(e.outcome));
    h.update_u64(e.sender);
    h.update_u64(e.receiver);
    h.update_u64(e.send_tick);
    h.update_u64(e.deliver_tick);
  }
  EXPECT_EQ(h.digest(), 17948217017054799834ULL);
  EXPECT_EQ(o.events.size(), 6400U);
  EXPECT_EQ(o.stats.sent, 6400U);
  EXPECT_EQ(o.stats.delivered, 1861U);
  EXPECT_EQ(o.stats.dropped_loss, 637U);
  EXPECT_EQ(o.stats.dropped_rate, 3200U);
  EXPECT_EQ(o.stats.dropped_jam, 680U);
  EXPECT_EQ(o.stats.dropped_network, 0U);
  EXPECT_EQ(o.net.timed_out + o.net.duplicates + o.net.unexpected, 0U);
  // Only the messages the model let through touched a socket: the delivered ones, and those
  // still in flight when the run ended.
  std::uint64_t scheduled = 0;
  for (const ChannelEvent& e : o.events) {
    scheduled += e.outcome == swarm::MessageOutcome::kScheduled ? 1 : 0;
  }
  EXPECT_EQ(o.net.udp.sent, scheduled);
  EXPECT_GE(scheduled, o.stats.delivered);
  EXPECT_LE(o.net.udp.received, o.net.udp.sent);
  EXPECT_GE(o.net.udp.received, o.stats.delivered);
}

TEST(SocketChannel, ThePerTickHashesAndEventsEqualTheDeterministicChannelForEveryThreadCount) {
  const std::uint64_t ticks = static_cast<std::uint64_t>(swarm::testing::scaled(300, 120));
  const Outcome reference = run(false, ticks, 1);
  for (std::size_t threads : {1U, 2U, 4U, 8U}) {
    const Outcome o = run(true, ticks, threads);
    EXPECT_EQ(o.hashes, reference.hashes) << "threads " << threads;
    ASSERT_EQ(o.events.size(), reference.events.size()) << "threads " << threads;
    for (std::size_t i = 0; i < o.events.size(); ++i) {
      ASSERT_TRUE(same_event(o.events[i], reference.events[i])) << "event " << i;
    }
    EXPECT_EQ(o.stats.delivered, reference.stats.delivered);
    EXPECT_EQ(o.stats.dropped_network, 0U);
  }
}

namespace {

// Drives a deterministic and a socket channel with the same traffic and checks, tick by tick, that
// they deliver the same messages in the same order. Returns the observed loss rate.
double compare_channels(swarm::LossModel model, double loss, std::uint64_t ticks) {
  swarm::ChannelConfig c;
  c.seed = 9;
  c.latency_ticks = 1;
  c.jitter_ticks = 2;
  c.loss_model = model;
  c.loss = loss;
  c.mean_burst = 4.0;
  swarm::DeterministicChannel simulated(4, c);
  SocketChannel real(4, c, 3000);

  for (std::uint64_t tick = 0; tick < ticks; ++tick) {
    for (std::uint32_t e = 0; e < 4; ++e) {
      for (std::uint32_t hop : {1U, 2U}) {
        const Message m = beacon(e, (e + hop) % 4, static_cast<std::uint8_t>(tick & 0xFF),
                                 static_cast<std::uint8_t>(e));
        simulated.send(tick, m);
        real.send(tick, m);
      }
    }
    simulated.deliver(tick);
    real.deliver(tick);
    for (std::uint32_t e = 0; e < 4; ++e) {
      const std::vector<Message> a = simulated.take_inbox(e);
      const std::vector<Message> b = real.take_inbox(e);
      EXPECT_EQ(a.size(), b.size()) << "tick " << tick << " endpoint " << e;
      if (a.size() != b.size()) {
        return -1.0;
      }
      for (std::size_t i = 0; i < a.size(); ++i) {
        if (!same_message(a[i], b[i])) {
          ADD_FAILURE() << "tick " << tick << " endpoint " << e << " message " << i;
          return -1.0;
        }
      }
    }
  }
  EXPECT_EQ(real.stats().dropped_network, 0U);
  EXPECT_EQ(simulated.stats().delivered, real.stats().delivered);
  const double sent = static_cast<double>(real.stats().sent);
  return static_cast<double>(real.stats().dropped_loss) / sent;
}

}  // namespace

TEST(SocketChannel, TheObservedLossMatchesTheSettingAndTheDeliveredSetEqualsTheSimulatedOne) {
  const std::uint64_t ticks = static_cast<std::uint64_t>(swarm::testing::scaled(2500, 800));
  const double tolerance = swarm::testing::kSanitized ? 0.04 : 0.02;
  // Independent loss: 20,000 messages, so the observed rate is within a few thousandths of 0.3.
  const double independent = compare_channels(swarm::LossModel::kIndependent, 0.3, ticks);
  RecordProperty("observed_independent_loss", std::to_string(independent));
  EXPECT_NEAR(independent, 0.3, tolerance);
  // Burst loss at the same mean: more variance, since drops come in runs.
  const double burst = compare_channels(swarm::LossModel::kBurst, 0.3, ticks);
  RecordProperty("observed_burst_loss", std::to_string(burst));
  EXPECT_NEAR(burst, 0.3, tolerance + 0.02);
}

TEST(SocketChannel, ABeaconTravelsAsAUdpDatagramAndACommandOverTcp) {
  swarm::ChannelConfig c;
  SocketChannel channel(3, c, 3000);
  for (std::uint32_t i = 0; i < 30; ++i) {
    channel.send(0, beacon(0, 1, static_cast<std::uint8_t>(i), 0));
  }
  for (std::uint32_t i = 0; i < 20; ++i) {
    Message m = beacon(2, i % 2, static_cast<std::uint8_t>(i), 1);
    m.type = MessageType::kCommand;
    channel.send(0, m);
  }
  channel.deliver(1);
  const SocketChannel::NetStats net = channel.net_stats();
  EXPECT_EQ(net.udp.sent, 30U);
  EXPECT_EQ(net.tcp.sent, 20U);
  EXPECT_EQ(net.udp.received, 30U);
  EXPECT_EQ(net.tcp.received, 20U);
  EXPECT_EQ(channel.stats().delivered, 50U);
  // Two connections: from the station to each of the two receivers it addressed.
  EXPECT_EQ(net.tcp.connections_opened, 2U);

  std::size_t commands = 0;
  for (std::uint32_t e = 0; e < 2; ++e) {
    for (const Message& m : channel.take_inbox(e)) {
      commands += m.type == MessageType::kCommand ? 1 : 0;
    }
  }
  EXPECT_EQ(commands, 20U);
}

TEST(SocketChannel, CommandsSurviveTotalLossInOrderAndMatchTheDeterministicChannel) {
  swarm::ChannelConfig c;
  c.loss_model = swarm::LossModel::kIndependent;
  c.loss = 1.0;  // every beacon is lost
  c.latency_ticks = 2;
  swarm::DeterministicChannel simulated(2, c);
  SocketChannel real(2, c, 3000);
  for (std::uint64_t tick = 0; tick < 30; ++tick) {
    for (swarm::Message m :
         {beacon(0, 1, 1, 1), beacon(0, 1, static_cast<std::uint8_t>(tick), 2)}) {
      if (m.payload[1] == 2) {
        m.type = MessageType::kCommand;
      }
      simulated.send(tick, m);
      real.send(tick, m);
    }
  }
  std::vector<Message> a;
  std::vector<Message> b;
  for (std::uint64_t tick = 0; tick < 40; ++tick) {
    simulated.deliver(tick);
    real.deliver(tick);
    for (const Message& m : simulated.take_inbox(1)) {
      a.push_back(m);
    }
    for (const Message& m : real.take_inbox(1)) {
      b.push_back(m);
    }
  }
  ASSERT_EQ(a.size(), 30U);  // only the commands
  ASSERT_EQ(b.size(), a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    ASSERT_TRUE(same_message(a[i], b[i])) << i;
    EXPECT_EQ(b[i].payload[0], i);  // in the order they were sent
  }
}

TEST(SocketChannel, AMessageTheNetworkLosesIsCountedAndTheRunContinues) {
  swarm::ChannelConfig c;
  c.latency_ticks = 1;
  SocketChannel channel(2, c, 50);  // a short timeout, so the missing ones are given up quickly
  channel.set_loss_in_transit([](const Message& m) { return m.seq % 10 == 3; });

  for (std::uint64_t tick = 0; tick < 100; ++tick) {
    channel.send(tick, beacon(0, 1, static_cast<std::uint8_t>(tick), 0));
  }
  std::size_t got = 0;
  for (std::uint64_t tick = 0; tick <= 101; ++tick) {
    channel.deliver(tick);
    got += channel.take_inbox(1).size();
  }
  EXPECT_EQ(got, 90U);
  EXPECT_EQ(channel.stats().sent, 100U);
  EXPECT_EQ(channel.stats().delivered, 90U);
  EXPECT_EQ(channel.stats().dropped_network, 10U);
  EXPECT_EQ(channel.net_stats().timed_out, 10U);

  // And it carries on afterwards.
  channel.set_loss_in_transit(nullptr);
  for (std::uint64_t tick = 200; tick < 220; ++tick) {
    channel.send(tick, beacon(0, 1, 7, 7));
    channel.deliver(tick);
    got += channel.take_inbox(1).size();
  }
  channel.deliver(221);
  got += channel.take_inbox(1).size();
  EXPECT_EQ(got, 110U);
  EXPECT_EQ(channel.stats().dropped_network, 10U);
}

TEST(SocketChannel, MisuseIsRejectedAndALongPayloadLeavesNoTrace) {
  EXPECT_THROW(SocketChannel(0, swarm::ChannelConfig{}), std::invalid_argument);
  EXPECT_THROW(SocketChannel(SocketChannel::kMaxEndpoints + 1, swarm::ChannelConfig{}),
               std::invalid_argument);
  EXPECT_THROW(SocketChannel(2, swarm::ChannelConfig{}, -1), std::invalid_argument);
  swarm::ChannelConfig bad;
  bad.latency_ticks = 0;
  EXPECT_THROW(SocketChannel(2, bad), std::invalid_argument);

  SocketChannel channel(2, swarm::ChannelConfig{}, 50);
  EXPECT_THROW(channel.send(0, beacon(5, 0, 0, 0)), std::out_of_range);
  EXPECT_THROW(channel.take_inbox(9), std::out_of_range);
  Message too_long = beacon(0, 1, 0, 0);
  too_long.payload.assign(swarm::net::kMaxWirePayload + 1, 0);
  EXPECT_THROW(channel.send(0, too_long), std::invalid_argument);
  EXPECT_EQ(channel.stats().sent, 0U);  // nothing was recorded for the refused message
  channel.deliver(5);
  EXPECT_EQ(channel.stats().dropped_network, 0U);  // and no phantom message timed out
}
