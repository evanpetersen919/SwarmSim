#include <gtest/gtest.h>

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
#include "swarm/simulation.hpp"

namespace {

// A harsh configuration that exercises every step of the channel: burst loss, jitter, a rate
// limit, a pulsed local jammer, and a global jammer that also affects senders.
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

struct Fingerprint {
  std::uint64_t final_hash = 0;
  std::uint64_t events = 0;  // a hash over every message event, in order
  swarm::ChannelStats stats;
  std::size_t event_count = 0;
};

Fingerprint run_harsh() {
  swarm::Simulation sim(harsh(), demo(8), 3);
  std::vector<swarm::ChannelEvent> events;
  sim.set_event_sink(&events);
  for (int t = 0; t < 400; ++t) {
    sim.step();
  }
  Fingerprint f;
  f.final_hash = sim.state_hash();
  f.stats = sim.channel().stats();
  f.event_count = events.size();
  swarm::Fnv1a64 h;
  for (const swarm::ChannelEvent& e : events) {
    h.update_u64(static_cast<std::uint64_t>(e.outcome));
    h.update_u64(e.sender);
    h.update_u64(e.receiver);
    h.update_u64(e.send_tick);
    h.update_u64(e.deliver_tick);
  }
  f.events = h.digest();
  return f;
}

swarm::Message message(std::uint32_t from, std::uint32_t to,
                       swarm::MessageType type = swarm::MessageType::kBeacon) {
  swarm::Message m;
  m.sender = from;
  m.receiver = to;
  m.type = type;
  m.payload = {1, 2, 3};
  return m;
}

}  // namespace

TEST(ChannelModel, TheHarshRunGivesTheFingerprintRecordedBeforeTheModelWasExtracted) {
  // These numbers were taken from the channel before its decisions moved into ChannelModel. If one
  // of them changes, the channel's behavior changed.
  const Fingerprint f = run_harsh();
  EXPECT_EQ(f.final_hash, 13965888112468217684ULL);
  EXPECT_EQ(f.events, 17948217017054799834ULL);
  EXPECT_EQ(f.event_count, 6400U);
  EXPECT_EQ(f.stats.sent, 6400U);
  EXPECT_EQ(f.stats.delivered, 1861U);
  EXPECT_EQ(f.stats.dropped_loss, 637U);
  EXPECT_EQ(f.stats.dropped_rate, 3200U);
  EXPECT_EQ(f.stats.dropped_jam, 680U);
}

TEST(ChannelModel, TheModelAloneDecidesExactlyWhatTheChannelDoes) {
  swarm::ChannelConfig config = harsh().channel;
  config.seed = 17;
  swarm::DeterministicChannel channel(8, config);
  swarm::ChannelModel model(8, config);
  std::vector<swarm::ChannelEvent> events;
  channel.set_event_sink(&events);

  std::vector<swarm::Vec3> positions;
  for (std::uint32_t i = 0; i < 8; ++i) {
    positions.push_back({10.0 * i, 0.0, 0.0});
  }
  std::size_t checked = 0;
  for (std::uint64_t tick = 0; tick < 300; ++tick) {
    channel.set_positions(positions);
    model.set_positions(positions);
    for (std::uint32_t from = 0; from < 8; ++from) {
      for (std::uint32_t to = 0; to < 8; ++to) {
        if (from == to) {
          continue;
        }
        const swarm::Message m = message(from, to);
        const swarm::ChannelModel::Decision d = model.decide(tick, m);
        channel.send(tick, m);
        const swarm::ChannelEvent& e = events.back();
        switch (d.outcome) {
          case swarm::ChannelModel::Outcome::kScheduled:
            ASSERT_EQ(e.outcome, swarm::MessageOutcome::kScheduled);
            ASSERT_EQ(e.deliver_tick, d.deliver_tick);
            break;
          case swarm::ChannelModel::Outcome::kDroppedLoss:
            ASSERT_EQ(e.outcome, swarm::MessageOutcome::kDroppedLoss);
            break;
          case swarm::ChannelModel::Outcome::kDroppedRate:
            ASSERT_EQ(e.outcome, swarm::MessageOutcome::kDroppedRate);
            break;
          case swarm::ChannelModel::Outcome::kDroppedJam:
            ASSERT_EQ(e.outcome, swarm::MessageOutcome::kDroppedJam);
            break;
        }
        ++checked;
      }
    }
  }
  EXPECT_EQ(checked, 300U * 56U);
}

TEST(ChannelModel, AReliableMessageIsNeverDroppedAndArrivesAfterExactlyTheBaseLatency) {
  swarm::ChannelConfig c;
  c.latency_ticks = 3;
  c.jitter_ticks = 5;
  c.loss_model = swarm::LossModel::kIndependent;
  c.loss = 1.0;  // every beacon is lost
  c.max_msgs_per_sender_per_tick = 1;
  swarm::Jammer j;
  j.radius = std::numeric_limits<double>::infinity();
  j.loss_inside = 1.0;
  c.jammers = {j};
  swarm::ChannelModel model(3, c);
  model.set_positions({swarm::Vec3{}, swarm::Vec3{}, swarm::Vec3{}});

  for (std::uint64_t tick = 10; tick < 60; ++tick) {
    const auto beacon = model.decide(tick, message(0, 1));
    EXPECT_NE(beacon.outcome, swarm::ChannelModel::Outcome::kScheduled);
    for (int k = 0; k < 4; ++k) {  // four commands in one tick: past the rate limit of one
      const auto command = model.decide(tick, message(0, 1, swarm::MessageType::kCommand));
      ASSERT_EQ(command.outcome, swarm::ChannelModel::Outcome::kScheduled);
      ASSERT_EQ(command.deliver_tick, tick + 3);  // no jitter
    }
  }
}

TEST(ChannelModel, ReliableMessagesArriveInTheOrderTheyWereSent) {
  swarm::ChannelConfig c;
  c.latency_ticks = 2;
  c.jitter_ticks = 4;
  c.loss_model = swarm::LossModel::kIndependent;
  c.loss = 0.5;
  c.seed = 3;
  swarm::DeterministicChannel channel(2, c);
  // Command number n carries n in its payload. Beacons are interleaved to stir the jitter stream.
  std::uint8_t next = 0;
  for (std::uint64_t tick = 0; tick < 60; ++tick) {
    channel.send(tick, message(0, 1));
    swarm::Message command = message(0, 1, swarm::MessageType::kCommand);
    command.payload = {next++};
    channel.send(tick, command);
    channel.send(tick, message(0, 1));
  }
  std::vector<std::uint8_t> arrived;
  for (std::uint64_t tick = 0; tick < 80; ++tick) {
    channel.deliver(tick);
    for (const swarm::Message& m : channel.take_inbox(1)) {
      if (m.type == swarm::MessageType::kCommand) {
        arrived.push_back(m.payload[0]);
      }
    }
  }
  ASSERT_EQ(arrived.size(), 60U);  // none lost, though half the beacons were
  for (std::size_t i = 0; i < arrived.size(); ++i) {
    EXPECT_EQ(arrived[i], i);
  }
}

TEST(ChannelModel, InterleavingReliableMessagesDoesNotChangeAnyBeaconsFate) {
  swarm::ChannelConfig c = harsh().channel;
  c.seed = 5;
  const std::vector<swarm::Vec3> positions = {{0, 0, 0}, {40, 0, 0}, {80, 0, 0}};
  swarm::ChannelModel plain(3, c);
  swarm::ChannelModel mixed(3, c);
  plain.set_positions(positions);
  mixed.set_positions(positions);
  for (std::uint64_t tick = 0; tick < 400; ++tick) {
    for (std::uint32_t from = 0; from < 3; ++from) {
      const swarm::Message beacon = message(from, (from + 1) % 3);
      const auto a = plain.decide(tick, beacon);
      if (tick % 3 == 0) {  // commands, in some ticks, before the beacon
        mixed.decide(tick, message(from, (from + 2) % 3, swarm::MessageType::kCommand));
      }
      const auto b = mixed.decide(tick, beacon);
      ASSERT_EQ(a.outcome, b.outcome) << "tick " << tick;
      ASSERT_EQ(a.deliver_tick, b.deliver_tick) << "tick " << tick;
    }
  }
}

TEST(ChannelModel, AnInvalidConfigurationAndAnEndpointOutOfRangeAreRejected) {
  swarm::ChannelConfig bad;
  bad.latency_ticks = 0;
  EXPECT_THROW(swarm::ChannelModel(2, bad), std::invalid_argument);
  swarm::ChannelModel model(2, swarm::ChannelConfig{});
  EXPECT_THROW(model.decide(0, message(5, 0)), std::out_of_range);
  EXPECT_THROW(model.decide(0, message(0, 2)), std::out_of_range);
}

namespace {

// A channel that wraps a deterministic one and counts what passes through it, to show that a
// simulation uses the channel it is given.
class CountingChannel : public swarm::IChannel {
 public:
  CountingChannel(std::uint32_t n, swarm::ChannelConfig c) : inner_(n, std::move(c)) {}
  void send(std::uint64_t t, swarm::Message m) override {
    ++sends;
    inner_.send(t, std::move(m));
  }
  void set_positions(const std::vector<swarm::Vec3>& p) override { inner_.set_positions(p); }
  void deliver(std::uint64_t t) override { inner_.deliver(t); }
  std::vector<swarm::Message> take_inbox(std::uint32_t r) override { return inner_.take_inbox(r); }
  const swarm::ChannelStats& stats() const override { return inner_.stats(); }
  std::uint32_t endpoint_count() const override { return inner_.endpoint_count(); }
  void set_event_sink(std::vector<swarm::ChannelEvent>* sink) override {
    ++sink_sets;
    inner_.set_event_sink(sink);
  }
  std::uint64_t sends = 0;
  int sink_sets = 0;

 private:
  swarm::DeterministicChannel inner_;
};

}  // namespace

TEST(ChannelModel, ASimulationUsesTheChannelItIsGivenAndEventsReachTheSinkThroughTheInterface) {
  const swarm::SimConfig config = harsh();
  swarm::ChannelConfig seeded = config.channel;
  seeded.seed = config.seed;
  auto owned = std::make_unique<CountingChannel>(8, seeded);
  CountingChannel* counting = owned.get();
  swarm::Simulation sim(config, demo(8), 2, std::move(owned));
  std::vector<swarm::ChannelEvent> events;
  sim.set_event_sink(&events);
  for (int t = 0; t < 100; ++t) {
    sim.step();
  }
  EXPECT_EQ(counting->sink_sets, 1);
  EXPECT_EQ(counting->sends, counting->stats().sent);
  EXPECT_GT(counting->sends, 0U);
  EXPECT_EQ(events.size(), counting->sends);

  // The same simulation with the default channel reaches the same state.
  swarm::Simulation reference(config, demo(8), 2);
  for (int t = 0; t < 100; ++t) {
    reference.step();
  }
  EXPECT_EQ(sim.state_hash(), reference.state_hash());
}

TEST(ChannelModel, ASimulationRejectsAChannelThatDoesNotFitItsAgents) {
  EXPECT_THROW(
      swarm::Simulation(harsh(), demo(4), 1,
                        std::make_unique<swarm::DeterministicChannel>(5, swarm::ChannelConfig{})),
      std::invalid_argument);
  // A null channel means "use the default".
  swarm::Simulation sim(harsh(), demo(4), 1, nullptr);
  sim.step();
  EXPECT_EQ(sim.agent_count(), 4U);
}
