#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarm/demo_agent.hpp"
#include "swarm/deterministic_channel.hpp"
#include "swarm/jammer.hpp"
#include "swarm/run.hpp"
#include "swarm/simulation.hpp"

using swarm::DeterministicChannel;
using swarm::JamAffects;
using swarm::Jammer;
using swarm::JamMode;
using swarm::Vec3;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

Jammer zone(Vec3 center, double radius) {
  Jammer j;
  j.center = center;
  j.radius = radius;
  return j;
}

// Endpoints 0, 1, 2 on a line, 100 m apart.
std::vector<Vec3> line_positions() { return {{0, 0, 0}, {100, 0, 0}, {200, 0, 0}}; }

swarm::ChannelConfig with(std::vector<Jammer> jammers) {
  swarm::ChannelConfig c;
  c.jammers = std::move(jammers);
  return c;
}

swarm::Message msg(std::uint32_t sender, std::uint32_t receiver) {
  swarm::Message m;
  m.sender = sender;
  m.receiver = receiver;
  return m;
}

// Sends one message at `tick` and reports whether it reaches the receiver.
bool delivered(DeterministicChannel& channel, std::uint64_t tick, std::uint32_t sender,
               std::uint32_t receiver) {
  channel.send(tick, msg(sender, receiver));
  channel.deliver(tick + 1);
  return !channel.take_inbox(receiver).empty();
}

}  // namespace

TEST(Jammer, CoverageIsInclusiveOfTheBoundary) {
  const Jammer j = zone({0, 0, 0}, 10.0);
  EXPECT_TRUE(swarm::jammer_covers(j, {0, 0, 0}));
  EXPECT_TRUE(swarm::jammer_covers(j, {10, 0, 0}));
  EXPECT_TRUE(swarm::jammer_covers(j, {6, 8, 0}));  // distance exactly 10
  EXPECT_FALSE(swarm::jammer_covers(j, {10.001, 0, 0}));
  EXPECT_TRUE(swarm::jammer_covers(zone({0, 0, 0}, kInf), {1e12, -1e12, 5e11}));
  EXPECT_FALSE(swarm::jammer_covers(j, {std::nan(""), 0, 0}));
}

TEST(Jammer, ConstantJammerIsActiveExactlyInsideItsWindow) {
  Jammer j = zone({0, 0, 0}, 1.0);
  j.start_tick = 10;
  j.end_tick = 20;
  EXPECT_FALSE(swarm::jammer_active(j, 9));
  EXPECT_TRUE(swarm::jammer_active(j, 10));
  EXPECT_TRUE(swarm::jammer_active(j, 19));
  EXPECT_FALSE(swarm::jammer_active(j, 20));
}

TEST(Jammer, PulsedJammerIsOnForDutyTimesPeriodTicksEveryPeriod) {
  Jammer j = zone({0, 0, 0}, 1.0);
  j.mode = JamMode::kPulsed;
  j.pulse_period_ticks = 10;
  j.pulse_duty = 0.3;
  std::set<std::uint64_t> on;
  for (std::uint64_t t = 0; t < 100; ++t) {
    if (swarm::jammer_active(j, t)) {
      on.insert(t);
    }
  }
  EXPECT_EQ(on.size(), 30U);
  for (std::uint64_t t : on) {
    EXPECT_LT(t % 10, 3U);
  }
}

TEST(Jammer, PulseOffsetShiftsThePattern) {
  Jammer j = zone({0, 0, 0}, 1.0);
  j.mode = JamMode::kPulsed;
  j.pulse_period_ticks = 10;
  j.pulse_duty = 0.3;
  j.pulse_offset_ticks = 2;
  for (std::uint64_t t = 0; t < 50; ++t) {
    const std::uint64_t phase = t % 10;
    EXPECT_EQ(swarm::jammer_active(j, t), phase == 8 || phase == 9 || phase == 0) << "tick " << t;
  }
}

TEST(Jammer, PulsePhaseIsMeasuredFromTheStartOfTheWindow) {
  Jammer j = zone({0, 0, 0}, 1.0);
  j.mode = JamMode::kPulsed;
  j.start_tick = 7;
  j.pulse_period_ticks = 4;
  j.pulse_duty = 0.5;
  EXPECT_TRUE(swarm::jammer_active(j, 7));
  EXPECT_TRUE(swarm::jammer_active(j, 8));
  EXPECT_FALSE(swarm::jammer_active(j, 9));
  EXPECT_FALSE(swarm::jammer_active(j, 10));
  EXPECT_TRUE(swarm::jammer_active(j, 11));
}

TEST(Jammer, ValidationRejectsMalformedJammers) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto rejects = [](Jammer j) { EXPECT_THROW(swarm::validate(j), std::invalid_argument); };
  EXPECT_NO_THROW(swarm::validate(zone({0, 0, 0}, kInf)));
  EXPECT_NO_THROW(swarm::validate(zone({0, 0, 0}, 0.0)));

  Jammer j = zone({0, 0, 0}, -1.0);
  rejects(j);
  j = zone({0, 0, 0}, nan);
  rejects(j);
  j = zone({nan, 0, 0}, 5.0);
  rejects(j);
  j = zone({0, 0, 0}, 5.0);
  j.start_tick = 10;
  j.end_tick = 10;  // empty window
  rejects(j);
  j = zone({0, 0, 0}, 5.0);
  j.loss_inside = 1.5;
  rejects(j);
  j.loss_inside = -0.1;
  rejects(j);
  j = zone({0, 0, 0}, 5.0);
  j.mode = JamMode::kPulsed;
  j.pulse_period_ticks = 0;
  rejects(j);
  j.pulse_period_ticks = 10;
  j.pulse_duty = 0.0;
  rejects(j);
  j.pulse_duty = 1.5;
  rejects(j);
  j.pulse_duty = 0.04;  // 0.4 ticks rounds to zero
  rejects(j);
  j.pulse_duty = 0.05;  // 0.5 rounds up to one tick
  EXPECT_NO_THROW(swarm::validate(j));
}

TEST(ChannelJamming, ReceiverInsideTheZoneIsDroppedAndOutsideIsDelivered) {
  DeterministicChannel channel(3, with({zone({100, 0, 0}, 20.0)}));
  channel.set_positions(line_positions());
  EXPECT_FALSE(delivered(channel, 0, 0, 1));  // receiver 1 is inside
  EXPECT_TRUE(delivered(channel, 1, 0, 2));   // receiver 2 is outside
  EXPECT_TRUE(delivered(channel, 2, 1, 0));   // receiver 0 is outside
  EXPECT_EQ(channel.stats().dropped_jam, 1U);
  EXPECT_EQ(channel.stats().dropped_loss, 0U);
}

TEST(ChannelJamming, AReceiverExactlyOnTheBoundaryIsInside) {
  DeterministicChannel channel(3, with({zone({0, 0, 0}, 100.0)}));
  channel.set_positions(line_positions());  // endpoint 1 is exactly 100 m away
  EXPECT_FALSE(delivered(channel, 0, 2, 1));
}

TEST(ChannelJamming, SenderInsideOnlyMattersInEitherMode) {
  Jammer receiver_only = zone({0, 0, 0}, 20.0);  // covers endpoint 0, the sender
  Jammer either = receiver_only;
  either.affects = JamAffects::kEither;

  DeterministicChannel a(3, with({receiver_only}));
  a.set_positions(line_positions());
  EXPECT_TRUE(delivered(a, 0, 0, 1));

  DeterministicChannel b(3, with({either}));
  b.set_positions(line_positions());
  EXPECT_FALSE(delivered(b, 0, 0, 1));
  EXPECT_FALSE(delivered(b, 1, 1, 0));  // and the receiver side still counts
  EXPECT_TRUE(delivered(b, 2, 1, 2));   // neither endpoint inside
}

TEST(ChannelJamming, NothingIsDroppedOutsideTheActiveWindow) {
  Jammer j = zone({100, 0, 0}, 20.0);
  j.start_tick = 10;
  j.end_tick = 20;
  DeterministicChannel channel(3, with({j}));
  channel.set_positions(line_positions());
  for (std::uint64_t tick = 0; tick < 30; ++tick) {
    const bool expected = tick < 10 || tick >= 20;
    EXPECT_EQ(delivered(channel, tick, 0, 1), expected) << "tick " << tick;
  }
}

TEST(ChannelJamming, PulsedJammerDropsOnlyDuringItsOnTicks) {
  Jammer j = zone({100, 0, 0}, 20.0);
  j.mode = JamMode::kPulsed;
  j.pulse_period_ticks = 10;
  j.pulse_duty = 0.3;
  DeterministicChannel channel(3, with({j}));
  channel.set_positions(line_positions());
  int dropped = 0;
  for (std::uint64_t tick = 0; tick < 100; ++tick) {
    const bool got = delivered(channel, tick, 0, 1);
    EXPECT_EQ(got, tick % 10 >= 3) << "tick " << tick;
    dropped += got ? 0 : 1;
  }
  EXPECT_EQ(dropped, 30);
}

TEST(ChannelJamming, PartialLossDropsTheConfiguredShare) {
  Jammer j = zone({100, 0, 0}, 20.0);
  j.loss_inside = 0.5;
  swarm::ChannelConfig config = with({j});
  config.seed = 4;
  DeterministicChannel channel(3, config);
  channel.set_positions(line_positions());
  int kept = 0;
  const int n = 20000;
  for (int i = 0; i < n; ++i) {
    kept += delivered(channel, static_cast<std::uint64_t>(i), 0, 1) ? 1 : 0;
  }
  EXPECT_NEAR(static_cast<double>(kept) / n, 0.5, 0.02);
}

TEST(ChannelJamming, AJammerWithFullLossUsesNoRandomDraws) {
  // Channel X has a full-loss jammer over endpoint 1 and a half-loss jammer over endpoint 2.
  // Channel Y has only the second. If the first drew random numbers, endpoint 2's fates would
  // differ.
  Jammer full = zone({100, 0, 0}, 20.0);
  Jammer half = zone({200, 0, 0}, 20.0);
  half.loss_inside = 0.5;
  swarm::ChannelConfig cx = with({full, half});
  swarm::ChannelConfig cy = with({half});
  cx.seed = cy.seed = 9;
  DeterministicChannel x(3, cx);
  DeterministicChannel y(3, cy);
  x.set_positions(line_positions());
  y.set_positions(line_positions());
  for (std::uint64_t tick = 0; tick < 500; ++tick) {
    delivered(x, tick, 0, 1);  // consumes nothing from the jam stream
    EXPECT_EQ(delivered(x, tick, 0, 2), delivered(y, tick, 0, 2)) << "tick " << tick;
  }
}

TEST(ChannelJamming, AddingAJammerDoesNotChangeWhichMessagesOutsideItAreLost) {
  swarm::ChannelConfig base;
  base.loss_model = swarm::LossModel::kIndependent;
  base.loss = 0.3;
  base.seed = 5;
  swarm::ChannelConfig jammed = base;
  jammed.jammers = {zone({100, 0, 0}, 20.0)};

  DeterministicChannel plain(3, base);
  DeterministicChannel with_jammer(3, jammed);
  plain.set_positions(line_positions());
  with_jammer.set_positions(line_positions());
  for (std::uint64_t tick = 0; tick < 2000; ++tick) {
    // Alternate receivers so both channels advance their loss stream identically.
    const std::uint32_t receiver = tick % 2 == 0 ? 1U : 2U;
    const bool a = delivered(plain, tick, 0, receiver);
    const bool b = delivered(with_jammer, tick, 0, receiver);
    if (receiver == 2) {
      EXPECT_EQ(a, b) << "tick " << tick;  // outside the zone: identical
    } else {
      EXPECT_FALSE(b);  // inside the zone: always dropped
    }
  }
}

TEST(ChannelJamming, SeveralJammersCombine) {
  DeterministicChannel channel(3, with({zone({100, 0, 0}, 20.0), zone({200, 0, 0}, 20.0)}));
  channel.set_positions(line_positions());
  EXPECT_TRUE(delivered(channel, 0, 1, 0));
  EXPECT_FALSE(delivered(channel, 1, 0, 1));
  EXPECT_FALSE(delivered(channel, 2, 0, 2));
}

TEST(ChannelJamming, ReceiverMovingInAndOutChangesItsFate) {
  DeterministicChannel channel(2, with({zone({0, 0, 0}, 50.0)}));
  for (std::uint64_t tick = 0; tick < 20; ++tick) {
    const double x = 200.0 - 20.0 * static_cast<double>(tick);  // sweeps through the zone
    channel.set_positions({{x, 0, 0}, {x, 0, 0}});
    const bool inside = std::abs(x) <= 50.0;
    EXPECT_EQ(delivered(channel, tick, 0, 1), !inside) << "x " << x;
  }
}

TEST(ChannelJamming, JammersWithoutPositionsAreAnError) {
  DeterministicChannel channel(3, with({zone({0, 0, 0}, 10.0)}));
  EXPECT_THROW(channel.send(0, msg(0, 1)), std::logic_error);
  channel.set_positions({{0, 0, 0}});  // wrong size
  EXPECT_THROW(channel.send(0, msg(0, 1)), std::logic_error);
}

TEST(ChannelJamming, ChannelsWithoutJammersIgnorePositions) {
  DeterministicChannel channel(3, swarm::ChannelConfig{});
  EXPECT_TRUE(delivered(channel, 0, 0, 1));  // never given positions
}

TEST(ChannelJamming, InvalidJammersAreRejectedByTheChannel) {
  Jammer bad = zone({0, 0, 0}, -5.0);
  EXPECT_THROW(DeterministicChannel(2, with({bad})), std::invalid_argument);
}

TEST(JammerText, RoundTripsEveryFieldIncludingInfiniteRadiusAndOpenWindow) {
  Jammer j;
  j.center = {1.5, -2.25, 3.125};
  j.radius = kInf;
  j.mode = JamMode::kPulsed;
  j.start_tick = 12;
  j.pulse_period_ticks = 40;
  j.pulse_duty = 0.35;
  j.pulse_offset_ticks = 7;
  j.loss_inside = 0.8;
  j.affects = JamAffects::kEither;

  Jammer back;
  std::string error;
  ASSERT_TRUE(swarm::parse_jammer(swarm::format_jammer(j), back, error)) << error;
  EXPECT_EQ(back.center, j.center);
  EXPECT_EQ(back.radius, kInf);
  EXPECT_EQ(back.mode, JamMode::kPulsed);
  EXPECT_EQ(back.start_tick, 12U);
  EXPECT_EQ(back.end_tick, std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(back.pulse_period_ticks, 40U);
  EXPECT_EQ(back.pulse_duty, 0.35);
  EXPECT_EQ(back.pulse_offset_ticks, 7U);
  EXPECT_EQ(back.loss_inside, 0.8);
  EXPECT_EQ(back.affects, JamAffects::kEither);
}

TEST(JammerText, MalformedTextIsRejectedWithAReason) {
  Jammer out;
  std::string error;
  EXPECT_FALSE(swarm::parse_jammer("1,2,3", out, error));
  EXPECT_NE(error.find("12"), std::string::npos);
  EXPECT_FALSE(swarm::parse_jammer("0,0,0,10,loud,0,100,0,0.5,0,1,receiver", out, error));
  EXPECT_FALSE(swarm::parse_jammer("0,0,0,10,constant,0,100,0,0.5,0,1,everyone", out, error));
  EXPECT_FALSE(swarm::parse_jammer("0,0,x,10,constant,0,100,0,0.5,0,1,receiver", out, error));
  EXPECT_FALSE(swarm::parse_jammer("0,0,0,10,constant,0,100,0,0.5,0,2.0,receiver", out, error));
  EXPECT_FALSE(swarm::parse_jammer("0,0,0,-3,constant,0,100,0,0.5,0,1,receiver", out, error));
  EXPECT_TRUE(swarm::parse_jammer("0,0,0,10,constant,0,100,0,0.5,0,1,receiver", out, error));
}

TEST(JammerText, RunConfigCarriesJammersInOrder) {
  swarm::RunConfig config;
  config.agents = 4;
  config.ticks = 10;
  Jammer a = zone({1, 2, 3}, 4.0);
  Jammer b = zone({5, 6, 7}, kInf);
  b.mode = JamMode::kPulsed;
  b.pulse_period_ticks = 8;
  config.sim.channel.jammers = {a, b};

  swarm::RunConfig parsed;
  std::string error;
  ASSERT_TRUE(swarm::from_log_header(swarm::to_log_header(config), parsed, error)) << error;
  ASSERT_EQ(parsed.sim.channel.jammers.size(), 2U);
  EXPECT_EQ(parsed.sim.channel.jammers[0].center, a.center);
  EXPECT_EQ(parsed.sim.channel.jammers[1].radius, kInf);
  EXPECT_EQ(parsed.sim.channel.jammers[1].mode, JamMode::kPulsed);
}

TEST(JammerText, MalformedJammerLinesInTheHeaderAreRejected) {
  swarm::LogHeader header = swarm::to_log_header(swarm::RunConfig{});
  header.config += "jammer=not,enough\n";
  swarm::RunConfig out;
  std::string error;
  EXPECT_FALSE(swarm::from_log_header(header, out, error));
  EXPECT_NE(error.find("jammer"), std::string::npos);
}

namespace {

using AgentList = std::vector<std::unique_ptr<swarm::IAgent>>;

// Sits on the line x = 10 * id and sends one message to every other agent each tick, recording
// how many messages arrive at each tick.
class Broadcaster : public swarm::IAgent {
 public:
  Broadcaster(std::uint32_t id, std::uint32_t count) : id_(id), count_(count) {}
  void step(swarm::AgentContext& ctx) override {
    arrivals_.resize(ctx.tick + 1, 0);
    arrivals_[ctx.tick] = ctx.inbox.size();
    for (std::uint32_t other = 0; other < count_; ++other) {
      if (other != id_) {
        swarm::Message m;
        m.receiver = other;
        ctx.outbox.push_back(m);
      }
    }
  }
  Vec3 position() const override { return {10.0 * id_, 0.0, 0.0}; }
  void hash_state(swarm::Fnv1a64& hash) const override { hash.update_u64(arrivals_.size()); }
  std::size_t arrivals_at(std::uint64_t tick) const {
    return tick < arrivals_.size() ? arrivals_[tick] : 0;
  }

 private:
  std::uint32_t id_;
  std::uint32_t count_;
  std::vector<std::size_t> arrivals_;
};

}  // namespace

TEST(SimulationJamming, AgentsInsideTheZoneHearNothingDuringTheWindowAndOthersAreUnaffected) {
  AgentList agents;
  for (std::uint32_t i = 0; i < 8; ++i) {
    agents.push_back(std::make_unique<Broadcaster>(i, 8));
  }
  swarm::SimConfig config;
  Jammer j = zone({45, 0, 0}, 12.0);  // covers the agents at x = 40 and x = 50
  j.start_tick = 20;
  j.end_tick = 60;
  config.channel.jammers = {j};
  swarm::Simulation sim(config, std::move(agents), 4);
  for (int t = 0; t < 100; ++t) {
    sim.step();
  }

  const auto agent = [&](std::size_t i) -> const Broadcaster& {
    return dynamic_cast<const Broadcaster&>(sim.agent(i));
  };
  // A message sent at tick t arrives at tick t + 1, so the jammed arrivals are ticks 21 to 60.
  for (std::size_t inside : {4U, 5U}) {
    EXPECT_EQ(agent(inside).arrivals_at(20), 7U);
    for (std::uint64_t t = 21; t <= 60; ++t) {
      ASSERT_EQ(agent(inside).arrivals_at(t), 0U) << "agent " << inside << " tick " << t;
    }
    EXPECT_EQ(agent(inside).arrivals_at(61), 7U);
  }
  for (std::size_t outside : {0U, 3U, 6U, 7U}) {
    for (std::uint64_t t = 1; t < 100; ++t) {
      ASSERT_EQ(agent(outside).arrivals_at(t), 7U) << "agent " << outside << " tick " << t;
    }
  }
  EXPECT_GT(sim.channel().stats().dropped_jam, 0U);
}

TEST(SimulationJamming, HashesAreIdenticalForAnyThreadCount) {
  swarm::SimConfig config;
  config.seed = 31;
  config.channel.latency_ticks = 2;
  config.channel.jitter_ticks = 1;
  config.channel.loss_model = swarm::LossModel::kBurst;
  config.channel.loss = 0.1;
  config.channel.mean_burst = 3.0;
  Jammer local = zone({80, 0, 0}, 30.0);
  local.loss_inside = 0.7;
  Jammer global = zone({0, 0, 0}, kInf);
  global.mode = JamMode::kPulsed;
  global.pulse_period_ticks = 25;
  global.pulse_duty = 0.2;
  global.affects = JamAffects::kEither;
  config.channel.jammers = {local, global};

  const auto run = [&](std::size_t threads) {
    AgentList agents;
    for (std::uint32_t i = 0; i < 16; ++i) {
      agents.push_back(std::make_unique<swarm::DemoAgent>(i, 16));
    }
    swarm::Simulation sim(config, std::move(agents), threads);
    std::vector<std::uint64_t> hashes;
    for (int t = 0; t < 200; ++t) {
      sim.step();
      hashes.push_back(sim.state_hash());
    }
    EXPECT_GT(sim.channel().stats().dropped_jam, 0U);
    return hashes;
  };
  const auto reference = run(1);
  for (std::size_t threads : {2U, 4U, 8U}) {
    EXPECT_EQ(run(threads), reference) << "threads " << threads;
  }
}

TEST(SimulationJamming, RecordedRunsWithJammersVerify) {
  swarm::RunConfig config;
  config.sim.seed = 3;
  config.agents = 12;
  config.ticks = 150;
  Jammer j = zone({40, 0, 0}, 25.0);
  j.mode = JamMode::kPulsed;
  j.start_tick = 30;
  j.end_tick = 120;
  j.pulse_period_ticks = 20;
  j.pulse_duty = 0.5;
  config.sim.channel.jammers = {j};

  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::record_run(config, 1, bytes), swarm::LogStatus::kOk);
  EXPECT_EQ(swarm::verify_log(bytes.data(), bytes.size(), 8).status, swarm::VerifyStatus::kOk);
}
