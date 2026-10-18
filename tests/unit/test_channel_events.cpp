#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "swarm/demo_agent.hpp"
#include "swarm/deterministic_channel.hpp"
#include "swarm/simulation.hpp"

using swarm::ChannelEvent;
using swarm::MessageOutcome;

namespace {

swarm::ChannelConfig harsh() {
  swarm::ChannelConfig c;
  c.seed = 5;
  c.latency_ticks = 2;
  c.jitter_ticks = 3;
  c.loss_model = swarm::LossModel::kBurst;
  c.loss = 0.2;
  c.mean_burst = 3.0;
  c.max_msgs_per_sender_per_tick = 4;
  swarm::Jammer j;
  j.radius = std::numeric_limits<double>::infinity();
  j.start_tick = 20;
  j.end_tick = 40;
  j.loss_inside = 0.5;
  c.jammers = {j};
  return c;
}

swarm::Message message(std::uint32_t from, std::uint32_t to) {
  swarm::Message m;
  m.sender = from;
  m.receiver = to;
  m.payload = {1, 2, 3};
  return m;
}

// Sends `per_tick` messages from every endpoint to the next one, for `ticks` ticks, delivering and
// draining every tick. Returns how many arrived, and the ticks they arrived at.
struct Traffic {
  std::vector<std::uint64_t> arrival_ticks;
  swarm::ChannelStats stats;
};

Traffic drive(swarm::DeterministicChannel& channel, std::uint32_t endpoints, int ticks,
              int per_tick) {
  Traffic t;
  const std::vector<swarm::Vec3> positions(endpoints, swarm::Vec3{});
  for (int tick = 0; tick < ticks + 10; ++tick) {
    channel.set_positions(positions);
    channel.deliver(static_cast<std::uint64_t>(tick));
    for (std::uint32_t e = 0; e < endpoints; ++e) {
      for (const swarm::Message& m : channel.take_inbox(e)) {
        EXPECT_EQ(m.deliver_tick, static_cast<std::uint64_t>(tick));
        t.arrival_ticks.push_back(m.deliver_tick);
      }
    }
    if (tick < ticks) {
      for (std::uint32_t e = 0; e < endpoints; ++e) {
        for (int k = 0; k < per_tick; ++k) {
          channel.send(static_cast<std::uint64_t>(tick), message(e, (e + 1) % endpoints));
        }
      }
    }
  }
  t.stats = channel.stats();
  return t;
}

}  // namespace

TEST(ChannelEvents, ASinkChangesNothingAboutWhatTheChannelDoes) {
  swarm::DeterministicChannel plain(4, harsh());
  swarm::DeterministicChannel watched(4, harsh());
  std::vector<ChannelEvent> events;
  watched.set_event_sink(&events);

  const Traffic a = drive(plain, 4, 80, 3);
  const Traffic b = drive(watched, 4, 80, 3);
  EXPECT_EQ(a.arrival_ticks, b.arrival_ticks);
  EXPECT_EQ(a.stats.sent, b.stats.sent);
  EXPECT_EQ(a.stats.delivered, b.stats.delivered);
  EXPECT_EQ(a.stats.dropped_loss, b.stats.dropped_loss);
  EXPECT_EQ(a.stats.dropped_rate, b.stats.dropped_rate);
  EXPECT_EQ(a.stats.dropped_jam, b.stats.dropped_jam);
  EXPECT_FALSE(events.empty());
}

TEST(ChannelEvents, EveryMessageProducesExactlyOneEventAndTheCountsMatchTheStatistics) {
  swarm::DeterministicChannel channel(4, harsh());
  std::vector<ChannelEvent> events;
  channel.set_event_sink(&events);
  const Traffic t = drive(channel, 4, 80, 6);  // 6 per tick is above the rate limit of 4

  EXPECT_EQ(events.size(), t.stats.sent);
  std::uint64_t scheduled = 0, loss = 0, rate = 0, jam = 0;
  for (const ChannelEvent& e : events) {
    switch (e.outcome) {
      case MessageOutcome::kScheduled:
        ++scheduled;
        break;
      case MessageOutcome::kDroppedLoss:
        ++loss;
        break;
      case MessageOutcome::kDroppedRate:
        ++rate;
        break;
      case MessageOutcome::kDroppedJam:
        ++jam;
        break;
    }
  }
  EXPECT_EQ(scheduled, t.stats.delivered);  // the run drains everything it schedules
  EXPECT_EQ(loss, t.stats.dropped_loss);
  EXPECT_EQ(rate, t.stats.dropped_rate);
  EXPECT_EQ(jam, t.stats.dropped_jam);
  EXPECT_EQ(scheduled + loss + rate + jam, t.stats.sent);
  EXPECT_GT(loss, 0U);
  EXPECT_GT(rate, 0U);
  EXPECT_GT(jam, 0U);
}

TEST(ChannelEvents, EachDropReasonIsReportedAsItself) {
  {
    swarm::ChannelConfig c;
    c.max_msgs_per_sender_per_tick = 2;
    swarm::DeterministicChannel channel(2, c);
    std::vector<ChannelEvent> events;
    channel.set_event_sink(&events);
    for (int i = 0; i < 5; ++i) {
      channel.send(7, message(0, 1));
    }
    ASSERT_EQ(events.size(), 5U);
    EXPECT_EQ(events[1].outcome, MessageOutcome::kScheduled);
    EXPECT_EQ(events[2].outcome, MessageOutcome::kDroppedRate);
    EXPECT_EQ(events[4].outcome, MessageOutcome::kDroppedRate);
    EXPECT_EQ(events[2].deliver_tick, 0U);
  }
  {
    swarm::ChannelConfig c;
    c.loss_model = swarm::LossModel::kIndependent;
    c.loss = 1.0;
    swarm::DeterministicChannel channel(2, c);
    std::vector<ChannelEvent> events;
    channel.set_event_sink(&events);
    channel.send(0, message(0, 1));
    ASSERT_EQ(events.size(), 1U);
    EXPECT_EQ(events[0].outcome, MessageOutcome::kDroppedLoss);
  }
  {
    swarm::ChannelConfig c;
    swarm::Jammer j;
    j.radius = std::numeric_limits<double>::infinity();
    c.jammers = {j};
    swarm::DeterministicChannel channel(2, c);
    channel.set_positions({swarm::Vec3{}, swarm::Vec3{}});
    std::vector<ChannelEvent> events;
    channel.set_event_sink(&events);
    channel.send(0, message(0, 1));
    ASSERT_EQ(events.size(), 1U);
    EXPECT_EQ(events[0].outcome, MessageOutcome::kDroppedJam);
  }
}

TEST(ChannelEvents, EventsCarryTheEndpointsAndTheTicks) {
  swarm::ChannelConfig c;
  c.latency_ticks = 4;
  swarm::DeterministicChannel channel(3, c);
  std::vector<ChannelEvent> events;
  channel.set_event_sink(&events);
  channel.send(11, message(2, 0));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].sender, 2U);
  EXPECT_EQ(events[0].receiver, 0U);
  EXPECT_EQ(events[0].send_tick, 11U);
  EXPECT_EQ(events[0].deliver_tick, 15U);
}

TEST(ChannelEvents, ScheduledDeliveryTicksAreWhenTheMessagesActuallyArrive) {
  swarm::DeterministicChannel channel(3, harsh());
  std::vector<ChannelEvent> events;
  channel.set_event_sink(&events);
  const Traffic t = drive(channel, 3, 60, 2);

  std::vector<std::uint64_t> scheduled;
  for (const ChannelEvent& e : events) {
    if (e.outcome == MessageOutcome::kScheduled) {
      scheduled.push_back(e.deliver_tick);
      const std::uint64_t delay = e.deliver_tick - e.send_tick;
      EXPECT_GE(delay, 2U);       // latency
      EXPECT_LE(delay, 2U + 3U);  // latency plus the largest jitter
    }
  }
  std::vector<std::uint64_t> arrived = t.arrival_ticks;
  std::sort(scheduled.begin(), scheduled.end());
  std::sort(arrived.begin(), arrived.end());
  EXPECT_EQ(scheduled, arrived);
}

TEST(ChannelEvents, TheEventListIsIdenticalForEveryThreadCount) {
  const auto run = [](std::size_t threads) {
    swarm::SimConfig config;
    config.seed = 9;
    config.channel = harsh();
    std::vector<std::unique_ptr<swarm::IAgent>> agents;
    for (std::uint32_t i = 0; i < 8; ++i) {
      agents.push_back(std::make_unique<swarm::DemoAgent>(i, 8));
    }
    swarm::Simulation sim(config, std::move(agents), threads);
    std::vector<ChannelEvent> events;
    sim.set_event_sink(&events);
    for (int i = 0; i < 100; ++i) {
      sim.step();
    }
    return events;
  };
  const std::vector<ChannelEvent> reference = run(1);
  ASSERT_GT(reference.size(), 100U);
  for (std::size_t threads : {2U, 4U, 8U}) {
    const std::vector<ChannelEvent> other = run(threads);
    ASSERT_EQ(other.size(), reference.size()) << "threads " << threads;
    for (std::size_t i = 0; i < reference.size(); ++i) {
      ASSERT_TRUE(other[i].outcome == reference[i].outcome &&
                  other[i].sender == reference[i].sender &&
                  other[i].receiver == reference[i].receiver &&
                  other[i].send_tick == reference[i].send_tick &&
                  other[i].deliver_tick == reference[i].deliver_tick)
          << "event " << i << " threads " << threads;
    }
  }
}
