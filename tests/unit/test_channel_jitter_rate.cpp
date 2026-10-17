#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

#include "swarm/deterministic_channel.hpp"
#include "swarm/rng.hpp"
#include "swarm/streams.hpp"

namespace {

swarm::Message msg(std::uint32_t sender, std::uint32_t receiver) {
  swarm::Message m;
  m.sender = sender;
  m.receiver = receiver;
  return m;
}

swarm::ChannelConfig jittered(std::uint32_t latency, std::uint32_t jitter, std::uint64_t seed = 1) {
  swarm::ChannelConfig c;
  c.latency_ticks = latency;
  c.jitter_ticks = jitter;
  c.seed = seed;
  return c;
}

}  // namespace

TEST(ChannelJitter, ZeroJitterDeliversExactlyAtLatency) {
  swarm::DeterministicChannel channel(2, jittered(2, 0));
  for (int i = 0; i < 100; ++i) {
    channel.send(5, msg(0, 1));
  }
  channel.deliver(100);
  for (const auto& m : channel.take_inbox(1)) {
    EXPECT_EQ(m.deliver_tick, 7U);
  }
}

TEST(ChannelJitter, DelayStaysInRangeAndCoversIt) {
  swarm::DeterministicChannel channel(2, jittered(2, 3));
  for (int i = 0; i < 4000; ++i) {
    channel.send(5, msg(0, 1));
  }
  channel.deliver(100);
  std::set<std::uint64_t> seen;
  for (const auto& m : channel.take_inbox(1)) {
    EXPECT_GE(m.deliver_tick, 7U);
    EXPECT_LE(m.deliver_tick, 10U);
    seen.insert(m.deliver_tick);
  }
  EXPECT_EQ(seen.size(), 4U);
}

TEST(ChannelJitter, CanReorderMessagesButInboxFollowsDeliveryTime) {
  swarm::DeterministicChannel channel(2, jittered(1, 6));
  for (std::uint64_t tick = 0; tick < 300; ++tick) {
    channel.send(tick, msg(0, 1));
  }
  channel.deliver(1000);
  const auto inbox = channel.take_inbox(1);
  ASSERT_EQ(inbox.size(), 300U);
  bool reordered = false;
  for (std::size_t i = 1; i < inbox.size(); ++i) {
    EXPECT_GE(inbox[i].deliver_tick, inbox[i - 1].deliver_tick);
    if (inbox[i].seq < inbox[i - 1].seq) {
      reordered = true;
    }
  }
  EXPECT_TRUE(reordered);
}

TEST(ChannelJitter, DoesNotChangeWhichMessagesAreLost) {
  const auto survivors = [](std::uint32_t jitter) {
    swarm::ChannelConfig c = jittered(1, jitter, 9);
    c.loss_model = swarm::LossModel::kIndependent;
    c.loss = 0.4;
    swarm::DeterministicChannel channel(2, c);
    for (int i = 0; i < 3000; ++i) {
      channel.send(0, msg(0, 1));
    }
    channel.deliver(1000);
    std::set<std::uint64_t> seqs;
    for (const auto& m : channel.take_inbox(1)) {
      seqs.insert(m.seq);
    }
    return seqs;
  };
  EXPECT_EQ(survivors(0), survivors(5));
}

TEST(ChannelJitter, DroppedMessagesDoNotConsumeJitterDraws) {
  swarm::ChannelConfig c = jittered(1, 9, 4);
  c.loss_model = swarm::LossModel::kIndependent;
  c.loss = 0.5;
  swarm::DeterministicChannel channel(2, c);
  for (int i = 0; i < 500; ++i) {
    channel.send(0, msg(0, 1));
  }
  channel.deliver(1000);
  auto inbox = channel.take_inbox(1);
  // Survivors in send order must have used consecutive draws of the jitter stream.
  std::vector<std::uint64_t> delay_by_seq(500, 0);
  std::vector<bool> got(500, false);
  for (const auto& m : inbox) {
    delay_by_seq[m.seq] = m.deliver_tick - 0 - 1;
    got[m.seq] = true;
  }
  swarm::Rng reference(4, swarm::kStreamChannelJitter);
  for (std::size_t seq = 0; seq < 500; ++seq) {
    if (got[seq]) {
      EXPECT_EQ(delay_by_seq[seq], reference.below(10));
    }
  }
}

TEST(ChannelJitter, MaxJitterIsRejected) {
  EXPECT_THROW(
      swarm::DeterministicChannel(2, jittered(1, std::numeric_limits<std::uint32_t>::max())),
      std::invalid_argument);
}

TEST(ChannelRateLimit, DropsMessagesBeyondTheCapWithinATick) {
  swarm::ChannelConfig c;
  c.max_msgs_per_sender_per_tick = 3;
  swarm::DeterministicChannel channel(2, c);
  for (int i = 0; i < 5; ++i) {
    channel.send(0, msg(0, 1));
  }
  channel.deliver(1);
  EXPECT_EQ(channel.take_inbox(1).size(), 3U);
  EXPECT_EQ(channel.stats().dropped_rate, 2U);
  EXPECT_EQ(channel.stats().dropped_loss, 0U);
}

TEST(ChannelRateLimit, CounterResetsOnTheNextTick) {
  swarm::ChannelConfig c;
  c.max_msgs_per_sender_per_tick = 2;
  swarm::DeterministicChannel channel(2, c);
  for (std::uint64_t tick = 0; tick < 5; ++tick) {
    channel.send(tick, msg(0, 1));
    channel.send(tick, msg(0, 1));
  }
  channel.deliver(100);
  EXPECT_EQ(channel.take_inbox(1).size(), 10U);
  EXPECT_EQ(channel.stats().dropped_rate, 0U);
}

TEST(ChannelRateLimit, OtherSendersAreUnaffected) {
  swarm::ChannelConfig c;
  c.max_msgs_per_sender_per_tick = 1;
  swarm::DeterministicChannel channel(3, c);
  for (int i = 0; i < 4; ++i) {
    channel.send(0, msg(0, 2));
    channel.send(0, msg(1, 2));
  }
  channel.deliver(1);
  const auto inbox = channel.take_inbox(2);
  ASSERT_EQ(inbox.size(), 2U);
  EXPECT_EQ(inbox[0].sender, 0U);
  EXPECT_EQ(inbox[1].sender, 1U);
}

TEST(ChannelRateLimit, ZeroMeansUnlimited) {
  swarm::DeterministicChannel channel(2, swarm::ChannelConfig{});
  for (int i = 0; i < 1000; ++i) {
    channel.send(0, msg(0, 1));
  }
  channel.deliver(1);
  EXPECT_EQ(channel.take_inbox(1).size(), 1000U);
}

TEST(ChannelRateLimit, LimitedMessagesDoNotConsumeLossDraws) {
  const auto run = [](std::uint32_t cap, int sends_per_tick) {
    swarm::ChannelConfig c;
    c.loss_model = swarm::LossModel::kIndependent;
    c.loss = 0.5;
    c.seed = 12;
    c.max_msgs_per_sender_per_tick = cap;
    swarm::DeterministicChannel channel(2, c);
    for (std::uint64_t tick = 0; tick < 300; ++tick) {
      for (int i = 0; i < sends_per_tick; ++i) {
        channel.send(tick, msg(0, 1));
      }
    }
    channel.deliver(1000);
    std::vector<std::uint64_t> ticks;
    for (const auto& m : channel.take_inbox(1)) {
      ticks.push_back(m.deliver_tick);
    }
    return ticks;
  };
  // Sending 3 per tick with a cap of 1 must behave like sending 1 per tick with no cap.
  EXPECT_EQ(run(1, 3), run(0, 1));
}
