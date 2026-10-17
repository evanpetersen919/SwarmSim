#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>

#include "swarm/deterministic_channel.hpp"

namespace {

swarm::Message msg(std::uint32_t sender, std::uint32_t receiver, std::uint8_t tag = 0) {
  swarm::Message m;
  m.sender = sender;
  m.receiver = receiver;
  m.payload = {tag};
  return m;
}

swarm::ChannelConfig config(std::uint32_t latency) {
  swarm::ChannelConfig c;
  c.latency_ticks = latency;
  return c;
}

}  // namespace

TEST(ChannelLatency, ArrivesExactlyAfterLatency) {
  swarm::DeterministicChannel channel(2, config(3));
  channel.send(10, msg(0, 1));

  channel.deliver(10);
  channel.deliver(11);
  channel.deliver(12);
  EXPECT_TRUE(channel.take_inbox(1).empty());

  channel.deliver(13);
  const auto inbox = channel.take_inbox(1);
  ASSERT_EQ(inbox.size(), 1U);
  EXPECT_EQ(inbox[0].deliver_tick, 13U);
}

TEST(ChannelLatency, MinimumLatencyIsNextTick) {
  swarm::DeterministicChannel channel(2, config(1));
  channel.send(0, msg(0, 1));
  channel.deliver(0);
  EXPECT_TRUE(channel.take_inbox(1).empty());
  channel.deliver(1);
  EXPECT_EQ(channel.take_inbox(1).size(), 1U);
}

TEST(ChannelLatency, ZeroLatencyIsRejected) {
  EXPECT_THROW(swarm::DeterministicChannel(2, config(0)), std::invalid_argument);
}

TEST(ChannelLatency, SeqIncreasesPerSenderIndependently) {
  swarm::DeterministicChannel channel(3, config(1));
  channel.send(0, msg(0, 2));
  channel.send(0, msg(1, 2));
  channel.send(0, msg(0, 2));
  channel.send(0, msg(1, 2));
  channel.send(0, msg(0, 2));
  channel.deliver(1);
  const auto inbox = channel.take_inbox(2);
  ASSERT_EQ(inbox.size(), 5U);
  // Ordered by (tick, sender, seq): sender 0 seqs 0,1,2 then sender 1 seqs 0,1.
  EXPECT_EQ(inbox[0].sender, 0U);
  EXPECT_EQ(inbox[0].seq, 0U);
  EXPECT_EQ(inbox[2].sender, 0U);
  EXPECT_EQ(inbox[2].seq, 2U);
  EXPECT_EQ(inbox[3].sender, 1U);
  EXPECT_EQ(inbox[3].seq, 0U);
  EXPECT_EQ(inbox[4].seq, 1U);
}

TEST(ChannelLatency, InboxesAreSeparatePerReceiver) {
  swarm::DeterministicChannel channel(3, config(1));
  channel.send(0, msg(0, 1, 11));
  channel.send(0, msg(0, 2, 22));
  channel.deliver(1);
  const auto for_one = channel.take_inbox(1);
  const auto for_two = channel.take_inbox(2);
  ASSERT_EQ(for_one.size(), 1U);
  ASSERT_EQ(for_two.size(), 1U);
  EXPECT_EQ(for_one[0].payload[0], 11);
  EXPECT_EQ(for_two[0].payload[0], 22);
  EXPECT_TRUE(channel.take_inbox(0).empty());
}

TEST(ChannelLatency, TakeInboxEmptiesIt) {
  swarm::DeterministicChannel channel(2, config(1));
  channel.send(0, msg(0, 1));
  channel.deliver(1);
  EXPECT_EQ(channel.take_inbox(1).size(), 1U);
  EXPECT_TRUE(channel.take_inbox(1).empty());
}

TEST(ChannelLatency, MessagesAccumulateUntilTaken) {
  swarm::DeterministicChannel channel(2, config(1));
  channel.send(0, msg(0, 1, 1));
  channel.deliver(1);
  channel.send(1, msg(0, 1, 2));
  channel.deliver(2);
  const auto inbox = channel.take_inbox(1);
  ASSERT_EQ(inbox.size(), 2U);
  EXPECT_EQ(inbox[0].payload[0], 1);
  EXPECT_EQ(inbox[1].payload[0], 2);
}

TEST(ChannelLatency, OutOfRangeEndpointsAreRejected) {
  swarm::DeterministicChannel channel(2, config(1));
  EXPECT_THROW(channel.send(0, msg(2, 0)), std::out_of_range);
  EXPECT_THROW(channel.send(0, msg(0, 2)), std::out_of_range);
  EXPECT_THROW(channel.take_inbox(2), std::out_of_range);
}

TEST(ChannelLatency, StatsCountSentAndDelivered) {
  swarm::DeterministicChannel channel(2, config(2));
  channel.send(0, msg(0, 1));
  channel.send(0, msg(1, 0));
  EXPECT_EQ(channel.stats().sent, 2U);
  EXPECT_EQ(channel.stats().delivered, 0U);
  channel.deliver(2);
  EXPECT_EQ(channel.stats().delivered, 2U);
  EXPECT_EQ(channel.stats().dropped_loss, 0U);
  EXPECT_EQ(channel.stats().dropped_rate, 0U);
}
