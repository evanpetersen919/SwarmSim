#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "swarm/deterministic_channel.hpp"

namespace {

swarm::ChannelConfig independent(double loss, std::uint64_t seed = 1) {
  swarm::ChannelConfig c;
  c.loss_model = swarm::LossModel::kIndependent;
  c.loss = loss;
  c.seed = seed;
  return c;
}

// Sends `count` messages from endpoint 0 to endpoint 1 and returns the seqs that arrived.
std::vector<std::uint64_t> arrivals(swarm::ChannelConfig config, int count) {
  swarm::DeterministicChannel channel(2, config);
  for (int i = 0; i < count; ++i) {
    swarm::Message m;
    m.sender = 0;
    m.receiver = 1;
    channel.send(0, m);
  }
  channel.deliver(1);
  std::vector<std::uint64_t> seqs;
  for (const auto& m : channel.take_inbox(1)) {
    seqs.push_back(m.seq);
  }
  return seqs;
}

}  // namespace

TEST(ChannelLoss, NoLossModelDropsNothing) {
  EXPECT_EQ(arrivals(swarm::ChannelConfig{}, 1000).size(), 1000U);
}

TEST(ChannelLoss, ZeroProbabilityDropsNothing) {
  EXPECT_EQ(arrivals(independent(0.0), 1000).size(), 1000U);
}

TEST(ChannelLoss, ProbabilityOneDropsEverything) {
  EXPECT_TRUE(arrivals(independent(1.0), 1000).empty());
}

TEST(ChannelLoss, ObservedRateMatchesTarget) {
  const int n = 100000;
  for (double target : {0.1, 0.3, 0.5}) {
    const auto received = arrivals(independent(target), n);
    const double observed = 1.0 - static_cast<double>(received.size()) / n;
    EXPECT_NEAR(observed, target, 0.01) << "target " << target;
  }
}

TEST(ChannelLoss, SameSeedSamePattern) {
  EXPECT_EQ(arrivals(independent(0.4, 7), 5000), arrivals(independent(0.4, 7), 5000));
}

TEST(ChannelLoss, DifferentSeedDifferentPattern) {
  EXPECT_NE(arrivals(independent(0.4, 7), 5000), arrivals(independent(0.4, 8), 5000));
}

TEST(ChannelLoss, DroppedMessagesAreCountedAndNeverDelivered) {
  swarm::DeterministicChannel channel(2, independent(0.5));
  for (int i = 0; i < 2000; ++i) {
    swarm::Message m;
    m.sender = 0;
    m.receiver = 1;
    channel.send(0, m);
  }
  channel.deliver(1);
  const auto& stats = channel.stats();
  EXPECT_EQ(stats.sent, 2000U);
  EXPECT_EQ(stats.delivered + stats.dropped_loss, stats.sent);
  EXPECT_EQ(channel.take_inbox(1).size(), stats.delivered);
  EXPECT_GT(stats.dropped_loss, 0U);
}

TEST(ChannelLoss, DroppedMessagesStillConsumeSeq) {
  const auto seqs = arrivals(independent(0.5), 200);
  ASSERT_GT(seqs.size(), 1U);
  EXPECT_LT(seqs.size(), 200U);
  // Gaps in seq show where messages were dropped.
  EXPECT_GT(seqs.back(), seqs.size() - 1);
}

TEST(ChannelLoss, InvalidProbabilityIsRejected) {
  EXPECT_THROW(swarm::DeterministicChannel(2, independent(-0.1)), std::invalid_argument);
  EXPECT_THROW(swarm::DeterministicChannel(2, independent(1.1)), std::invalid_argument);
  EXPECT_THROW(swarm::DeterministicChannel(2, independent(std::nan(""))), std::invalid_argument);
}
