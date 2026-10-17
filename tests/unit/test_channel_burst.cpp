#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <vector>

#include "swarm/deterministic_channel.hpp"

namespace {

swarm::ChannelConfig burst(double loss, double mean_burst, std::uint64_t seed = 1) {
  swarm::ChannelConfig c;
  c.loss_model = swarm::LossModel::kBurst;
  c.loss = loss;
  c.mean_burst = mean_burst;
  c.seed = seed;
  return c;
}

swarm::ChannelConfig independent(double loss, std::uint64_t seed = 1) {
  swarm::ChannelConfig c;
  c.loss_model = swarm::LossModel::kIndependent;
  c.loss = loss;
  c.seed = seed;
  return c;
}

struct LinkTrace {
  std::vector<bool> dropped;  // indexed by the message's position on the link
};

// Sends `count` messages on the single link 0 -> 1 and records which were dropped.
LinkTrace trace_single_link(swarm::ChannelConfig config, int count) {
  swarm::DeterministicChannel channel(2, config);
  for (int i = 0; i < count; ++i) {
    swarm::Message m;
    m.sender = 0;
    m.receiver = 1;
    channel.send(0, m);
  }
  channel.deliver(1);
  std::set<std::uint64_t> arrived;
  for (const auto& m : channel.take_inbox(1)) {
    arrived.insert(m.seq);
  }
  LinkTrace trace;
  trace.dropped.resize(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    trace.dropped[static_cast<std::size_t>(i)] = arrived.count(static_cast<std::uint64_t>(i)) == 0;
  }
  return trace;
}

double loss_rate(const LinkTrace& t) {
  std::size_t drops = 0;
  for (bool d : t.dropped) {
    drops += d ? 1U : 0U;
  }
  return static_cast<double>(drops) / static_cast<double>(t.dropped.size());
}

double mean_run_length(const LinkTrace& t) {
  std::size_t runs = 0;
  std::size_t total = 0;
  std::size_t current = 0;
  for (bool d : t.dropped) {
    if (d) {
      ++current;
    } else if (current > 0) {
      ++runs;
      total += current;
      current = 0;
    }
  }
  if (current > 0) {
    ++runs;
    total += current;
  }
  return runs == 0 ? 0.0 : static_cast<double>(total) / static_cast<double>(runs);
}

}  // namespace

TEST(ChannelBurst, MeanLossMatchesTarget) {
  for (double target : {0.1, 0.2, 0.4}) {
    const auto trace = trace_single_link(burst(target, 5.0), 200000);
    EXPECT_NEAR(loss_rate(trace), target, 0.015) << "target " << target;
  }
}

TEST(ChannelBurst, MeanBurstLengthMatchesTarget) {
  for (double length : {2.0, 5.0, 10.0}) {
    const auto trace = trace_single_link(burst(0.2, length), 400000);
    EXPECT_NEAR(mean_run_length(trace), length, 0.3 * length / 5.0 + 0.1) << "length " << length;
  }
}

TEST(ChannelBurst, BurstierThanIndependentAtEqualMeanLoss) {
  const auto bursty = trace_single_link(burst(0.2, 8.0), 200000);
  const auto random = trace_single_link(independent(0.2), 200000);
  EXPECT_NEAR(loss_rate(bursty), loss_rate(random), 0.02);
  // Independent drops at p = 0.2 have mean run 1 / (1 - p) = 1.25.
  EXPECT_GT(mean_run_length(bursty), 3.0 * mean_run_length(random));
}

TEST(ChannelBurst, StateIsPerDirectedLink) {
  // Alternate sends on two links from one sender. Each link must show the target loss, the
  // links must differ from each other, and their joint drop rate must be near p squared.
  const double p = 0.2;
  const int n = 200000;
  swarm::DeterministicChannel channel(3, burst(p, 5.0));
  for (int i = 0; i < n; ++i) {
    for (std::uint32_t receiver : {1U, 2U}) {
      swarm::Message m;
      m.sender = 0;
      m.receiver = receiver;
      channel.send(0, m);
    }
  }
  channel.deliver(1);
  std::vector<bool> got1(static_cast<std::size_t>(2 * n), false);
  std::vector<bool> got2(static_cast<std::size_t>(2 * n), false);
  for (const auto& m : channel.take_inbox(1)) {
    got1[m.seq] = true;
  }
  for (const auto& m : channel.take_inbox(2)) {
    got2[m.seq] = true;
  }
  std::size_t drop1 = 0;
  std::size_t drop2 = 0;
  std::size_t both = 0;
  for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
    const bool d1 = !got1[2 * i];      // seq 2i goes to receiver 1
    const bool d2 = !got2[2 * i + 1];  // seq 2i+1 goes to receiver 2
    drop1 += d1 ? 1U : 0U;
    drop2 += d2 ? 1U : 0U;
    both += (d1 && d2) ? 1U : 0U;
  }
  EXPECT_NEAR(static_cast<double>(drop1) / n, p, 0.015);
  EXPECT_NEAR(static_cast<double>(drop2) / n, p, 0.015);
  EXPECT_NEAR(static_cast<double>(both) / n, p * p, 0.01);
}

TEST(ChannelBurst, ZeroLossDropsNothingAndFullLossDropsAll) {
  EXPECT_DOUBLE_EQ(loss_rate(trace_single_link(burst(0.0, 4.0), 5000)), 0.0);
  EXPECT_DOUBLE_EQ(loss_rate(trace_single_link(burst(1.0, 4.0), 5000)), 1.0);
}

TEST(ChannelBurst, BoundaryOfFeasibleParametersIsAccepted) {
  // loss = 0.5 with mean_burst = 1 needs P(good to bad) = 1: strict alternation.
  const auto trace = trace_single_link(burst(0.5, 1.0), 10000);
  EXPECT_NEAR(loss_rate(trace), 0.5, 0.001);
  EXPECT_DOUBLE_EQ(mean_run_length(trace), 1.0);
}

TEST(ChannelBurst, InfeasibleOrInvalidParametersAreRejected) {
  EXPECT_THROW(swarm::DeterministicChannel(2, burst(0.6, 1.0)), std::invalid_argument);
  EXPECT_THROW(swarm::DeterministicChannel(2, burst(0.2, 0.5)), std::invalid_argument);
  EXPECT_THROW(swarm::DeterministicChannel(2, burst(-0.1, 4.0)), std::invalid_argument);
  EXPECT_THROW(swarm::DeterministicChannel(2, burst(1.1, 4.0)), std::invalid_argument);
}

TEST(ChannelBurst, SameSeedSamePatternDifferentSeedDiffers) {
  const auto a = trace_single_link(burst(0.3, 6.0, 5), 5000);
  const auto b = trace_single_link(burst(0.3, 6.0, 5), 5000);
  const auto c = trace_single_link(burst(0.3, 6.0, 6), 5000);
  EXPECT_EQ(a.dropped, b.dropped);
  EXPECT_NE(a.dropped, c.dropped);
}
