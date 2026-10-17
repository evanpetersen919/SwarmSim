#include <gtest/gtest.h>

#include <cstdint>

#include "swarm/rng.hpp"

// Reference outputs of PCG32 seeded with (42, 54), from the pcg-c minimal demo.
TEST(Rng, MatchesPcg32ReferenceSequence) {
  swarm::Rng rng(42, 54);
  const std::uint32_t expected[] = {0xa15c02b7U, 0x7b47f409U, 0xba1d3330U,
                                    0x83d2f293U, 0xbfa4784bU, 0xcbed606eU};
  for (std::uint32_t value : expected) {
    EXPECT_EQ(rng.next_u32(), value);
  }
}

TEST(Rng, UniformStaysInUnitInterval) {
  swarm::Rng rng(1, 0);
  for (int i = 0; i < 100000; ++i) {
    const double u = rng.uniform();
    ASSERT_GE(u, 0.0);
    ASSERT_LT(u, 1.0);
  }
}

TEST(Rng, NormalHasRoughlyZeroMeanAndUnitVariance) {
  swarm::Rng rng(7, 0);
  const int n = 200000;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int i = 0; i < n; ++i) {
    const double x = rng.normal();
    sum += x;
    sum_sq += x * x;
  }
  const double mean = sum / n;
  const double var = sum_sq / n - mean * mean;
  EXPECT_NEAR(mean, 0.0, 0.02);
  EXPECT_NEAR(var, 1.0, 0.03);
}

TEST(Rng, BernoulliMatchesProbability) {
  swarm::Rng rng(3, 0);
  const int n = 200000;
  int hits = 0;
  for (int i = 0; i < n; ++i) {
    if (rng.bernoulli(0.3)) {
      ++hits;
    }
  }
  EXPECT_NEAR(static_cast<double>(hits) / n, 0.3, 0.01);
}
