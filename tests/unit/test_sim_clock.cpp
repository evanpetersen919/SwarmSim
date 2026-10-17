#include <gtest/gtest.h>

#include <stdexcept>

#include "swarm/sim_clock.hpp"

TEST(SimClock, StartsAtZero) {
  swarm::SimClock clock(0.01);
  EXPECT_EQ(clock.tick(), 0U);
  EXPECT_DOUBLE_EQ(clock.time(), 0.0);
}

TEST(SimClock, TimeIsTickTimesDt) {
  swarm::SimClock clock(0.01);
  for (int i = 0; i < 1000; ++i) {
    clock.advance();
  }
  EXPECT_EQ(clock.tick(), 1000U);
  EXPECT_DOUBLE_EQ(clock.time(), 1000.0 * 0.01);
}

TEST(SimClock, RejectsNonPositiveStep) {
  EXPECT_THROW(swarm::SimClock(0.0), std::invalid_argument);
  EXPECT_THROW(swarm::SimClock(-0.1), std::invalid_argument);
}
