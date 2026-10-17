#include <gtest/gtest.h>

#include <cstdint>

#include "swarm/format/hash.hpp"

TEST(Fnv1a64, EmptyInputIsOffsetBasis) {
  swarm::Fnv1a64 hash;
  EXPECT_EQ(hash.digest(), 14695981039346656037ULL);
}

TEST(Fnv1a64, KnownAnswerForSingleByte) {
  swarm::Fnv1a64 hash;
  hash.update("a", 1);
  EXPECT_EQ(hash.digest(), 0xaf63dc4c8601ec8cULL);
}

TEST(Fnv1a64, OrderMatters) {
  swarm::Fnv1a64 a;
  a.update_u64(1);
  a.update_u64(2);
  swarm::Fnv1a64 b;
  b.update_u64(2);
  b.update_u64(1);
  EXPECT_NE(a.digest(), b.digest());
}

TEST(Fnv1a64, DoubleHashesItsBitPattern) {
  swarm::Fnv1a64 a;
  a.update_f64(1.5);
  swarm::Fnv1a64 b;
  b.update_f64(1.5);
  swarm::Fnv1a64 c;
  c.update_f64(1.5000000000000002);
  EXPECT_EQ(a.digest(), b.digest());
  EXPECT_NE(a.digest(), c.digest());
}
