#include <gtest/gtest.h>

#include <cstdint>

#include "swarm/format/hash.hpp"
#include "swarm/rng.hpp"

namespace {

std::uint64_t hash_of_draws(std::uint64_t seed, std::uint64_t stream, int count) {
  swarm::Rng rng(seed, stream);
  swarm::Fnv1a64 hash;
  for (int i = 0; i < count; ++i) {
    hash.update_f64(rng.normal());
  }
  return hash.digest();
}

}  // namespace

TEST(RngStreams, SameSeedAndStreamReproduceExactly) {
  EXPECT_EQ(hash_of_draws(42, 0, 10000), hash_of_draws(42, 0, 10000));
}

TEST(RngStreams, DifferentSeedsDiverge) {
  EXPECT_NE(hash_of_draws(42, 0, 100), hash_of_draws(43, 0, 100));
}

TEST(RngStreams, DifferentStreamsDiverge) {
  EXPECT_NE(hash_of_draws(42, 0, 100), hash_of_draws(42, 1, 100));
}

TEST(RngStreams, ExtraDrawsInOneStreamDoNotShiftAnother) {
  swarm::Rng a(42, 1);
  swarm::Rng b(42, 2);
  const std::uint32_t b_first = swarm::Rng(42, 2).next_u32();
  for (int i = 0; i < 50; ++i) {
    a.next_u32();
  }
  EXPECT_EQ(b.next_u32(), b_first);
}
