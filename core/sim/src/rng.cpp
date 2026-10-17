#include "swarm/rng.hpp"

#include <cmath>

namespace swarm {

namespace {
constexpr std::uint64_t kMultiplier = 6364136223846793005ULL;
constexpr double kTwoPi = 6.283185307179586476925286766559;
}  // namespace

Rng::Rng(std::uint64_t seed, std::uint64_t stream) {
  state_ = 0;
  inc_ = (stream << 1U) | 1U;
  next_u32();
  state_ += seed;
  next_u32();
}

std::uint32_t Rng::next_u32() {
  const std::uint64_t old = state_;
  state_ = old * kMultiplier + inc_;
  const auto xorshifted = static_cast<std::uint32_t>(((old >> 18U) ^ old) >> 27U);
  const auto rot = static_cast<std::uint32_t>(old >> 59U);
  return (xorshifted >> rot) | (xorshifted << ((~rot + 1U) & 31U));
}

double Rng::uniform() { return static_cast<double>(next_u32() >> 8U) * (1.0 / 16777216.0); }

double Rng::uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

double Rng::normal() {
  // 1 - u is in (0, 1], so the logarithm is always finite.
  const double u1 = 1.0 - uniform();
  const double u2 = uniform();
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(kTwoPi * u2);
}

bool Rng::bernoulli(double p) { return uniform() < p; }

}  // namespace swarm
