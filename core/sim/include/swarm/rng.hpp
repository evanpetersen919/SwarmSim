#pragma once

#include <cstdint>

namespace swarm {

// PCG32 (XSH-RR). Own implementation because the standard distributions are
// implementation-defined and would break cross-platform replay. Give every subsystem its
// own stream id (sensor noise, channel, weather, ...) so changing one does not shift the
// others.
class Rng {
 public:
  Rng(std::uint64_t seed, std::uint64_t stream);

  std::uint32_t next_u32();
  // Uniform integer in [0, bound) without modulo bias. `bound` must be at least 1.
  std::uint32_t below(std::uint32_t bound);
  // Uniform in [0, 1) with 24 bits of resolution.
  double uniform();
  // Uniform in [lo, hi).
  double uniform(double lo, double hi);
  // Standard normal via Box-Muller. Consumes two uniform draws per call.
  double normal();
  // True with probability p.
  bool bernoulli(double p);

 private:
  std::uint64_t state_ = 0;
  std::uint64_t inc_ = 0;
};

}  // namespace swarm
