#pragma once

#include <cstdint>

namespace swarm {

// Fixed-step simulation clock. Time is derived from an integer tick count, so it never
// accumulates floating-point error and never depends on the host clock.
class SimClock {
 public:
  explicit SimClock(double dt_seconds);

  void advance() { ++tick_; }
  std::uint64_t tick() const { return tick_; }
  double dt() const { return dt_; }
  double time() const { return static_cast<double>(tick_) * dt_; }

 private:
  double dt_;
  std::uint64_t tick_ = 0;
};

}  // namespace swarm
