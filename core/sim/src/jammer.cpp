#include "swarm/jammer.hpp"

#include <cmath>
#include <stdexcept>

namespace swarm {

namespace {

std::uint64_t on_ticks(const Jammer& j) {
  return static_cast<std::uint64_t>(
      std::floor(j.pulse_duty * static_cast<double>(j.pulse_period_ticks) + 0.5));
}

}  // namespace

void validate(const Jammer& j) {
  if (!std::isfinite(j.center.x) || !std::isfinite(j.center.y) || !std::isfinite(j.center.z)) {
    throw std::invalid_argument("Jammer: center must be finite");
  }
  if (!(j.radius >= 0.0)) {  // also rejects NaN; infinity is allowed
    throw std::invalid_argument("Jammer: radius must be non-negative");
  }
  if (j.end_tick <= j.start_tick) {
    throw std::invalid_argument("Jammer: the active window must contain at least one tick");
  }
  if (!(j.loss_inside >= 0.0 && j.loss_inside <= 1.0)) {
    throw std::invalid_argument("Jammer: loss_inside must be in [0, 1]");
  }
  if (j.mode == JamMode::kPulsed) {
    if (j.pulse_period_ticks < 1) {
      throw std::invalid_argument("Jammer: pulse period must be at least 1 tick");
    }
    if (!(j.pulse_duty > 0.0 && j.pulse_duty <= 1.0)) {
      throw std::invalid_argument("Jammer: pulse duty must be in (0, 1]");
    }
    if (on_ticks(j) < 1) {
      throw std::invalid_argument("Jammer: duty * period rounds to zero on-ticks");
    }
  }
}

bool jammer_active(const Jammer& j, std::uint64_t tick) {
  if (tick < j.start_tick || tick >= j.end_tick) {
    return false;
  }
  if (j.mode == JamMode::kConstant) {
    return true;
  }
  const std::uint64_t phase = (tick - j.start_tick + j.pulse_offset_ticks) % j.pulse_period_ticks;
  return phase < on_ticks(j);
}

bool jammer_covers(const Jammer& j, const Vec3& point) {
  return norm_squared(point - j.center) <= j.radius * j.radius;
}

}  // namespace swarm
