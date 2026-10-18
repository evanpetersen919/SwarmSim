#pragma once

#include <cstdint>
#include <limits>

#include "swarm/vec3.hpp"

namespace swarm {

enum class JamMode {
  kConstant,  // active for the whole window
  kPulsed,    // active for `duty` of every `pulse_period_ticks`
};

enum class JamAffects {
  kReceiver,  // a message is affected when its receiver is inside the zone
  kEither,    // ... when its sender or its receiver is inside
};

// A radio jammer: inside its zone, during its active ticks, messages are dropped with probability
// `loss_inside`.
struct Jammer {
  Vec3 center;
  double radius = 100.0;  // meters; infinity covers everything; a point at exactly radius is inside
  JamMode mode = JamMode::kConstant;
  std::uint64_t start_tick = 0;  // first active tick
  std::uint64_t end_tick =
      std::numeric_limits<std::uint64_t>::max();  // first tick after the window
  // Pulsed mode only. The jammer is on when (tick - start + offset) % period < on_ticks, where
  // on_ticks = round(duty * period). Both the period and on_ticks must be at least 1.
  std::uint64_t pulse_period_ticks = 0;
  double pulse_duty = 0.5;  // in (0, 1]
  std::uint64_t pulse_offset_ticks = 0;
  double loss_inside = 1.0;  // in [0, 1]; 1 drops everything and uses no random draws
  JamAffects affects = JamAffects::kReceiver;
};

// Throws std::invalid_argument if the jammer is malformed.
void validate(const Jammer& jammer);

// Whether the jammer is transmitting at `tick`.
bool jammer_active(const Jammer& jammer, std::uint64_t tick);

// Whether `point` is inside the jammer's zone (distance <= radius).
bool jammer_covers(const Jammer& jammer, const Vec3& point);

}  // namespace swarm
