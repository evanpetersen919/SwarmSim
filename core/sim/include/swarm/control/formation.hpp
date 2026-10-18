#pragma once

// Controller headers may include only this directory, swarm/estimation/, sensor_data.hpp,
// vec3.hpp, drone_command.hpp, and swarm/format/byte_io.hpp. They must never include vehicle truth.
// Enforced by scripts/check_core_invariants.sh.

#include <cstddef>
#include <vector>

#include "swarm/vec3.hpp"

namespace swarm {

enum class FormationShape {
  kLine,   // slots in a row along y
  kWedge,  // a V with its apex at the front (largest x) and the arms trailing in -x
  kGrid,   // rows of the smallest square-ish column count that fits
};

inline constexpr std::size_t kMaxFormationSlots = 4096;

// Slot offsets for `count` drones, in the world frame, centered so that the offsets average to
// the origin. The closest pair of slots is exactly `spacing` apart (for count >= 2). The offsets
// use only basic arithmetic and fixed constants, so they are identical on every platform.
//
// Throws std::invalid_argument if `spacing` is not finite and positive or `count` exceeds
// kMaxFormationSlots.
std::vector<Vec3> make_formation(FormationShape shape, std::size_t count, double spacing);

}  // namespace swarm
