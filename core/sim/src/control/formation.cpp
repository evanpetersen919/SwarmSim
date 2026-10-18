#include "swarm/control/formation.hpp"

#include <cmath>
#include <stdexcept>

namespace swarm {

namespace {

// sqrt(3) / 2: the rows of a wedge whose arms make 60 degrees with each other.
constexpr double kHalfSqrt3 = 0.86602540378443864676;

void center(std::vector<Vec3>& slots) {
  if (slots.empty()) {
    return;
  }
  Vec3 mean;
  for (const Vec3& s : slots) {
    mean += s;
  }
  mean = mean / static_cast<double>(slots.size());
  for (Vec3& s : slots) {
    s -= mean;
  }
}

}  // namespace

std::vector<Vec3> make_formation(FormationShape shape, std::size_t count, double spacing) {
  if (!(std::isfinite(spacing) && spacing > 0.0)) {
    throw std::invalid_argument("make_formation: spacing must be finite and positive");
  }
  if (count > kMaxFormationSlots) {
    throw std::invalid_argument("make_formation: too many slots");
  }

  std::vector<Vec3> slots;
  slots.reserve(count);

  switch (shape) {
    case FormationShape::kLine:
      for (std::size_t k = 0; k < count; ++k) {
        slots.push_back({0.0, static_cast<double>(k) * spacing, 0.0});
      }
      break;

    case FormationShape::kWedge:
      // Slot 0 is the apex. Later slots alternate sides, one more row back for every pair. Along
      // an arm and between the apex and the first row the spacing is exact; across the arms it is
      // row * spacing.
      for (std::size_t k = 0; k < count; ++k) {
        if (k == 0) {
          slots.push_back({0.0, 0.0, 0.0});
          continue;
        }
        const double row = static_cast<double>((k + 1) / 2);
        const double side = (k % 2 == 1) ? 1.0 : -1.0;
        slots.push_back({-row * spacing * kHalfSqrt3, side * row * spacing * 0.5, 0.0});
      }
      break;

    case FormationShape::kGrid: {
      std::size_t columns = 1;
      while (columns * columns < count) {
        ++columns;
      }
      for (std::size_t k = 0; k < count; ++k) {
        slots.push_back({static_cast<double>(k / columns) * spacing,
                         static_cast<double>(k % columns) * spacing, 0.0});
      }
      break;
    }
  }

  center(slots);
  return slots;
}

}  // namespace swarm
