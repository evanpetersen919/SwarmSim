#pragma once

// Controller headers may include only this directory, swarm/estimation/, sensor_data.hpp,
// vec3.hpp, drone_command.hpp, and swarm/format/byte_io.hpp. They must never include vehicle truth.
// Enforced by scripts/check_core_invariants.sh.

#include <cstdint>
#include <vector>

#include "swarm/vec3.hpp"

namespace swarm {

struct SeparationParams {
  double d_safe = 8.0;      // meters, the margin for a neighbor known exactly
  double kappa = 2.0;       // meters of extra margin per meter of position sigma
  double gain = 4.0;        // m/s^2 of repulsion per meter of penetration
  double max_accel = 12.0;  // m/s^2, cap on the repulsion from each neighbor and on the sum
};

// A neighbor as separation sees it: its slot (used only to break ties), the estimated position,
// and the predictor's position uncertainty.
struct SeparationNeighbor {
  std::uint32_t slot = 0;
  Vec3 position;
  double sigma = 0.0;
};

// Repulsive acceleration for the drone in slot `self` at `own_position`.
//
// A neighbor with uncertainty sigma has the margin d_safe + kappa * sigma. A neighbor farther away
// than its margin contributes nothing. Inside it, the contribution points away from the neighbor
// with magnitude gain * (margin - distance), capped at max_accel. The contributions are summed and
// the sum is capped at max_accel.
//
// When the two positions coincide the direction is along +x for the higher slot and -x for the
// lower one, so the pair still receives equal and opposite pushes. Neighbors that are not finite,
// have a negative sigma, or share this drone's slot are ignored.
//
// Throws std::invalid_argument if the parameters are not finite and positive (kappa may be zero)
// or `own_position` is not finite.
Vec3 separation_accel(std::uint32_t self, const Vec3& own_position,
                      const std::vector<SeparationNeighbor>& neighbors,
                      const SeparationParams& params);

// Adds the formation command to the separation command with the separation term taking priority.
// The separation term is kept whole (up to max_accel); the formation term is limited to the
// headroom that remains under max_accel. A plain clamped sum would let the formation pull cancel
// part of the repulsion exactly when the repulsion is needed.
Vec3 combine_accel(const Vec3& formation, const Vec3& separation, double max_accel);

}  // namespace swarm
