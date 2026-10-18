#pragma once

// Controller headers may include only this directory, swarm/estimation/, sensor_data.hpp,
// vec3.hpp, drone_command.hpp, and swarm/format/byte_io.hpp. They must never include vehicle truth.
// Enforced by scripts/check_core_invariants.sh.

#include <cstdint>
#include <vector>

#include "swarm/control/mission.hpp"
#include "swarm/estimation/neighbor_predictor.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

struct FormationParams {
  // Weight on the shared mission reference versus the neighbor consensus, in [0, 1]. With no usable
  // neighbor the reference gets all of the weight whatever this is set to.
  double alpha = 0.5;
  double kp = 1.0;           // position gain, 1/s^2
  double kd = 1.8;           // velocity gain, 1/s (a damping ratio of 0.9 with the default kp)
  double sigma_floor = 0.5;  // meters, added in quadrature to every estimate's sigma
  double max_accel = 8.0;    // m/s^2, the airframe limit the command is clamped to
};

// A neighbor as the controller knows it: the slot it occupies and the predictor's current estimate.
struct FormationNeighbor {
  std::uint32_t slot = 0;
  NeighborEstimate estimate;
};

struct FormationTarget {
  Vec3 position;  // where this drone should be
  Vec3 velocity;  // how fast it should be moving
  std::size_t neighbors_used = 0;
};

// Where a drone in slot `self` should be, from what it knows.
//
// Each drone estimates the center of the formation as its own position minus its slot offset. The
// formation center is taken to be
//
//   alpha * reference + (1 - alpha) * consensus
//
// where the consensus is the inverse-variance weighted mean of the center estimates of this drone
// and of every usable neighbor. A neighbor's weight is 1 / (sigma^2 + sigma_floor^2), so an old
// prediction, which carries a larger sigma, counts for less. The target position is that center
// plus this drone's offset, and the target velocity is blended the same way.
//
// A neighbor is unusable if its estimate is not valid, not finite, or its slot is outside the
// formation or is this drone's own. With no usable neighbor the target is the reference alone.
//
// Throws std::invalid_argument if `self` is outside the formation, the parameters are invalid, or
// the own estimate is not finite.
FormationTarget formation_target(std::uint32_t self, const Vec3& own_position,
                                 const Vec3& own_velocity, double own_sigma,
                                 const std::vector<FormationNeighbor>& neighbors,
                                 const std::vector<Vec3>& offsets,
                                 const MissionReference& reference, const FormationParams& params);

// Proportional-derivative acceleration toward the target, clamped to params.max_accel:
//   a = kp * (target.position - position) + kd * (target.velocity - velocity)
Vec3 formation_accel(const FormationTarget& target, const Vec3& own_position,
                     const Vec3& own_velocity, const FormationParams& params);

}  // namespace swarm
