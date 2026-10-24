#pragma once
// Controller headers may include only this directory, swarm/estimation/, sensor_data.hpp,
// vec3.hpp, drone_command.hpp, message.hpp, and swarm/format/{byte_io,hash}.hpp. They must
// never include vehicle truth.
// Enforced by scripts/check_core_invariants.sh.

#include <cstdint>
#include <optional>
#include <vector>

#include "swarm/control/separation.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

// Guidance for a swarm of vehicles that cannot stop (ADR 0010): every drone flies a circle around
// a shared center. The reference is a pure function of time, as the mission path is, so every
// drone computes everyone's reference position from the tick alone and nominal flight needs no
// message.
//
// For slot i of n at time t, with omega = speed / radius:
//   radius R_i   = radius + (i mod 2) * radius_step
//   altitude z_i = center.z + i * altitude_step
//   angle        = 2 pi i / n + omega t, counterclockwise from +x
//   speed V_i    = omega R_i
// Every drone has the same angular rate, so the angular spacing does not drift.
//
// All values here are assumptions (docs/MODELS.md). The limits are copied from the vehicle by the
// scenario builder, because controllers do not see the vehicle.
struct OrbitParams {
  Vec3 center;                  // horizontal center; z is the altitude of slot 0
  double radius = 150.0;        // m, base radius
  double radius_step = 12.0;    // m, added to odd slots
  double altitude_step = 10.0;  // m, added per slot
  double speed = 17.0;          // m/s on the base radius
  double min_speed = 12.0;      // m/s, the vehicle's limits
  double max_speed = 25.0;
  double speed_margin = 1.0;  // m/s kept clear of both limits when asking for a speed
  // ADR 0019. The most the phase correction may ask for above the commanded speed, m/s. Empty means
  // no cap beyond the speed limit, as before. Slowing down to correct the phase is not capped.
  std::optional<double> max_catchup;

  double radial_gain = 1.0;       // cut toward the circle per unit of relative radial error
  double phase_gain = 0.05;       // 1/s: m/s of speed per meter of arc error
  double velocity_gain = 1.0;     // 1/s: acceleration per m/s of velocity error
  double altitude_gain = 0.5;     // 1/s^2
  double altitude_damping = 1.5;  // 1/s
  double max_accel = 10.0;        // m/s^2, limit of the combined command

  // Weight in [0, 1] of the neighbors' view of this drone's phase against the shared clock's.
  // 0 flies the clock alone, as before; 1 trusts the neighbors alone. With no usable neighbor the
  // clock gets all of the weight. `consensus_sigma_floor` (m) is added in quadrature to every
  // neighbor's position sigma when weighting.
  double consensus_weight = 0.5;
  double consensus_sigma_floor = 5.0;

  // Closest-approach separation. The margin of a neighbor is min(d_safe + kappa * sigma,
  // max_margin); `lookahead_s` is how far ahead the closest approach is looked for.
  SeparationParams separation{30.0, 2.0, 60.0, 0.2, 4.0};
  double lookahead_s = 8.0;
};

// Throws std::invalid_argument unless every value is finite, the speeds and radii are positive,
// the steps are not negative, min_speed < speed < max_speed with the margin inside, the consensus
// weight is in [0, 1], and the separation parameters are valid.
void validate(const OrbitParams& params);

struct OrbitReference {
  Vec3 position;
  Vec3 velocity;
  double radius = 0.0;
  double angle = 0.0;  // radians, counterclockwise from +x
  double speed = 0.0;
};

// Throws std::invalid_argument if `slot` is not below `n`, `n` is zero, `time_s` is not finite, or
// the parameters are invalid.
OrbitReference orbit_reference(std::uint32_t slot, std::uint32_t n, double time_s,
                               const OrbitParams& params);

// Signed angle in (-pi, pi] from the drone's angle around `center` to `reference_angle`; positive
// means the drone is behind the reference.
double orbit_phase_error(const Vec3& position, const Vec3& center, double reference_angle);

// Acceleration that takes a drone onto the circle of `radius` around `center` and along it at
// `speed`, with the speed adjusted to remove `phase_error` (radians, positive when behind), and
// onto the altitude `z_ref`.
//
// With p the horizontal position relative to the center, r = |p|, e_r the outward unit vector and
// e_t the counterclockwise tangent:
//   direction = normalize(e_t - clamp(radial_gain * (r - radius) / radius, -1, 1) * e_r)
//   desired speed = clamp(speed + phase_gain * radius * phase_error, min + margin, max - margin)
//   horizontal a = -(desired speed^2 / radius) e_r + velocity_gain * gain_scale *
//                  (desired speed * direction - v)
//   vertical a   = gain_scale * (altitude_gain * (z_ref - z) - altitude_damping * v_z)
// The centripetal term is the feedforward of a steady circle. Throws std::invalid_argument for
// non-finite input, a non-positive radius or speed, a negative gain scale, or invalid parameters.
Vec3 orbit_guidance(const Vec3& position, const Vec3& velocity, const Vec3& center, double radius,
                    double speed, double z_ref, double phase_error, const OrbitParams& params,
                    double gain_scale);

// A neighbor as closest-approach separation sees it.
struct OrbitNeighbor {
  std::uint32_t slot = 0;
  Vec3 position;
  Vec3 velocity;
  double sigma = 0.0;  // position uncertainty, meters
};

// The phase error of this drone that its neighbors imply, in radians, positive when behind.
//
// If every other drone were exactly on its reference, a drone off its own reference by delta
// would see each neighbor j at the angle theta_j + delta' where only the relative angle matters.
// Neighbor j is expected a fixed angle ahead of this drone, theta_ref_j - theta_ref_self, so
//   e_j = wrap(angle_j - (theta_ref_j - theta_ref_self) - own angle)
// is the correction this drone should make, from the neighbor's estimated angle around the
// center. The result is the circular mean of the e_j weighted by 1 / (sigma_j^2 + floor^2), with
// sigma_j the neighbor's position uncertainty. Neighbors that are not finite, have a negative
// sigma, or share this drone's slot are ignored. Returns false, leaving `error` unchanged, when
// none is usable. Throws std::invalid_argument for a slot not below `n`, a non-finite time or
// position, or invalid parameters.
bool orbit_consensus_phase_error(std::uint32_t self, std::uint32_t n, double time_s,
                                 const Vec3& position, const std::vector<OrbitNeighbor>& neighbors,
                                 const OrbitParams& params, double& error);

// Avoidance acceleration for the drone in slot `self`.
//
// For each neighbor with relative position r and relative velocity v, the closest approach inside
// the lookahead is at t* = clamp(-r.v / |v|^2, 0, lookahead) with miss vector m = r + v t*. When
// |m| is below the neighbor's margin the drone is pushed along -m (away from the predicted closest
// approach, vertical included) with magnitude gain * (margin - |m|), capped at max_accel. When the
// closest approach is exactly zero there is no side to dodge to, and the push is to the right of
// the drone's own heading (along +x for the higher slot and -x for the lower when it has no
// heading), so two aircraft head-on pass on their right. The higher slot applies the whole push and
// the lower slot half of it, so a pair does not both dodge the same way and cancel. Neighbors that
// are not finite, have a negative sigma, or share this drone's slot are ignored. The sum is capped
// at max_accel.
//
// Throws std::invalid_argument for a non-finite own position or velocity, or invalid parameters.
Vec3 orbit_separation_accel(std::uint32_t self, const Vec3& position, const Vec3& velocity,
                            const std::vector<OrbitNeighbor>& neighbors, const OrbitParams& params,
                            double margin_scale);

}  // namespace swarm
