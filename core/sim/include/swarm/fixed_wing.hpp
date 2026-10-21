#pragma once

#include "swarm/drone_command.hpp"
#include "swarm/dynamics.hpp"
#include "swarm/vehicle.hpp"

namespace swarm {

// Limits of a fixed-wing aircraft flown as a coordinated-turn point mass (ADR 0010).
//
// The defaults are an ALTIUS-class parameter set. They are not the figures of any real aircraft:
//   published (press reports for the ALTIUS-600): max_speed is the reported maximum, 90 km/h.
//   assumed: every other value. Public sources give no stall speed, bank limit, or rate limits.
// docs/MODELS.md lists each value with its provenance.
struct FixedWingParams {
  double min_speed = 12.0;          // m/s, horizontal; the aircraft never flies slower
  double max_speed = 25.0;          // m/s, horizontal
  double max_accel = 2.0;           // m/s^2 along the heading, speeding up
  double max_decel = 2.0;           // m/s^2 along the heading, slowing down
  double max_bank = 0.61;           // rad, about 35 degrees
  double max_climb_rate = 3.0;      // m/s
  double max_descent_rate = 4.0;    // m/s
  double max_vertical_accel = 2.0;  // m/s^2
  double gravity = 9.80665;         // m/s^2

  // The largest sideways acceleration in a level coordinated turn, g * tan(max_bank).
  double max_lateral_accel() const;
};

// Throws std::invalid_argument unless every limit is finite and positive, min_speed is below
// max_speed, and max_bank is below pi/2.
void validate(const FixedWingParams& params);

// A fixed-wing aircraft cannot hover. Each step:
//   - The heading is the direction of the horizontal velocity, or state.yaw while that is zero.
//   - The horizontal speed is held in [min_speed, max_speed]. An aircraft that starts slower
//     (for example at rest) is flying at min_speed along its heading after the first step.
//   - The command's acceleration is split into its components along the heading, sideways, and
//     vertical. Along the heading it is clamped to [-max_decel, max_accel]. Sideways it is clamped
//     to +-max_lateral_accel() and turns the heading at rate a_sideways / speed, which is a
//     coordinated turn at bank angle atan(a_sideways / g). Vertical acceleration is clamped to
//     max_vertical_accel and the climb rate to [-max_descent_rate, max_climb_rate].
//   - The command's yaw rate is ignored: the heading changes only by banking.
//   - Integration is semi-implicit Euler, as in dynamics.hpp. `accel` in the result is
//     (v' - v)/dt. There is no wind.
class FixedWingVehicle final : public IVehicle {
 public:
  explicit FixedWingVehicle(const FixedWingParams& params) : params_(params) {}

  // Throws std::invalid_argument for a non-positive or non-finite `dt`, a non-finite command or
  // state, or invalid parameters.
  DroneState step(const DroneState& state, const DroneCommand& command, double dt) const override;

  const FixedWingParams& params() const { return params_; }

 private:
  FixedWingParams params_;
};

}  // namespace swarm
