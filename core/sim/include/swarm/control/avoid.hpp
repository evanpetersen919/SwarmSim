#pragma once
// Controller headers may include only this directory, swarm/estimation/, sensor_data.hpp,
// vec3.hpp, drone_command.hpp, message.hpp, and swarm/format/{byte_io,hash}.hpp. They must
// never include vehicle truth.
// Enforced by scripts/check_core_invariants.sh.

#include <limits>
#include <optional>
#include <vector>

#include "swarm/control/hazard.hpp"
#include "swarm/sensor_data.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

// How a return is judged clearable by climbing (ADR 0015).
enum class AvoidFeasibility {
  // The climb needed against a fixed share of the climb limit times the time, as in ADR 0014.
  kBudget,
  // The climb needed against the altitude the vehicle can actually gain in that time under its
  // climb and vertical acceleration limits, from its present vertical speed.
  kKinematic,
};

// Avoiding terrain and obstacles by turning and slowing, from the forward obstacle sensor
// (ADR 0014, ADR 0015). The altitude floors raise the altitude command, which is limited by the
// climb rate; a return that climbing cannot clear in time is avoided sideways instead. Every value
// is an assumption (docs/MODELS.md).
struct AvoidParams {
  bool enabled = false;
  // A return is only avoided when the aircraft would reach it within this many seconds.
  double horizon_s = 12.0;
  // After the last threat the aircraft keeps its heading this long before the guidance takes it
  // back.
  double hold_s = 3.0;
  // The part of the vehicle's largest sideways acceleration that a turn may use, in (0, 1].
  double bank_fraction = 0.9;
  // The largest sideways acceleration of the vehicle, g * tan(max_bank), m/s^2. Set by the
  // scenario builder, because controllers do not see the vehicle.
  double max_lateral_accel = 6.85;
  // The largest vertical acceleration of the vehicle, m/s^2. Set by the scenario builder. While
  // avoiding, the vertical command is limited to it: the vehicle cannot use more, and the climb
  // that an unclearable return asks for would otherwise take the whole of the acceleration limit
  // and leave nothing for the turn. The kinematic judgment uses it as well.
  double max_vertical_accel = 2.0;
  // The time in which a turn aims to close the angle to the column it turns toward, seconds. It is
  // also the time over which the rejoin preference looks ahead.
  double turn_time_s = 1.5;

  // ADR 0015. All three default to the behavior of ADR 0014.
  // Prefer the free column nearest the heading the guidance is already turning toward, and break a
  // tie toward the side of its sideways acceleration, instead of the column nearest the aircraft's
  // own heading and the right on a tie.
  bool rejoin = false;
  AvoidFeasibility feasibility = AvoidFeasibility::kBudget;
  // The clearance a return must be cleared by to count as clearable, meters. Empty means the
  // terrain guidance's clearance.
  std::optional<double> clearance_m;
};

// Throws std::invalid_argument unless the horizon, hold, and turn time are finite and positive, the
// bank fraction is in (0, 1], the accelerations are finite and positive, and a clearance, when set,
// is finite and not negative.
void validate(const AvoidParams& params);

struct AvoidInput {
  // The latest forward reading, and the pose estimate's position when it was taken. The rays are
  // in row-major order with `columns` rays to a row, so ray i is in column i % columns.
  const ForwardSample* sample = nullptr;
  Vec3 sample_position;
  // The aircraft now.
  Vec3 position;
  Vec3 velocity;
  // The clearance to keep (the terrain guidance's `clearance`) and the climb rate a planned climb
  // may use (`climb_fraction * climb_rate`), for the budget judgment.
  double clearance = 60.0;
  double climb_budget = 1.8;
  // The vehicle's climb limit, m/s, for the kinematic judgment.
  double climb_limit = 3.0;
  // The horizontal acceleration the guidance asks for now (only x and y are used), for the rejoin
  // preference.
  Vec3 guidance_accel;
  // A return's range is moved by this many standard deviations in the direction that puts the
  // point higher, as the altitude floor does.
  double margin_sigmas = 2.0;
  double min_speed = 12.0;
  // Points known from memory or from other aircraft (ADR 0016), judged as returns are: a point is
  // given the column of its bearing when that is within half a column spacing of a column and is
  // ignored otherwise, and its height is raised by `margin_sigmas` of its sigma. Points farther off
  // the heading than `fan_half_rad` plus half a column spacing are ignored.
  const std::vector<HazardPoint>* hazards = nullptr;
  double fan_half_rad = 0.35;
};

struct AvoidResult {
  // The column nearest the aircraft's heading has a return that climbing cannot clear in time.
  bool threatened = false;
  // Every column is blocked.
  bool all_blocked = false;
  // The angle to the column turned toward, radians, positive to the left of the heading.
  double turn_angle = 0.0;
  // The sideways acceleration commanded, m/s^2, positive to the left.
  double turn_accel = 0.0;
  // The time is shorter than the turn needs, so the speed should come down.
  bool slow = false;
  // The time to the nearest unclearable return in the column nearest the heading, seconds.
  double time_to_threat = std::numeric_limits<double>::infinity();
};

// The altitude, in meters, that a vehicle with climb limit `climb_limit` and vertical acceleration
// limit `accel` can gain in `t` seconds starting at vertical speed `v0` (negative when descending),
// accelerating at `accel` until it reaches the limit and climbing at the limit after:
//   v0 t + accel t^2 / 2                  for t <= (limit - v0) / accel
//   limit t - (limit - v0)^2 / (2 accel)  after
// A `v0` above the limit counts as the limit. Throws std::invalid_argument for non-finite input, a
// negative `t`, or a non-positive limit or acceleration.
double reachable_climb(double v0, double climb_limit, double accel, double t);

// Which way to turn.
//
// For each detected ray the point it implies is `sample_position + direction * range` (the range
// moved by the margin), at horizontal distance d from the aircraft, reached in t = d / speed
// seconds at the horizontal speed (at least `min_speed`). The ray is *unclearable* when t is below
// the horizon and the climb it needs, `point.z + clearance - position.z`, is more than the climb
// the aircraft can count on in t: `climb_budget * t`, or `reachable_climb` under the kinematic
// judgment. `clearance` is the avoidance clearance when one is set. A column is blocked when any
// of its rays is unclearable.
//
// The avoidance is triggered when the column whose azimuth is nearest the aircraft's heading is
// blocked. The aircraft turns toward the free column nearest its heading (the right one on a tie);
// with `rejoin`, toward the free column nearest the preferred heading, the direction of the
// horizontal velocity after `turn_time_s` under the guidance acceleration, with a tie going to the
// side of that acceleration's sideways component. When no column is free, toward the column whose
// nearest unclearable return is farthest in time. The sideways acceleration is speed * angle /
// turn_time_s, limited to bank_fraction of the vehicle's largest, and the speed should come down
// when the time to the threat is less than the time that turn needs. Returns the default result
// (not threatened) for no sample, a sample with no rays or no columns, or an aircraft with no
// horizontal velocity.
//
// Throws std::invalid_argument for non-finite input or invalid parameters.
AvoidResult assess_avoidance(const AvoidParams& params, const AvoidInput& input);

}  // namespace swarm
