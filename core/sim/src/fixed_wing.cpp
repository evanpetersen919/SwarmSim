#include "swarm/fixed_wing.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swarm {

namespace {

bool finite(Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// Below this horizontal speed the velocity has no usable direction.
constexpr double kMinDirectionSpeed = 1e-6;

}  // namespace

double FixedWingParams::max_lateral_accel() const { return gravity * std::tan(max_bank); }

void validate(const FixedWingParams& params) {
  const auto ok = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!ok(params.min_speed) || !ok(params.max_speed) || !ok(params.max_accel) ||
      !ok(params.max_decel) || !ok(params.max_bank) || !ok(params.max_climb_rate) ||
      !ok(params.max_descent_rate) || !ok(params.max_vertical_accel) || !ok(params.gravity)) {
    throw std::invalid_argument("FixedWingParams: limits must be finite and positive");
  }
  if (params.min_speed >= params.max_speed) {
    throw std::invalid_argument("FixedWingParams: min_speed must be below max_speed");
  }
  if (params.max_bank >= 0.5 * kPi) {
    throw std::invalid_argument("FixedWingParams: max_bank must be below pi/2");
  }
}

DroneState FixedWingVehicle::step(const DroneState& state, const DroneCommand& command,
                                  double dt) const {
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument("FixedWingVehicle::step: dt must be positive and finite");
  }
  if (!finite(command.accel) || !std::isfinite(command.yaw_rate) || !finite(state.pos) ||
      !finite(state.vel) || !std::isfinite(state.yaw)) {
    throw std::invalid_argument("FixedWingVehicle::step: state and command must be finite");
  }
  validate(params_);

  const double horizontal = std::hypot(state.vel.x, state.vel.y);
  const double heading =
      horizontal > kMinDirectionSpeed ? std::atan2(state.vel.y, state.vel.x) : state.yaw;
  const double speed = std::clamp(horizontal, params_.min_speed, params_.max_speed);

  const double ahead_x = std::cos(heading);
  const double ahead_y = std::sin(heading);
  const double along = command.accel.x * ahead_x + command.accel.y * ahead_y;
  const double sideways = -command.accel.x * ahead_y + command.accel.y * ahead_x;

  const double a_along = std::clamp(along, -params_.max_decel, params_.max_accel);
  const double lateral_limit = params_.max_lateral_accel();
  const double a_side = std::clamp(sideways, -lateral_limit, lateral_limit);
  const double a_up =
      std::clamp(command.accel.z, -params_.max_vertical_accel, params_.max_vertical_accel);

  const double next_speed = std::clamp(speed + a_along * dt, params_.min_speed, params_.max_speed);
  const double next_heading = wrap_angle(heading + a_side / speed * dt);
  const double next_climb =
      std::clamp(state.vel.z + a_up * dt, -params_.max_descent_rate, params_.max_climb_rate);

  DroneState next;
  next.vel = {next_speed * std::cos(next_heading), next_speed * std::sin(next_heading), next_climb};
  next.accel = (next.vel - state.vel) / dt;
  next.pos = state.pos + next.vel * dt;
  next.yaw = next_heading;
  next.yaw_rate = wrap_angle(next_heading - heading) / dt;
  return next;
}

}  // namespace swarm
