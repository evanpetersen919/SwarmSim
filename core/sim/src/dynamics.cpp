#include "swarm/dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swarm {

namespace {

bool finite(Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

}  // namespace

double wrap_angle(double angle) {
  double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
  if (wrapped <= 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

void validate(const DroneParams& params) {
  const auto ok = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!ok(params.max_speed) || !ok(params.max_accel) || !ok(params.max_yaw_rate)) {
    throw std::invalid_argument("DroneParams: limits must be finite and positive");
  }
}

DroneState step(const DroneState& state, const DroneCommand& command, const DroneParams& params,
                double dt) {
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument("step: dt must be positive and finite");
  }
  if (!finite(command.accel) || !std::isfinite(command.yaw_rate)) {
    throw std::invalid_argument("step: command must be finite");
  }
  validate(params);

  const Vec3 commanded = clamp_norm(command.accel, params.max_accel);
  const Vec3 velocity = clamp_norm(state.vel + commanded * dt, params.max_speed);

  DroneState next;
  next.accel = (velocity - state.vel) / dt;
  next.vel = velocity;
  next.pos = state.pos + velocity * dt;
  next.yaw_rate = std::clamp(command.yaw_rate, -params.max_yaw_rate, params.max_yaw_rate);
  next.yaw = wrap_angle(state.yaw + next.yaw_rate * dt);
  return next;
}

}  // namespace swarm
