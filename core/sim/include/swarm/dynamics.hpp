#pragma once

#include "swarm/drone_command.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

inline constexpr double kPi = 3.14159265358979323846;

struct DroneParams {
  double max_speed = 15.0;    // m/s
  double max_accel = 8.0;     // m/s^2
  double max_yaw_rate = kPi;  // rad/s
};

struct DroneState {
  Vec3 pos;
  Vec3 vel;
  Vec3 accel;  // acceleration actually applied during the last step
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

// Wraps an angle in radians into (-pi, pi].
double wrap_angle(double angle);

// Throws std::invalid_argument unless every limit is finite and positive.
void validate(const DroneParams& params);

// Advances a kinematic point mass by `dt` seconds with semi-implicit Euler integration:
//   a = clamp(command.accel, max_accel)
//   v' = clamp(v + a*dt, max_speed)       (direction kept)
//   p' = p + v'*dt
// `accel` in the result is (v' - v)/dt, which is what an accelerometer would feel after limits.
// Throws std::invalid_argument if `dt` is not positive and finite, if the command is not finite,
// or if the parameters are invalid.
DroneState step(const DroneState& state, const DroneCommand& command, const DroneParams& params,
                double dt);

}  // namespace swarm
