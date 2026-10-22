#include "swarm/fixed_wing_energy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swarm {

namespace {

bool finite(Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// Below this horizontal air speed the heading has no usable direction.
constexpr double kMinDirectionSpeed = 1e-6;
// Speed above the hold speed that a climb may spend, per unit of that speed, as a fraction of
// the weight: a climb is allowed to trade a little speed for height.
constexpr double kEnergyTradeGain = 0.2;
// The airspeed is never taken below this when it divides something.
constexpr double kMinAirspeed = 1.0;

}  // namespace

double EnergyParams::stall_speed(double gravity) const {
  return std::sqrt(2.0 * mass * gravity / (air_density * wing_area * cl_max));
}

double EnergyParams::hold_speed(double gravity) const {
  return stall_margin * stall_speed(gravity);
}

double EnergyParams::drag(double speed, double load_factor, double gravity) const {
  const double v = std::max(speed, kMinAirspeed);
  const double q = 0.5 * air_density * v * v;
  const double cl = load_factor * mass * gravity / (q * wing_area);
  const double aspect = wingspan * wingspan / wing_area;
  const double k = 1.0 / (kPi * aspect * oswald);
  return q * wing_area * (cd0 + k * cl * cl);
}

double EnergyParams::steady_climb_rate(double speed, double gravity) const {
  const double v = std::max(speed, kMinAirspeed);
  const double thrust = std::min(max_thrust, max_power / v);
  return (thrust - drag(v, 1.0, gravity)) * v / (mass * gravity);
}

void validate(const EnergyParams& p) {
  const auto ok = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!ok(p.mass) || !ok(p.wing_area) || !ok(p.wingspan) || !ok(p.cd0) || !ok(p.oswald) ||
      !ok(p.cl_max) || !ok(p.air_density) || !ok(p.max_thrust) || !ok(p.max_power) ||
      !ok(p.stall_margin) || !ok(p.max_roll_rate) || !ok(p.roll_tau) || !ok(p.pitch_tau) ||
      !ok(p.max_path_rate) || !ok(p.max_path_angle)) {
    throw std::invalid_argument("EnergyParams: values must be finite and positive");
  }
  if (p.stall_margin < 1.0) {
    throw std::invalid_argument("EnergyParams: stall_margin must be at least 1");
  }
  if (p.oswald > 1.0) {
    throw std::invalid_argument("EnergyParams: oswald must be at most 1");
  }
  if (p.max_path_angle >= 0.5 * kPi) {
    throw std::invalid_argument("EnergyParams: max_path_angle must be below pi/2");
  }
}

double FixedWingEnergyVehicle::drag(double v, double load_factor) const {
  return energy_.drag(v, load_factor, limits_.gravity);
}

DroneState FixedWingEnergyVehicle::step_in_wind(const DroneState& state,
                                                const DroneCommand& command, double dt,
                                                const Vec3& wind) const {
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument("FixedWingEnergyVehicle::step: dt must be positive and finite");
  }
  if (!finite(command.accel) || !std::isfinite(command.yaw_rate) || !finite(state.pos) ||
      !finite(state.vel) || !std::isfinite(state.yaw) || !finite(wind)) {
    throw std::invalid_argument(
        "FixedWingEnergyVehicle::step: state, command, and wind must be finite");
  }
  validate(limits_);
  validate(energy_);
  const double g = limits_.gravity;
  const double weight = energy_.mass * g;

  // The air-relative motion: airspeed, heading, and path angle.
  const Vec3 air = state.vel - wind;
  const double horizontal = std::hypot(air.x, air.y);
  const double heading = horizontal > kMinDirectionSpeed ? std::atan2(air.y, air.x) : state.yaw;
  const double speed = std::max(std::sqrt(horizontal * horizontal + air.z * air.z), kMinAirspeed);
  const double gamma = std::atan2(air.z, horizontal);

  // What the controller asked for, in the frame of the heading.
  const double ahead_x = std::cos(heading);
  const double ahead_y = std::sin(heading);
  const double along = command.accel.x * ahead_x + command.accel.y * ahead_y;
  const double sideways = -command.accel.x * ahead_y + command.accel.y * ahead_x;

  // Bank. It starts from the bank the last turn implies, follows the command with a lag and a
  // rate limit, and is held below what the wing can carry at this speed.
  const double stall_1g = energy_.stall_speed(g);
  const double bank_now = std::atan2(speed * state.yaw_rate, g);
  const double bank_for_stall =
      std::acos(std::clamp(stall_1g * stall_1g / (speed * speed), 0.05, 1.0));
  const double bank_limit = std::min(limits_.max_bank, bank_for_stall);
  const double bank_cmd = std::clamp(std::atan2(sideways, g), -bank_limit, bank_limit);
  double bank_step = (bank_cmd - bank_now) * (1.0 - std::exp(-dt / energy_.roll_tau));
  const double bank_step_limit = energy_.max_roll_rate * dt;
  bank_step = std::clamp(bank_step, -bank_step_limit, bank_step_limit);
  const double bank = std::clamp(bank_now + bank_step, -bank_limit, bank_limit);
  const double load = 1.0 / std::cos(bank);

  const double stall_speed = stall_1g * std::sqrt(load);
  const double hold = energy_.stall_margin * stall_speed;
  const double drag_now = drag(speed, load);
  const double thrust_available = std::min(energy_.max_thrust, energy_.max_power / speed);

  // The path angle the controller wants, capped by what the propulsion supports and by the
  // speed it must not spend.
  const double vertical_accel =
      std::clamp(command.accel.z, -limits_.max_vertical_accel, limits_.max_vertical_accel);
  // The climb rate that, reached through the pitch lag, gives the wished acceleration: a lag of
  // time constant tau moves toward its target at (target - current) / tau.
  const double climb_wish = std::clamp(air.z + vertical_accel * energy_.pitch_tau,
                                       -limits_.max_descent_rate, limits_.max_climb_rate);
  double gamma_cmd = std::asin(std::clamp(climb_wish / speed, -1.0, 1.0));
  const double surplus = (thrust_available - drag_now) / weight +
                         kEnergyTradeGain * (speed - hold) / std::max(hold, kMinAirspeed);
  gamma_cmd = std::min(gamma_cmd, std::asin(std::clamp(surplus, -1.0, 1.0)));
  gamma_cmd = std::clamp(gamma_cmd, -energy_.max_path_angle, energy_.max_path_angle);

  double gamma_step = (gamma_cmd - gamma) * (1.0 - std::exp(-dt / energy_.pitch_tau));
  const double gamma_step_limit = energy_.max_path_rate * dt;
  gamma_step = std::clamp(gamma_step, -gamma_step_limit, gamma_step_limit);
  if (speed < stall_speed) {
    // The wing cannot carry the weight: the nose drops.
    const double lost = 1.0 - (speed * speed) / (stall_speed * stall_speed);
    gamma_step -= lost * g / speed * dt;
  }
  const double next_gamma = std::clamp(gamma + gamma_step, -0.5 * kPi + 0.05, 0.5 * kPi - 0.05);

  // Thrust makes up the drag and the weight along the path, plus the speed change asked for, and
  // keeps the speed above the hold speed.
  double speed_wish = along;
  if (speed < hold) {
    speed_wish = std::max(speed_wish, (hold - speed) / energy_.roll_tau);
  }
  double thrust = drag_now + energy_.mass * (speed_wish + g * std::sin(gamma));
  thrust = std::clamp(thrust, 0.0, thrust_available);
  const double speed_rate = (thrust - drag_now) / energy_.mass - g * std::sin(next_gamma);
  double next_speed = std::max(speed + speed_rate * dt, kMinAirspeed);
  if (next_speed > limits_.max_speed) {
    next_speed = limits_.max_speed;  // drag brakes
  }

  const double next_heading = wrap_angle(heading + g * std::tan(bank) / speed * dt);
  const double next_horizontal = next_speed * std::cos(next_gamma);

  DroneState next;
  next.vel = {next_horizontal * std::cos(next_heading) + wind.x,
              next_horizontal * std::sin(next_heading) + wind.y,
              next_speed * std::sin(next_gamma) + wind.z};
  next.accel = (next.vel - state.vel) / dt;
  next.pos = state.pos + next.vel * dt;
  next.yaw = next_heading;
  next.yaw_rate = wrap_angle(next_heading - heading) / dt;
  return next;
}

}  // namespace swarm
