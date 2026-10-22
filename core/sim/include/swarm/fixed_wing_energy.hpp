#pragma once

#include <cstdint>

#include "swarm/fixed_wing.hpp"
#include "swarm/vehicle.hpp"

namespace swarm {

// Which model flies a fixed-wing aircraft: the coordinated-turn point mass of ADR 0010, or the
// energy model below (ADR 0011).
enum class FixedWingDynamics : std::uint8_t {
  kSimple = 0,
  kEnergy = 1,
};

// What the energy model adds to FixedWingParams (ADR 0011): a mass, a wing, a drag polar, a
// propulsion limit, and the speed of the roll and pitch response.
//
//   published (press reports for the ALTIUS-600): mass about 12 kg, wingspan 2.54 m.
//   assumed: everything else. Wing area, the drag polar, maximum lift coefficient, and propulsion
//   were chosen so the aircraft cruises near the published 60 km/h and can climb at about the
//   rate FixedWingParams assumes; the stall speed, the roll rate, and the rest are not from any
//   source. docs/MODELS.md lists each value.
struct EnergyParams {
  double mass = 12.0;            // kg
  double wing_area = 0.9;        // m^2
  double wingspan = 2.54;        // m
  double cd0 = 0.035;            // zero-lift drag coefficient
  double oswald = 0.8;           // span efficiency
  double cl_max = 1.4;           // maximum lift coefficient, which sets the stall speed
  double air_density = 1.225;    // kg/m^3
  double max_thrust = 60.0;      // N, at low speed
  double max_power = 600.0;      // W, propulsive: thrust at speed v is at most max_power / v
  double stall_margin = 1.15;    // the autopilot holds this multiple of the stall speed
  double max_roll_rate = 1.2;    // rad/s
  double roll_tau = 0.3;         // s, first-order lag of the bank angle
  double pitch_tau = 0.6;        // s, first-order lag of the flight-path angle
  double max_path_rate = 0.4;    // rad/s
  double max_path_angle = 0.35;  // rad, about 20 degrees

  // The stall speed in level flight, and the speed the autopilot will not fly below in it.
  double stall_speed(double gravity) const;
  double hold_speed(double gravity) const;

  // The drag in N at airspeed `speed` carrying `load_factor` times the weight.
  double drag(double speed, double load_factor, double gravity) const;

  // The climb rate in m/s the propulsion can hold at airspeed `speed` in level-speed flight: the
  // thrust left after drag, through the power limit, times the speed over the weight. Negative when
  // the aircraft cannot hold the speed in level flight. Terrain guidance plans against it.
  double steady_climb_rate(double speed, double gravity) const;
};

// Throws std::invalid_argument unless every value is finite and positive, the margin is at least
// 1, the span efficiency is at most 1, and the path-angle limit is below pi/2.
void validate(const EnergyParams& params);

// A fixed-wing aircraft flown as a point mass in the air mass, with thrust, drag, and a lift limit
// (ADR 0011). The state is the ground-frame DroneState; the air velocity is the ground velocity
// minus the wind. From the air velocity it takes the airspeed V, the heading psi, and the
// flight-path angle gamma. The command's acceleration is read as the controller's wish:
//   along (horizontal, along the heading)  -> a thrust demand that changes the speed by that much
//   sideways                               -> a bank angle atan(a / g), within the bank limit
//   vertical                               -> a climb rate wish, which becomes a path angle
// Each step:
//   - The bank angle follows its command with a first-order lag and a roll-rate limit; it starts
//     from the bank implied by the last turn rate. The bank is also limited so that the load
//     factor 1 / cos(bank) does not stall the wing at the current speed.
//   - The heading changes at g tan(bank) / V.
//   - Thrust makes up the drag and the weight component along the path, plus the speed change
//     asked for, within min(max_thrust, max_power / V). The autopilot also raises thrust to keep
//     the speed above the hold speed for the current load factor.
//   - The path angle follows its command with a first-order lag and a rate limit. A climb the
//     propulsion cannot support is cut: the path angle is capped by the thrust left after drag,
//     so a climb costs speed only as far as the speed above the hold speed allows.
//   - Below the stall speed for the load factor the wing cannot carry the weight, and the path
//     angle falls.
//   - Airspeed above max_speed is cut back, as if by drag brakes. Climb and descent rates are
//     limited to those of FixedWingParams.
// Integration is semi-implicit Euler. The command's yaw rate is ignored. `accel` in the result is
// (v' - v) / dt in the ground frame.
class FixedWingEnergyVehicle final : public IVehicle {
 public:
  FixedWingEnergyVehicle(const FixedWingParams& limits, const EnergyParams& energy)
      : limits_(limits), energy_(energy) {}

  // Flies in still air.
  DroneState step(const DroneState& state, const DroneCommand& command, double dt) const override {
    return step_in_wind(state, command, dt, Vec3{});
  }
  DroneState step_in_wind(const DroneState& state, const DroneCommand& command, double dt,
                          const Vec3& wind) const override;

  const FixedWingParams& limits() const { return limits_; }
  const EnergyParams& energy() const { return energy_; }

  // The drag of the aircraft at airspeed `v` carrying `load_factor` times its weight.
  double drag(double v, double load_factor) const;

 private:
  FixedWingParams limits_;
  EnergyParams energy_;
};

}  // namespace swarm
