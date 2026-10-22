#include "swarm/vehicle.hpp"

#include <cmath>
#include <stdexcept>

namespace swarm {

DroneState IVehicle::step_in_wind(const DroneState& state, const DroneCommand& command, double dt,
                                  const Vec3& wind) const {
  if (!std::isfinite(wind.x) || !std::isfinite(wind.y) || !std::isfinite(wind.z)) {
    throw std::invalid_argument("IVehicle::step_in_wind: wind must be finite");
  }
  DroneState air = state;
  air.vel = state.vel - wind;
  DroneState next = step(air, command, dt);
  next.vel = next.vel + wind;
  next.pos = state.pos + next.vel * dt;
  next.accel = (next.vel - state.vel) / dt;
  return next;
}

}  // namespace swarm
