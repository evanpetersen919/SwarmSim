#pragma once

#include <cstdint>

#include "swarm/drone_command.hpp"
#include "swarm/dynamics.hpp"

namespace swarm {

// Which vehicle a drone is. The agent builds the vehicle from this and the matching parameters.
enum class VehicleKind : std::uint8_t {
  kQuad = 0,
  kFixedWing = 1,
};

// The truth-side half of a drone that turns a command into motion (ADR 0005, ADR 0010). A vehicle
// owns its limits and nothing else; the state stays with the agent. Controllers never see this
// header.
class IVehicle {
 public:
  virtual ~IVehicle() = default;

  // Advances `state` by `dt` seconds under `command`, within the vehicle's limits. Throws
  // std::invalid_argument for a non-positive or non-finite `dt`, a non-finite command, or invalid
  // limits.
  virtual DroneState step(const DroneState& state, const DroneCommand& command,
                          double dt) const = 0;
};

// The kinematic point mass of dynamics.hpp: it can accelerate in any direction, stop, and hover.
class QuadVehicle final : public IVehicle {
 public:
  explicit QuadVehicle(const DroneParams& params) : params_(params) {}

  DroneState step(const DroneState& state, const DroneCommand& command, double dt) const override {
    return swarm::step(state, command, params_, dt);
  }

  const DroneParams& params() const { return params_; }

 private:
  DroneParams params_;
};

}  // namespace swarm
