#pragma once

#include "swarm/vec3.hpp"

namespace swarm {

// What a drone's brain asks of the vehicle. It lives in its own header so controllers can name
// their output without including the truth-side dynamics header.
struct DroneCommand {
  Vec3 accel;  // desired acceleration, world frame, gravity already compensated
  double yaw_rate = 0.0;
};

}  // namespace swarm
