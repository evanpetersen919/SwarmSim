#pragma once

#include "swarm/vec3.hpp"

// What sensors report. Estimators consume these types and nothing else from the simulation: this
// header deliberately includes no vehicle truth. scripts/check_core_invariants.sh enforces that
// estimator sources include only this header, vector types, and each other.

namespace swarm {

// Inertial sensor output: specific force in the world frame and yaw rate.
struct ImuSample {
  Vec3 accel;
  double gyro_z = 0.0;
};

// A position measurement derived from a LiDAR scan. When `available` is false there is no fix and
// the other fields must not be used (`position` is zero and `sigma` is the sensor's ceiling).
struct PositionFix {
  bool available = false;
  Vec3 position;       // meters, world frame
  double sigma = 0.0;  // standard deviation per axis, meters
};

}  // namespace swarm
