#pragma once

#include <vector>

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

// A height measurement from a radar altimeter: the distance straight down to the highest solid
// surface below the aircraft. When `available` is false there is no reading and the other fields
// must not be used.
struct AltimeterSample {
  bool available = false;
  double height = 0.0;  // meters above the surface
  double sigma = 0.0;   // standard deviation, meters
};

// One ray of the forward obstacle sensor (ADR 0013): its unit direction in the world frame and,
// when something solid was detected along it, the measured range to it and the standard deviation
// of that range. A ray with no return has `hit` false and the other fields are not to be used.
struct ForwardRay {
  Vec3 direction;
  bool hit = false;
  double range = 0.0;  // meters, measured
  double sigma = 0.0;  // standard deviation of the range, meters
};

// A reading of the forward obstacle sensor, present on the ticks the sensor made one. The rays are
// in a fixed order for a given configuration. When `available` is false there is no reading.
struct ForwardSample {
  bool available = false;
  std::vector<ForwardRay> rays;
};

}  // namespace swarm
