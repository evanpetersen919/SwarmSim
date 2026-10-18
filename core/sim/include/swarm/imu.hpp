#pragma once

#include <cstdint>

#include "swarm/dynamics.hpp"
#include "swarm/rng.hpp"
#include "swarm/sensor_data.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

struct ImuParams {
  double accel_noise_density = 0.0;       // (m/s^2) / sqrt(Hz)
  double accel_bias_walk = 0.0;           // (m/s^2) / sqrt(s)
  double accel_initial_bias_sigma = 0.0;  // m/s^2, per axis
  double gyro_noise_density = 0.0;        // (rad/s) / sqrt(Hz)
  double gyro_bias_walk = 0.0;            // (rad/s) / sqrt(s)
  double gyro_initial_bias_sigma = 0.0;   // rad/s
  Vec3 gravity{0.0, 0.0, -9.81};          // m/s^2

  // Order-of-magnitude values for a consumer MEMS part. Placeholders, not taken from a datasheet.
  static ImuParams typical_mems();
};

// Orientation-free IMU: a world-frame accelerometer and a yaw-rate gyro, each with a random-walk
// bias and white noise. Reports specific force `accel_true - gravity`, so a hovering drone reads
// +9.81 on z.
//
// Every call to sample() draws the same number of random values whatever the parameters are, so
// changing one parameter never shifts the noise of another channel.
class Imu {
 public:
  // Throws std::invalid_argument for negative or non-finite parameters or a non-positive dt.
  Imu(const ImuParams& params, double dt, std::uint64_t seed, std::uint32_t drone_id);

  ImuSample sample(const DroneState& truth);

  // Current bias values, for tests and evaluation only. Estimators must not read these.
  Vec3 accel_bias() const { return accel_bias_; }
  double gyro_bias() const { return gyro_bias_; }

 private:
  ImuParams params_;
  double noise_scale_accel_;
  double noise_scale_gyro_;
  double walk_scale_accel_;
  double walk_scale_gyro_;
  Rng rng_;
  Vec3 accel_bias_;
  double gyro_bias_ = 0.0;
};

}  // namespace swarm
