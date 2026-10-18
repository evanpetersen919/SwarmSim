#pragma once

// Estimator headers may include only this directory, sensor_data.hpp, and vector types. They must
// never include vehicle truth (dynamics, scene, simulation). Enforced by
// scripts/check_core_invariants.sh.

#include <cstdint>

#include "swarm/estimation/kalman.hpp"
#include "swarm/sensor_data.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

struct PoseEstimatorConfig {
  double dt = 0.01;  // IMU sample period, seconds
  // Known constants from the sensor specification, not from ground truth.
  Vec3 gravity{0.0, 0.0, -9.81};
  double accel_noise_density = 0.0;  // (m/s^2) / sqrt(Hz)
  double accel_bias_walk = 0.0;      // (m/s^2) / sqrt(s)
  // A position fix is rejected if its normalized innovation squared exceeds this. The default is
  // the 0.999 quantile of chi-square with three degrees of freedom.
  double fix_nis_gate = 16.27;
};

// The launch state: where the drone was deployed and how well that is known. A one-time input.
struct PoseLaunch {
  Vec3 position;
  Vec3 velocity;
  double position_sigma = 0.1;  // per axis, meters
  double velocity_sigma = 0.1;  // per axis, m/s
  double bias_sigma = 0.05;     // accelerometer bias, per axis, m/s^2
};

struct PoseEstimate {
  Vec3 position;
  Vec3 velocity;
  Vec3 accel_bias;
  Vec3 position_sigma;  // per-axis standard deviation
  Vec3 velocity_sigma;
};

struct PoseEstimatorCounters {
  std::uint64_t fixes_applied = 0;
  std::uint64_t fixes_gated = 0;        // rejected as implausible
  std::uint64_t fixes_singular = 0;     // rejected for a degenerate innovation covariance
  std::uint64_t fixes_unavailable = 0;  // no fix was available
};

// Kalman filter over position, velocity, and accelerometer bias in the world frame.
//
//   a_meas = specific_force + gravity                 (world-frame acceleration reading)
//   p' = p + v dt + 0.5 (a_meas - b) dt^2
//   v' = v + (a_meas - b) dt
//   b' = b
//
// predict() takes one IMU sample per call, so the estimator advances by `dt` each time. update()
// fuses a position fix. The gyro and yaw are not used.
class PoseEstimator {
 public:
  // Throws std::invalid_argument for an invalid configuration or launch state.
  PoseEstimator(const PoseEstimatorConfig& config, const PoseLaunch& launch);

  // Throws std::invalid_argument for a non-finite sample.
  void predict(const ImuSample& imu);

  // Fuses a fix. Unavailable fixes are counted and ignored. Throws std::invalid_argument if an
  // available fix is not finite or its sigma is not positive.
  void update(const PositionFix& fix);

  PoseEstimate estimate() const;
  const PoseEstimatorCounters& counters() const { return counters_; }

 private:
  PoseEstimatorConfig config_;
  LinearKalmanFilter<9> filter_;
  SquareMatrix<9> transition_;
  SquareMatrix<9> process_noise_;
  Eigen::Matrix<double, 9, 3> control_;
  PoseEstimatorCounters counters_;
};

}  // namespace swarm
