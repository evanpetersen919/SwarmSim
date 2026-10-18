#include "swarm/imu.hpp"

#include <cmath>
#include <stdexcept>

#include "swarm/streams.hpp"

namespace swarm {

namespace {

bool non_negative(double v) { return std::isfinite(v) && v >= 0.0; }

}  // namespace

ImuParams ImuParams::typical_mems() {
  ImuParams p;
  p.accel_noise_density = 4.0e-3;
  p.accel_bias_walk = 1.0e-4;
  p.accel_initial_bias_sigma = 2.0e-2;
  p.gyro_noise_density = 1.0e-4;
  p.gyro_bias_walk = 1.0e-5;
  p.gyro_initial_bias_sigma = 1.0e-3;
  return p;
}

Imu::Imu(const ImuParams& params, double dt, std::uint64_t seed, std::uint32_t drone_id)
    : params_(params),
      noise_scale_accel_(0.0),
      noise_scale_gyro_(0.0),
      walk_scale_accel_(0.0),
      walk_scale_gyro_(0.0),
      rng_(seed, kStreamImuBase + drone_id) {
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument("Imu: dt must be positive and finite");
  }
  if (!non_negative(params.accel_noise_density) || !non_negative(params.accel_bias_walk) ||
      !non_negative(params.accel_initial_bias_sigma) || !non_negative(params.gyro_noise_density) ||
      !non_negative(params.gyro_bias_walk) || !non_negative(params.gyro_initial_bias_sigma)) {
    throw std::invalid_argument("Imu: noise parameters must be finite and non-negative");
  }
  if (!std::isfinite(params.gravity.x) || !std::isfinite(params.gravity.y) ||
      !std::isfinite(params.gravity.z)) {
    throw std::invalid_argument("Imu: gravity must be finite");
  }

  // Discrete-time equivalents of the continuous-time densities.
  const double root_dt = std::sqrt(dt);
  noise_scale_accel_ = params.accel_noise_density / root_dt;
  noise_scale_gyro_ = params.gyro_noise_density / root_dt;
  walk_scale_accel_ = params.accel_bias_walk * root_dt;
  walk_scale_gyro_ = params.gyro_bias_walk * root_dt;

  // Initial bias: four normals, always drawn.
  accel_bias_.x = params.accel_initial_bias_sigma * rng_.normal();
  accel_bias_.y = params.accel_initial_bias_sigma * rng_.normal();
  accel_bias_.z = params.accel_initial_bias_sigma * rng_.normal();
  gyro_bias_ = params.gyro_initial_bias_sigma * rng_.normal();
}

ImuSample Imu::sample(const DroneState& truth) {
  // Eight normals per sample, in a fixed order: bias steps, then measurement noise.
  accel_bias_.x += walk_scale_accel_ * rng_.normal();
  accel_bias_.y += walk_scale_accel_ * rng_.normal();
  accel_bias_.z += walk_scale_accel_ * rng_.normal();
  gyro_bias_ += walk_scale_gyro_ * rng_.normal();

  const Vec3 noise{noise_scale_accel_ * rng_.normal(), noise_scale_accel_ * rng_.normal(),
                   noise_scale_accel_ * rng_.normal()};
  const double gyro_noise = noise_scale_gyro_ * rng_.normal();

  ImuSample out;
  out.accel = (truth.accel - params_.gravity) + accel_bias_ + noise;
  out.gyro_z = truth.yaw_rate + gyro_bias_ + gyro_noise;
  return out;
}

}  // namespace swarm
