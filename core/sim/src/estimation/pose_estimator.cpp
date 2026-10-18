#include "swarm/estimation/pose_estimator.hpp"

#include <cmath>
#include <stdexcept>

namespace swarm {

namespace {

bool finite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

Eigen::Vector3d to_eigen(const Vec3& v) { return Eigen::Vector3d(v.x, v.y, v.z); }

Vec3 to_vec3(const Eigen::Vector3d& v) { return Vec3{v(0), v(1), v(2)}; }

void validate(const PoseEstimatorConfig& c) {
  if (!(c.dt > 0.0) || !std::isfinite(c.dt)) {
    throw std::invalid_argument("PoseEstimatorConfig: dt must be positive and finite");
  }
  if (!finite(c.gravity)) {
    throw std::invalid_argument("PoseEstimatorConfig: gravity must be finite");
  }
  if (!(std::isfinite(c.accel_noise_density) && c.accel_noise_density >= 0.0) ||
      !(std::isfinite(c.accel_bias_walk) && c.accel_bias_walk >= 0.0)) {
    throw std::invalid_argument("PoseEstimatorConfig: noise parameters must be finite and >= 0");
  }
  if (!(c.fix_nis_gate > 0.0)) {
    throw std::invalid_argument("PoseEstimatorConfig: fix_nis_gate must be positive");
  }
}

void validate(const PoseLaunch& l) {
  const auto sigma = [](double s) { return std::isfinite(s) && s >= 0.0; };
  if (!finite(l.position) || !finite(l.velocity)) {
    throw std::invalid_argument("PoseLaunch: position and velocity must be finite");
  }
  if (!sigma(l.position_sigma) || !sigma(l.velocity_sigma) || !sigma(l.bias_sigma)) {
    throw std::invalid_argument("PoseLaunch: sigmas must be finite and >= 0");
  }
}

StateVector<9> initial_state(const PoseLaunch& l) {
  StateVector<9> x = StateVector<9>::Zero();
  x.segment<3>(0) = to_eigen(l.position);
  x.segment<3>(3) = to_eigen(l.velocity);
  return x;
}

SquareMatrix<9> initial_covariance(const PoseLaunch& l) {
  SquareMatrix<9> p = SquareMatrix<9>::Zero();
  p.block<3, 3>(0, 0) = l.position_sigma * l.position_sigma * Eigen::Matrix3d::Identity();
  p.block<3, 3>(3, 3) = l.velocity_sigma * l.velocity_sigma * Eigen::Matrix3d::Identity();
  p.block<3, 3>(6, 6) = l.bias_sigma * l.bias_sigma * Eigen::Matrix3d::Identity();
  return p;
}

}  // namespace

PoseEstimator::PoseEstimator(const PoseEstimatorConfig& config, const PoseLaunch& launch)
    : config_(config),
      filter_((validate(config), validate(launch), initial_state(launch)),
              initial_covariance(launch)) {
  const double dt = config_.dt;
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

  transition_ = SquareMatrix<9>::Identity();
  transition_.block<3, 3>(0, 3) = dt * identity;
  transition_.block<3, 3>(0, 6) = -0.5 * dt * dt * identity;
  transition_.block<3, 3>(3, 6) = -dt * identity;

  control_ = Eigen::Matrix<double, 9, 3>::Zero();
  control_.block<3, 3>(0, 0) = 0.5 * dt * dt * identity;
  control_.block<3, 3>(3, 0) = dt * identity;

  // White noise of density d on the acceleration reading is a per-sample sigma of d / sqrt(dt),
  // entering the state through the control matrix. The bias is a random walk.
  const double accel_variance = config_.accel_noise_density * config_.accel_noise_density / dt;
  process_noise_ = accel_variance * control_ * control_.transpose();
  process_noise_.block<3, 3>(6, 6) =
      config_.accel_bias_walk * config_.accel_bias_walk * dt * identity;
}

void PoseEstimator::predict(const ImuSample& imu) {
  if (!finite(imu.accel) || !std::isfinite(imu.gyro_z)) {
    throw std::invalid_argument("PoseEstimator: IMU sample must be finite");
  }
  const Vec3 acceleration = imu.accel + config_.gravity;
  filter_.predict<3>(transition_, process_noise_, control_,
                     StateVector<3>(acceleration.x, acceleration.y, acceleration.z));
}

void PoseEstimator::update(const PositionFix& fix) {
  if (!fix.available) {
    ++counters_.fixes_unavailable;
    return;
  }
  if (!finite(fix.position) || !(std::isfinite(fix.sigma) && fix.sigma > 0.0)) {
    throw std::invalid_argument(
        "PoseEstimator: an available fix needs a finite position and a positive sigma");
  }

  Eigen::Matrix<double, 3, 9> h = Eigen::Matrix<double, 3, 9>::Zero();
  h.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  const SquareMatrix<3> r = fix.sigma * fix.sigma * Eigen::Matrix3d::Identity();
  const auto result = filter_.update<3>(to_eigen(fix.position), h, r, config_.fix_nis_gate);
  switch (result.status) {
    case UpdateStatus::kApplied:
      ++counters_.fixes_applied;
      break;
    case UpdateStatus::kGated:
      ++counters_.fixes_gated;
      break;
    case UpdateStatus::kSingular:
      ++counters_.fixes_singular;
      break;
  }
}

PoseEstimate PoseEstimator::estimate() const {
  const StateVector<9>& x = filter_.state();
  const SquareMatrix<9>& p = filter_.covariance();
  PoseEstimate e;
  e.position = to_vec3(x.segment<3>(0));
  e.velocity = to_vec3(x.segment<3>(3));
  e.accel_bias = to_vec3(x.segment<3>(6));
  e.position_sigma = Vec3{std::sqrt(p(0, 0)), std::sqrt(p(1, 1)), std::sqrt(p(2, 2))};
  e.velocity_sigma = Vec3{std::sqrt(p(3, 3)), std::sqrt(p(4, 4)), std::sqrt(p(5, 5))};
  return e;
}

}  // namespace swarm
