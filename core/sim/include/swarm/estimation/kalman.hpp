#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

// Linear Kalman filter on fixed-size Eigen types. Eigen is used here and in other core/sim code
// only; it never appears in core/format or anything the Unreal plugin compiles.
//
// Estimator sources may include only this header, the sensor output types, and vector types. They
// must not include dynamics, scene, or simulation truth headers (see
// scripts/check_core_invariants.sh).

namespace swarm {

template <int N>
using StateVector = Eigen::Matrix<double, N, 1>;
template <int N>
using SquareMatrix = Eigen::Matrix<double, N, N>;

enum class UpdateStatus {
  kApplied,
  kGated,     // the normalized innovation exceeded the gate; the filter was not changed
  kSingular,  // the innovation covariance is not positive definite; the filter was not changed
};

template <int M>
struct UpdateResult {
  UpdateStatus status = UpdateStatus::kSingular;
  Eigen::Matrix<double, M, 1> innovation = Eigen::Matrix<double, M, 1>::Zero();
  SquareMatrix<M> innovation_covariance = SquareMatrix<M>::Zero();
  double nis = 0.0;  // normalized innovation squared: y' S^-1 y
};

namespace detail {

template <typename Derived>
void require_finite(const Eigen::MatrixBase<Derived>& m, const char* what) {
  if (!m.allFinite()) {
    throw std::invalid_argument(std::string("Kalman filter: ") + what + " must be finite");
  }
}

// A covariance-like matrix must be symmetric (to rounding) with a non-negative diagonal.
template <typename Derived>
void require_covariance(const Eigen::MatrixBase<Derived>& m, const char* what) {
  require_finite(m, what);
  const double scale = std::max(1.0, m.cwiseAbs().maxCoeff());
  if ((m - m.transpose()).cwiseAbs().maxCoeff() > 1e-9 * scale) {
    throw std::invalid_argument(std::string("Kalman filter: ") + what + " must be symmetric");
  }
  if (m.diagonal().minCoeff() < 0.0) {
    throw std::invalid_argument(std::string("Kalman filter: ") + what +
                                " must have a non-negative diagonal");
  }
}

}  // namespace detail

// State estimate x and covariance P for a linear model.
//
//   predict:  x' = F x (+ B u),   P' = F P F' + Q
//   update:   y = z - H x,   S = H P H' + R,   K = P H' S^-1
//             x' = x + K y,  P' = (I - K H) P (I - K H)' + K R K'      (Joseph form)
//
// The Joseph form and an explicit symmetrization keep P symmetric and positive semi-definite
// despite rounding. S is factored with LDLT; no matrix is inverted explicitly.
//
// Invalid arguments (non-finite values, asymmetric or negative covariances) throw
// std::invalid_argument, since they indicate a programming error. A measurement that is
// implausible or numerically unusable is not an error: update() reports it in its result and
// leaves the filter unchanged.
template <int N>
class LinearKalmanFilter {
 public:
  LinearKalmanFilter(const StateVector<N>& x0, const SquareMatrix<N>& p0) : x_(x0), p_(p0) {
    detail::require_finite(x0, "initial state");
    detail::require_covariance(p0, "initial covariance");
  }

  const StateVector<N>& state() const { return x_; }
  const SquareMatrix<N>& covariance() const { return p_; }

  void predict(const SquareMatrix<N>& f, const SquareMatrix<N>& q) {
    check_predict(f, q);
    x_ = f * x_;
    finish_predict(f, q);
  }

  // Prediction with a control input u (for example an IMU reading): x' = F x + B u.
  template <int K>
  void predict(const SquareMatrix<N>& f, const SquareMatrix<N>& q,
               const Eigen::Matrix<double, N, K>& b, const StateVector<K>& u) {
    check_predict(f, q);
    detail::require_finite(b, "control matrix");
    detail::require_finite(u, "control input");
    x_ = f * x_ + b * u;
    finish_predict(f, q);
  }

  // Fuses a measurement z = H x + noise with covariance R. If `nis_gate` is finite, a measurement
  // whose normalized innovation squared exceeds it is rejected.
  template <int M>
  UpdateResult<M> update(const Eigen::Matrix<double, M, 1>& z, const Eigen::Matrix<double, M, N>& h,
                         const SquareMatrix<M>& r,
                         double nis_gate = std::numeric_limits<double>::infinity()) {
    detail::require_finite(z, "measurement");
    detail::require_finite(h, "measurement matrix");
    detail::require_covariance(r, "measurement covariance");

    UpdateResult<M> result;
    result.innovation = z - h * x_;
    SquareMatrix<M> s = h * p_ * h.transpose() + r;
    s = 0.5 * (s + s.transpose());
    result.innovation_covariance = s;

    Eigen::Matrix<double, N, M> k;
    if constexpr (M == 1) {
      // A scalar innovation covariance is divided directly. Factoring a 1x1 matrix with LDLT is
      // correct, but GCC 13 at -O3 reports a false array-bounds warning from the unreachable
      // pivot-swap path, and the direct form is simpler anyway.
      const double s_value = s(0, 0);
      if (!(s_value > 0.0)) {
        result.status = UpdateStatus::kSingular;
        return result;
      }
      result.nis = result.innovation(0) * result.innovation(0) / s_value;
      if (result.nis > nis_gate) {
        result.status = UpdateStatus::kGated;
        return result;
      }
      k = p_ * h.transpose() / s_value;
    } else {
      const Eigen::LDLT<SquareMatrix<M>> ldlt(s);
      const double scale = s.cwiseAbs().maxCoeff();
      if (ldlt.info() != Eigen::Success || !(scale > 0.0) ||
          !(ldlt.vectorD().minCoeff() > 1e-12 * scale)) {
        result.status = UpdateStatus::kSingular;
        return result;
      }

      result.nis = result.innovation.dot(ldlt.solve(result.innovation));
      if (result.nis > nis_gate) {
        result.status = UpdateStatus::kGated;
        return result;
      }

      // K' = S^-1 H P, so K = (S^-1 H P)'. S and P are symmetric.
      const Eigen::Matrix<double, M, N> k_transpose = ldlt.solve(h * p_);
      k = k_transpose.transpose();
    }
    x_ += k * result.innovation;
    const SquareMatrix<N> i_kh = SquareMatrix<N>::Identity() - k * h;
    p_ = i_kh * p_ * i_kh.transpose() + k * r * k.transpose();
    p_ = 0.5 * (p_ + p_.transpose());
    result.status = UpdateStatus::kApplied;
    return result;
  }

 private:
  void check_predict(const SquareMatrix<N>& f, const SquareMatrix<N>& q) const {
    detail::require_finite(f, "transition matrix");
    detail::require_covariance(q, "process noise");
  }

  void finish_predict(const SquareMatrix<N>& f, const SquareMatrix<N>& q) {
    p_ = f * p_ * f.transpose() + q;
    p_ = 0.5 * (p_ + p_.transpose());
  }

  StateVector<N> x_;
  SquareMatrix<N> p_;
};

// Constant-velocity model for a state [position (3), velocity (3)].
//   F = [I dt I; 0 I]
//   Q = q * [dt^3/3 I, dt^2/2 I; dt^2/2 I, dt I]
// where q (m^2/s^3) is the spectral density of white acceleration noise: after t seconds without
// information the velocity uncertainty is about sqrt(q t).
inline SquareMatrix<6> constant_velocity_transition(double dt) {
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument("constant_velocity_transition: dt must be positive and finite");
  }
  SquareMatrix<6> f = SquareMatrix<6>::Identity();
  f.topRightCorner<3, 3>() = dt * Eigen::Matrix3d::Identity();
  return f;
}

inline SquareMatrix<6> constant_velocity_process_noise(double dt, double q) {
  if (!(dt > 0.0) || !std::isfinite(dt) || !(q >= 0.0) || !std::isfinite(q)) {
    throw std::invalid_argument(
        "constant_velocity_process_noise: dt must be positive and q non-negative, both finite");
  }
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  SquareMatrix<6> noise = SquareMatrix<6>::Zero();
  noise.topLeftCorner<3, 3>() = q * dt * dt * dt / 3.0 * identity;
  noise.topRightCorner<3, 3>() = q * dt * dt / 2.0 * identity;
  noise.bottomLeftCorner<3, 3>() = q * dt * dt / 2.0 * identity;
  noise.bottomRightCorner<3, 3>() = q * dt * identity;
  return noise;
}

}  // namespace swarm
