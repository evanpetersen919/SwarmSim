#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "swarm/estimation/kalman.hpp"
#include "swarm/rng.hpp"

using swarm::LinearKalmanFilter;
using swarm::SquareMatrix;
using swarm::StateVector;
using swarm::UpdateStatus;

namespace {

using V1 = StateVector<1>;
using M1 = SquareMatrix<1>;

M1 scalar(double v) {
  M1 m;
  m << v;
  return m;
}

V1 vec1(double v) {
  V1 x;
  x << v;
  return x;
}

}  // namespace

TEST(Kalman, TextbookScalarUpdate) {
  LinearKalmanFilter<1> kf(vec1(0.0), scalar(1.0));
  const auto r = kf.update<1>(vec1(1.0), scalar(1.0), scalar(1.0));
  EXPECT_EQ(r.status, UpdateStatus::kApplied);
  EXPECT_DOUBLE_EQ(r.innovation(0), 1.0);
  EXPECT_DOUBLE_EQ(r.innovation_covariance(0, 0), 2.0);
  EXPECT_DOUBLE_EQ(r.nis, 0.5);
  EXPECT_NEAR(kf.state()(0), 0.5, 1e-15);
  EXPECT_NEAR(kf.covariance()(0, 0), 0.5, 1e-15);
}

TEST(Kalman, ScalarRandomWalkReachesTheRiccatiSteadyState) {
  const double q = 0.1;
  const double r = 0.5;
  const double prior = 0.5 * (q + std::sqrt(q * q + 4.0 * q * r));
  const double posterior = prior * r / (prior + r);

  LinearKalmanFilter<1> kf(vec1(0.0), scalar(10.0));
  for (int i = 0; i < 500; ++i) {
    kf.predict(scalar(1.0), scalar(q));
    kf.update<1>(vec1(0.0), scalar(1.0), scalar(r));
  }
  EXPECT_NEAR(kf.covariance()(0, 0), posterior, 1e-12);
}

TEST(Kalman, ConstantVelocityTrackerConvergesAndStaysConsistentWithItsOwnCovariance) {
  const double dt = 0.1;
  const double sigma = 0.5;
  const Eigen::Vector3d p0(3.0, -2.0, 10.0);
  const Eigen::Vector3d v(2.0, -1.0, 0.5);

  swarm::Rng rng(1, 0);
  StateVector<6> x0 = StateVector<6>::Zero();
  SquareMatrix<6> p_init = SquareMatrix<6>::Identity() * 100.0;
  LinearKalmanFilter<6> kf(x0, p_init);

  const auto f = swarm::constant_velocity_transition(dt);
  const auto q = swarm::constant_velocity_process_noise(dt, 0.01);
  Eigen::Matrix<double, 3, 6> h = Eigen::Matrix<double, 3, 6>::Zero();
  h.leftCols<3>() = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d r = sigma * sigma * Eigen::Matrix3d::Identity();

  const int steps = 600;
  for (int i = 1; i <= steps; ++i) {
    kf.predict(f, q);
    const double nx = rng.normal();
    const double ny = rng.normal();
    const double nz = rng.normal();
    const Eigen::Vector3d truth = p0 + v * (dt * i);
    const Eigen::Vector3d z = truth + sigma * Eigen::Vector3d(nx, ny, nz);
    kf.update<3>(z, h, r);
  }

  const Eigen::Vector3d p_true = p0 + v * (dt * steps);
  for (int axis = 0; axis < 3; ++axis) {
    const double pos_sd = std::sqrt(kf.covariance()(axis, axis));
    const double vel_sd = std::sqrt(kf.covariance()(axis + 3, axis + 3));
    EXPECT_LT(std::abs(kf.state()(axis) - p_true(axis)), 4.0 * pos_sd) << "axis " << axis;
    EXPECT_LT(std::abs(kf.state()(axis + 3) - v(axis)), 4.0 * vel_sd) << "axis " << axis;
    EXPECT_LT(pos_sd, sigma);  // better than a single measurement
    EXPECT_LT(vel_sd, 0.3);
  }
}

TEST(Kalman, CovarianceStaysSymmetricAndPositiveSemiDefiniteOverManySteps) {
  const double dt = 0.05;
  LinearKalmanFilter<6> kf(StateVector<6>::Zero(), SquareMatrix<6>::Identity() * 4.0);
  const auto f = swarm::constant_velocity_transition(dt);
  const auto q = swarm::constant_velocity_process_noise(dt, 0.5);
  Eigen::Matrix<double, 3, 6> h = Eigen::Matrix<double, 3, 6>::Zero();
  h.leftCols<3>() = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d r = 0.25 * Eigen::Matrix3d::Identity();

  swarm::Rng rng(2, 0);
  for (int i = 0; i < 10000; ++i) {
    kf.predict(f, q);
    if (i % 3 != 0) {  // leave gaps so the covariance also grows between updates
      const double nx = rng.normal();
      const double ny = rng.normal();
      const double nz = rng.normal();
      kf.update<3>(Eigen::Vector3d(nx, ny, nz), h, r);
    }
    if (i % 20 == 0) {
      const auto& p = kf.covariance();
      const double scale = std::max(1.0, p.cwiseAbs().maxCoeff());
      ASSERT_LE((p - p.transpose()).cwiseAbs().maxCoeff(), 1e-12 * scale);
      Eigen::SelfAdjointEigenSolver<SquareMatrix<6>> eigen(p);
      ASSERT_GE(eigen.eigenvalues().minCoeff(), -1e-10 * scale) << "step " << i;
    }
  }
}

TEST(Kalman, JosephFormMatchesTheTextbookFormForTheOptimalGain) {
  SquareMatrix<2> p;
  p << 2.0, 0.6, 0.6, 1.0;
  LinearKalmanFilter<2> kf(StateVector<2>(1.0, -1.0), p);
  Eigen::Matrix<double, 1, 2> h;
  h << 1.0, 0.0;
  const double r = 0.7;

  const double s = (h * p * h.transpose())(0, 0) + r;
  const Eigen::Matrix<double, 2, 1> k = p * h.transpose() / s;
  const SquareMatrix<2> textbook = (SquareMatrix<2>::Identity() - k * h) * p;

  kf.update<1>(vec1(0.5), h, scalar(r));
  EXPECT_LT((kf.covariance() - textbook).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(Kalman, AnUpdateNeverIncreasesTheCovarianceTraceAndAPredictionWithIdentityNeverDecreasesIt) {
  swarm::Rng rng(3, 0);
  for (int trial = 0; trial < 200; ++trial) {
    SquareMatrix<3> a = SquareMatrix<3>::Zero();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        a(i, j) = rng.uniform(-1.0, 1.0);
      }
    }
    const SquareMatrix<3> p0 = a * a.transpose() + 0.1 * SquareMatrix<3>::Identity();
    LinearKalmanFilter<3> kf(StateVector<3>::Zero(), p0);

    Eigen::Matrix<double, 2, 3> h;
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 3; ++j) {
        h(i, j) = rng.uniform(-1.0, 1.0);
      }
    }
    const SquareMatrix<2> r = SquareMatrix<2>::Identity() * rng.uniform(0.05, 2.0);
    Eigen::Vector2d z(0.0, 0.0);
    const double before = kf.covariance().trace();
    kf.update<2>(z, h, r);
    ASSERT_LE(kf.covariance().trace(), before + 1e-12) << "trial " << trial;

    const double mid = kf.covariance().trace();
    kf.predict(SquareMatrix<3>::Identity(), 0.01 * SquareMatrix<3>::Identity());
    ASSERT_GE(kf.covariance().trace(), mid) << "trial " << trial;
  }
}

TEST(Kalman, AnUnobservedStateIsUntouchedByAnUpdate) {
  SquareMatrix<2> p;
  p << 1.0, 0.0, 0.0, 4.0;
  LinearKalmanFilter<2> kf(StateVector<2>(0.0, 7.0), p);
  Eigen::Matrix<double, 1, 2> h;
  h << 1.0, 0.0;
  kf.update<1>(vec1(3.0), h, scalar(1.0));
  EXPECT_EQ(kf.state()(1), 7.0);
  EXPECT_EQ(kf.covariance()(1, 1), 4.0);
  EXPECT_EQ(kf.covariance()(0, 1), 0.0);
}

TEST(Kalman, AMeasurementWithEnormousNoiseBarelyMovesTheEstimate) {
  LinearKalmanFilter<1> kf(vec1(2.0), scalar(1.0));
  kf.update<1>(vec1(100.0), scalar(1.0), scalar(1e12));
  EXPECT_NEAR(kf.state()(0), 2.0, 1e-9);
  EXPECT_NEAR(kf.covariance()(0, 0), 1.0, 1e-9);
}

TEST(Kalman, NisIsReportedAndGatingRejectsWithoutChangingTheFilter) {
  // P = 4, R = 1 so S = 5; z = 3 so y = 3 and NIS = 9 / 5 = 1.8.
  LinearKalmanFilter<1> gated(vec1(0.0), scalar(4.0));
  const auto rejected = gated.update<1>(vec1(3.0), scalar(1.0), scalar(1.0), 1.79);
  EXPECT_EQ(rejected.status, UpdateStatus::kGated);
  EXPECT_NEAR(rejected.nis, 1.8, 1e-12);
  EXPECT_EQ(gated.state()(0), 0.0);
  EXPECT_EQ(gated.covariance()(0, 0), 4.0);

  LinearKalmanFilter<1> accepted(vec1(0.0), scalar(4.0));
  const auto applied = accepted.update<1>(vec1(3.0), scalar(1.0), scalar(1.0), 1.81);
  EXPECT_EQ(applied.status, UpdateStatus::kApplied);
  EXPECT_NEAR(accepted.state()(0), 2.4, 1e-12);  // gain 4/5 times innovation 3
}

TEST(Kalman, TheChiSquareGateForThreeDofRejectsAboutOneInAThousandValidMeasurements) {
  // With a correct model the NIS is chi-square distributed with 3 degrees of freedom, and the
  // 0.999 quantile is about 16.27. Draw measurements that are exactly consistent with the model.
  SquareMatrix<3> p;
  p << 2.0, 0.5, 0.1, 0.5, 1.5, 0.3, 0.1, 0.3, 1.0;
  const Eigen::Matrix3d r = 0.5 * Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d s = p + r;
  const Eigen::Matrix3d l = Eigen::LLT<Eigen::Matrix3d>(s).matrixL();

  swarm::Rng rng(4, 0);
  const int trials = 200000;
  int rejected = 0;
  for (int i = 0; i < trials; ++i) {
    const double a = rng.normal();
    const double b = rng.normal();
    const double c = rng.normal();
    const Eigen::Vector3d z = l * Eigen::Vector3d(a, b, c);
    LinearKalmanFilter<3> kf(StateVector<3>::Zero(), p);
    const auto result = kf.update<3>(z, Eigen::Matrix3d::Identity(), r);
    rejected += result.nis > 16.27 ? 1 : 0;
  }
  const double rate = static_cast<double>(rejected) / trials;
  EXPECT_GT(rate, 0.0006);
  EXPECT_LT(rate, 0.0014);
}

TEST(Kalman, ASingularInnovationCovarianceIsReportedNotThrown) {
  LinearKalmanFilter<1> kf(vec1(1.0), scalar(0.0));
  const auto result = kf.update<1>(vec1(5.0), scalar(1.0), scalar(0.0));
  EXPECT_EQ(result.status, UpdateStatus::kSingular);
  EXPECT_EQ(kf.state()(0), 1.0);
  EXPECT_EQ(kf.covariance()(0, 0), 0.0);
}

TEST(Kalman, NonFiniteOrMalformedInputThrows) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(LinearKalmanFilter<1>(vec1(nan), scalar(1.0)), std::invalid_argument);
  EXPECT_THROW(LinearKalmanFilter<1>(vec1(0.0), scalar(nan)), std::invalid_argument);
  EXPECT_THROW(LinearKalmanFilter<1>(vec1(0.0), scalar(-1.0)), std::invalid_argument);

  SquareMatrix<2> asymmetric;
  asymmetric << 1.0, 0.5, -0.5, 1.0;
  EXPECT_THROW(LinearKalmanFilter<2>(StateVector<2>::Zero(), asymmetric), std::invalid_argument);

  LinearKalmanFilter<1> kf(vec1(0.0), scalar(1.0));
  EXPECT_THROW(kf.predict(scalar(nan), scalar(0.1)), std::invalid_argument);
  EXPECT_THROW(kf.predict(scalar(1.0), scalar(-0.1)), std::invalid_argument);
  EXPECT_THROW(kf.update<1>(vec1(nan), scalar(1.0), scalar(1.0)), std::invalid_argument);
  EXPECT_THROW(kf.update<1>(vec1(0.0), scalar(inf), scalar(1.0)), std::invalid_argument);
  EXPECT_THROW(kf.update<1>(vec1(0.0), scalar(1.0), scalar(-1.0)), std::invalid_argument);
}

TEST(Kalman, PredictionWithAControlInputMatchesConstantAccelerationKinematics) {
  const double dt = 0.1;
  const double accel = 2.0;
  SquareMatrix<2> f;
  f << 1.0, dt, 0.0, 1.0;
  Eigen::Matrix<double, 2, 1> b;
  b << 0.5 * dt * dt, dt;

  LinearKalmanFilter<2> kf(StateVector<2>::Zero(), SquareMatrix<2>::Zero());
  const int steps = 50;
  for (int i = 0; i < steps; ++i) {
    kf.predict<1>(f, SquareMatrix<2>::Zero(), b, vec1(accel));
  }
  const double t = dt * steps;
  EXPECT_NEAR(kf.state()(0), 0.5 * accel * t * t, 1e-9);
  EXPECT_NEAR(kf.state()(1), accel * t, 1e-9);
}

TEST(Kalman, ConstantVelocityModelHasTheDocumentedStructure) {
  const double dt = 0.5;
  const double q = 2.0;
  const auto f = swarm::constant_velocity_transition(dt);
  EXPECT_EQ(f(0, 0), 1.0);
  EXPECT_EQ(f(0, 3), dt);
  EXPECT_EQ(f(1, 4), dt);
  EXPECT_EQ(f(3, 3), 1.0);
  EXPECT_EQ(f(3, 0), 0.0);

  const auto n = swarm::constant_velocity_process_noise(dt, q);
  EXPECT_NEAR(n(0, 0), q * dt * dt * dt / 3.0, 1e-15);
  EXPECT_NEAR(n(0, 3), q * dt * dt / 2.0, 1e-15);
  EXPECT_NEAR(n(3, 0), q * dt * dt / 2.0, 1e-15);
  EXPECT_NEAR(n(3, 3), q * dt, 1e-15);
  EXPECT_EQ(n(0, 1), 0.0);  // axes are independent
  EXPECT_LE((n - n.transpose()).cwiseAbs().maxCoeff(), 0.0);
  Eigen::SelfAdjointEigenSolver<SquareMatrix<6>> eigen(n);
  EXPECT_GE(eigen.eigenvalues().minCoeff(), -1e-15);

  EXPECT_THROW(swarm::constant_velocity_transition(0.0), std::invalid_argument);
  EXPECT_THROW(swarm::constant_velocity_process_noise(0.1, -1.0), std::invalid_argument);
  EXPECT_THROW(swarm::constant_velocity_process_noise(-0.1, 1.0), std::invalid_argument);
}
