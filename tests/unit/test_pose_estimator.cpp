#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "swarm/dynamics.hpp"
#include "swarm/estimation/pose_estimator.hpp"
#include "swarm/imu.hpp"
#include "swarm/lidar.hpp"
#include "swarm/position_fix.hpp"
#include "swarm/rng.hpp"

using swarm::DroneCommand;
using swarm::DroneParams;
using swarm::DroneState;
using swarm::Imu;
using swarm::ImuParams;
using swarm::PoseEstimate;
using swarm::PoseEstimator;
using swarm::PoseEstimatorConfig;
using swarm::PoseLaunch;
using swarm::PositionFix;
using swarm::PositionFixConfig;
using swarm::PositionFixSensor;
using swarm::Scan;
using swarm::Vec3;

namespace {

constexpr double kDt = 0.01;

// A scan in which half of the rays returned, so the fix sensor reports its base sigma.
Scan half_valid_scan() {
  Scan s;
  s.beams = 1;
  s.azimuth_samples = 100;
  s.returns.assign(100, swarm::LidarReturn{});
  for (std::size_t i = 0; i < 50; ++i) {
    s.returns[i].valid = true;
  }
  return s;
}

PoseEstimatorConfig config_for(const ImuParams& imu) {
  PoseEstimatorConfig c;
  c.dt = kDt;
  c.accel_noise_density = imu.accel_noise_density;
  c.accel_bias_walk = imu.accel_bias_walk;
  return c;
}

PoseLaunch launch_at(const Vec3& position, const Vec3& velocity = {}) {
  PoseLaunch l;
  l.position = position;
  l.velocity = velocity;
  l.position_sigma = 0.1;
  l.velocity_sigma = 0.1;
  l.bias_sigma = 0.1;
  return l;
}

ImuParams biased_imu() {
  ImuParams p = ImuParams::typical_mems();
  p.accel_initial_bias_sigma = 0.05;
  return p;
}

// A drone that hovers at a fixed point with a biased accelerometer.
struct Hover {
  Vec3 position{0.0, 0.0, 20.0};
  DroneState truth;
  Imu imu;
  PositionFixSensor fixer;
  PoseEstimator estimator;
  Scan scan = half_valid_scan();
  std::uint64_t tick = 0;

  explicit Hover(std::uint64_t seed)
      : imu(biased_imu(), kDt, seed, 0),
        fixer(PositionFixConfig{}, seed, 0),
        estimator(config_for(biased_imu()), launch_at({0.0, 0.0, 20.0})) {
    truth.pos = position;
  }

  void step(bool with_fix) {
    estimator.predict(imu.sample(truth));
    ++tick;
    if (with_fix && tick % 10 == 0) {
      estimator.update(fixer.measure(truth.pos, scan));
    }
  }

  double position_error() const {
    const Vec3 e = estimator.estimate().position - truth.pos;
    return swarm::norm(e);
  }
};

double norm_of(const Vec3& v) { return swarm::norm(v); }

}  // namespace

TEST(PoseEstimator, TheBiasOfAHoveringDroneIsLearnedFromPositionFixes) {
  Hover h(3);
  ASSERT_GT(norm_of(h.imu.accel_bias()), 0.03) << "pick a seed whose bias is not tiny";
  for (int i = 0; i < 6000; ++i) {
    h.step(true);
  }
  const PoseEstimate e = h.estimator.estimate();
  const Vec3 bias_error = e.accel_bias - h.imu.accel_bias();
  EXPECT_LT(std::abs(bias_error.x), 0.01);
  EXPECT_LT(std::abs(bias_error.y), 0.01);
  EXPECT_LT(std::abs(bias_error.z), 0.01);
  EXPECT_LT(h.position_error(), 1.0);
  EXPECT_LE(h.estimator.counters().fixes_gated, 4U);
  EXPECT_EQ(h.estimator.counters().fixes_unavailable, 0U);
  EXPECT_GE(h.estimator.counters().fixes_applied, 590U);
}

TEST(PoseEstimator, WithoutFixesTheErrorAndTheReportedUncertaintyBothGrow) {
  Hover h(3);
  ASSERT_GT(norm_of(h.imu.accel_bias()), 0.03);
  double error_at_10 = 0.0;
  double sigma_at_10 = 0.0;
  for (int i = 1; i <= 3000; ++i) {
    h.step(false);
    if (i == 1000) {
      error_at_10 = h.position_error();
      sigma_at_10 = norm_of(h.estimator.estimate().position_sigma);
    }
  }
  const double error_at_30 = h.position_error();
  const double sigma_at_30 = norm_of(h.estimator.estimate().position_sigma);
  EXPECT_GT(error_at_30, 2.0 * error_at_10);  // drift is quadratic in time
  EXPECT_GT(error_at_30, 2.0);
  EXPECT_GT(sigma_at_30, 2.0 * sigma_at_10);  // and the filter knows it is getting worse

  Hover with_fixes(3);
  for (int i = 0; i < 3000; ++i) {
    with_fixes.step(true);
  }
  EXPECT_LT(with_fixes.position_error(), 0.5 * error_at_30);
}

TEST(PoseEstimator, CovarianceGrowsBetweenFixesAndShrinksAtOne) {
  Hover h(5);
  double previous = norm_of(h.estimator.estimate().position_sigma);
  for (int i = 0; i < 9; ++i) {
    h.step(false);
    const double sigma = norm_of(h.estimator.estimate().position_sigma);
    EXPECT_GT(sigma, previous) << "step " << i;
    previous = sigma;
  }
  h.step(true);  // the tenth step carries a fix
  EXPECT_LT(norm_of(h.estimator.estimate().position_sigma), previous);
}

TEST(PoseEstimator, AGrosslyWrongFixIsRejectedAndLeavesTheEstimateUnchanged) {
  Hover h(7);
  for (int i = 0; i < 500; ++i) {
    h.step(true);
  }
  const PoseEstimate before = h.estimator.estimate();
  const std::uint64_t gated_before = h.estimator.counters().fixes_gated;

  PositionFix bad;
  bad.available = true;
  bad.position = before.position + Vec3{100.0, 0.0, 0.0};
  bad.sigma = 0.5;
  h.estimator.update(bad);

  const PoseEstimate after = h.estimator.estimate();
  EXPECT_EQ(h.estimator.counters().fixes_gated, gated_before + 1U);
  EXPECT_EQ(after.position, before.position);
  EXPECT_EQ(after.velocity, before.velocity);
  EXPECT_EQ(after.accel_bias, before.accel_bias);
}

TEST(PoseEstimator, ErrorStaysWithinThreeReportedSigmaForAtLeastNinetyFivePercentOfSamples) {
  // A moving drone over many seeded runs. A consistent filter keeps its error inside its own
  // stated uncertainty almost always.
  std::size_t inside = 0;
  std::size_t total = 0;
  for (std::uint64_t seed = 1; seed <= 40; ++seed) {
    swarm::Rng command_rng(seed, 99);
    DroneState truth;
    truth.pos = {0.0, 0.0, 30.0};
    truth.vel = {2.0, 0.0, 0.0};
    Imu imu(biased_imu(), kDt, seed, 0);
    PositionFixSensor fixer(PositionFixConfig{}, seed, 0);
    PoseEstimator estimator(config_for(biased_imu()), launch_at(truth.pos, truth.vel));
    const Scan scan = half_valid_scan();
    DroneCommand cmd;

    for (int tick = 1; tick <= 6000; ++tick) {
      if (tick % 100 == 1) {  // change the commanded acceleration once a second
        cmd.accel = {1.5 * command_rng.normal(), 1.5 * command_rng.normal(),
                     0.5 * command_rng.normal()};
      }
      truth = swarm::step(truth, cmd, DroneParams{}, kDt);
      estimator.predict(imu.sample(truth));
      if (tick % 10 == 0) {
        estimator.update(fixer.measure(truth.pos, scan));
      }
      if (tick % 50 == 0) {
        const PoseEstimate e = estimator.estimate();
        const Vec3 err = e.position - truth.pos;
        inside += (std::abs(err.x) <= 3.0 * e.position_sigma.x) ? 1U : 0U;
        inside += (std::abs(err.y) <= 3.0 * e.position_sigma.y) ? 1U : 0U;
        inside += (std::abs(err.z) <= 3.0 * e.position_sigma.z) ? 1U : 0U;
        total += 3U;
      }
    }
  }
  EXPECT_GE(static_cast<double>(inside) / static_cast<double>(total), 0.95);
}

TEST(PoseEstimator, IdenticalInputsGiveIdenticalOutputsAndEstimatorsDoNotInterfere) {
  Hover a(11);
  Hover b(11);
  Hover other(12);  // a different stream, stepped in between
  for (int i = 0; i < 2000; ++i) {
    a.step(true);
    other.step(true);
    b.step(true);
  }
  const PoseEstimate ea = a.estimator.estimate();
  const PoseEstimate eb = b.estimator.estimate();
  EXPECT_EQ(ea.position, eb.position);
  EXPECT_EQ(ea.velocity, eb.velocity);
  EXPECT_EQ(ea.accel_bias, eb.accel_bias);
  EXPECT_EQ(ea.position_sigma, eb.position_sigma);
  EXPECT_NE(ea.accel_bias, other.estimator.estimate().accel_bias);
}

TEST(PoseEstimator, UnavailableFixesAreCountedAndIgnored) {
  Hover h(2);
  const PoseEstimate before = h.estimator.estimate();
  PositionFix none;  // available == false, other fields unusable
  none.position = {1e9, 1e9, 1e9};
  none.sigma = -5.0;
  h.estimator.update(none);
  h.estimator.update(none);
  EXPECT_EQ(h.estimator.counters().fixes_unavailable, 2U);
  EXPECT_EQ(h.estimator.estimate().position, before.position);
}

TEST(PoseEstimator, InvalidInputsAreRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  Hover h(2);

  swarm::ImuSample bad_imu;
  bad_imu.accel = {nan, 0.0, 9.81};
  EXPECT_THROW(h.estimator.predict(bad_imu), std::invalid_argument);
  swarm::ImuSample bad_gyro;
  bad_gyro.gyro_z = std::numeric_limits<double>::infinity();
  EXPECT_THROW(h.estimator.predict(bad_gyro), std::invalid_argument);

  PositionFix fix;
  fix.available = true;
  fix.position = {nan, 0.0, 0.0};
  fix.sigma = 0.5;
  EXPECT_THROW(h.estimator.update(fix), std::invalid_argument);
  fix.position = {0.0, 0.0, 0.0};
  fix.sigma = 0.0;
  EXPECT_THROW(h.estimator.update(fix), std::invalid_argument);
  fix.sigma = -1.0;
  EXPECT_THROW(h.estimator.update(fix), std::invalid_argument);
}

TEST(PoseEstimator, InvalidConfigurationAndLaunchAreRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto bad_config = [](PoseEstimatorConfig c) {
    EXPECT_THROW(PoseEstimator(c, PoseLaunch{}), std::invalid_argument);
  };
  PoseEstimatorConfig c;
  c.dt = 0.0;
  bad_config(c);
  c = PoseEstimatorConfig{};
  c.gravity = {0.0, nan, -9.81};
  bad_config(c);
  c = PoseEstimatorConfig{};
  c.accel_noise_density = -1.0;
  bad_config(c);
  c = PoseEstimatorConfig{};
  c.accel_bias_walk = nan;
  bad_config(c);
  c = PoseEstimatorConfig{};
  c.fix_nis_gate = 0.0;
  bad_config(c);

  const auto bad_launch = [](PoseLaunch l) {
    EXPECT_THROW(PoseEstimator(PoseEstimatorConfig{}, l), std::invalid_argument);
  };
  PoseLaunch l;
  l.position = {nan, 0.0, 0.0};
  bad_launch(l);
  l = PoseLaunch{};
  l.velocity = {0.0, 0.0, std::numeric_limits<double>::infinity()};
  bad_launch(l);
  l = PoseLaunch{};
  l.position_sigma = -0.1;
  bad_launch(l);
  l = PoseLaunch{};
  l.bias_sigma = nan;
  bad_launch(l);
}
