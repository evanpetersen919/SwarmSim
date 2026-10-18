#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "swarm/demo_agent.hpp"
#include "swarm/imu.hpp"
#include "swarm/simulation.hpp"
#include "swarm/streams.hpp"

using swarm::DroneState;
using swarm::Imu;
using swarm::ImuParams;
using swarm::Vec3;

namespace {

constexpr double kDt = 0.01;

double variance(const std::vector<double>& v) {
  double mean = 0.0;
  for (double x : v) {
    mean += x;
  }
  mean /= static_cast<double>(v.size());
  double sum = 0.0;
  for (double x : v) {
    sum += (x - mean) * (x - mean);
  }
  return sum / static_cast<double>(v.size() - 1);
}

ImuParams accel_noise_only(double density) {
  ImuParams p;
  p.accel_noise_density = density;
  return p;
}

}  // namespace

TEST(Imu, NoiseFreeReportsSpecificForceAndYawRate) {
  Imu imu(ImuParams{}, kDt, 1, 0);
  DroneState truth;
  truth.accel = {1.0, -2.0, 0.5};
  truth.yaw_rate = 0.3;
  const auto s = imu.sample(truth);
  EXPECT_DOUBLE_EQ(s.accel.x, 1.0);
  EXPECT_DOUBLE_EQ(s.accel.y, -2.0);
  EXPECT_DOUBLE_EQ(s.accel.z, 0.5 + 9.81);
  EXPECT_DOUBLE_EQ(s.gyro_z, 0.3);
}

TEST(Imu, StationaryDroneReadsPlusOneGOnZ) {
  Imu imu(ImuParams{}, kDt, 1, 0);
  const auto s = imu.sample(DroneState{});
  EXPECT_DOUBLE_EQ(s.accel.x, 0.0);
  EXPECT_DOUBLE_EQ(s.accel.y, 0.0);
  EXPECT_DOUBLE_EQ(s.accel.z, 9.81);
  EXPECT_DOUBLE_EQ(s.gyro_z, 0.0);
}

TEST(Imu, WhiteNoiseVarianceMatchesDensityOverDt) {
  const double density = 0.02;
  Imu imu(accel_noise_only(density), kDt, 7, 0);
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  for (int i = 0; i < 200000; ++i) {
    const auto s = imu.sample(DroneState{});
    x.push_back(s.accel.x);
    y.push_back(s.accel.y);
    z.push_back(s.accel.z - 9.81);
  }
  const double expected = density * density / kDt;
  EXPECT_NEAR(variance(x) / expected, 1.0, 0.03);
  EXPECT_NEAR(variance(y) / expected, 1.0, 0.03);
  EXPECT_NEAR(variance(z) / expected, 1.0, 0.03);
}

TEST(Imu, GyroNoiseVarianceMatchesDensityOverDt) {
  ImuParams p;
  p.gyro_noise_density = 0.005;
  Imu imu(p, kDt, 3, 1);
  std::vector<double> g;
  for (int i = 0; i < 200000; ++i) {
    g.push_back(imu.sample(DroneState{}).gyro_z);
  }
  EXPECT_NEAR(variance(g) / (p.gyro_noise_density * p.gyro_noise_density / kDt), 1.0, 0.03);
}

TEST(Imu, BiasRandomWalkVarianceGrowsLinearlyWithSteps) {
  ImuParams p;
  p.accel_bias_walk = 0.05;
  p.gyro_bias_walk = 0.002;
  const int imus = 2000;
  const int steps = 400;
  std::vector<double> accel_bias;
  std::vector<double> gyro_bias;
  for (int i = 0; i < imus; ++i) {
    Imu imu(p, kDt, 11, static_cast<std::uint32_t>(i));
    for (int k = 0; k < steps; ++k) {
      imu.sample(DroneState{});
    }
    accel_bias.push_back(imu.accel_bias().x);
    accel_bias.push_back(imu.accel_bias().y);
    accel_bias.push_back(imu.accel_bias().z);
    gyro_bias.push_back(imu.gyro_bias());
  }
  const double accel_expected = steps * p.accel_bias_walk * p.accel_bias_walk * kDt;
  const double gyro_expected = steps * p.gyro_bias_walk * p.gyro_bias_walk * kDt;
  EXPECT_NEAR(variance(accel_bias) / accel_expected, 1.0, 0.06);
  EXPECT_NEAR(variance(gyro_bias) / gyro_expected, 1.0, 0.12);
}

TEST(Imu, InitialBiasSpreadMatchesSigma) {
  ImuParams p;
  p.accel_initial_bias_sigma = 0.03;
  p.gyro_initial_bias_sigma = 0.004;
  std::vector<double> accel_bias;
  std::vector<double> gyro_bias;
  for (std::uint32_t i = 0; i < 3000; ++i) {
    Imu imu(p, kDt, 5, i);
    accel_bias.push_back(imu.accel_bias().x);
    gyro_bias.push_back(imu.gyro_bias());
  }
  EXPECT_NEAR(variance(accel_bias) / (0.03 * 0.03), 1.0, 0.08);
  EXPECT_NEAR(variance(gyro_bias) / (0.004 * 0.004), 1.0, 0.08);
}

TEST(Imu, DifferentDronesAreUncorrelated) {
  Imu a(accel_noise_only(0.02), kDt, 9, 0);
  Imu b(accel_noise_only(0.02), kDt, 9, 1);
  const int n = 100000;
  double sum_ab = 0.0;
  double sum_aa = 0.0;
  double sum_bb = 0.0;
  for (int i = 0; i < n; ++i) {
    const double xa = a.sample(DroneState{}).accel.x;
    const double xb = b.sample(DroneState{}).accel.x;
    sum_ab += xa * xb;
    sum_aa += xa * xa;
    sum_bb += xb * xb;
  }
  EXPECT_LT(std::abs(sum_ab / std::sqrt(sum_aa * sum_bb)), 0.015);
}

TEST(Imu, SameSeedAndIdReproduceExactly) {
  Imu a(ImuParams::typical_mems(), kDt, 21, 4);
  Imu b(ImuParams::typical_mems(), kDt, 21, 4);
  DroneState truth;
  truth.accel = {0.1, 0.2, 0.3};
  for (int i = 0; i < 1000; ++i) {
    const auto sa = a.sample(truth);
    const auto sb = b.sample(truth);
    ASSERT_EQ(sa.accel, sb.accel);
    ASSERT_EQ(sa.gyro_z, sb.gyro_z);
  }
}

TEST(Imu, CreatingAnotherImuDoesNotChangeAnExistingStream) {
  const auto series = [](bool create_neighbor) {
    std::unique_ptr<Imu> neighbor;
    if (create_neighbor) {
      neighbor = std::make_unique<Imu>(ImuParams::typical_mems(), kDt, 2, 3);
    }
    Imu imu(ImuParams::typical_mems(), kDt, 2, 2);
    std::vector<double> out;
    for (int i = 0; i < 100; ++i) {
      out.push_back(imu.sample(DroneState{}).accel.x);
    }
    return out;
  };
  EXPECT_EQ(series(false), series(true));
}

TEST(Imu, ChangingOneChannelsParametersDoesNotShiftAnothersNoise) {
  // The gyro output must be identical whether or not the accelerometer has noise enabled,
  // because both consume the same number of draws per sample.
  ImuParams quiet_accel;
  quiet_accel.gyro_noise_density = 0.01;
  quiet_accel.gyro_bias_walk = 0.001;
  ImuParams noisy_accel = quiet_accel;
  noisy_accel.accel_noise_density = 0.05;
  noisy_accel.accel_bias_walk = 0.01;
  Imu a(quiet_accel, kDt, 6, 0);
  Imu b(noisy_accel, kDt, 6, 0);
  for (int i = 0; i < 500; ++i) {
    ASSERT_EQ(a.sample(DroneState{}).gyro_z, b.sample(DroneState{}).gyro_z);
  }
}

TEST(Imu, RejectsInvalidArguments) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(Imu(ImuParams{}, 0.0, 1, 0), std::invalid_argument);
  EXPECT_THROW(Imu(ImuParams{}, -0.01, 1, 0), std::invalid_argument);
  EXPECT_THROW(Imu(ImuParams{}, nan, 1, 0), std::invalid_argument);

  ImuParams p;
  p.accel_noise_density = -1.0;
  EXPECT_THROW(Imu(p, kDt, 1, 0), std::invalid_argument);
  p = ImuParams{};
  p.gyro_bias_walk = nan;
  EXPECT_THROW(Imu(p, kDt, 1, 0), std::invalid_argument);
  p = ImuParams{};
  p.accel_initial_bias_sigma = std::numeric_limits<double>::infinity();
  EXPECT_THROW(Imu(p, kDt, 1, 0), std::invalid_argument);
  p = ImuParams{};
  p.gravity = {0.0, nan, -9.81};
  EXPECT_THROW(Imu(p, kDt, 1, 0), std::invalid_argument);
}

TEST(Simulation, RejectsMoreAgentsThanTheStreamRangesAllow) {
  std::vector<std::unique_ptr<swarm::IAgent>> too_many;
  for (std::uint32_t i = 0; i <= swarm::kMaxAgents; ++i) {
    too_many.push_back(std::make_unique<swarm::DemoAgent>(i, swarm::kMaxAgents + 1U));
  }
  EXPECT_THROW(swarm::Simulation(swarm::SimConfig{}, std::move(too_many), 1),
               std::invalid_argument);
}
