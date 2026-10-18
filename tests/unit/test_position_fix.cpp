#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "swarm/lidar.hpp"
#include "swarm/position_fix.hpp"
#include "swarm/scene.hpp"
#include "swarm/terrain.hpp"
#include "swarm/weather.hpp"

using swarm::PositionFix;
using swarm::PositionFixConfig;
using swarm::PositionFixSensor;
using swarm::Scan;
using swarm::Vec3;

namespace {

// A scan with `total` rays of which the first `valid` returned.
Scan scan_with(std::size_t valid, std::size_t total) {
  Scan s;
  s.beams = 1;
  s.azimuth_samples = static_cast<std::uint32_t>(total);
  s.returns.assign(total, swarm::LidarReturn{});
  for (std::size_t i = 0; i < valid; ++i) {
    s.returns[i].valid = true;
    s.returns[i].range = 10.0;
  }
  return s;
}

double std_dev(const std::vector<double>& v) {
  double mean = 0.0;
  for (double x : v) {
    mean += x;
  }
  mean /= static_cast<double>(v.size());
  double sum = 0.0;
  for (double x : v) {
    sum += (x - mean) * (x - mean);
  }
  return std::sqrt(sum / static_cast<double>(v.size() - 1));
}

}  // namespace

TEST(PositionFix, NoFixBelowTheMinimumFractionAndAFixAtOrAboveIt) {
  PositionFixSensor sensor(PositionFixConfig{}, 1, 0);  // minimum fraction 0.05
  EXPECT_FALSE(sensor.measure({0, 0, 0}, scan_with(0, 100)).available);
  EXPECT_FALSE(sensor.measure({0, 0, 0}, scan_with(4, 100)).available);
  EXPECT_TRUE(sensor.measure({0, 0, 0}, scan_with(5, 100)).available);
  EXPECT_TRUE(sensor.measure({0, 0, 0}, scan_with(100, 100)).available);
  EXPECT_FALSE(sensor.measure({0, 0, 0}, Scan{}).available);  // an empty scan has no information
}

TEST(PositionFix, AnUnavailableFixCarriesTheCeilingSigmaAndNoPosition) {
  PositionFixSensor sensor(PositionFixConfig{}, 1, 0);
  const PositionFix fix = sensor.measure({10, 20, 30}, scan_with(0, 100));
  EXPECT_FALSE(fix.available);
  EXPECT_EQ(fix.sigma, PositionFixConfig{}.max_sigma);
  EXPECT_EQ(fix.position, Vec3{});
}

TEST(PositionFix, SigmaScalesAsTheInverseSquareRootOfTheValidFraction) {
  PositionFixSensor sensor(PositionFixConfig{}, 1, 0);  // base 0.5 m at fraction 0.5
  EXPECT_DOUBLE_EQ(sensor.measure({0, 0, 0}, scan_with(50, 100)).sigma, 0.5);
  EXPECT_DOUBLE_EQ(sensor.measure({0, 0, 0}, scan_with(125, 1000)).sigma, 1.0);  // fraction 0.125
  EXPECT_DOUBLE_EQ(sensor.measure({0, 0, 0}, scan_with(500, 1000)).sigma, 0.5);
  EXPECT_NEAR(sensor.measure({0, 0, 0}, scan_with(100, 100)).sigma, 0.5 * std::sqrt(0.5), 1e-15);
}

TEST(PositionFix, SigmaNeverExceedsTheCeiling) {
  PositionFixConfig c;
  c.base_sigma = 5.0;
  c.min_fraction = 0.01;
  c.max_sigma = 20.0;
  PositionFixSensor sensor(c, 1, 0);
  // fraction 0.01 would give 5 * sqrt(50) = 35.4 m, which the ceiling caps.
  EXPECT_DOUBLE_EQ(sensor.measure({0, 0, 0}, scan_with(1, 100)).sigma, 20.0);
}

TEST(PositionFix, MeasurementErrorHasTheReportedStandardDeviation) {
  PositionFixSensor sensor(PositionFixConfig{}, 7, 0);
  const Scan scan = scan_with(50, 100);
  std::vector<double> ex;
  std::vector<double> ey;
  std::vector<double> ez;
  const Vec3 truth{100.0, -50.0, 20.0};
  for (int i = 0; i < 100000; ++i) {
    const PositionFix fix = sensor.measure(truth, scan);
    ex.push_back(fix.position.x - truth.x);
    ey.push_back(fix.position.y - truth.y);
    ez.push_back(fix.position.z - truth.z);
  }
  EXPECT_NEAR(std_dev(ex), 0.5, 0.01);
  EXPECT_NEAR(std_dev(ey), 0.5, 0.01);
  EXPECT_NEAR(std_dev(ez), 0.5, 0.01);
}

TEST(PositionFix, FogDegradesAndEventuallyRemovesTheFixOnARealScan) {
  swarm::TerrainConfig t;
  t.size_x = 400.0;
  t.size_y = 400.0;
  t.cell_size = 4.0;
  t.type = swarm::TerrainType::kField;
  const swarm::Scene scene(swarm::generate_terrain(t, 1));
  const Vec3 position{0.0, 0.0, 10.0};

  const auto fix_in = [&](const swarm::Weather& weather) {
    swarm::Lidar lidar(swarm::LidarConfig{}, 5, 0);
    PositionFixSensor sensor(PositionFixConfig{}, 5, 0);
    return sensor.measure(position, lidar.scan(position, 0.0, scene, weather));
  };

  const PositionFix clear = fix_in(swarm::Weather::clear());
  const PositionFix light_fog = fix_in(swarm::Weather::fog(150.0));
  const PositionFix dense_fog = fix_in(swarm::Weather::fog(40.0));

  ASSERT_TRUE(clear.available);
  ASSERT_TRUE(light_fog.available);
  EXPECT_GT(light_fog.sigma, clear.sigma);  // fewer returns, less certain
  EXPECT_FALSE(dense_fog.available);        // nothing comes back
}

TEST(PositionFix, TheNoiseOfACallDoesNotDependOnEarlierCallsOutcomes) {
  PositionFixSensor a(PositionFixConfig{}, 11, 3);
  PositionFixSensor b(PositionFixConfig{}, 11, 3);
  const Scan good = scan_with(50, 100);
  const Vec3 truth{1.0, 2.0, 3.0};

  a.measure(truth, scan_with(0, 100));  // no fix on the first call
  b.measure(truth, good);               // a fix on the first call
  const PositionFix second_a = a.measure(truth, good);
  const PositionFix second_b = b.measure(truth, good);
  EXPECT_EQ(second_a.position, second_b.position);
}

TEST(PositionFix, SameSeedAndDroneReproduceAndDifferentDronesDiffer) {
  const Scan scan = scan_with(50, 100);
  PositionFixSensor a(PositionFixConfig{}, 5, 1);
  PositionFixSensor b(PositionFixConfig{}, 5, 1);
  PositionFixSensor other(PositionFixConfig{}, 5, 2);
  const PositionFix fa = a.measure({0, 0, 0}, scan);
  EXPECT_EQ(fa.position, b.measure({0, 0, 0}, scan).position);
  EXPECT_NE(fa.position, other.measure({0, 0, 0}, scan).position);
}

TEST(PositionFix, InvalidConfigurationAndInputAreRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto rejects = [](PositionFixConfig c) {
    EXPECT_THROW(PositionFixSensor(c, 1, 0), std::invalid_argument);
  };
  PositionFixConfig c;
  c.base_sigma = 0.0;
  rejects(c);
  c = PositionFixConfig{};
  c.base_sigma = nan;
  rejects(c);
  c = PositionFixConfig{};
  c.reference_fraction = 0.0;
  rejects(c);
  c = PositionFixConfig{};
  c.reference_fraction = 1.5;
  rejects(c);
  c = PositionFixConfig{};
  c.min_fraction = 0.0;
  rejects(c);
  c = PositionFixConfig{};
  c.max_sigma = 0.1;  // below base_sigma
  rejects(c);

  PositionFixSensor sensor(PositionFixConfig{}, 1, 0);
  EXPECT_THROW(sensor.measure({nan, 0, 0}, scan_with(50, 100)), std::invalid_argument);
}
