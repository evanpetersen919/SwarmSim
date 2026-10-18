#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "swarm/dynamics.hpp"
#include "swarm/lidar.hpp"
#include "swarm/scene.hpp"
#include "swarm/terrain.hpp"

using swarm::Lidar;
using swarm::LidarConfig;
using swarm::Scene;
using swarm::SceneData;
using swarm::Vec3;

namespace {

SceneData flat_data() {
  swarm::TerrainConfig c;
  c.size_x = 400.0;
  c.size_y = 400.0;
  c.cell_size = 4.0;
  c.type = swarm::TerrainType::kField;
  return swarm::generate_terrain(c, 1);
}

LidarConfig quiet() {
  LidarConfig c;
  c.range_noise_sigma = 0.0;
  c.range_noise_sigma_per_m = 0.0;
  c.dropout_probability = 0.0;
  return c;
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

constexpr double kDeg = swarm::kPi / 180.0;

}  // namespace

TEST(LidarPattern, ElevationsAreSpreadEvenlyAcrossTheVerticalFieldOfView) {
  LidarConfig c = quiet();
  c.beams = 3;
  c.azimuth_samples = 4;
  c.vertical_fov_min_deg = -30.0;
  c.vertical_fov_max_deg = 10.0;
  const Lidar lidar(c, 1, 0);
  EXPECT_NEAR(lidar.beam_direction(0, 0, 0.0).z, std::sin(-30.0 * kDeg), 1e-12);
  EXPECT_NEAR(lidar.beam_direction(1, 0, 0.0).z, std::sin(-10.0 * kDeg), 1e-12);
  EXPECT_NEAR(lidar.beam_direction(2, 0, 0.0).z, std::sin(10.0 * kDeg), 1e-12);
}

TEST(LidarPattern, ASingleBeamSitsAtTheMiddleOfTheVerticalFieldOfView) {
  LidarConfig c = quiet();
  c.beams = 1;
  c.vertical_fov_min_deg = -30.0;
  c.vertical_fov_max_deg = 10.0;
  EXPECT_NEAR(Lidar(c, 1, 0).beam_direction(0, 0, 0.0).z, std::sin(-10.0 * kDeg), 1e-12);
}

TEST(LidarPattern, AzimuthsAreCenteredOnTheHeading) {
  LidarConfig c = quiet();
  c.beams = 1;
  c.vertical_fov_min_deg = 0.0;
  c.vertical_fov_max_deg = 0.0;
  c.azimuth_samples = 4;
  c.horizontal_fov_deg = 360.0;
  const Lidar full(c, 1, 0);
  const double expected_deg[] = {-135.0, -45.0, 45.0, 135.0};
  for (std::uint32_t k = 0; k < 4; ++k) {
    const Vec3 d = full.beam_direction(0, k, 0.0);
    EXPECT_NEAR(std::atan2(d.y, d.x), expected_deg[k] * kDeg, 1e-12);
  }

  c.azimuth_samples = 3;
  c.horizontal_fov_deg = 90.0;
  const Lidar narrow(c, 1, 0);
  const Vec3 middle = narrow.beam_direction(0, 1, 0.0);
  EXPECT_NEAR(middle.x, 1.0, 1e-12);  // the middle sample points straight ahead
  EXPECT_NEAR(middle.y, 0.0, 1e-12);
  EXPECT_NEAR(std::atan2(narrow.beam_direction(0, 0, 0.0).y, narrow.beam_direction(0, 0, 0.0).x),
              -30.0 * kDeg, 1e-12);
}

TEST(LidarPattern, YawRotatesTheWholePattern) {
  const Lidar lidar(quiet(), 1, 0);
  const double yaw = 0.7;
  for (std::uint32_t b = 0; b < lidar.config().beams; b += 5) {
    for (std::uint32_t k = 0; k < lidar.config().azimuth_samples; k += 7) {
      const Vec3 a = lidar.beam_direction(b, k, 0.0);
      const Vec3 r = lidar.beam_direction(b, k, yaw);
      EXPECT_NEAR(r.x, a.x * std::cos(yaw) - a.y * std::sin(yaw), 1e-12);
      EXPECT_NEAR(r.y, a.x * std::sin(yaw) + a.y * std::cos(yaw), 1e-12);
      EXPECT_NEAR(r.z, a.z, 1e-12);
    }
  }
}

TEST(LidarPattern, EveryDirectionIsAUnitVectorAndTheRayCountIsBeamsTimesSamples) {
  const Lidar lidar(quiet(), 1, 0);
  EXPECT_EQ(lidar.ray_count(), 16U * 90U);
  for (std::uint32_t b = 0; b < 16; ++b) {
    for (std::uint32_t k = 0; k < 90; ++k) {
      ASSERT_NEAR(swarm::norm(lidar.beam_direction(b, k, 1.3)), 1.0, 1e-12);
    }
  }
}

TEST(LidarScan, FlatGroundReadsHeightOverSineOfDepression) {
  const Scene scene(flat_data());
  LidarConfig c = quiet();
  c.max_range = 100.0;
  Lidar lidar(c, 1, 0);
  const double height = 10.0;
  const auto scan = lidar.scan({0, 0, height}, 0.3, scene);
  ASSERT_EQ(scan.returns.size(), lidar.ray_count());

  std::uint32_t valid = 0;
  for (std::uint32_t b = 0; b < c.beams; ++b) {
    const double elevation = lidar.beam_direction(b, 0, 0.0).z;  // sin(elevation)
    for (std::uint32_t k = 0; k < c.azimuth_samples; ++k) {
      const auto& r = scan.at(b, k);
      if (elevation < 0.0 && height / -elevation <= c.max_range) {
        ASSERT_TRUE(r.valid) << "beam " << b << " sample " << k;
        ASSERT_NEAR(r.range, height / -elevation, 1e-9);
        ++valid;
      } else {
        ASSERT_FALSE(r.valid) << "beam " << b << " sample " << k;
        ASSERT_EQ(r.range, 0.0);
      }
    }
  }
  EXPECT_GT(valid, 0U);
}

TEST(LidarScan, RotatingBySomeSamplesShiftsTheReturnsByTheSameIndices) {
  SceneData data = flat_data();
  swarm::Obstacle pillar;
  pillar.kind = swarm::ObstacleKind::kCylinder;
  pillar.cx = 12.0;
  pillar.cy = 7.0;
  pillar.base_z = 0.0;
  pillar.height = 40.0;
  pillar.half_x = 2.0;
  data.obstacles.push_back(pillar);
  const Scene scene(data);

  LidarConfig c = quiet();
  c.max_range = 100.0;
  Lidar a(c, 1, 0);
  Lidar b(c, 1, 0);
  const auto base = a.scan({0, 0, 10}, 0.0, scene);
  const double step = 2.0 * swarm::kPi / c.azimuth_samples;
  const auto shifted = b.scan({0, 0, 10}, 3.0 * step, scene);
  for (std::uint32_t beam = 0; beam < c.beams; ++beam) {
    for (std::uint32_t k = 0; k < c.azimuth_samples; ++k) {
      const auto& expected = base.at(beam, (k + 3U) % c.azimuth_samples);
      const auto& got = shifted.at(beam, k);
      ASSERT_EQ(got.valid, expected.valid) << "beam " << beam << " sample " << k;
      if (got.valid) {
        ASSERT_NEAR(got.range, expected.range, 1e-9);
      }
    }
  }
}

TEST(LidarScan, RangeNoiseHasTheConfiguredConstantAndPerMeterSigma) {
  const Scene scene(flat_data());
  LidarConfig c = quiet();
  c.beams = 1;
  c.vertical_fov_min_deg = -90.0;
  c.vertical_fov_max_deg = -90.0;
  c.azimuth_samples = 1000;
  c.min_range = 0.1;
  c.range_noise_sigma = 0.05;
  c.range_noise_sigma_per_m = 0.01;
  Lidar lidar(c, 3, 0);
  const double height = 10.0;  // straight down, so the true range is exactly the height
  std::vector<double> errors;
  for (int s = 0; s < 200; ++s) {
    for (const auto& r : lidar.scan({0, 0, height}, 0.0, scene).returns) {
      ASSERT_TRUE(r.valid);
      errors.push_back(r.range - height);
    }
  }
  EXPECT_NEAR(std_dev(errors) / (0.05 + 0.01 * height), 1.0, 0.02);
}

TEST(LidarScan, DropoutRateMatchesTheConfiguredProbability) {
  const Scene scene(flat_data());
  LidarConfig c = quiet();
  c.beams = 1;
  c.vertical_fov_min_deg = -90.0;
  c.vertical_fov_max_deg = -90.0;
  c.azimuth_samples = 1000;
  c.dropout_probability = 0.2;
  Lidar lidar(c, 4, 0);
  std::size_t lost = 0;
  const int scans = 100;
  for (int s = 0; s < scans; ++s) {
    for (const auto& r : lidar.scan({0, 0, 5}, 0.0, scene).returns) {
      lost += r.valid ? 0U : 1U;
    }
  }
  EXPECT_NEAR(static_cast<double>(lost) / (scans * 1000.0), 0.2, 0.01);
}

TEST(LidarScan, HitsBelowTheMinimumAndBeyondTheMaximumRangeAreNotReported) {
  SceneData data = flat_data();
  swarm::Obstacle wall;
  wall.kind = swarm::ObstacleKind::kBox;
  wall.cx = 10.0;
  wall.cy = 0.0;
  wall.base_z = 0.0;
  wall.height = 20.0;
  wall.half_x = 1.0;
  wall.half_y = 30.0;
  data.obstacles.push_back(wall);
  const Scene scene(data);

  LidarConfig c = quiet();
  c.beams = 1;
  c.vertical_fov_min_deg = 0.0;
  c.vertical_fov_max_deg = 0.0;
  c.azimuth_samples = 1;
  c.horizontal_fov_deg = 1.0;  // one ray straight ahead along +x
  // The wall face is 9 m away.
  c.min_range = 5.0;
  c.max_range = 50.0;
  {
    Lidar lidar(c, 1, 0);
    const auto r = lidar.scan({0, 0, 5}, 0.0, scene).returns[0];
    ASSERT_TRUE(r.valid);
    EXPECT_NEAR(r.range, 9.0, 1e-9);
  }
  c.min_range = 9.5;  // hit is closer than the minimum
  EXPECT_FALSE(Lidar(c, 1, 0).scan({0, 0, 5}, 0.0, scene).returns[0].valid);
  c.min_range = 0.5;
  c.max_range = 8.0;  // hit is farther than the maximum
  EXPECT_FALSE(Lidar(c, 1, 0).scan({0, 0, 5}, 0.0, scene).returns[0].valid);
}

TEST(LidarScan, ChangingTheSceneDoesNotShiftTheNoiseOnUnrelatedRays) {
  const SceneData plain = flat_data();
  SceneData busy = flat_data();
  swarm::Obstacle pillar;
  pillar.kind = swarm::ObstacleKind::kCylinder;
  pillar.cx = 15.0;
  pillar.cy = 5.0;
  pillar.base_z = 0.0;
  pillar.height = 30.0;
  pillar.half_x = 3.0;
  busy.obstacles.push_back(pillar);
  const Scene scene_plain(plain);
  const Scene scene_busy(busy);

  LidarConfig c;  // noise and dropout enabled
  c.dropout_probability = 0.1;
  c.range_noise_sigma = 0.05;
  c.max_range = 100.0;
  Lidar a(c, 12, 2);
  Lidar b(c, 12, 2);
  std::size_t compared = 0;
  for (int s = 0; s < 5; ++s) {
    const auto sa = a.scan({0, 0, 10}, 0.1 * s, scene_plain);
    const auto sb = b.scan({0, 0, 10}, 0.1 * s, scene_busy);
    ASSERT_EQ(sa.returns.size(), sb.returns.size());
    for (std::size_t i = 0; i < sa.returns.size(); ++i) {
      // Where the pillar is not involved both scans see the same ground, so the same noise.
      if (sa.returns[i].valid && sb.returns[i].valid &&
          std::abs(sa.returns[i].range - sb.returns[i].range) < 1e-12) {
        ++compared;
      }
    }
  }
  // Most rays are unaffected by the pillar; they must agree exactly.
  EXPECT_GT(compared, 5U * 16U * 90U / 2U);
}

TEST(LidarScan, SameSeedAndDroneReproduceAndDifferentDronesDiffer) {
  const Scene scene(flat_data());
  LidarConfig c;
  c.max_range = 100.0;
  Lidar a(c, 5, 1);
  Lidar b(c, 5, 1);
  Lidar other(c, 5, 2);
  const auto sa = a.scan({0, 0, 10}, 0.2, scene);
  const auto sb = b.scan({0, 0, 10}, 0.2, scene);
  const auto so = other.scan({0, 0, 10}, 0.2, scene);
  bool differs = false;
  for (std::size_t i = 0; i < sa.returns.size(); ++i) {
    ASSERT_EQ(sa.returns[i].valid, sb.returns[i].valid);
    ASSERT_EQ(sa.returns[i].range, sb.returns[i].range);
    if (sa.returns[i].valid && so.returns[i].valid && sa.returns[i].range != so.returns[i].range) {
      differs = true;
    }
  }
  EXPECT_TRUE(differs);
}

TEST(LidarConfigValidation, InvalidConfigurationsAreRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto rejects = [](LidarConfig c) { EXPECT_THROW(Lidar(c, 1, 0), std::invalid_argument); };
  LidarConfig c;
  c.beams = 0;
  rejects(c);
  c = LidarConfig{};
  c.azimuth_samples = 0;
  rejects(c);
  c = LidarConfig{};
  c.vertical_fov_min_deg = 20.0;  // above the maximum
  rejects(c);
  c = LidarConfig{};
  c.vertical_fov_min_deg = -100.0;
  rejects(c);
  c = LidarConfig{};
  c.horizontal_fov_deg = 0.0;
  rejects(c);
  c = LidarConfig{};
  c.horizontal_fov_deg = 361.0;
  rejects(c);
  c = LidarConfig{};
  c.min_range = -1.0;
  rejects(c);
  c = LidarConfig{};
  c.min_range = 70.0;  // above the maximum
  rejects(c);
  c = LidarConfig{};
  c.range_noise_sigma = -0.1;
  rejects(c);
  c = LidarConfig{};
  c.range_noise_sigma_per_m = nan;
  rejects(c);
  c = LidarConfig{};
  c.dropout_probability = 1.5;
  rejects(c);
  c = LidarConfig{};
  c.max_range = nan;
  rejects(c);
}
