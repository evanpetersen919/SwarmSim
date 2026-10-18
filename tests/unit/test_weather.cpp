#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "swarm/lidar.hpp"
#include "swarm/scene.hpp"
#include "swarm/terrain.hpp"
#include "swarm/weather.hpp"

using swarm::Lidar;
using swarm::LidarConfig;
using swarm::Scene;
using swarm::SceneData;
using swarm::Weather;

namespace {

// Flat ground with one tall wall across the +x direction at x = wall_x (face), so a level beam
// along +x measures the wall.
Scene scene_with_wall(double wall_x) {
  swarm::TerrainConfig c;
  c.size_x = 600.0;
  c.size_y = 600.0;
  c.cell_size = 4.0;
  c.type = swarm::TerrainType::kField;
  SceneData data = swarm::generate_terrain(c, 1);
  swarm::Obstacle wall;
  wall.kind = swarm::ObstacleKind::kBox;
  wall.cx = wall_x + 1.0;
  wall.cy = 0.0;
  wall.base_z = 0.0;
  wall.height = 50.0;
  wall.half_x = 1.0;
  wall.half_y = 40.0;
  data.obstacles.push_back(wall);
  return Scene(data);
}

// A single level beam straight ahead.
LidarConfig forward_beam() {
  LidarConfig c;
  c.beams = 1;
  c.azimuth_samples = 1;
  c.horizontal_fov_deg = 1.0;
  c.vertical_fov_min_deg = 0.0;
  c.vertical_fov_max_deg = 0.0;
  c.min_range = 0.5;
  c.max_range = 250.0;
  c.range_noise_sigma = 0.0;
  c.range_noise_sigma_per_m = 0.0;
  return c;
}

}  // namespace

TEST(Weather, FogExtinctionFollowsKoschmieder) {
  EXPECT_NEAR(Weather::fog(1000.0).extinction_per_m(), 3.912 / 1000.0, 1e-15);
  EXPECT_NEAR(Weather::fog(50.0).extinction_per_m(), 3.912 / 50.0, 1e-15);
  EXPECT_NEAR(Weather::clear().extinction_per_m(), 3.912 / 20000.0, 1e-15);
}

TEST(Weather, RainAddsAPowerLawAndAddsToFog) {
  Weather w = Weather::rain(25.0, 1000.0);
  const double expected = 3.912 / 1000.0 + w.rain_coefficient * std::pow(25.0, w.rain_exponent);
  EXPECT_NEAR(w.extinction_per_m(), expected, 1e-15);
}

TEST(Weather, HeavierRainAttenuatesMore) {
  double previous = Weather::rain(0.0).extinction_per_m();
  for (double rate : {1.0, 5.0, 25.0, 100.0}) {
    const double alpha = Weather::rain(rate).extinction_per_m();
    EXPECT_GT(alpha, previous) << "rate " << rate;
    previous = alpha;
  }
}

TEST(Weather, PlaceholderRainConstantsGiveTheRangeQuotedInTheModelDocs) {
  // docs/MODELS.md says 100 mm/h limits a 0.6-reflectivity target to roughly 190 m.
  EXPECT_NEAR(Weather::rain(100.0).max_detection_range(0.6, 0.05), 190.0, 2.0);
  // And that 200 m visibility limits it to about 63 m.
  EXPECT_NEAR(Weather::fog(200.0).max_detection_range(0.6, 0.05), 63.5, 0.5);
}

TEST(Weather, ZeroRainCoefficientRemovesTheRainEffect) {
  Weather w = Weather::rain(80.0, 5000.0);
  w.rain_coefficient = 0.0;
  EXPECT_NEAR(w.extinction_per_m(), Weather::fog(5000.0).extinction_per_m(), 1e-15);
}

TEST(Weather, IntensityIsReflectivityTimesTwoWayTransmission) {
  const Weather w = Weather::fog(500.0);
  const double alpha = 3.912 / 500.0;
  EXPECT_NEAR(w.intensity(0.6, 20.0), 0.6 * std::exp(-2.0 * alpha * 20.0), 1e-15);
  EXPECT_DOUBLE_EQ(w.intensity(0.6, 0.0), 0.6);
}

TEST(Weather, MaximumDetectionRangeMatchesTheClosedForm) {
  const Weather w = Weather::fog(200.0);
  EXPECT_NEAR(w.max_detection_range(0.6, 0.05), std::log(12.0) / (2.0 * 3.912 / 200.0), 1e-9);
  // At exactly that range the intensity equals the threshold.
  EXPECT_NEAR(w.intensity(0.6, w.max_detection_range(0.6, 0.05)), 0.05, 1e-12);
  // Denser fog shortens it.
  EXPECT_LT(Weather::fog(100.0).max_detection_range(0.6, 0.05), w.max_detection_range(0.6, 0.05));
}

TEST(Weather, MaximumDetectionRangeHandlesDegenerateCases) {
  const double inf = std::numeric_limits<double>::infinity();
  Weather no_extinction;
  no_extinction.visibility_m = inf;  // alpha_fog = 0
  EXPECT_EQ(no_extinction.max_detection_range(0.6, 0.05), inf);
  EXPECT_EQ(Weather::fog(200.0).max_detection_range(0.6, 0.0), inf);    // no threshold
  EXPECT_EQ(Weather::fog(200.0).max_detection_range(0.04, 0.05), 0.0);  // too weak at any range
}

TEST(Weather, InvalidValuesAreRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(Weather::fog(0.0).validate(), std::invalid_argument);
  EXPECT_THROW(Weather::fog(-10.0).validate(), std::invalid_argument);
  EXPECT_THROW(Weather::fog(nan).validate(), std::invalid_argument);
  EXPECT_THROW(Weather::rain(-1.0).validate(), std::invalid_argument);
  Weather w;
  w.rain_coefficient = -1.0;
  EXPECT_THROW(w.validate(), std::invalid_argument);
  w = Weather{};
  w.rain_exponent = 0.0;
  EXPECT_THROW(w.validate(), std::invalid_argument);
  EXPECT_NO_THROW(Weather::clear().validate());
}

TEST(LidarWeather, AWallJustInsideTheDetectionRangeIsSeenAndJustOutsideIsNot) {
  const Weather fog = Weather::fog(200.0);
  LidarConfig c = forward_beam();
  const double limit = fog.max_detection_range(c.obstacle_reflectivity, c.intensity_threshold);
  ASSERT_GT(limit, 50.0);
  ASSERT_LT(limit, c.max_range);

  {
    const Scene near(scene_with_wall(limit - 1.0));
    Lidar lidar(c, 1, 0);
    const auto r = lidar.scan({0, 0, 5}, 0.0, near, fog).returns[0];
    ASSERT_TRUE(r.valid);
    EXPECT_NEAR(r.range, limit - 1.0, 1e-9);
    EXPECT_GE(r.intensity, c.intensity_threshold);
  }
  {
    const Scene far(scene_with_wall(limit + 1.0));
    Lidar lidar(c, 1, 0);
    const auto r = lidar.scan({0, 0, 5}, 0.0, far, fog).returns[0];
    EXPECT_FALSE(r.valid);
    EXPECT_EQ(r.intensity, 0.0);
  }
}

TEST(LidarWeather, ReportedIntensityIsTheAttenuatedReflectivity) {
  const Scene scene = scene_with_wall(30.0);
  LidarConfig c = forward_beam();
  const Weather w = Weather::fog(400.0);
  Lidar lidar(c, 1, 0);
  const auto r = lidar.scan({0, 0, 5}, 0.0, scene, w).returns[0];
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.intensity, c.obstacle_reflectivity * std::exp(-2.0 * (3.912 / 400.0) * 30.0),
              1e-12);

  // Ground uses the terrain reflectivity.
  LidarConfig down = forward_beam();
  down.vertical_fov_min_deg = -90.0;
  down.vertical_fov_max_deg = -90.0;
  Lidar ground(down, 1, 0);
  const auto g = ground.scan({0, 0, 12}, 0.0, scene, w).returns[0];
  ASSERT_TRUE(g.valid);
  EXPECT_NEAR(g.intensity, down.terrain_reflectivity * std::exp(-2.0 * (3.912 / 400.0) * 12.0),
              1e-12);
}

TEST(LidarWeather, ClearAirLeavesAStandardScanUnchanged) {
  const Scene scene = scene_with_wall(40.0);
  LidarConfig c;
  c.max_range = 100.0;
  Lidar a(c, 3, 0);
  Lidar b(c, 3, 0);
  const auto without = a.scan({0, 0, 8}, 0.4, scene);
  const auto with_clear = b.scan({0, 0, 8}, 0.4, scene, Weather::clear());
  for (std::size_t i = 0; i < without.returns.size(); ++i) {
    ASSERT_EQ(without.returns[i].valid, with_clear.returns[i].valid);
    ASSERT_EQ(without.returns[i].range, with_clear.returns[i].range);
  }
}

TEST(LidarWeather, FogRemovesReturnsButDoesNotShiftTheNoiseOfTheRest) {
  const Scene scene = scene_with_wall(40.0);
  LidarConfig c;
  c.max_range = 100.0;
  c.range_noise_sigma = 0.05;
  c.dropout_probability = 0.05;
  Lidar clear_lidar(c, 21, 1);
  Lidar fog_lidar(c, 21, 1);
  const auto clear_scan = clear_lidar.scan({0, 0, 8}, 0.4, scene);
  const auto fog_scan = fog_lidar.scan({0, 0, 8}, 0.4, scene, Weather::fog(150.0));

  std::size_t clear_valid = 0;
  std::size_t fog_valid = 0;
  for (std::size_t i = 0; i < clear_scan.returns.size(); ++i) {
    clear_valid += clear_scan.returns[i].valid ? 1U : 0U;
    fog_valid += fog_scan.returns[i].valid ? 1U : 0U;
    if (fog_scan.returns[i].valid) {
      ASSERT_TRUE(clear_scan.returns[i].valid);  // fog only removes returns
      ASSERT_EQ(fog_scan.returns[i].range, clear_scan.returns[i].range);  // same noise draw
    }
  }
  EXPECT_LT(fog_valid, clear_valid);
  EXPECT_GT(fog_valid, 0U);
}

TEST(LidarWeather, HeavierRainWeakensTheReturnAndCanDropItBelowThreshold) {
  const Scene scene = scene_with_wall(60.0);
  LidarConfig c = forward_beam();
  c.intensity_threshold = 0.5;  // a demanding detector, so rain matters within 60 m
  const auto scan = [&](const Weather& w) {
    Lidar lidar(c, 1, 0);
    return lidar.scan({0, 0, 5}, 0.0, scene, w).returns[0];
  };
  const auto light = scan(Weather::rain(1.0));
  const auto heavy = scan(Weather::rain(100.0));
  EXPECT_TRUE(light.valid);
  EXPECT_FALSE(heavy.valid);

  // With a normal threshold both are detected, and the heavier rain gives the weaker return.
  c.intensity_threshold = 0.05;
  const auto light_normal = scan(Weather::rain(1.0));
  const auto heavy_normal = scan(Weather::rain(100.0));
  ASSERT_TRUE(light_normal.valid);
  ASSERT_TRUE(heavy_normal.valid);
  EXPECT_LT(heavy_normal.intensity, light_normal.intensity);
}

TEST(LidarWeather, InvalidWeatherAndConfigurationAreRejected) {
  const Scene scene = scene_with_wall(30.0);
  Lidar lidar(forward_beam(), 1, 0);
  EXPECT_THROW(lidar.scan({0, 0, 5}, 0.0, scene, Weather::fog(-1.0)), std::invalid_argument);

  LidarConfig c = forward_beam();
  c.terrain_reflectivity = 0.0;
  EXPECT_THROW(Lidar(c, 1, 0), std::invalid_argument);
  c = forward_beam();
  c.obstacle_reflectivity = 1.5;
  EXPECT_THROW(Lidar(c, 1, 0), std::invalid_argument);
  c = forward_beam();
  c.intensity_threshold = 1.0;
  EXPECT_THROW(Lidar(c, 1, 0), std::invalid_argument);
  c = forward_beam();
  c.intensity_threshold = -0.1;
  EXPECT_THROW(Lidar(c, 1, 0), std::invalid_argument);
}
