#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarm/control/drone_brain.hpp"
#include "swarm/control/formation.hpp"
#include "swarm/control/terrain_map.hpp"
#include "swarm/drone_agent.hpp"
#include "swarm/forward_sensor.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"
#include "swarm/scene.hpp"
#include "swarm/simulation.hpp"
#include "swarm/weather.hpp"

using swarm::BrainConfig;
using swarm::BrainInput;
using swarm::DroneBrain;
using swarm::ForwardRay;
using swarm::ForwardSample;
using swarm::ForwardSensor;
using swarm::ForwardSensorConfig;
using swarm::Vec3;
using swarm::Weather;

namespace {

// Flat ground at 0 m over 2 km by 2 km, with a wall (a box) across x from 380 m to 420 m, 200 m
// high, if `wall` is set.
swarm::Scene make_scene(bool wall) {
  swarm::SceneData data;
  data.terrain.cells_x = 20;
  data.terrain.cells_y = 20;
  data.terrain.origin_x = -1000.0;
  data.terrain.origin_y = -1000.0;
  data.terrain.cell_size = 100.0;
  data.terrain.heights.assign(21 * 21, 0.0);
  if (wall) {
    swarm::Obstacle box;
    box.kind = swarm::ObstacleKind::kBox;
    box.cx = 400.0;
    box.cy = 0.0;
    box.base_z = 0.0;
    box.height = 200.0;
    box.half_x = 20.0;
    box.half_y = 500.0;
    data.obstacles.push_back(box);
  }
  return swarm::Scene(std::move(data));
}

ForwardSensorConfig one_ray() {
  ForwardSensorConfig c;
  c.azimuth_rays = 1;
  c.elevation_rays = 1;
  c.pitch_deg = 0.0;
  c.dropout_probability = 0.0;
  return c;
}

}  // namespace

TEST(ForwardSensor, AnEmptyWorldGivesRaysWithoutReturns) {
  const swarm::Scene scene = make_scene(false);
  // Heading along +x at 500 m with the fan tilted up: nothing ahead, and no ray reaches the ground.
  ForwardSensorConfig level = ForwardSensorConfig{};
  level.pitch_deg = 10.0;
  level.vertical_fov_deg = 0.0;
  ForwardSensor up(level, 1, 0);
  const ForwardSample s = up.measure({0.0, 0.0, 500.0}, 1.0, 0.0, scene, Weather{});
  ASSERT_TRUE(s.available);
  ASSERT_EQ(s.rays.size(), 21U);
  for (const ForwardRay& ray : s.rays) {
    EXPECT_FALSE(ray.hit);
  }
}

TEST(ForwardSensor, TheFanHasTheStatedShapeAndOrder) {
  const swarm::Scene scene = make_scene(false);
  ForwardSensorConfig c;
  c.azimuth_rays = 3;
  c.elevation_rays = 2;
  c.horizontal_fov_deg = 40.0;
  c.vertical_fov_deg = 20.0;
  c.pitch_deg = 0.0;
  ForwardSensor sensor(c, 1, 0);
  const ForwardSample s = sensor.measure({0.0, 0.0, 500.0}, 1.0, 0.0, scene, Weather{});
  ASSERT_EQ(s.rays.size(), 6U);
  const double kDeg = 3.14159265358979323846 / 180.0;
  for (const ForwardRay& ray : s.rays) {
    EXPECT_NEAR(std::sqrt(ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y +
                          ray.direction.z * ray.direction.z),
                1.0, 1e-12);
  }
  // Row-major from the lowest row, and from the leftmost column (+y is to the left of +x).
  EXPECT_NEAR(s.rays[0].direction.z, std::sin(-10.0 * kDeg), 1e-12);
  EXPECT_NEAR(s.rays[3].direction.z, std::sin(10.0 * kDeg), 1e-12);
  EXPECT_GT(s.rays[0].direction.y, 0.0);           // leftmost
  EXPECT_NEAR(s.rays[1].direction.y, 0.0, 1e-12);  // center
  EXPECT_LT(s.rays[2].direction.y, 0.0);           // rightmost
  EXPECT_NEAR(s.rays[0].direction.y, -s.rays[2].direction.y, 1e-12);
  // A heading that is not a unit vector, and not along an axis, turns the whole fan.
  const ForwardSample turned = sensor.measure({0.0, 0.0, 500.0}, 0.0, 5.0, scene, Weather{});
  EXPECT_NEAR(turned.rays[1].direction.y, std::cos(10.0 * kDeg), 1e-12);
  EXPECT_NEAR(turned.rays[1].direction.x, 0.0, 1e-12);
}

TEST(ForwardSensor, ReportsTheRangeToAWallWithNoiseOfTheStatedSize) {
  const swarm::Scene scene = make_scene(true);
  ForwardSensor sensor(one_ray(), 3, 0);
  const double distance = 380.0;  // the near face of the wall
  // Even the default weather has some haze: sigma is the floor or the range term over T.
  const double transmission = std::exp(-Weather{}.extinction_per_m() * distance);
  const double sigma = std::max(2.0, 0.01 * distance) / transmission;
  double sum = 0.0;
  double sum2 = 0.0;
  const int n = 3000;
  for (int i = 0; i < n; ++i) {
    const ForwardSample s = sensor.measure({0.0, 0.0, 50.0}, 1.0, 0.0, scene, Weather{});
    ASSERT_TRUE(s.rays[0].hit);
    EXPECT_NEAR(s.rays[0].sigma, sigma, 1e-9 * sigma + 1e-9);
    sum += s.rays[0].range - distance;
    sum2 += (s.rays[0].range - distance) * (s.rays[0].range - distance);
  }
  EXPECT_NEAR(sum / n, 0.0, 4.0 * sigma / std::sqrt(static_cast<double>(n)));
  EXPECT_NEAR(std::sqrt(sum2 / n), sigma, 0.06 * sigma);
  // Facing away from the wall there is nothing to see.
  EXPECT_FALSE(sensor.measure({0.0, 0.0, 50.0}, -1.0, 0.0, scene, Weather{}).rays[0].hit);
  // And a wall beyond the range is not seen.
  ForwardSensorConfig short_range = one_ray();
  short_range.max_range_m = 300.0;
  ForwardSensor near_only(short_range, 3, 0);
  EXPECT_FALSE(near_only.measure({0.0, 0.0, 50.0}, 1.0, 0.0, scene, Weather{}).rays[0].hit);
}

TEST(ForwardSensor, FogShortensTheRangeAtWhichAWallIsSeenAndBlursItFirst) {
  const swarm::Scene scene = make_scene(true);
  // Extinction is 3 / visibility, so the transmission over 380 m is exp(-1140 / visibility).
  ForwardSensor clear(one_ray(), 4, 0);
  ForwardSensor light(one_ray(), 4, 0);
  ForwardSensor thick(one_ray(), 4, 0);
  const ForwardSample a = clear.measure({0.0, 0.0, 50.0}, 1.0, 0.0, scene, Weather::fog(20000.0));
  const ForwardSample b = light.measure({0.0, 0.0, 50.0}, 1.0, 0.0, scene, Weather::fog(2000.0));
  const ForwardSample c = thick.measure({0.0, 0.0, 50.0}, 1.0, 0.0, scene, Weather::fog(300.0));
  ASSERT_TRUE(a.rays[0].hit);
  ASSERT_TRUE(b.rays[0].hit);
  EXPECT_GT(b.rays[0].sigma, a.rays[0].sigma);  // blurred by the loss of transmission
  EXPECT_FALSE(c.rays[0].hit);                  // below the minimum transmission: not seen
}

TEST(ForwardSensor, DropsOutAtTheConfiguredRate) {
  const swarm::Scene scene = make_scene(true);
  ForwardSensorConfig c = one_ray();
  c.dropout_probability = 0.25;
  ForwardSensor sensor(c, 7, 0);
  int hits = 0;
  const int n = 4000;
  for (int i = 0; i < n; ++i) {
    hits += sensor.measure({0.0, 0.0, 50.0}, 1.0, 0.0, scene, Weather{}).rays[0].hit ? 1 : 0;
  }
  EXPECT_NEAR(static_cast<double>(hits) / n, 0.75, 0.03);
}

TEST(ForwardSensor, TheNoiseDoesNotDependOnWhatTheRaysHit) {
  const swarm::Scene wall = make_scene(true);
  // The same number of draws per call whether or not anything is hit: a sensor whose first call
  // saw the wall and one whose first call saw nothing are at the same place in their noise, so
  // their second calls, which both see the wall, give the same range.
  ForwardSensor a(one_ray(), 9, 0);
  ForwardSensor b(one_ray(), 9, 0);
  a.measure({0.0, 0.0, 50.0}, 1.0, 0.0, wall, Weather{});
  b.measure({0.0, 0.0, 50.0}, -1.0, 0.0, wall, Weather{});  // looks away: no hit
  const ForwardSample second_a = a.measure({0.0, 0.0, 50.0}, 1.0, 0.0, wall, Weather{});
  const ForwardSample second_b = b.measure({0.0, 0.0, 50.0}, 1.0, 0.0, wall, Weather{});
  ASSERT_TRUE(second_a.rays[0].hit);
  ASSERT_TRUE(second_b.rays[0].hit);
  EXPECT_DOUBLE_EQ(second_a.rays[0].range, second_b.rays[0].range);
}

TEST(ForwardSensor, EachAircraftAndSeedHasItsOwnNoiseAndARepeatIsIdentical) {
  const swarm::Scene scene = make_scene(true);
  ForwardSensor one(one_ray(), 5, 0);
  ForwardSensor repeat(one_ray(), 5, 0);
  ForwardSensor other_drone(one_ray(), 5, 1);
  ForwardSensor other_seed(one_ray(), 6, 0);
  const double r = one.measure({0.0, 0.0, 50.0}, 1.0, 0.0, scene, Weather{}).rays[0].range;
  EXPECT_DOUBLE_EQ(repeat.measure({0.0, 0.0, 50.0}, 1.0, 0.0, scene, Weather{}).rays[0].range, r);
  EXPECT_NE(other_drone.measure({0.0, 0.0, 50.0}, 1.0, 0.0, scene, Weather{}).rays[0].range, r);
  EXPECT_NE(other_seed.measure({0.0, 0.0, 50.0}, 1.0, 0.0, scene, Weather{}).rays[0].range, r);
}

TEST(ForwardSensor, ParametersAndInputsAreChecked) {
  const swarm::Scene scene = make_scene(true);
  ForwardSensorConfig c;
  c.horizontal_fov_deg = 0.0;
  EXPECT_THROW(ForwardSensor(c, 1, 0), std::invalid_argument);
  c = ForwardSensorConfig{};
  c.vertical_fov_deg = 90.0;
  EXPECT_THROW(ForwardSensor(c, 1, 0), std::invalid_argument);
  c = ForwardSensorConfig{};
  c.azimuth_rays = 0;
  EXPECT_THROW(ForwardSensor(c, 1, 0), std::invalid_argument);
  c = ForwardSensorConfig{};
  c.dropout_probability = 1.0;
  EXPECT_THROW(ForwardSensor(c, 1, 0), std::invalid_argument);
  c = ForwardSensorConfig{};
  c.min_transmission = 0.0;
  EXPECT_THROW(ForwardSensor(c, 1, 0), std::invalid_argument);
  c = ForwardSensorConfig{};
  c.period_ticks = 0;
  EXPECT_THROW(ForwardSensor(c, 1, 0), std::invalid_argument);
  ForwardSensor sensor(ForwardSensorConfig{}, 1, 0);
  EXPECT_THROW(sensor.measure({0.0, 0.0, std::nan("")}, 1.0, 0.0, scene, Weather{}),
               std::invalid_argument);
  EXPECT_THROW(sensor.measure({0.0, 0.0, 50.0}, 0.0, 0.0, scene, Weather{}), std::invalid_argument);
  EXPECT_THROW(sensor.measure({0.0, 0.0, 50.0}, std::nan(""), 0.0, scene, Weather{}),
               std::invalid_argument);
}

namespace {

// One aircraft orbiting at 150 m over a map that says the ground is flat at 0 m, as in the
// altimeter tests: the map asks for no climb, so only a sensor can.
BrainConfig orbit_over_flat_map(bool use_forward) {
  BrainConfig c;
  c.id = 0;
  c.num_drones = 1;
  c.dt = 0.02;
  c.offsets = swarm::make_formation(swarm::FormationShape::kLine, 1, 15.0);
  c.mission.waypoints = {{0, 0, 150}};
  c.pose.dt = c.dt;
  c.predictor_config.dt = c.dt;
  c.launch.position = {300.0, 0.0, 150.0};
  c.launch.velocity = {0.0, 16.7, 0.0};
  c.launch.position_sigma = 0.1;
  c.launch.velocity_sigma = 0.1;
  c.launch.bias_sigma = 0.05;
  c.guidance = swarm::Guidance::kOrbit;
  c.orbit.center = {0.0, 0.0, 150.0};
  c.orbit.radius = 300.0;
  c.orbit.speed = 16.7;
  c.orbit.min_speed = 12.0;
  c.orbit.max_speed = 25.0;
  c.terrain = std::make_shared<const swarm::TerrainMap>(10, 10, -2000.0, -2000.0, 400.0,
                                                        std::vector<double>(11 * 11, 0.0));
  c.terrain_guidance.enabled = true;
  c.terrain_guidance.clearance = 60.0;
  c.use_forward_sensor = use_forward;
  return c;
}

// A reading with one ray that hit: from the aircraft at (300, 0, 150) along (dx, dy, dz) at
// `range`.
ForwardSample one_hit(double dx, double dy, double dz, double range, double sigma = 1.0) {
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
  ForwardSample s;
  s.available = true;
  ForwardRay ray;
  ray.direction = {dx / length, dy / length, dz / length};
  ray.hit = true;
  ray.range = range;
  ray.sigma = sigma;
  s.rays.push_back(ray);
  return s;
}

double vertical_command(DroneBrain& brain, std::uint64_t tick, const ForwardSample& reading) {
  BrainInput in;
  in.imu.accel = {0.0, 0.0, 9.81};
  in.fix.available = true;
  in.fix.position = {300.0, 0.0, 150.0};
  in.fix.sigma = 0.2;
  in.forward = reading;
  return brain.step(in, tick).command.accel.z;
}

}  // namespace

TEST(ForwardGuidance, ARisingGroundAheadThatTheMapDoesNotHaveRaisesTheCommand) {
  DroneBrain with(orbit_over_flat_map(true));
  DroneBrain without(orbit_over_flat_map(false));
  const ForwardSample none;
  for (std::uint64_t t = 0; t < 5; ++t) {
    vertical_command(with, t, none);
    vertical_command(without, t, none);
  }
  // A surface 63 m above the aircraft's altitude, 190 m ahead along its heading.
  const ForwardSample high = one_hit(0.0, 0.95, 0.32, 200.0);
  EXPECT_GT(vertical_command(with, 5, high), 1.0);
  EXPECT_NEAR(vertical_command(without, 5, high), 0.0, 0.5);
}

TEST(ForwardGuidance, GroundBelowTheClearanceAndMissingReturnsChangeNothing) {
  DroneBrain with(orbit_over_flat_map(true));
  DroneBrain without(orbit_over_flat_map(false));
  // A surface 57 m below the aircraft: its floor (93 + 60 minus the climb gained on the way) is
  // under the nominal altitude, so the command is not touched, and not lowered either.
  const ForwardSample low = one_hit(0.0, 1.0, -0.2, 300.0);
  ForwardSample miss = low;
  miss.rays[0].hit = false;
  for (std::uint64_t t = 0; t < 20; ++t) {
    EXPECT_DOUBLE_EQ(vertical_command(with, t, low), vertical_command(without, t, low));
  }
  DroneBrain with_miss(orbit_over_flat_map(true));
  DroneBrain without_miss(orbit_over_flat_map(false));
  for (std::uint64_t t = 0; t < 20; ++t) {
    EXPECT_DOUBLE_EQ(vertical_command(with_miss, t, miss), vertical_command(without_miss, t, miss));
  }
}

TEST(ForwardGuidance, ADistantPeakAsksForLessThanANearOneAndTheRangeErrorCountsInTheSafeDirection) {
  // A surface 10 m above the aircraft, seen at 200 m and at 600 m: the far one lets the climb
  // start later, so it asks for less now.
  DroneBrain near(orbit_over_flat_map(true));
  DroneBrain far(orbit_over_flat_map(true));
  const ForwardSample none;
  for (std::uint64_t t = 0; t < 5; ++t) {
    vertical_command(near, t, none);
    vertical_command(far, t, none);
  }
  const double asks_near = vertical_command(near, 5, one_hit(0.0, 1.0, 0.05, 200.0));
  const double asks_far = vertical_command(far, 5, one_hit(0.0, 1.0, 0.0167, 600.0));
  EXPECT_GT(asks_far, 0.0);
  EXPECT_GT(asks_near, asks_far);
  // A rising ray with a large range error is taken at the farther, higher end.
  DroneBrain sure(orbit_over_flat_map(true));
  DroneBrain unsure(orbit_over_flat_map(true));
  for (std::uint64_t t = 0; t < 5; ++t) {
    vertical_command(sure, t, none);
    vertical_command(unsure, t, none);
  }
  EXPECT_GT(vertical_command(unsure, 5, one_hit(0.0, 0.95, 0.32, 200.0, 40.0)),
            vertical_command(sure, 5, one_hit(0.0, 0.95, 0.32, 200.0, 1.0)));
}

TEST(ForwardGuidance, AReadingThatIsTooOldIsNotUsed) {
  BrainConfig c = orbit_over_flat_map(true);
  c.forward_valid_ticks = 10;
  DroneBrain with(c);
  DroneBrain without(orbit_over_flat_map(false));
  const ForwardSample none;
  for (std::uint64_t t = 0; t < 3; ++t) {
    vertical_command(with, t, none);
    vertical_command(without, t, none);
  }
  EXPECT_GT(vertical_command(with, 3, one_hit(0.0, 0.95, 0.32, 200.0)), 1.0);
  vertical_command(without, 3, one_hit(0.0, 0.95, 0.32, 200.0));
  EXPECT_GT(vertical_command(with, 8, none), 0.0);  // within the valid time it still counts
  for (std::uint64_t t = 9; t < 20; ++t) {
    vertical_command(with, t, none);
    vertical_command(without, t, none);
  }
  EXPECT_NEAR(vertical_command(with, 20, none), vertical_command(without, 20, none), 0.5);
}

namespace {

std::string forward_scenario(int version, const std::string& vehicle, const std::string& sensors,
                             const std::string& control = std::string()) {
  return "version: " + std::to_string(version) +
         "\nseed: 1\nduration_s: 5\n"
         "scene: {type: field, size_m: [2000, 2000], cell_m: 20, obstacles: {density_per_m2: "
         "0.0}}\n"
         "swarm: {drones: 2}\n"
         "mission:\n  waypoints:\n    - [0, 0, 150]\n  speed_mps: 17\n" +
         vehicle + (sensors.empty() ? std::string() : "sensors:\n" + sensors) + "control:\n" +
         control + "  orbit: {radius_m: 300, radius_step_m: 12, altitude_step_m: 10}\n";
}

std::string first_error(const swarm::LoadResult& r) {
  return r.errors.empty() ? std::string() : swarm::to_string(r.errors.front());
}

}  // namespace

TEST(ForwardScenario, TheSensorKeysAreReadRoundTripAndWrittenOnlyWhenUsed) {
  const std::string fw = "vehicle: {type: fixed_wing}\n";
  const std::string forward =
      "  forward: {horizontal_fov_deg: 60, vertical_fov_deg: 30, pitch_deg: -3, azimuth_rays: 9, "
      "elevation_rays: 5, max_range_m: 900, sigma_floor_m: 3, sigma_per_m: 0.02, "
      "dropout_probability: 0.1, min_transmission: 0.3, period_ticks: 5}\n";
  const swarm::LoadResult r = swarm::load_scenario(forward_scenario(3, fw, forward));
  ASSERT_TRUE(r.ok()) << first_error(r);
  EXPECT_TRUE(r.scenario.forward_used);
  EXPECT_DOUBLE_EQ(r.scenario.forward.horizontal_fov_deg, 60.0);
  EXPECT_EQ(r.scenario.forward.azimuth_rays, 9U);
  EXPECT_EQ(r.scenario.forward.elevation_rays, 5U);
  EXPECT_DOUBLE_EQ(r.scenario.forward.max_range_m, 900.0);
  EXPECT_EQ(r.scenario.forward.period_ticks, 5U);
  const std::string text = swarm::canonical_text(r.scenario);
  EXPECT_NE(text.find("  forward:"), std::string::npos);
  const swarm::LoadResult again = swarm::load_scenario(text);
  ASSERT_TRUE(again.ok()) << first_error(again);
  EXPECT_EQ(swarm::canonical_text(again.scenario), text);

  const swarm::LoadResult plain = swarm::load_scenario(forward_scenario(3, fw, ""));
  ASSERT_TRUE(plain.ok());
  EXPECT_FALSE(plain.scenario.forward_used);
  EXPECT_EQ(swarm::canonical_text(plain.scenario).find("forward"), std::string::npos);
}

TEST(ForwardScenario, TheSensorNeedsVersionThreeAFixedWingVehicleAndValidValues) {
  const std::string fw = "vehicle: {type: fixed_wing}\n";
  const std::string forward = "  forward: {period_ticks: 5}\n";
  EXPECT_FALSE(swarm::load_scenario(forward_scenario(2, fw, forward)).ok());
  EXPECT_FALSE(swarm::load_scenario(forward_scenario(3, "vehicle: {type: quad}\n", forward)).ok());
  for (const char* bad :
       {"horizontal_fov_deg: 0", "horizontal_fov_deg: 181", "vertical_fov_deg: 90", "pitch_deg: 90",
        "azimuth_rays: 0", "elevation_rays: 0", "max_range_m: 0", "sigma_floor_m: 0",
        "dropout_probability: 1.0", "min_transmission: 0", "min_transmission: 1.5",
        "period_ticks: 0", "unknown_key: 1"}) {
    EXPECT_FALSE(
        swarm::load_scenario(forward_scenario(3, fw, std::string("  forward: {") + bad + "}\n"))
            .ok())
        << bad;
  }
  EXPECT_TRUE(
      swarm::load_scenario(forward_scenario(3, fw, "  forward: {min_transmission: 1.0}\n")).ok());
}

TEST(ForwardScenario, TheMapFaultKeysAreReadRoundTripAndWrittenOnlyWhenUsed) {
  const std::string fw = "vehicle: {type: fixed_wing}\n";
  const std::string control =
      "  terrain: {clearance_m: 60, offset_m: -30, error_sigma_m: 10, error_correlation_m: 200,\n"
      "            omit: [{x_m: [0, 100], y_m: [-50, 50]}, {x_m: [200, 300], y_m: [0, 10]}]}\n";
  const swarm::LoadResult r = swarm::load_scenario(forward_scenario(3, fw, "", control));
  ASSERT_TRUE(r.ok()) << first_error(r);
  const swarm::TerrainMapFaults& f = r.scenario.map_faults;
  EXPECT_DOUBLE_EQ(f.offset_m, -30.0);
  EXPECT_DOUBLE_EQ(f.error_sigma_m, 10.0);
  EXPECT_DOUBLE_EQ(f.error_correlation_m, 200.0);
  ASSERT_EQ(f.omit.size(), 2U);
  EXPECT_DOUBLE_EQ(f.omit[0].x_max, 100.0);
  EXPECT_DOUBLE_EQ(f.omit[1].y_max, 10.0);
  const std::string text = swarm::canonical_text(r.scenario);
  const swarm::LoadResult again = swarm::load_scenario(text);
  ASSERT_TRUE(again.ok()) << first_error(again);
  EXPECT_EQ(swarm::canonical_text(again.scenario), text);

  // A terrain section without faults writes none, so earlier canonical text is unchanged.
  const swarm::LoadResult plain =
      swarm::load_scenario(forward_scenario(3, fw, "", "  terrain: {clearance_m: 60}\n"));
  ASSERT_TRUE(plain.ok());
  EXPECT_FALSE(plain.scenario.map_faults.used());
  const std::string plain_text = swarm::canonical_text(plain.scenario);
  for (const char* key : {"offset_m", "error_sigma_m", "omit"}) {
    EXPECT_EQ(plain_text.find(key), std::string::npos) << key;
  }
}

TEST(ForwardScenario, TheMapFaultKeysAreChecked) {
  const std::string fw = "vehicle: {type: fixed_wing}\n";
  const auto load = [&](const std::string& terrain) {
    return swarm::load_scenario(forward_scenario(3, fw, "", "  terrain: {" + terrain + "}\n"));
  };
  EXPECT_FALSE(load("error_sigma_m: -1").ok());
  EXPECT_FALSE(load("error_correlation_m: 0").ok());
  EXPECT_FALSE(load("omit: [{x_m: [100, 0], y_m: [0, 10]}]").ok());  // min above max
  EXPECT_FALSE(load("omit: [{x_m: [0, 100]}]").ok());                // y_m missing
  EXPECT_FALSE(load("omit: [{x_m: [0, 100], y_m: [0, 10], z: 1}]").ok());
  EXPECT_FALSE(load("omit: 5").ok());
  EXPECT_TRUE(load("omit: []").ok());
}

namespace {

// A flat scene at 5 m with a 45 m bump at the middle sample, 10 by 10 cells of 10 m.
swarm::SceneData bump_scene() {
  swarm::SceneData scene;
  scene.terrain.cells_x = 10;
  scene.terrain.cells_y = 10;
  scene.terrain.origin_x = -50.0;
  scene.terrain.origin_y = -50.0;
  scene.terrain.cell_size = 10.0;
  scene.terrain.heights.assign(11 * 11, 5.0);
  scene.terrain.heights[5 * 11 + 5] = 50.0;
  return scene;
}

}  // namespace

TEST(MapFaults, AnExactMapIsTheDefaultAndFaultsOfZeroChangeNothing) {
  const swarm::SceneData scene = bump_scene();
  const swarm::TerrainMap exact = swarm::make_terrain_map(scene);
  const swarm::TerrainMap zero = swarm::make_terrain_map(scene, swarm::TerrainMapFaults{}, 99);
  for (double x : {-40.0, 0.0, 30.0}) {
    for (double y : {-40.0, 0.0, 30.0}) {
      EXPECT_EQ(exact.max_height_near(x, y, 3.0), zero.max_height_near(x, y, 3.0));
    }
  }
  EXPECT_EQ(exact.max_height_near(0.0, 0.0, 1.0), 50.0);
}

TEST(MapFaults, AnOffsetMovesEveryHeightOfTheMap) {
  swarm::TerrainMapFaults faults;
  faults.offset_m = -30.0;
  const swarm::TerrainMap map = swarm::make_terrain_map(bump_scene(), faults, 1);
  EXPECT_EQ(map.max_height_near(0.0, 0.0, 1.0), 20.0);
  EXPECT_EQ(map.max_height_near(-40.0, 40.0, 1.0), -25.0);
}

TEST(MapFaults, AnOmittedRectangleIsFlattenedToTheLowestHeightOnItsEdge) {
  swarm::TerrainMapFaults faults;
  faults.omit.push_back({-20.0, 20.0, -20.0, 20.0});
  const swarm::TerrainMap map = swarm::make_terrain_map(bump_scene(), faults, 1);
  EXPECT_EQ(map.max_height_near(0.0, 0.0, 1.0), 5.0);  // the bump is gone
  // The ground outside the rectangle is as it was.
  EXPECT_EQ(map.max_height_near(-45.0, -45.0, 1.0), 5.0);
  // A rectangle that contains no sample changes nothing.
  swarm::TerrainMapFaults between;
  between.omit.push_back({1.0, 4.0, 1.0, 4.0});
  EXPECT_EQ(swarm::make_terrain_map(bump_scene(), between, 1).max_height_near(0.0, 0.0, 1.0), 50.0);
  // The edge sets the level: a rectangle whose edge is higher flattens to that.
  swarm::SceneData raised = bump_scene();
  for (double& h : raised.terrain.heights) {
    h += 100.0;
  }
  EXPECT_EQ(swarm::make_terrain_map(raised, faults, 1).max_height_near(0.0, 0.0, 1.0), 105.0);
}

TEST(MapFaults, TheRandomErrorIsDeterministicSeededAndOfAboutTheStatedSize) {
  swarm::SceneData scene;
  scene.terrain.cells_x = 100;
  scene.terrain.cells_y = 100;
  scene.terrain.origin_x = -500.0;
  scene.terrain.origin_y = -500.0;
  scene.terrain.cell_size = 10.0;
  scene.terrain.heights.assign(101 * 101, 0.0);
  swarm::TerrainMapFaults faults;
  faults.error_sigma_m = 10.0;
  faults.error_correlation_m = 100.0;
  const swarm::TerrainMap a = swarm::make_terrain_map(scene, faults, 1);
  const swarm::TerrainMap again = swarm::make_terrain_map(scene, faults, 1);
  const swarm::TerrainMap other = swarm::make_terrain_map(scene, faults, 2);
  bool differs = false;
  bool varies = false;
  const double first = a.max_height_near(0.0, 0.0, 1.0);
  for (double x = -400.0; x <= 400.0; x += 100.0) {
    for (double y = -400.0; y <= 400.0; y += 100.0) {
      EXPECT_EQ(a.max_height_near(x, y, 1.0), again.max_height_near(x, y, 1.0));
      differs = differs || a.max_height_near(x, y, 1.0) != other.max_height_near(x, y, 1.0);
      varies = varies || a.max_height_near(x, y, 1.0) != first;
    }
  }
  EXPECT_TRUE(differs);
  EXPECT_TRUE(varies);
  // The highest point of a field of about a hundred independent lattice values of sigma 10 is a few
  // sigma above zero.
  const double highest = a.max_height_near(0.0, 0.0, 800.0);
  EXPECT_GT(highest, 10.0);
  EXPECT_LT(highest, 50.0);
}

namespace {

// Four aircraft orbiting over hills of 100 m with ground contact on, and a map that is 100 m too
// low: the guidance asks for an altitude that clears hills that are not where it thinks. Returns
// how many crashed.
int aircraft_lost(bool with_forward) {
  const std::string text =
      "version: 3\n"
      "seed: 5\n"
      "duration_s: 150\n"
      "scene: {type: hills, size_m: [2000, 2000], cell_m: 10, amplitude_m: 100,\n"
      "        obstacles: {density_per_m2: 0.0}}\n"
      "swarm: {drones: 4}\n"
      "mission:\n  waypoints:\n    - [0, 0, 8]\n  speed_mps: 17\n"
      "vehicle: {type: fixed_wing, dynamics: energy}\n"
      "environment: {ground_contact_crashes: true}\n"
      "weather: {visibility_m: 20000}\n" +
      std::string(with_forward ? "sensors:\n  forward: {period_ticks: 5}\n" : "") +
      "control:\n  orbit: {radius_m: 150, radius_step_m: 12, altitude_step_m: 10}\n";
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok()) << first_error(r);
  const swarm::ScenarioWorld world = swarm::build_world(r.scenario);
  const std::unique_ptr<swarm::Simulation> sim = world.make_simulation(1);
  for (std::uint64_t tick = 0; tick < r.scenario.duration_ticks; ++tick) {
    sim->step();
  }
  int lost = 0;
  for (std::size_t i = 0; i < r.scenario.drones; ++i) {
    lost += dynamic_cast<const swarm::DroneAgent&>(sim->agent(i)).crashed() ? 1 : 0;
  }
  return lost;
}

}  // namespace

TEST(ForwardFlight, WithoutAMapTheForwardSensorKeepsAircraftOffHillsThatOtherwiseCatchThem) {
  EXPECT_GT(aircraft_lost(false), 0);
  EXPECT_LT(aircraft_lost(true), aircraft_lost(false));
}
