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
#include "swarm/radar_altimeter.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"
#include "swarm/simulation.hpp"

using swarm::AltimeterSample;
using swarm::BrainConfig;
using swarm::BrainInput;
using swarm::DroneBrain;
using swarm::RadarAltimeter;
using swarm::RadarAltimeterConfig;
using swarm::Vec3;

TEST(RadarAltimeter, ReportsTheHeightAboveTheSurfaceWithNoiseOfTheStatedSize) {
  RadarAltimeterConfig c;
  c.dropout_probability = 0.0;
  RadarAltimeter sensor(c, 5, 0);
  const double height = 400.0;
  const double sigma = std::max(c.sigma_floor_m, c.sigma_per_m * height);
  double sum = 0.0;
  double sum2 = 0.0;
  const int n = 4000;
  for (int i = 0; i < n; ++i) {
    const AltimeterSample s = sensor.measure({10.0, 20.0, 500.0}, 100.0);
    ASSERT_TRUE(s.available);
    EXPECT_DOUBLE_EQ(s.sigma, sigma);
    sum += s.height - height;
    sum2 += (s.height - height) * (s.height - height);
  }
  EXPECT_NEAR(sum / n, 0.0, 4.0 * sigma / std::sqrt(static_cast<double>(n)));
  EXPECT_NEAR(std::sqrt(sum2 / n), sigma, 0.05 * sigma);
}

TEST(RadarAltimeter, HasNoReadingBelowTheSurfaceOrBeyondItsRange) {
  RadarAltimeterConfig c;
  c.dropout_probability = 0.0;
  RadarAltimeter sensor(c, 5, 0);
  EXPECT_FALSE(sensor.measure({0.0, 0.0, 100.0}, 100.0).available);  // on the surface
  EXPECT_FALSE(sensor.measure({0.0, 0.0, 90.0}, 100.0).available);   // below it
  EXPECT_FALSE(sensor.measure({0.0, 0.0, 100.0 + c.max_range_m + 1.0}, 100.0).available);
  EXPECT_TRUE(sensor.measure({0.0, 0.0, 100.0 + c.max_range_m - 1.0}, 100.0).available);
}

TEST(RadarAltimeter, DropsOutAtTheConfiguredRate) {
  RadarAltimeterConfig c;
  c.dropout_probability = 0.2;
  RadarAltimeter sensor(c, 9, 3);
  int missing = 0;
  const int n = 5000;
  for (int i = 0; i < n; ++i) {
    missing += sensor.measure({0.0, 0.0, 300.0}, 0.0).available ? 0 : 1;
  }
  EXPECT_NEAR(static_cast<double>(missing) / n, 0.2, 0.03);
}

TEST(RadarAltimeter, TheNoiseDoesNotDependOnEarlierOutcomes) {
  // A sensor that was out of range on its first call and one that was not draw the same noise on
  // the second call, because every call draws the same numbers.
  RadarAltimeterConfig c;
  c.dropout_probability = 0.0;
  RadarAltimeter a(c, 5, 1);
  RadarAltimeter b(c, 5, 1);
  a.measure({0.0, 0.0, 99999.0}, 0.0);  // out of range
  b.measure({0.0, 0.0, 300.0}, 0.0);
  const AltimeterSample sa = a.measure({0.0, 0.0, 300.0}, 0.0);
  const AltimeterSample sb = b.measure({0.0, 0.0, 300.0}, 0.0);
  EXPECT_DOUBLE_EQ(sa.height, sb.height);
}

TEST(RadarAltimeter, EachAircraftAndSeedHasItsOwnNoise) {
  RadarAltimeterConfig c;
  c.dropout_probability = 0.0;
  RadarAltimeter first(c, 5, 0);
  RadarAltimeter repeat(c, 5, 0);
  RadarAltimeter other_drone(c, 5, 1);
  RadarAltimeter other_seed(c, 6, 0);
  const AltimeterSample s = first.measure({0.0, 0.0, 300.0}, 0.0);
  EXPECT_DOUBLE_EQ(repeat.measure({0.0, 0.0, 300.0}, 0.0).height, s.height);
  EXPECT_NE(other_drone.measure({0.0, 0.0, 300.0}, 0.0).height, s.height);
  EXPECT_NE(other_seed.measure({0.0, 0.0, 300.0}, 0.0).height, s.height);
}

TEST(RadarAltimeter, ParametersAndInputsAreChecked) {
  RadarAltimeterConfig c;
  c.sigma_floor_m = 0.0;
  EXPECT_THROW(RadarAltimeter(c, 1, 0), std::invalid_argument);
  c = RadarAltimeterConfig{};
  c.dropout_probability = 1.0;
  EXPECT_THROW(RadarAltimeter(c, 1, 0), std::invalid_argument);
  c = RadarAltimeterConfig{};
  c.period_ticks = 0;
  EXPECT_THROW(RadarAltimeter(c, 1, 0), std::invalid_argument);
  RadarAltimeter sensor(RadarAltimeterConfig{}, 1, 0);
  EXPECT_THROW(sensor.measure({0.0, 0.0, std::nan("")}, 0.0), std::invalid_argument);
  EXPECT_THROW(sensor.measure({0.0, 0.0, 10.0}, std::nan("")), std::invalid_argument);
}

namespace {

// One aircraft orbiting at 150 m over a map that says the ground is flat at 0 m, with a clearance
// of 60 m: the map asks for no climb. The world may disagree with the map; the altimeter is what
// tells the aircraft.
BrainConfig orbit_over_flat_map(bool use_altimeter) {
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
  c.use_altimeter = use_altimeter;
  return c;
}

// The vertical acceleration the brain commands on tick `tick`, given a reading on that tick.
double vertical_command(DroneBrain& brain, std::uint64_t tick, const AltimeterSample& reading) {
  BrainInput in;
  in.imu.accel = {0.0, 0.0, 9.81};
  in.fix.available = true;
  in.fix.position = {300.0, 0.0, 150.0};
  in.fix.sigma = 0.2;
  in.altimeter = reading;
  return brain.step(in, tick).command.accel.z;
}

AltimeterSample reading(double height) {
  AltimeterSample s;
  s.available = true;
  s.height = height;
  s.sigma = 1.0;
  return s;
}

}  // namespace

TEST(AltimeterGuidance, AMapThatDisagreesWithTheWorldIsOverriddenByTheMeasuredHeight) {
  DroneBrain with(orbit_over_flat_map(true));
  DroneBrain without(orbit_over_flat_map(false));
  const AltimeterSample none;
  for (std::uint64_t t = 0; t < 5; ++t) {
    vertical_command(with, t, none);
    vertical_command(without, t, none);
  }
  // The map says 150 m is 150 m above ground. The altimeter says the ground is 20 m below.
  const double raised = vertical_command(with, 5, reading(20.0));
  const double ignored = vertical_command(without, 5, reading(20.0));
  EXPECT_GT(raised, 1.0);
  EXPECT_NEAR(ignored, 0.0, 0.5);
}

TEST(AltimeterGuidance, EnoughMeasuredClearanceChangesNothing) {
  DroneBrain with(orbit_over_flat_map(true));
  DroneBrain without(orbit_over_flat_map(false));
  for (std::uint64_t t = 0; t < 5; ++t) {
    EXPECT_DOUBLE_EQ(vertical_command(with, t, reading(150.0)),
                     vertical_command(without, t, reading(150.0)));
  }
  // The margin counts: a reading just above the clearance but within two sigma of it still climbs.
  DroneBrain edge(orbit_over_flat_map(true));
  for (std::uint64_t t = 0; t < 5; ++t) {
    vertical_command(edge, t, AltimeterSample{});
  }
  EXPECT_GT(vertical_command(edge, 5, reading(61.0)), 0.0);
}

TEST(AltimeterGuidance, ItNeverLowersTheCommand) {
  DroneBrain with(orbit_over_flat_map(true));
  DroneBrain without(orbit_over_flat_map(false));
  for (std::uint64_t t = 0; t < 20; ++t) {
    // A very large height: the aircraft is far above ground. It must not descend for that.
    EXPECT_DOUBLE_EQ(vertical_command(with, t, reading(1400.0)),
                     vertical_command(without, t, reading(1400.0)));
  }
}

TEST(AltimeterGuidance, AReadingThatIsTooOldIsNotUsedAndNoReadingFallsBackToThePlan) {
  BrainConfig c = orbit_over_flat_map(true);
  c.altimeter_valid_ticks = 10;
  DroneBrain with(c);
  DroneBrain without(orbit_over_flat_map(false));
  const AltimeterSample none;
  for (std::uint64_t t = 0; t < 3; ++t) {
    vertical_command(with, t, none);
    vertical_command(without, t, none);
  }
  EXPECT_GT(vertical_command(with, 3, reading(20.0)), 1.0);
  vertical_command(without, 3, reading(20.0));
  // Within the valid time the last reading still counts.
  EXPECT_GT(vertical_command(with, 8, none), 0.0);
  // After it, the plan alone applies.
  for (std::uint64_t t = 9; t < 20; ++t) {
    vertical_command(with, t, none);
    vertical_command(without, t, none);
  }
  EXPECT_NEAR(vertical_command(with, 20, none), vertical_command(without, 20, none), 0.5);
}

namespace {

std::string altimeter_scenario(int version, const std::string& vehicle, const std::string& extra) {
  return "version: " + std::to_string(version) +
         "\nseed: 1\nduration_s: 5\n"
         "scene: {type: field, size_m: [2000, 2000], cell_m: 20, obstacles: {density_per_m2: "
         "0.0}}\n"
         "swarm: {drones: 2}\n"
         "mission:\n  waypoints:\n    - [0, 0, 150]\n  speed_mps: 17\n" +
         vehicle + (extra.empty() ? std::string() : "sensors:\n" + extra) +
         "control:\n  orbit: {radius_m: 300, radius_step_m: 12, altitude_step_m: 10}\n";
}

}  // namespace

TEST(AltimeterScenario, TheKeysAreReadRoundTripAndWrittenOnlyWhenUsed) {
  const std::string fw = "vehicle: {type: fixed_wing}\n";
  const std::string altimeter =
      "  radar_altimeter: {sigma_floor_m: 2.0, sigma_per_m: 0.02, max_range_m: 900, "
      "dropout_probability: 0.1, period_ticks: 5}\n";
  const swarm::LoadResult r = swarm::load_scenario(altimeter_scenario(3, fw, altimeter));
  ASSERT_TRUE(r.ok()) << (r.errors.empty() ? "" : swarm::to_string(r.errors.front()));
  EXPECT_TRUE(r.scenario.altimeter_used);
  EXPECT_DOUBLE_EQ(r.scenario.altimeter.sigma_floor_m, 2.0);
  EXPECT_DOUBLE_EQ(r.scenario.altimeter.max_range_m, 900.0);
  EXPECT_EQ(r.scenario.altimeter.period_ticks, 5U);
  const std::string text = swarm::canonical_text(r.scenario);
  EXPECT_NE(text.find("radar_altimeter:"), std::string::npos);
  const swarm::LoadResult again = swarm::load_scenario(text);
  ASSERT_TRUE(again.ok()) << (again.errors.empty() ? "" : swarm::to_string(again.errors.front()));
  EXPECT_EQ(swarm::canonical_text(again.scenario), text);

  // Without the section the text has no trace of it, and the scenario stays at the version it
  // needs.
  const swarm::LoadResult plain = swarm::load_scenario(altimeter_scenario(3, fw, ""));
  ASSERT_TRUE(plain.ok());
  EXPECT_FALSE(plain.scenario.altimeter_used);
  EXPECT_EQ(swarm::canonical_text(plain.scenario).find("radar_altimeter"), std::string::npos);
}

TEST(AltimeterScenario, ItNeedsVersionThreeAFixedWingVehicleAndValidValues) {
  const std::string fw = "vehicle: {type: fixed_wing}\n";
  const std::string altimeter = "  radar_altimeter: {period_ticks: 5}\n";
  EXPECT_FALSE(swarm::load_scenario(altimeter_scenario(2, fw, altimeter)).ok());
  EXPECT_FALSE(
      swarm::load_scenario(altimeter_scenario(3, "vehicle: {type: quad}\n", altimeter)).ok());
  EXPECT_FALSE(swarm::load_scenario(
                   altimeter_scenario(3, fw, "  radar_altimeter: {dropout_probability: 1.0}\n"))
                   .ok());
  EXPECT_FALSE(
      swarm::load_scenario(altimeter_scenario(3, fw, "  radar_altimeter: {sigma_floor_m: 0}\n"))
          .ok());
  EXPECT_FALSE(
      swarm::load_scenario(altimeter_scenario(3, fw, "  radar_altimeter: {unknown_key: 1}\n"))
          .ok());
}

namespace {

// Four aircraft orbiting at 8 m over hills of 100 m, with ground contact on and no terrain map, so
// only the altimeter can keep them off the ground.
int aircraft_lost(bool with_altimeter) {
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
      std::string(with_altimeter ? "sensors:\n  radar_altimeter: {period_ticks: 5}\n" : "") +
      "control:\n  orbit: {radius_m: 150, radius_step_m: 12, altitude_step_m: 10}\n";
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok()) << (r.errors.empty() ? "" : swarm::to_string(r.errors.front()));
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

TEST(AltimeterFlight, WithoutAMapTheAltimeterKeepsAircraftOffHillsThatOtherwiseCatchThem) {
  EXPECT_GT(aircraft_lost(false), 0);
  EXPECT_EQ(aircraft_lost(true), 0);
}
