#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "swarm/format/log.hpp"
#include "swarm/scenario/metrics.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"

namespace {

std::string pattern_scenario(const std::string& pattern, const std::string& extra = "") {
  return "version: 3\n"
         "seed: 11\n"
         "duration_s: 240\n"
         "scene: {type: field, size_m: [3000, 3000], cell_m: 20, obstacles: {density_per_m2: "
         "0.0}}\n"
         "swarm: {drones: 4}\n"
         "mission:\n"
         "  waypoints:\n"
         "    - [0, 0, 120]\n"
         "  speed_mps: 17\n"
         "vehicle: {type: fixed_wing}\n"
         "weather: {visibility_m: 20000}\n"
         "comms: {latency_ticks: 2, loss_model: independent, loss: 0.05}\n"
         "metrics: {settle_s: 120}\n"
         "control:\n"
         "  predictor: kalman\n"
         "  orbit: {radius_step_m: 0, altitude_step_m: 15}\n"
         "  pattern: " +
         pattern + "\n" + extra;
}

swarm::MetricsOutcome fly(const std::string& text, std::size_t threads = 2) {
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok()) << (r.errors.empty() ? "" : swarm::to_string(r.errors.front()));
  std::vector<std::uint8_t> bytes;
  EXPECT_EQ(swarm::run_scenario(r.scenario, threads, bytes), swarm::LogStatus::kOk);
  return swarm::analyze_log(bytes.data(), bytes.size());
}

}  // namespace

TEST(FlightPatterns, AStaggeredRacetrackIsFlownOnItsPath) {
  const swarm::MetricsOutcome m = fly(pattern_scenario(
      "{type: racetrack, leg_length_m: 500, turn_radius_m: 100, heading_deg: 20, stagger_s: 12}"));
  ASSERT_TRUE(m.ok()) << m.message;
  // After the settle time every aircraft is near its place on the racetrack.
  ASSERT_TRUE(m.result.orbit_radial_rmse.has_value());
  EXPECT_LT(*m.result.orbit_radial_rmse, 10.0);
  EXPECT_LT(*m.result.orbit_phase_rmse, 60.0);
  EXPECT_GT(*m.result.time_on_station, 0.7);
  EXPECT_EQ(m.result.collision_episodes, 0U);
  EXPECT_GT(*m.result.min_separation, 30.0);
}

TEST(FlightPatterns, ParallelLanesAreFlownWithoutCollision) {
  const swarm::MetricsOutcome m = fly(pattern_scenario(
      "{type: lanes, leg_length_m: 500, turn_radius_m: 100, lane_spacing_m: 300, heading_deg: 0, "
      "stagger_s: 12}"));
  ASSERT_TRUE(m.ok()) << m.message;
  EXPECT_LT(*m.result.orbit_radial_rmse, 10.0);
  EXPECT_EQ(m.result.collision_episodes, 0U);
  EXPECT_GT(*m.result.min_separation, 30.0);
}

TEST(FlightPatterns, ARacetrackRunIsTheSameOnEveryThreadCount) {
  const std::string text = pattern_scenario("{type: racetrack, leg_length_m: 400, stagger_s: 10}");
  const swarm::LoadResult r = swarm::load_scenario(text);
  ASSERT_TRUE(r.ok());
  std::uint64_t one = 0;
  std::uint64_t four = 0;
  std::vector<std::uint8_t> a;
  std::vector<std::uint8_t> b;
  ASSERT_EQ(swarm::run_scenario(r.scenario, 1, a, &one), swarm::LogStatus::kOk);
  ASSERT_EQ(swarm::run_scenario(r.scenario, 4, b, &four), swarm::LogStatus::kOk);
  EXPECT_EQ(one, four);
  EXPECT_EQ(a, b);
}

TEST(FlightPatterns, AnInfeasibleTurnIsRefusedWhenTheScenarioLoads) {
  // A 30 m turn at about 17 m/s needs about 9.6 m/s^2, far over the bank limit.
  const swarm::LoadResult r = swarm::load_scenario(
      pattern_scenario("{type: racetrack, leg_length_m: 400, turn_radius_m: 30}"));
  EXPECT_FALSE(r.ok());
  // Lanes closer together than twice the turn radius plus the safe distance are refused.
  EXPECT_FALSE(swarm::load_scenario(
                   pattern_scenario("{type: lanes, turn_radius_m: 100, lane_spacing_m: 220}"))
                   .ok());
}

TEST(FlightPatterns, TheMetricsReportHeightAboveGroundAndContacts) {
  const swarm::MetricsOutcome m = fly(pattern_scenario("{type: racetrack, leg_length_m: 400}"));
  ASSERT_TRUE(m.ok()) << m.message;
  ASSERT_TRUE(m.result.min_agl.has_value());
  // Over flat ground at 0 m, the lowest aircraft is at about its layer altitude of 120 m.
  EXPECT_GT(*m.result.min_agl, 80.0);
  // The mean is over every aircraft and record, so it is not below the minimum, and over flat
  // ground with the layers of the swarm it is above the lowest layer.
  ASSERT_TRUE(m.result.mean_agl.has_value());
  EXPECT_GE(*m.result.mean_agl, *m.result.min_agl);
  EXPECT_GT(*m.result.mean_agl, 80.0);
  ASSERT_TRUE(m.result.ground_contacts.has_value());
  EXPECT_EQ(*m.result.ground_contacts, 0U);
}

TEST(FlightPatterns, ACrashIsCountedAsAGroundContact) {
  // Orbiting at 5 m over hills that rise to 100 m, with ground contact on.
  const swarm::MetricsOutcome m =
      fly("version: 3\nseed: 5\nduration_s: 90\n"
          "scene: {type: hills, size_m: [2000, 2000], cell_m: 10, amplitude_m: 100, "
          "obstacles: {density_per_m2: 0.0}}\n"
          "swarm: {drones: 4}\nmission:\n  waypoints:\n    - [0, 0, 8]\n  speed_mps: 17\n"
          "vehicle: {type: fixed_wing}\nenvironment: {ground_contact_crashes: true}\n"
          "weather: {visibility_m: 20000}\n");
  ASSERT_TRUE(m.ok()) << m.message;
  EXPECT_GT(*m.result.ground_contacts, 0U);
  EXPECT_LE(*m.result.min_agl, 0.0);
}
