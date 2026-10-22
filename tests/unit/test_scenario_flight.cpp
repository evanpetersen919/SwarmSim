#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "swarm/control/path.hpp"
#include "swarm/format/log.hpp"
#include "swarm/scenario/metrics.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"

using swarm::FixedWingDynamics;
using swarm::LoadResult;

namespace {

const char* kBody =
    "seed: 1\n"
    "duration_s: 10\n"
    "swarm: {drones: 3}\n"
    "mission:\n"
    "  waypoints:\n"
    "    - [0, 0, 100]\n"
    "  speed_mps: 17\n";

std::string file(int version, const std::string& rest = "") {
  return "version: " + std::to_string(version) + "\n" + kBody + rest;
}

const char* kEnergy = "vehicle: {type: fixed_wing, dynamics: energy}\n";
const char* kWind = "environment:\n  wind: {mean_mps: [4, -2], gust_mps: 1.5, gust_scale_m: 120}\n";

bool has_error(const LoadResult& r, const std::string& path, const std::string& text) {
  for (const swarm::ScenarioError& e : r.errors) {
    if (e.path == path && e.message.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}

std::uint64_t hash_of(const std::string& text, std::size_t threads) {
  const LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok());
  std::vector<std::uint8_t> bytes;
  std::uint64_t hash = 0;
  EXPECT_EQ(swarm::run_scenario(r.scenario, threads, bytes, &hash), swarm::LogStatus::kOk);
  return hash;
}

}  // namespace

TEST(ScenarioFlight, TheEnergyModelAndTheWindNeedVersionThree) {
  EXPECT_TRUE(
      has_error(swarm::load_scenario(file(2, kEnergy)), "vehicle.dynamics", "needs version 3"));
  EXPECT_TRUE(
      has_error(swarm::load_scenario(file(2, std::string("vehicle: {type: fixed_wing}\n") + kWind)),
                "environment", "needs version 3"));
  EXPECT_TRUE(swarm::load_scenario(file(3, kEnergy)).ok());
}

TEST(ScenarioFlight, TheEnergyModelDerivesTheMinimumSpeedFromTheWing) {
  const LoadResult r = swarm::load_scenario(file(3, kEnergy));
  ASSERT_TRUE(r.ok());
  const swarm::VehicleSettings& v = r.scenario.vehicle;
  EXPECT_EQ(v.dynamics, FixedWingDynamics::kEnergy);
  EXPECT_NEAR(v.fixed_wing.min_speed, v.energy.hold_speed(v.fixed_wing.gravity), 1e-12);
  EXPECT_TRUE(has_error(swarm::load_scenario(file(3,
                                                  "vehicle: {type: fixed_wing, dynamics: "
                                                  "energy, min_speed_mps: 13}\n")),
                        "vehicle.min_speed_mps", "follows from the wing"));
}

TEST(ScenarioFlight, EnergyParametersAreOverriddenInTheirUnitsAndNeedTheEnergyModel) {
  const LoadResult r = swarm::load_scenario(
      file(3,
           "vehicle:\n  type: fixed_wing\n  dynamics: energy\n  energy: {mass_kg: 10, "
           "wing_area_m2: 0.8, max_power_w: 500}\n"));
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.scenario.vehicle.energy.mass, 10.0);
  EXPECT_EQ(r.scenario.vehicle.energy.wing_area, 0.8);
  EXPECT_EQ(r.scenario.vehicle.energy.max_power, 500.0);
  EXPECT_TRUE(has_error(swarm::load_scenario(file(3,
                                                  "vehicle: {type: fixed_wing, energy: "
                                                  "{mass_kg: 10}}\n")),
                        "vehicle.energy", "needs dynamics: energy"));
  EXPECT_TRUE(has_error(swarm::load_scenario(file(3,
                                                  "vehicle: {type: fixed_wing, dynamics: "
                                                  "energy, energy: {stall_margin: 0.5}}\n")),
                        "vehicle.energy.stall_margin", ""));
}

TEST(ScenarioFlight, TheWindIsReadAndBadValuesAreErrors) {
  const LoadResult r = swarm::load_scenario(file(3, std::string(kEnergy) + kWind));
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.scenario.wind.mean.x, 4.0);
  EXPECT_EQ(r.scenario.wind.mean.y, -2.0);
  EXPECT_EQ(r.scenario.wind.gust_sigma, 1.5);
  EXPECT_EQ(r.scenario.wind.gust_scale, 120.0);
  EXPECT_TRUE(has_error(swarm::load_scenario(file(3, "environment:\n  wind: {gust_mps: -1}\n")),
                        "environment.wind.gust_mps", ""));
  EXPECT_TRUE(
      has_error(swarm::load_scenario(file(3, "environment:\n  wind: {mean_mps: [1, 2, 3]}\n")),
                "environment.wind.mean_mps", "list of 2"));
}

TEST(ScenarioFlight, CanonicalTextNamesTheLowestVersionAndRoundTrips) {
  // Nothing new: the text is as it was.
  const std::string plain = swarm::canonical_text(
      swarm::load_scenario(file(3, "vehicle: {type: fixed_wing}\n")).scenario);
  EXPECT_EQ(plain.rfind("version: 2\n", 0), 0U);
  EXPECT_EQ(plain.find("environment:"), std::string::npos);
  EXPECT_EQ(plain.find("dynamics:"), std::string::npos);

  const swarm::Scenario flight =
      swarm::load_scenario(file(3, std::string(kEnergy) + kWind)).scenario;
  const std::string text = swarm::canonical_text(flight);
  EXPECT_EQ(text.rfind("version: 3\n", 0), 0U);
  EXPECT_NE(text.find("dynamics: energy"), std::string::npos);
  EXPECT_EQ(text.find("min_speed_mps"), std::string::npos);
  const LoadResult again = swarm::load_scenario(text);
  ASSERT_TRUE(again.ok());
  EXPECT_EQ(swarm::canonical_text(again.scenario), text);
  EXPECT_EQ(again.scenario.wind.gust_sigma, flight.wind.gust_sigma);
  EXPECT_EQ(again.scenario.vehicle.energy.mass, flight.vehicle.energy.mass);

  // Wind alone, on the simple model, is version 3 too.
  const std::string windy = swarm::canonical_text(
      swarm::load_scenario(file(3, std::string("vehicle: {type: fixed_wing}\n") + kWind)).scenario);
  EXPECT_EQ(windy.rfind("version: 3\n", 0), 0U);
}

TEST(ScenarioFlight, TheWorldHandsTheWindAndTheEnergyModelToEveryDrone) {
  const swarm::Scenario s = swarm::load_scenario(file(3, std::string(kEnergy) + kWind)).scenario;
  const swarm::ScenarioWorld world = swarm::build_world(s);
  ASSERT_EQ(world.agents.size(), 3U);
  for (const swarm::DroneAgentConfig& a : world.agents) {
    EXPECT_EQ(a.dynamics, FixedWingDynamics::kEnergy);
    EXPECT_EQ(a.wind.mean.x, 4.0);
    EXPECT_EQ(a.wind.gust_sigma, 1.5);
  }
}

TEST(ScenarioFlight, AWindyEnergyRunIsRepeatableAndDiffersFromACalmOne) {
  const std::string calm = file(3, kEnergy);
  const std::string windy = file(3, std::string(kEnergy) + kWind);
  EXPECT_EQ(hash_of(windy, 1), hash_of(windy, 1));
  EXPECT_EQ(hash_of(windy, 1), hash_of(windy, 3));
  EXPECT_NE(hash_of(windy, 1), hash_of(calm, 1));
}

TEST(ScenarioFlight, ASimpleFixedWingRunWithoutWindIsTheSameAsBefore) {
  // The same run written as a version 2 file and as a version 3 file with no new keys has the same
  // canonical text and so the same hash.
  const std::string v2 = file(2, "vehicle: {type: fixed_wing}\n");
  const std::string v3 = file(3, "vehicle: {type: fixed_wing}\n");
  EXPECT_EQ(swarm::canonical_text(swarm::load_scenario(v2).scenario),
            swarm::canonical_text(swarm::load_scenario(v3).scenario));
  EXPECT_EQ(hash_of(v2, 1), hash_of(v3, 1));
}

TEST(ScenarioFlight, TheFlightScenarioFileFliesItsPatternAboveTheGround) {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/fixed_wing_flight.yaml", std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  LoadResult r = swarm::load_scenario(text);
  ASSERT_TRUE(r.ok());
  r.scenario.duration_ticks = 4500;  // 90 s, enough to join the racetrack and settle
  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(r.scenario, 2, bytes), swarm::LogStatus::kOk);
  const swarm::MetricsOutcome m = swarm::analyze_log(bytes.data(), bytes.size());
  ASSERT_TRUE(m.ok()) << m.message;
  EXPECT_EQ(m.result.alive_at_end, 8U);
  EXPECT_EQ(m.result.collision_episodes, 0U);
  EXPECT_EQ(*m.result.ground_contacts, 0U);
  EXPECT_GT(*m.result.min_agl, 40.0);
  EXPECT_LT(*m.result.orbit_radial_rmse, 10.0);
}

TEST(ScenarioFlight, TheFormationScenarioFileHoldsItsFormationThroughTheSequence) {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/fixed_wing_formation.yaml",
                   std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const LoadResult r = swarm::load_scenario(text);
  ASSERT_TRUE(r.ok()) << (r.errors.empty() ? "" : swarm::to_string(r.errors.front()));
  EXPECT_EQ(r.scenario.drones, 4U);
  EXPECT_EQ(r.scenario.duration_ticks, 3000U);  // 60 s
  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(r.scenario, 2, bytes), swarm::LogStatus::kOk);
  const swarm::MetricsOutcome m = swarm::analyze_log(bytes.data(), bytes.size());
  ASSERT_TRUE(m.ok()) << m.message;
  EXPECT_EQ(m.result.alive_at_end, 4U);
  EXPECT_EQ(m.result.collision_episodes, 0U);
  EXPECT_EQ(*m.result.ground_contacts, 0U);
  EXPECT_GT(*m.result.min_separation, 15.0);
  EXPECT_LT(*m.result.orbit_radial_rmse, 6.0);
  EXPECT_LT(*m.result.orbit_phase_rmse, 8.0);
}

TEST(ScenarioFlight, TheRakeScenarioFileSweepsAWideStripWithoutCollision) {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/fixed_wing_rake.yaml", std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const LoadResult r = swarm::load_scenario(text);
  ASSERT_TRUE(r.ok()) << (r.errors.empty() ? "" : swarm::to_string(r.errors.front()));
  EXPECT_EQ(r.scenario.drones, 15U);
  EXPECT_NEAR(swarm::rake_spacing(r.scenario.pattern.formation), 739.0, 1.0);
  const std::string canonical = swarm::canonical_text(r.scenario);
  EXPECT_NE(canonical.find("shape: rake"), std::string::npos);
  const LoadResult again = swarm::load_scenario(canonical);
  ASSERT_TRUE(again.ok()) << (again.errors.empty() ? "" : swarm::to_string(again.errors.front()));
  EXPECT_EQ(swarm::canonical_text(again.scenario), canonical);

  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(r.scenario, 2, bytes), swarm::LogStatus::kOk);
  const swarm::MetricsOutcome m = swarm::analyze_log(bytes.data(), bytes.size());
  ASSERT_TRUE(m.ok()) << m.message;
  EXPECT_EQ(m.result.alive_at_end, 15U);
  EXPECT_EQ(m.result.collision_episodes, 0U);
  EXPECT_EQ(*m.result.ground_contacts, 0U);
  // Neighbors are a swath apart; the closest pair cannot be nearer than the 20 m of altitude.
  EXPECT_GT(*m.result.min_separation, 20.0);
  EXPECT_LT(*m.result.orbit_radial_rmse, 10.0);
}
