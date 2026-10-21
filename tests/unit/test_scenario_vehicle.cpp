#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "swarm/format/log.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"

using swarm::FixedWingParams;
using swarm::LoadResult;
using swarm::VehicleKind;

namespace {

const char* kBody =
    "seed: 1\n"
    "duration_s: 10\n"
    "swarm: {drones: 3}\n"
    "mission:\n"
    "  waypoints:\n"
    "    - [0, 0, 20]\n";

std::string file(int version, const std::string& vehicle = "") {
  return "version: " + std::to_string(version) + "\n" + kBody + vehicle;
}

bool has_error(const LoadResult& r, const std::string& path, const std::string& text) {
  for (const swarm::ScenarioError& e : r.errors) {
    if (e.path == path && e.message.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(ScenarioVehicle, AScenarioWithoutAVehicleIsAQuadInBothVersions) {
  EXPECT_EQ(swarm::load_scenario(file(1)).scenario.vehicle.kind, VehicleKind::kQuad);
  EXPECT_EQ(swarm::load_scenario(file(2)).scenario.vehicle.kind, VehicleKind::kQuad);
}

TEST(ScenarioVehicle, AFixedWingVehicleTakesTheDefaultsWhenNothingIsOverridden) {
  const LoadResult r = swarm::load_scenario(file(2, "vehicle: {type: fixed_wing}\n"));
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.scenario.vehicle.kind, VehicleKind::kFixedWing);
  const FixedWingParams defaults;
  EXPECT_EQ(r.scenario.vehicle.fixed_wing.min_speed, defaults.min_speed);
  EXPECT_EQ(r.scenario.vehicle.fixed_wing.max_bank, defaults.max_bank);
}

TEST(ScenarioVehicle, OverridesAreReadInTheUnitsOfTheirKeys) {
  const LoadResult r = swarm::load_scenario(
      file(2, "vehicle:\n  type: fixed_wing\n  min_speed_mps: 14\n  max_speed_mps: 30\n"
              "  max_bank_deg: 45\n"));
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.scenario.vehicle.fixed_wing.min_speed, 14.0);
  EXPECT_EQ(r.scenario.vehicle.fixed_wing.max_speed, 30.0);
  EXPECT_NEAR(r.scenario.vehicle.fixed_wing.max_bank, 0.78539816339744828, 1e-12);
}

TEST(ScenarioVehicle, AVersionOneFileCannotHaveAVehicleSection) {
  const LoadResult r = swarm::load_scenario(file(1, "vehicle: {type: fixed_wing}\n"));
  EXPECT_FALSE(r.ok());
  EXPECT_TRUE(has_error(r, "vehicle", "needs version 2"));
}

TEST(ScenarioVehicle, BadVehicleValuesAreErrorsWithTheirPaths) {
  EXPECT_TRUE(has_error(swarm::load_scenario(file(2, "vehicle: {type: balloon}\n")), "vehicle.type",
                        "must be one of quad, fixed_wing"));
  EXPECT_TRUE(has_error(
      swarm::load_scenario(file(2, "vehicle: {type: fixed_wing, max_bank_deg: 90}\n")),
      "vehicle.max_bank_deg", ""));
  EXPECT_TRUE(has_error(
      swarm::load_scenario(file(2, "vehicle: {type: fixed_wing, min_speed_mps: 30}\n")), "vehicle",
      "min_speed must be below max_speed"));
  // A quad takes no fixed-wing keys.
  EXPECT_TRUE(has_error(swarm::load_scenario(file(2, "vehicle: {type: quad, min_speed_mps: 5}\n")),
                        "vehicle.min_speed_mps", "unknown key"));
}

TEST(ScenarioVehicle, ACanonicalTextRoundTripsAndNamesTheVersionItNeeds) {
  const swarm::Scenario quad = swarm::load_scenario(file(2)).scenario;
  const std::string quad_text = swarm::canonical_text(quad);
  EXPECT_EQ(quad_text.rfind("version: 1\n", 0), 0U);
  EXPECT_EQ(quad_text.find("vehicle:"), std::string::npos);

  const swarm::Scenario wing =
      swarm::load_scenario(file(2, "vehicle: {type: fixed_wing, max_bank_deg: 40}\n")).scenario;
  const std::string wing_text = swarm::canonical_text(wing);
  EXPECT_EQ(wing_text.rfind("version: 2\n", 0), 0U);
  const LoadResult again = swarm::load_scenario(wing_text);
  ASSERT_TRUE(again.ok());
  EXPECT_EQ(swarm::canonical_text(again.scenario), wing_text);
  EXPECT_EQ(again.scenario.vehicle.fixed_wing.max_bank, wing.vehicle.fixed_wing.max_bank);
}

TEST(ScenarioVehicle, TheWorldGivesFixedWingDronesAFlyingStart) {
  const swarm::Scenario s =
      swarm::load_scenario(file(2, "vehicle: {type: fixed_wing}\n")).scenario;
  const swarm::ScenarioWorld world = swarm::build_world(s);
  ASSERT_EQ(world.agents.size(), 3U);
  for (const swarm::DroneAgentConfig& a : world.agents) {
    EXPECT_EQ(a.vehicle_kind, VehicleKind::kFixedWing);
    const double speed = std::hypot(a.start_velocity.x, a.start_velocity.y);
    EXPECT_GE(speed, a.fixed_wing.min_speed);
    EXPECT_LE(speed, a.fixed_wing.max_speed);
  }
}

TEST(ScenarioVehicle, AFixedWingRunIsRepeatableAndDiffersFromAQuadRun) {
  const auto hash_of = [](const std::string& text, std::size_t threads) {
    const swarm::LoadResult r = swarm::load_scenario(text);
    EXPECT_TRUE(r.ok());
    std::vector<std::uint8_t> bytes;
    std::uint64_t hash = 0;
    EXPECT_EQ(swarm::run_scenario(r.scenario, threads, bytes, &hash), swarm::LogStatus::kOk);
    return hash;
  };
  const std::string wing = file(2, "vehicle: {type: fixed_wing}\n");
  EXPECT_EQ(hash_of(wing, 1), hash_of(wing, 1));
  EXPECT_EQ(hash_of(wing, 1), hash_of(wing, 3));
  EXPECT_NE(hash_of(wing, 1), hash_of(file(2), 1));
}
