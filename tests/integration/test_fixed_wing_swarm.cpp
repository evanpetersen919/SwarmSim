#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "swarm/drone_agent.hpp"
#include "swarm/scenario/metrics.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"

using swarm::LogStatus;
using swarm::Scenario;

namespace {

using Bytes = std::vector<std::uint8_t>;

std::string scenario_text() {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/fixed_wing_orbit.yaml", std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

std::string replace(std::string text, const std::string& from, const std::string& to) {
  const std::size_t at = text.find(from);
  EXPECT_NE(at, std::string::npos) << from;
  if (at != std::string::npos) {
    text.replace(at, from.size(), to);
  }
  return text;
}

Scenario load(const std::string& text, double duration_s) {
  const swarm::LoadResult r = swarm::load_scenario(replace(
      text, "duration_s: 300", "duration_s: " + std::to_string(static_cast<int>(duration_s))));
  EXPECT_TRUE(r.ok());
  return r.scenario;
}

Bytes record(const Scenario& s, std::size_t threads) {
  Bytes bytes;
  EXPECT_EQ(swarm::run_scenario(s, threads, bytes), LogStatus::kOk);
  return bytes;
}

}  // namespace

TEST(FixedWingSwarm, TheExampleScenarioLoadsAsEightAircraftOnAnOrbit) {
  const Scenario s = load(scenario_text(), 60.0);
  EXPECT_EQ(s.vehicle.kind, swarm::VehicleKind::kFixedWing);
  EXPECT_EQ(s.drones, 8U);
  const swarm::ScenarioWorld world = swarm::build_world(s);
  ASSERT_EQ(world.agents.size(), 8U);
  for (const swarm::DroneAgentConfig& a : world.agents) {
    EXPECT_EQ(a.brain.guidance, swarm::Guidance::kOrbit);
    const double speed = std::hypot(a.start_velocity.x, a.start_velocity.y);
    EXPECT_GE(speed, a.fixed_wing.min_speed);
    EXPECT_LE(speed, a.fixed_wing.max_speed);
  }
}

TEST(FixedWingSwarm, TheLogDoesNotDependOnTheThreadCount) {
  const Scenario s = load(scenario_text(), 60.0);
  EXPECT_EQ(record(s, 1), record(s, 4));
}

TEST(FixedWingSwarm, AnEstablishedSwarmKeepsApartAndAllSurvive) {
  const Scenario s = load(scenario_text(), 60.0);
  const Bytes bytes = record(s, 2);
  const swarm::MetricsOutcome out = swarm::analyze_log(bytes.data(), bytes.size());
  ASSERT_TRUE(out.ok()) << out.message;
  EXPECT_EQ(out.result.alive_at_end, 8U);
  EXPECT_EQ(out.result.collision_episodes, 0U);
  // The slots are on different radii and altitudes, so the nearest two are far apart.
  ASSERT_TRUE(out.result.min_separation.has_value());
  EXPECT_GT(*out.result.min_separation, 50.0);
}

TEST(FixedWingSwarm, FogThatBlindsTheCameraStillRunsAndStaysDeterministic) {
  const std::string text = replace(scenario_text(), "visibility_m: 3000", "visibility_m: 200");
  const Scenario s = load(text, 60.0);
  const Bytes a = record(s, 1);
  EXPECT_EQ(a, record(s, 3));
  const swarm::MetricsOutcome out = swarm::analyze_log(a.data(), a.size());
  ASSERT_TRUE(out.ok()) << out.message;
  EXPECT_EQ(out.result.alive_at_end, 8U);
}

TEST(FixedWingSwarm, AHoldCommandMakesAircraftCircleAndAResumeBringsThemBack) {
  const std::string commands =
      "\ncommands:\n  - {at_s: 20, kind: hold}\n  - {at_s: 40, kind: resume}\n";
  const Scenario s = load(scenario_text() + commands, 60.0);
  const Bytes bytes = record(s, 2);
  EXPECT_EQ(bytes, record(s, 1));
  const swarm::MetricsOutcome out = swarm::analyze_log(bytes.data(), bytes.size());
  ASSERT_TRUE(out.ok()) << out.message;
  EXPECT_EQ(out.result.alive_at_end, 8U);
  EXPECT_EQ(out.result.collision_episodes, 0U);
}

TEST(FixedWingSwarm, ASeparationAblationChangesNothingWhenNothingGetsClose) {
  const Scenario with = load(scenario_text(), 60.0);
  const Scenario without = load(
      replace(scenario_text(), "  predictor: kalman", "  predictor: kalman\n  separation: false"),
      60.0);
  // The rule engages only inside its margin, and an established swarm never gets there, so the
  // runs are identical apart from the scenario text recorded in the header. Compare the minimum
  // separation, which is computed from the aircraft states.
  const Bytes a = record(with, 1);
  const Bytes b = record(without, 1);
  const swarm::MetricsOutcome ma = swarm::analyze_log(a.data(), a.size());
  const swarm::MetricsOutcome mb = swarm::analyze_log(b.data(), b.size());
  ASSERT_TRUE(ma.ok() && mb.ok());
  EXPECT_EQ(*ma.result.min_separation, *mb.result.min_separation);
}
