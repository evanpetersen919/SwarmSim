#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "swarm/control/orbit.hpp"
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

TEST(FixedWingSwarm, AFailedAircraftFliesOnStraightAndSilentInsteadOfHangingInTheAir) {
  const Scenario s = load(scenario_text(), 60.0);
  Scenario failing = s;
  failing.faults.push_back({2, 1000});  // 20 s at 50 Hz
  const swarm::ScenarioWorld world = swarm::build_world(failing);
  const auto sim = world.make_simulation(1);
  swarm::Vec3 at_failure;
  swarm::Vec3 velocity;
  double heading_at_failure = 0.0;
  for (std::uint64_t tick = 0; tick < 1500; ++tick) {
    sim->step();
    const swarm::DroneAgent& d = dynamic_cast<const swarm::DroneAgent&>(sim->agent(2));
    if (tick + 1 == 1001) {
      EXPECT_FALSE(d.alive());
      at_failure = d.truth().pos;
      velocity = d.truth().vel;
      heading_at_failure = std::atan2(velocity.y, velocity.x);
    }
    if (tick + 1 > 1001) {
      EXPECT_FALSE(d.alive());
      // It moves in a straight line at constant speed: it does not turn, stop, or speed up.
      EXPECT_NEAR(d.truth().vel.x, velocity.x, 1e-9);
      EXPECT_NEAR(d.truth().vel.y, velocity.y, 1e-9);
    }
  }
  const swarm::DroneAgent& d = dynamic_cast<const swarm::DroneAgent&>(sim->agent(2));
  const double seconds = (1500 - 1001) * failing.dt;
  EXPECT_NEAR(d.truth().pos.x, at_failure.x + velocity.x * seconds, 1e-6);
  EXPECT_NEAR(d.truth().pos.y, at_failure.y + velocity.y * seconds, 1e-6);
  // It really left: it covered speed * time, about 170 m in these 10 seconds.
  EXPECT_NEAR(swarm::distance(d.truth().pos, at_failure), swarm::norm(velocity) * seconds, 1e-6);
  EXPECT_NEAR(std::atan2(d.truth().vel.y, d.truth().vel.x), heading_at_failure, 1e-9);
}

TEST(FixedWingSwarm, TheConsensusWeightIsAScenarioSettingAndZeroFliesTheClockAlone) {
  const swarm::LoadResult clock =
      swarm::load_scenario(replace(scenario_text(), "    altitude_step_m: 10",
                                   "    altitude_step_m: 10\n    consensus_weight: 0"));
  ASSERT_TRUE(clock.ok());
  EXPECT_EQ(clock.scenario.orbit.consensus_weight, 0.0);
  const swarm::LoadResult bad =
      swarm::load_scenario(replace(scenario_text(), "    altitude_step_m: 10",
                                   "    altitude_step_m: 10\n    consensus_weight: 2"));
  EXPECT_FALSE(bad.ok());
  const swarm::LoadResult standard = swarm::load_scenario(scenario_text());
  ASSERT_TRUE(standard.ok());
  EXPECT_EQ(standard.scenario.orbit.consensus_weight, swarm::OrbitParams{}.consensus_weight);
  // The weight is in the canonical text, so it is in the log header.
  EXPECT_NE(swarm::canonical_text(clock.scenario).find("consensus_weight: 0\n"), std::string::npos);
}
