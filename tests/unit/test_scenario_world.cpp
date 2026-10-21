#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarm/format/log.hpp"
#include "swarm/format/scene.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"

using swarm::Scenario;
using swarm::ScenarioWorld;
using swarm::Vec3;

namespace {

Scenario load(const std::string& name) {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/" + name, std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok()) << name;
  return r.scenario;
}

Scenario with_obstacles() {
  Scenario s = load("formation_jam.yaml");
  s.terrain.type = swarm::TerrainType::kHills;
  s.terrain.amplitude = 10.0;
  s.obstacles.density_per_m2 = 0.0004;
  s.obstacles.spawn_clear_radius = 60.0;
  return s;
}

std::uint64_t scene_hash_of(const ScenarioWorld& w) { return swarm::scene_hash(w.scene->data()); }

}  // namespace

TEST(ScenarioWorld, TheBuiltWorldMatchesTheScenario) {
  const Scenario s = load("formation_jam.yaml");
  const ScenarioWorld w = swarm::build_world(s);
  ASSERT_EQ(w.agents.size(), 8U);
  EXPECT_EQ(w.offsets, swarm::make_formation(swarm::FormationShape::kWedge, 8, 20.0));
  EXPECT_EQ(w.sim.seed, 7U);
  EXPECT_DOUBLE_EQ(w.sim.dt, 0.02);
  EXPECT_EQ(w.sim.channel.latency_ticks, 2U);
  EXPECT_EQ(w.sim.channel.jitter_ticks, 1U);
  EXPECT_EQ(w.sim.channel.loss_model, swarm::LossModel::kBurst);
  ASSERT_EQ(w.sim.channel.jammers.size(), 1U);
  EXPECT_EQ(w.sim.channel.jammers[0].start_tick, 100U);

  for (std::uint32_t i = 0; i < 8; ++i) {
    const swarm::DroneAgentConfig& a = w.agents[i];
    EXPECT_EQ(a.brain.id, i);
    EXPECT_EQ(a.brain.num_drones, 8U);
    EXPECT_EQ(a.brain.offsets, w.offsets);
    EXPECT_DOUBLE_EQ(a.brain.dt, 0.02);
    EXPECT_DOUBLE_EQ(a.brain.pose.dt, 0.02);
    EXPECT_DOUBLE_EQ(a.brain.predictor_config.dt, 0.02);
    EXPECT_EQ(a.brain.predictor, swarm::PredictorKind::kKalman);
    EXPECT_EQ(a.brain.report_period_ticks, 5U);
    EXPECT_EQ(a.brain.staleness_ticks, 50U);
    ASSERT_EQ(a.brain.mission.waypoints.size(), 2U);
    EXPECT_DOUBLE_EQ(a.brain.mission.speed, 4.0);
    EXPECT_EQ(a.lidar.beams, 8U);
    EXPECT_EQ(a.lidar.azimuth_samples, 36U);
    EXPECT_EQ(a.fix_period_ticks, 5U);
    EXPECT_EQ(a.seed, 7U);
    EXPECT_DOUBLE_EQ(a.vehicle.max_accel, a.brain.separation.max_accel);
  }
}

TEST(ScenarioWorld, ThePredictorChoiceReachesTheBrain) {
  Scenario s = load("formation_jam.yaml");
  s.predictor = swarm::PredictorChoice::kDeadReckoning;
  EXPECT_EQ(swarm::build_world(s).agents[0].brain.predictor, swarm::PredictorKind::kDeadReckoning);
  s.predictor = swarm::PredictorChoice::kHoldLast;
  EXPECT_EQ(swarm::build_world(s).agents[0].brain.predictor, swarm::PredictorKind::kHoldLast);
}

TEST(ScenarioWorld, TheSameScenarioBuildsTheSameWorldAndADifferentSeedDoesNot) {
  const Scenario s = with_obstacles();
  const ScenarioWorld a = swarm::build_world(s);
  const ScenarioWorld b = swarm::build_world(s);
  EXPECT_EQ(scene_hash_of(a), scene_hash_of(b));
  for (std::size_t i = 0; i < a.agents.size(); ++i) {
    EXPECT_EQ(a.agents[i].start, b.agents[i].start);
  }

  Scenario other = s;
  other.seed = s.seed + 1;
  const ScenarioWorld c = swarm::build_world(other);
  EXPECT_NE(scene_hash_of(a), scene_hash_of(c));
  EXPECT_NE(a.agents[0].start, c.agents[0].start);
}

TEST(ScenarioWorld, StartsStayInsideTheLayoutAndApartOverManySeeds) {
  Scenario s = load("formation_jam.yaml");  // 8 drones, 4 columns, spacing 35, jitter 8
  const Vec3 center = s.mission.waypoints.front();
  const double bound = s.start.spacing_m - 2.0 * s.start.jitter_m;  // adjacent cells, worst case
  ASSERT_GT(bound, 0.0);
  for (std::uint64_t seed = 0; seed < 100; ++seed) {
    s.seed = seed;
    const std::vector<Vec3> starts = swarm::start_positions(s);
    ASSERT_EQ(starts.size(), 8U);
    for (std::uint32_t i = 0; i < 8; ++i) {
      const double col = static_cast<double>(i % 4) - 1.5;
      const double row = static_cast<double>(i / 4) - 0.5;
      EXPECT_LE(std::abs(starts[i].x - (center.x + col * 35.0)), 8.0);
      EXPECT_LE(std::abs(starts[i].y - (center.y + row * 35.0)), 8.0);
      EXPECT_LE(std::abs(starts[i].z - s.start.altitude_m), s.start.altitude_jitter_m);
      for (std::uint32_t j = i + 1; j < 8; ++j) {
        ASSERT_GE(swarm::distance(starts[i], starts[j]), bound) << "seed " << seed;
      }
    }
  }
}

TEST(ScenarioWorld, ZeroJitterGivesTheExactGridAndTheDrawsDoNotShiftWithTheJitterAmount) {
  Scenario s = load("formation_jam.yaml");
  s.start.jitter_m = 0.0;
  s.start.altitude_jitter_m = 0.0;
  const std::vector<Vec3> grid = swarm::start_positions(s);
  EXPECT_DOUBLE_EQ(grid[0].x, s.mission.waypoints.front().x - 1.5 * 35.0);
  EXPECT_DOUBLE_EQ(grid[5].y, s.mission.waypoints.front().y + 0.5 * 35.0);
  EXPECT_DOUBLE_EQ(grid[3].z, s.start.altitude_m);

  // The jitter of drone i, as a fraction of the jitter amount, is the same whatever the amount is.
  s.start.jitter_m = 2.0;
  const std::vector<Vec3> small = swarm::start_positions(s);
  s.start.jitter_m = 8.0;
  const std::vector<Vec3> large = swarm::start_positions(s);
  for (std::size_t i = 0; i < grid.size(); ++i) {
    EXPECT_NEAR((small[i].x - grid[i].x) / 2.0, (large[i].x - grid[i].x) / 8.0, 1e-12);
    EXPECT_NEAR((small[i].y - grid[i].y) / 2.0, (large[i].y - grid[i].y) / 8.0, 1e-12);
  }
}

TEST(ScenarioWorld, ChangingTheDroneCountDoesNotChangeTheTerrainOrTheObstacles) {
  Scenario s = with_obstacles();
  const std::uint64_t base = scene_hash_of(swarm::build_world(s));
  EXPECT_GT(swarm::build_world(s).scene->data().obstacles.size(), 0U);
  s.drones = 12;
  EXPECT_EQ(scene_hash_of(swarm::build_world(s)), base);
  s.drones = 3;
  EXPECT_EQ(scene_hash_of(swarm::build_world(s)), base);
}

TEST(ScenarioWorld, NoObstacleIsPlacedNearTheStartArea) {
  const Scenario s = with_obstacles();
  const ScenarioWorld w = swarm::build_world(s);
  const Vec3 start = s.mission.waypoints.front();
  for (const swarm::Obstacle& o : w.scene->data().obstacles) {
    EXPECT_GE(std::hypot(o.cx - start.x, o.cy - start.y), s.obstacles.spawn_clear_radius);
  }
}

TEST(ScenarioWorld, FormationsTighterThanTheSeparationMarginAreRejected) {
  Scenario s = load("formation_jam.yaml");
  s.formation_spacing_m = 10.0;
  try {
    swarm::build_world(s);
    FAIL() << "expected an exception";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("spacing_m"), std::string::npos);
  }
  s.formation_spacing_m = 12.0;  // equal to the margin is not enough
  EXPECT_THROW(swarm::build_world(s), std::invalid_argument);
  s.formation_spacing_m = 12.5;
  EXPECT_NO_THROW(swarm::build_world(s));
}

TEST(ScenarioWorld, TheOraclePredictorSetsTheOracleFlagOnEveryDrone) {
  Scenario s = load("formation_jam.yaml");
  for (const auto& a : swarm::build_world(s).agents) {
    EXPECT_FALSE(a.oracle);
  }
  s.predictor = swarm::PredictorChoice::kOracle;
  for (const auto& a : swarm::build_world(s).agents) {
    EXPECT_TRUE(a.oracle);
  }
}

TEST(ScenarioWorld, ASingleDroneBuilds) {
  Scenario s = load("minimal.yaml");
  s.drones = 1;
  const ScenarioWorld w = swarm::build_world(s);
  EXPECT_EQ(w.agents.size(), 1U);
  EXPECT_NE(w.make_simulation(1), nullptr);
}

TEST(ScenarioWorld, ABuiltSimulationStepsAndItsHashesAreIdenticalForEveryThreadCount) {
  Scenario s = load("formation_jam.yaml");
  const ScenarioWorld w = swarm::build_world(s);

  const auto run = [&](std::size_t threads) {
    const auto sim = w.make_simulation(threads);
    std::vector<std::uint64_t> hashes;
    for (int i = 0; i < 300; ++i) {
      sim->step();
      hashes.push_back(sim->state_hash());
    }
    return hashes;
  };
  const std::vector<std::uint64_t> reference = run(1);
  EXPECT_NE(reference.front(), reference.back());
  for (std::size_t threads : {2U, 4U, 8U}) {
    EXPECT_EQ(run(threads), reference) << "threads " << threads;
  }
  EXPECT_EQ(run(1), reference);  // a second simulation from the same world starts from scratch
}

TEST(ScenarioWorld, WithoutCommandsTheBuiltSimulationIsExactlyTheDronesAndNothingElse) {
  // No hash is pinned here: a full drone run uses sin, cos, and Eigen, whose results are not
  // promised to match across platforms. Instead the built simulation is compared with one put
  // together by hand from the drone agents alone, which must be identical on every platform.
  const Scenario s = load("formation_jam.yaml");
  const ScenarioWorld w = swarm::build_world(s);
  EXPECT_FALSE(w.station.has_value());

  const auto built = w.make_simulation(2);
  EXPECT_EQ(built->agent_count(), 8U);  // the drones and nothing else

  std::vector<std::unique_ptr<swarm::IAgent>> agents;
  for (const swarm::DroneAgentConfig& config : w.agents) {
    agents.push_back(std::make_unique<swarm::DroneAgent>(config, *w.scene));
  }
  swarm::Simulation by_hand(w.sim, std::move(agents), 2);
  for (int i = 0; i < 300; ++i) {
    built->step();
    by_hand.step();
    ASSERT_EQ(built->state_hash(), by_hand.state_hash()) << "tick " << i;
  }
}

TEST(ScenarioWorld, CommandsAddAGroundStationAsTheLastEndpoint) {
  Scenario s = load("formation_jam.yaml");
  s.commands = {{500, swarm::CommandKind::kHold}, {900, swarm::CommandKind::kResume}};
  const ScenarioWorld w = swarm::build_world(s);
  ASSERT_TRUE(w.station.has_value());
  EXPECT_EQ(w.station->drones, 8U);
  ASSERT_EQ(w.station->schedule.size(), 2U);
  EXPECT_EQ(w.station->schedule[0].tick, 500U);
  EXPECT_EQ(w.station->schedule[1].kind, swarm::CommandKind::kResume);

  const auto sim = w.make_simulation(1);
  ASSERT_EQ(sim->agent_count(), 9U);  // eight drones, then the station
  EXPECT_NE(dynamic_cast<const swarm::GroundStationAgent*>(&sim->agent(8)), nullptr);
  for (std::size_t i = 0; i < 8; ++i) {
    EXPECT_NE(dynamic_cast<const swarm::DroneAgent*>(&sim->agent(i)), nullptr);
  }
  // The drones still address only each other: the brain's neighbors are the drones.
  EXPECT_EQ(w.agents[0].brain.num_drones, 8U);
}

TEST(ScenarioWorld, ASwarmThatIsToldToHoldIsCommandedThroughTheBuiltSimulation) {
  Scenario s = load("formation_jam.yaml");
  s.channel.jammers.clear();
  s.commands = {{100, swarm::CommandKind::kHold}};
  const ScenarioWorld w = swarm::build_world(s);
  const auto sim = w.make_simulation(2);
  for (int i = 0; i < 160; ++i) {
    sim->step();
  }
  for (std::size_t i = 0; i < 8; ++i) {
    const auto& d = dynamic_cast<const swarm::DroneAgent&>(sim->agent(i));
    EXPECT_TRUE(d.brain().holding_commanded()) << "drone " << i;
  }
}

// ---- scenes from files ----------------------------------------------------------------------

namespace {

std::string scene_path(const std::string& name) {
  return (std::filesystem::temp_directory_path() / ("swarmsim_test_" + name + ".swscene")).string();
}

void write_scene_file(const std::string& path, const swarm::SceneData& scene) {
  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::serialize_scene(scene, bytes), swarm::SceneStatus::kOk);
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out.write(reinterpret_cast<const char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
}

// A short scenario on the generated scene, and the same scenario reading that scene from a file.
struct FileScenes {
  Scenario generated;
  Scenario from_file;
  std::string path;
  std::uint64_t hash = 0;
};

FileScenes make_file_scenes(const std::string& name) {
  FileScenes f;
  f.generated = with_obstacles();
  f.generated.duration_ticks = 100;
  const ScenarioWorld w = swarm::build_world(f.generated);
  f.hash = swarm::scene_hash(w.scene->data());
  f.path = scene_path(name);
  write_scene_file(f.path, w.scene->data());
  f.from_file = f.generated;
  f.from_file.scene_file.used = true;
  f.from_file.scene_file.path = f.path;
  return f;
}

}  // namespace

TEST(SceneFromFile, TheWorldHasTheFilesSceneAndRecordsItsHash) {
  const FileScenes f = make_file_scenes("world");
  const ScenarioWorld w = swarm::build_world(f.from_file);
  EXPECT_EQ(scene_hash_of(w), f.hash);
  EXPECT_FALSE(f.from_file.scene_file.hash.has_value());
  ASSERT_TRUE(w.scenario.scene_file.hash.has_value());
  EXPECT_EQ(*w.scenario.scene_file.hash, f.hash);
}

TEST(SceneFromFile, AMatchingHashIsAccepted) {
  FileScenes f = make_file_scenes("match");
  f.from_file.scene_file.hash = f.hash;
  EXPECT_NO_THROW(swarm::build_world(f.from_file));
}

TEST(SceneFromFile, AWrongHashAMissingFileAndAnInvalidFileAreRefused) {
  FileScenes f = make_file_scenes("refused");
  Scenario wrong = f.from_file;
  wrong.scene_file.hash = f.hash ^ 1U;
  EXPECT_THROW(swarm::build_world(wrong), std::invalid_argument);

  Scenario missing = f.from_file;
  missing.scene_file.path = scene_path("does_not_exist");
  EXPECT_THROW(swarm::build_world(missing), std::invalid_argument);

  const std::string junk = scene_path("junk");
  std::ofstream(junk, std::ios::binary) << "not a scene";
  Scenario invalid = f.from_file;
  invalid.scene_file.path = junk;
  EXPECT_THROW(swarm::build_world(invalid), std::invalid_argument);
}

TEST(SceneFromFile, ARunOnTheFileIsTheRunOnTheGeneratedScene) {
  const FileScenes f = make_file_scenes("same_run");
  std::vector<std::uint8_t> generated_log;
  std::vector<std::uint8_t> file_log;
  std::uint64_t generated_hash = 0;
  std::uint64_t file_hash = 0;
  ASSERT_EQ(swarm::run_scenario(f.generated, 1, generated_log, &generated_hash),
            swarm::LogStatus::kOk);
  ASSERT_EQ(swarm::run_scenario(f.from_file, 1, file_log, &file_hash), swarm::LogStatus::kOk);
  EXPECT_EQ(file_hash, generated_hash);
}

TEST(SceneFromFile, TheLogHeaderHoldsTheHashAndTheLogVerifies) {
  FileScenes f = make_file_scenes("verify");
  std::vector<std::uint8_t> log;
  ASSERT_EQ(swarm::run_scenario(f.from_file, 1, log), swarm::LogStatus::kOk);

  swarm::LogReader reader(log.data(), log.size());
  ASSERT_EQ(reader.read_header(), swarm::LogStatus::kOk);
  char hex[20];
  std::snprintf(hex, sizeof(hex), "%016llx", static_cast<unsigned long long>(f.hash));
  EXPECT_NE(reader.header().config.find(std::string("hash: ") + hex), std::string::npos);

  EXPECT_EQ(swarm::verify_run_log(log.data(), log.size(), 1).status, swarm::VerifyStatus::kOk);

  // A different scene in the same file: the log no longer belongs to it.
  swarm::SceneData other = swarm::build_world(f.generated).scene->data();
  other.terrain.heights[0] += 1.0;
  write_scene_file(f.path, other);
  const swarm::VerifyResult changed = swarm::verify_run_log(log.data(), log.size(), 1);
  EXPECT_EQ(changed.status, swarm::VerifyStatus::kBadConfig);
  EXPECT_NE(changed.message.find("hash"), std::string::npos);
}

TEST(SceneFromFile, AGeneratedScenarioKeepsItsCanonicalText) {
  // Nothing about the scene source may change the text of a scenario that generates its scene,
  // because logs and golden files hold that text.
  const Scenario s = load("formation_jam.yaml");
  EXPECT_EQ(swarm::canonical_text(s).find("source:"), std::string::npos);
}
