#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "swarm/control/formation.hpp"
#include "swarm/control/mission.hpp"
#include "swarm/drone_agent.hpp"
#include "swarm/rng.hpp"
#include "swarm/scene.hpp"
#include "swarm/simulation.hpp"
#include "swarm/terrain.hpp"

using swarm::DroneAgent;
using swarm::FallbackState;
using swarm::Scene;
using swarm::Vec3;

namespace {

constexpr double kDt = 0.02;  // 50 Hz
constexpr double kSpacing = 20.0;
constexpr double kAltitude = 20.0;

swarm::SceneData make_scene() {
  swarm::TerrainConfig t;
  t.size_x = 1000.0;
  t.size_y = 1000.0;
  t.cell_size = 8.0;
  t.amplitude = 4.0;
  return swarm::generate_terrain(t, 5);
}

swarm::Mission mission() {
  swarm::Mission m;
  m.waypoints = {{-150, 0, kAltitude}, {150, 0, kAltitude}};
  m.speed = 4.0;
  return m;
}

// Starts on a 4-column grid, 35 m apart, then each moved by up to 8 m in x and y, so no two start
// closer than 19 m and the formation has to be assembled from a scattered start.
std::vector<Vec3> scattered_starts(std::uint32_t count, std::uint64_t seed) {
  swarm::Rng rng(seed, 0x700);
  std::vector<Vec3> starts;
  for (std::uint32_t i = 0; i < count; ++i) {
    starts.push_back({-170.0 + 35.0 * (i % 4) + rng.uniform(-8, 8),
                      -50.0 + 35.0 * (i / 4) + rng.uniform(-8, 8), kAltitude + rng.uniform(-3, 3)});
  }
  return starts;
}

struct Options {
  std::uint32_t drones = 8;
  std::uint64_t seed = 7;
  std::size_t threads = 2;
  std::uint64_t ticks = 3000;  // 60 s
  bool jam = false;
  std::uint64_t jam_start = 1500;
  std::uint64_t jam_end = 1650;  // 3 s
  swarm::LossModel loss = swarm::LossModel::kBurst;
};

struct Result {
  std::vector<std::uint64_t> hashes;
  double final_mean_error = 0.0;  // mean distance from each drone's slot on the reference, meters
  double final_max_error = 0.0;
  double min_separation = std::numeric_limits<double>::infinity();  // true, over the whole run
  std::set<FallbackState> visited;                                  // by any drone
  std::vector<FallbackState> final_states;
  std::uint64_t jam_drops = 0;
};

Result run(const Scene& scene, const Options& o) {
  swarm::SimConfig config;
  config.seed = o.seed;
  config.dt = kDt;
  config.channel.latency_ticks = 2;
  config.channel.jitter_ticks = 1;
  config.channel.loss_model = o.loss;
  config.channel.loss = 0.05;
  config.channel.mean_burst = 2.0;
  if (o.jam) {
    swarm::Jammer j;
    j.radius = std::numeric_limits<double>::infinity();
    j.start_tick = o.jam_start;
    j.end_tick = o.jam_end;
    j.affects = swarm::JamAffects::kEither;
    config.channel.jammers = {j};
  }

  const auto offsets = swarm::make_formation(swarm::FormationShape::kWedge, o.drones, kSpacing);
  const auto starts = scattered_starts(o.drones, o.seed);

  std::vector<std::unique_ptr<swarm::IAgent>> agents;
  for (std::uint32_t i = 0; i < o.drones; ++i) {
    swarm::DroneAgentConfig c;
    c.brain.id = i;
    c.brain.num_drones = o.drones;
    c.brain.dt = kDt;
    c.brain.offsets = offsets;
    c.brain.mission = mission();
    c.brain.pose.dt = kDt;
    c.brain.predictor_config.dt = kDt;
    c.brain.report_period_ticks = 5;
    c.brain.staleness_ticks = 50;
    c.start = starts[i];
    c.vehicle.max_accel = c.brain.separation.max_accel;
    c.lidar.beams = 8;
    c.lidar.azimuth_samples = 36;
    c.seed = o.seed;
    agents.push_back(std::make_unique<DroneAgent>(c, scene));
  }
  swarm::Simulation sim(config, std::move(agents), o.threads);

  const auto drone = [&](std::size_t i) -> const DroneAgent& {
    return dynamic_cast<const DroneAgent&>(sim.agent(i));
  };

  Result out;
  const swarm::MissionPath path(mission());
  for (std::uint64_t tick = 0; tick < o.ticks; ++tick) {
    sim.step();
    out.hashes.push_back(sim.state_hash());
    for (std::size_t i = 0; i < o.drones; ++i) {
      out.visited.insert(drone(i).brain().state());
      for (std::size_t j = i + 1; j < o.drones; ++j) {
        out.min_separation = std::min(out.min_separation,
                                      swarm::distance(drone(i).truth().pos, drone(j).truth().pos));
      }
    }
  }

  const Vec3 center = path.at(static_cast<double>(o.ticks) * kDt).position;
  double sum = 0.0;
  for (std::size_t i = 0; i < o.drones; ++i) {
    const double e = swarm::distance(drone(i).truth().pos, center + offsets[i]);
    sum += e;
    out.final_max_error = std::max(out.final_max_error, e);
    out.final_states.push_back(drone(i).brain().state());
  }
  out.final_mean_error = sum / o.drones;
  out.jam_drops = sim.channel().stats().dropped_jam;
  return out;
}

}  // namespace

TEST(FormationSwarm, TwoDronesWithNoLossReachTheFormationAndStayThere) {
  const Scene scene(make_scene());
  Options o;
  o.drones = 2;
  o.loss = swarm::LossModel::kNone;
  const Result r = run(scene, o);
  RecordProperty("two_drones_max_error_m", std::to_string(r.final_max_error));
  EXPECT_LT(r.final_max_error, 0.5);
  EXPECT_GT(r.min_separation, 10.0);
  EXPECT_EQ(r.visited, std::set<FallbackState>{FallbackState::kNominal});
}

TEST(FormationSwarm, EightScatteredDronesConvergeToTheFormationAndStayApart) {
  const Scene scene(make_scene());
  const Result r = run(scene, Options{});
  RecordProperty("eight_drones_mean_error_m", std::to_string(r.final_mean_error));
  RecordProperty("eight_drones_max_error_m", std::to_string(r.final_max_error));
  RecordProperty("eight_drones_min_separation_m", std::to_string(r.min_separation));
  EXPECT_LT(r.final_mean_error, 0.4);
  EXPECT_LT(r.final_max_error, 0.6);
  EXPECT_GT(r.min_separation, 1.0);
}

TEST(FormationSwarm, TenSeedsAllConvergeAndKeepTheirSeparation) {
  // The separation rule is a margin, not a guarantee, so the true minimum over the run is measured
  // for each seed. The closest approaches happen while the formation is assembled.
  const Scene scene(make_scene());
  double worst_separation = std::numeric_limits<double>::infinity();
  double worst_error = 0.0;
  for (std::uint64_t seed = 0; seed < 10; ++seed) {
    Options o;
    o.seed = seed;
    const Result r = run(scene, o);
    worst_separation = std::min(worst_separation, r.min_separation);
    worst_error = std::max(worst_error, r.final_max_error);
  }
  RecordProperty("worst_min_separation_m", std::to_string(worst_separation));
  RecordProperty("worst_final_error_m", std::to_string(worst_error));
  EXPECT_GT(worst_separation, 1.0);
  EXPECT_LT(worst_error, 0.8);
}

TEST(FormationSwarm,
     AJamWhileTheFormationIsAssemblingDrivesDegradedCommsAndTheFormationStillForms) {
  const Scene scene(make_scene());
  double worst_separation = std::numeric_limits<double>::infinity();
  double worst_error = 0.0;
  for (std::uint64_t seed = 0; seed < 10; ++seed) {
    Options o;
    o.seed = seed;
    o.jam = true;
    o.jam_start = 100;
    o.jam_end = 400;  // 6 s, starting 2 s in
    const Result r = run(scene, o);
    EXPECT_GT(r.jam_drops, 0U) << "seed " << seed;
    EXPECT_TRUE(r.visited.count(FallbackState::kDegradedComms) > 0) << "seed " << seed;
    for (FallbackState s : r.final_states) {
      EXPECT_EQ(s, FallbackState::kNominal) << "seed " << seed;
    }
    worst_separation = std::min(worst_separation, r.min_separation);
    worst_error = std::max(worst_error, r.final_max_error);
  }
  RecordProperty("jam_worst_min_separation_m", std::to_string(worst_separation));
  RecordProperty("jam_worst_final_error_m", std::to_string(worst_error));
  EXPECT_GT(worst_separation, 1.0);
  EXPECT_LT(worst_error, 0.8);
}

TEST(FormationSwarm, HashesAreIdenticalForEveryThreadCount) {
  const Scene scene(make_scene());
  Options o;
  o.jam = true;
  o.jam_start = 100;
  o.jam_end = 400;
  o.ticks = 2000;
  o.threads = 1;
  const Result reference = run(scene, o);
  for (std::size_t threads : {2U, 4U, 8U}) {
    o.threads = threads;
    EXPECT_EQ(run(scene, o).hashes, reference.hashes) << "threads " << threads;
  }
}

TEST(FormationSwarm, SameSeedReproducesAndADifferentSeedDiffers) {
  const Scene scene(make_scene());
  Options o;
  o.ticks = 500;
  const auto base = run(scene, o).hashes.back();
  EXPECT_EQ(run(scene, o).hashes.back(), base);
  o.seed = 8;
  EXPECT_NE(run(scene, o).hashes.back(), base);
}
