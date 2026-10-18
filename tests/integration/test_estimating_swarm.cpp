#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "estimating_drone.hpp"
#include "swarm/obstacles.hpp"
#include "swarm/scene.hpp"
#include "swarm/simulation.hpp"
#include "swarm/terrain.hpp"

using swarm::Scene;
using swarm::SceneData;
using swarm::Vec3;
using swarm::Weather;
using swarm::testing::EstimatingDrone;

namespace {

constexpr std::uint32_t kDrones = 8;
constexpr double kDt = 0.02;            // 50 Hz
constexpr std::uint64_t kTicks = 3000;  // 60 s
// Every drone's messages are lost during this window, so every prediction goes stale.
constexpr std::uint64_t kJamStart = 1000;
constexpr std::uint64_t kJamEnd = 1150;  // 3 s

SceneData make_scene() {
  swarm::TerrainConfig t;
  t.size_x = 800.0;
  t.size_y = 800.0;
  t.cell_size = 8.0;
  t.type = swarm::TerrainType::kField;
  SceneData data = swarm::generate_terrain(t, 3);
  swarm::ObstacleConfig o;
  o.density_per_m2 = 0.0002;
  o.spawn_clear_radius = 0.0;
  swarm::place_obstacles(data, o, 3);
  return data;
}

swarm::SimConfig make_config(std::uint64_t seed, bool jam) {
  swarm::SimConfig c;
  c.seed = seed;
  c.dt = kDt;
  c.channel.latency_ticks = 2;
  c.channel.jitter_ticks = 1;
  c.channel.loss_model = swarm::LossModel::kBurst;
  c.channel.loss = 0.05;
  c.channel.mean_burst = 2.0;
  if (jam) {
    swarm::Jammer j;
    j.center = {0, 0, 0};
    j.radius = std::numeric_limits<double>::infinity();
    j.start_tick = kJamStart;
    j.end_tick = kJamEnd;
    j.affects = swarm::JamAffects::kEither;
    c.channel.jammers = {j};
  }
  return c;
}

struct Result {
  std::vector<std::uint64_t> hashes;
  double rms_error_second_half = 0.0;      // estimated vs true position, over all drones
  double mean_error[3] = {0.0, 0.0, 0.0};  // hold-last, dead reckoning, Kalman, during the jam
  std::size_t prediction_samples = 0;
  std::uint64_t fixes_applied = 0;
  std::uint64_t fixes_unavailable = 0;
  std::uint64_t reports_accepted = 0;
  std::uint64_t reports_malformed = 0;
  std::uint64_t jam_drops = 0;
};

Result run(const Scene& scene, const Weather& weather, std::uint64_t seed, std::size_t threads,
           bool jam) {
  const swarm::SimConfig config = make_config(seed, jam);
  std::vector<std::unique_ptr<swarm::IAgent>> agents;
  for (std::uint32_t i = 0; i < kDrones; ++i) {
    agents.push_back(std::make_unique<EstimatingDrone>(i, kDrones, seed, kDt, scene, weather));
  }
  swarm::Simulation sim(config, std::move(agents), threads);

  const auto drone = [&](std::size_t i) -> const EstimatingDrone& {
    return dynamic_cast<const EstimatingDrone&>(sim.agent(i));
  };

  Result out;
  double squared = 0.0;
  std::size_t count = 0;
  for (std::uint64_t tick = 0; tick < kTicks; ++tick) {
    sim.step();
    out.hashes.push_back(sim.state_hash());

    if (tick >= kTicks / 2) {
      for (std::size_t i = 0; i < kDrones; ++i) {
        const Vec3 err = drone(i).estimate().position - drone(i).truth().pos;
        squared += swarm::norm_squared(err);
        ++count;
      }
    }

    // During the last second of the jam, compare what each drone believes about each neighbor
    // with where that neighbor really is.
    if (jam && tick >= kJamEnd - 50 && tick < kJamEnd) {
      for (std::size_t i = 0; i < kDrones; ++i) {
        for (std::size_t j = 0; j < kDrones; ++j) {
          if (i == j) {
            continue;
          }
          bool all_valid = true;
          double e[3];
          for (std::size_t kind = 0; kind < 3; ++kind) {
            const auto est = drone(i).predicted(static_cast<std::uint32_t>(j), kind, tick);
            all_valid = all_valid && est.valid;
            e[kind] = swarm::norm(est.position - drone(j).truth().pos);
          }
          if (all_valid) {
            for (std::size_t kind = 0; kind < 3; ++kind) {
              out.mean_error[kind] += e[kind];
            }
            ++out.prediction_samples;
          }
        }
      }
    }
  }
  out.rms_error_second_half = std::sqrt(squared / static_cast<double>(count));
  if (out.prediction_samples > 0) {
    for (double& m : out.mean_error) {
      m /= static_cast<double>(out.prediction_samples);
    }
  }
  for (std::size_t i = 0; i < kDrones; ++i) {
    out.fixes_applied += drone(i).pose_counters().fixes_applied;
    out.fixes_unavailable += drone(i).pose_counters().fixes_unavailable;
    out.reports_accepted += drone(i).table_counters().accepted;
    out.reports_malformed += drone(i).table_counters().malformed;
  }
  out.jam_drops = sim.channel().stats().dropped_jam;
  return out;
}

}  // namespace

TEST(EstimatingSwarm, HashesAreIdenticalForEveryThreadCount) {
  const Scene scene(make_scene());
  const Result reference = run(scene, Weather::clear(), 7, 1, true);
  for (std::size_t threads : {2U, 4U, 8U}) {
    EXPECT_EQ(run(scene, Weather::clear(), 7, threads, true).hashes, reference.hashes)
        << "threads " << threads;
  }
}

TEST(EstimatingSwarm, TheWholeStackActuallyRan) {
  const Scene scene(make_scene());
  const Result r = run(scene, Weather::clear(), 7, 2, true);
  EXPECT_GT(r.fixes_applied, 0U);
  EXPECT_EQ(r.fixes_unavailable, 0U);  // clear air at 15 m altitude always gives a fix
  EXPECT_GT(r.reports_accepted, 0U);
  EXPECT_EQ(r.reports_malformed, 0U);
  EXPECT_GT(r.jam_drops, 0U);
  EXPECT_GT(r.prediction_samples, 0U);
}

TEST(EstimatingSwarm, PoseErrorStaysSmallWithFixesAndGrowsWithoutThem) {
  const Scene scene(make_scene());
  const Result clear = run(scene, Weather::clear(), 7, 2, false);
  const Result fog = run(scene, Weather::fog(40.0), 7, 2, false);  // too dense to see the ground

  RecordProperty("clear_rms_position_error_m", std::to_string(clear.rms_error_second_half));
  RecordProperty("fog_rms_position_error_m", std::to_string(fog.rms_error_second_half));
  RecordProperty("clear_fixes_applied", std::to_string(clear.fixes_applied));
  RecordProperty("fog_fixes_applied", std::to_string(fog.fixes_applied));
  RecordProperty("fog_fixes_unavailable", std::to_string(fog.fixes_unavailable));
  EXPECT_LT(clear.rms_error_second_half, 2.0);
  EXPECT_GT(fog.fixes_unavailable, 0U);
  // Almost no fix survives: a drone near a tall obstacle can still get returns inside the fog's
  // short detection range, so the count is small but not zero.
  EXPECT_LT(fog.fixes_applied, clear.fixes_applied / 10);
  EXPECT_GT(fog.rms_error_second_half, 5.0 * clear.rms_error_second_half);
  EXPECT_GT(fog.rms_error_second_half, 5.0);
}

TEST(EstimatingSwarm, DuringAJamDeadReckoningAndKalmanBothBeatHoldLastByAWideMargin) {
  // The neighbors here maneuver (a new random acceleration every second), so this is not the
  // straight-flight case. At the default process noise the Kalman predictor is about as good as
  // dead reckoning and not clearly better; the recorded numbers show it. Only the large gap to
  // hold-last is asserted, and no ordering between Kalman and dead reckoning is claimed.
  const Scene scene(make_scene());
  const Result r = run(scene, Weather::clear(), 7, 2, true);
  ASSERT_GT(r.prediction_samples, 100U);

  RecordProperty("jam_hold_last_mean_error_m", std::to_string(r.mean_error[0]));
  RecordProperty("jam_dead_reckoning_mean_error_m", std::to_string(r.mean_error[1]));
  RecordProperty("jam_kalman_mean_error_m", std::to_string(r.mean_error[2]));
  EXPECT_LT(r.mean_error[1], 0.25 * r.mean_error[0]);
  EXPECT_LT(r.mean_error[2], 0.25 * r.mean_error[0]);
}

TEST(EstimatingSwarm, SameSeedReproducesAndADifferentSeedDiffers) {
  const Scene scene(make_scene());
  const auto base = run(scene, Weather::clear(), 7, 2, true).hashes.back();
  EXPECT_EQ(run(scene, Weather::clear(), 7, 2, true).hashes.back(), base);
  EXPECT_NE(run(scene, Weather::clear(), 8, 2, true).hashes.back(), base);
}
