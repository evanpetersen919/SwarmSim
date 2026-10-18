#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "sensed_drone.hpp"
#include "swarm/lockstep_pool.hpp"
#include "swarm/obstacles.hpp"
#include "swarm/rng.hpp"
#include "swarm/scene.hpp"
#include "swarm/simulation.hpp"
#include "swarm/terrain.hpp"

using swarm::Scene;
using swarm::SceneData;
using swarm::Vec3;
using swarm::Weather;
using swarm::testing::SensedDrone;

namespace {

constexpr std::uint32_t kDrones = 16;
constexpr int kTicks = 200;

SceneData make_scene(std::uint64_t seed) {
  swarm::TerrainConfig t;
  t.size_x = 400.0;
  t.size_y = 400.0;
  t.cell_size = 4.0;
  t.type = swarm::TerrainType::kHills;
  t.amplitude = 20.0;
  t.feature_scale = 90.0;
  SceneData data = swarm::generate_terrain(t, seed);
  swarm::ObstacleConfig o;
  o.density_per_m2 = 0.001;
  o.spawn_clear_radius = 0.0;
  swarm::place_obstacles(data, o, seed);
  return data;
}

swarm::SimConfig make_config(std::uint64_t seed) {
  swarm::SimConfig c;
  c.seed = seed;
  c.dt = 0.01;
  c.channel.latency_ticks = 2;
  c.channel.jitter_ticks = 1;
  c.channel.loss_model = swarm::LossModel::kBurst;
  c.channel.loss = 0.1;
  c.channel.mean_burst = 3.0;
  swarm::Jammer j;
  j.center = {0, 0, 0};
  j.radius = 40.0;
  j.mode = swarm::JamMode::kPulsed;
  j.pulse_period_ticks = 30;
  j.pulse_duty = 0.4;
  c.channel.jammers = {j};
  return c;
}

struct RunResult {
  std::vector<std::uint64_t> hashes;
  std::uint64_t valid_returns = 0;
  std::uint64_t total_rays = 0;
  std::uint64_t received = 0;
  std::uint64_t dropped_jam = 0;
};

RunResult run(const Scene& scene, const Weather& weather, std::uint64_t seed, std::size_t threads) {
  const swarm::SimConfig config = make_config(seed);
  std::vector<std::unique_ptr<swarm::IAgent>> agents;
  for (std::uint32_t i = 0; i < kDrones; ++i) {
    agents.push_back(std::make_unique<SensedDrone>(i, kDrones, seed, config.dt, scene, weather));
  }
  swarm::Simulation sim(config, std::move(agents), threads);

  RunResult out;
  for (int t = 0; t < kTicks; ++t) {
    sim.step();
    out.hashes.push_back(sim.state_hash());
  }
  for (std::size_t i = 0; i < sim.agent_count(); ++i) {
    const auto& drone = dynamic_cast<const SensedDrone&>(sim.agent(i));
    out.valid_returns += drone.valid_returns();
    out.total_rays += drone.total_rays();
    out.received += drone.received();
  }
  out.dropped_jam = sim.channel().stats().dropped_jam;
  return out;
}

}  // namespace

TEST(SensedSwarm, HashesAreIdenticalForEveryThreadCount) {
  const Scene scene(make_scene(7));
  const Weather fog = Weather::fog(300.0);
  const RunResult reference = run(scene, fog, 42, 1);
  for (std::size_t threads : {2U, 4U, 8U}) {
    const RunResult other = run(scene, fog, 42, threads);
    EXPECT_EQ(other.hashes, reference.hashes) << "threads " << threads;
  }
}

TEST(SensedSwarm, TheSensorsAndTheChannelActuallyDidSomething) {
  const Scene scene(make_scene(7));
  const RunResult r = run(scene, Weather::fog(300.0), 42, 4);
  EXPECT_GT(r.total_rays, 0U);
  EXPECT_GT(r.valid_returns, 0U);
  EXPECT_LT(r.valid_returns, r.total_rays);  // sky, range limits, and fog remove some
  EXPECT_GT(r.received, 0U);
  EXPECT_GT(r.dropped_jam, 0U);
}

TEST(SensedSwarm, SameInputsReproduceAndChangedInputsDiffer) {
  const Scene scene(make_scene(7));
  const Scene other_scene(make_scene(8));
  const Weather fog = Weather::fog(300.0);
  const auto base = run(scene, fog, 42, 2).hashes.back();
  EXPECT_EQ(run(scene, fog, 42, 2).hashes.back(), base);
  EXPECT_NE(run(scene, fog, 43, 2).hashes.back(), base);                 // different run seed
  EXPECT_NE(run(other_scene, fog, 42, 2).hashes.back(), base);           // different scene
  EXPECT_NE(run(scene, Weather::fog(80.0), 42, 2).hashes.back(), base);  // different weather
}

TEST(SensedSwarm, ASharedSceneGivesTheSameAnswersWhenQueriedFromManyThreads) {
  const Scene scene(make_scene(11));

  struct Ray {
    Vec3 origin;
    Vec3 direction;
    double range;
  };
  struct Answer {
    bool hit = false;
    double distance = 0.0;
    swarm::HitKind kind = swarm::HitKind::kNone;
    int obstacle = -1;
  };

  swarm::Rng rng(5, 0);
  std::vector<Ray> rays(100000);
  for (Ray& r : rays) {
    r.origin = {rng.uniform(-190.0, 190.0), rng.uniform(-190.0, 190.0), rng.uniform(0.0, 50.0)};
    r.direction = swarm::normalized(Vec3{rng.normal(), rng.normal(), rng.normal() - 0.4});
    r.range = rng.uniform(10.0, 120.0);
  }

  const auto cast = [&](const Ray& r) {
    Answer a;
    swarm::RayHit hit;
    a.hit = scene.raycast(r.origin, r.direction, r.range, hit);
    if (a.hit) {
      a.distance = hit.distance;
      a.kind = hit.kind;
      a.obstacle = hit.obstacle_index;
    }
    return a;
  };

  std::vector<Answer> sequential(rays.size());
  for (std::size_t i = 0; i < rays.size(); ++i) {
    sequential[i] = cast(rays[i]);
  }

  swarm::LockstepPool pool(8);
  std::vector<Answer> parallel(rays.size());
  pool.for_each(rays.size(), [&](std::size_t i) { parallel[i] = cast(rays[i]); });

  std::size_t hits = 0;
  for (std::size_t i = 0; i < rays.size(); ++i) {
    ASSERT_EQ(parallel[i].hit, sequential[i].hit) << "ray " << i;
    ASSERT_EQ(parallel[i].distance, sequential[i].distance) << "ray " << i;
    ASSERT_EQ(parallel[i].kind, sequential[i].kind) << "ray " << i;
    ASSERT_EQ(parallel[i].obstacle, sequential[i].obstacle) << "ray " << i;
    hits += sequential[i].hit ? 1U : 0U;
  }
  EXPECT_GT(hits, 10000U);
}
