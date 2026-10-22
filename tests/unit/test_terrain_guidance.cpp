#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarm/control/terrain_map.hpp"
#include "swarm/drone_agent.hpp"
#include "swarm/format/scene.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"
#include "swarm/simulation.hpp"

using swarm::PathSample;
using swarm::TerrainGuidanceParams;
using swarm::TerrainMap;
using swarm::Vec3;

namespace {

// A 10 x 10 grid of 10 m cells centered on the origin, flat at 5 m except one sample at 80 m.
TerrainMap ridge_map() {
  std::vector<double> h(11 * 11, 5.0);
  h[5 * 11 + 8] = 80.0;  // sample (8, 5): x = -50 + 80 = 30, y = 0
  return TerrainMap(10, 10, -50.0, -50.0, 10.0, h);
}

TerrainGuidanceParams guidance() {
  TerrainGuidanceParams p;
  p.enabled = true;
  p.clearance = 50.0;
  p.footprint = 0.0;
  p.climb_rate = 3.0;
  p.climb_fraction = 0.5;
  p.sample_spacing = 10.0;
  p.lookahead = 100.0;
  return p;
}

}  // namespace

TEST(TerrainMap, TheHighestSampleNearAPointIsFound) {
  const TerrainMap map = ridge_map();
  EXPECT_EQ(map.max_height_near(0.0, 0.0, 5.0), 5.0);
  // The ridge sample is at (30, 0). A disc of radius 12 around (20, 0) touches its cell.
  EXPECT_EQ(map.max_height_near(20.0, 0.0, 12.0), 80.0);
  // A disc just short of the cells next to the ridge does not see it.
  EXPECT_EQ(map.max_height_near(-30.0, 0.0, 5.0), 5.0);
}

TEST(TerrainMap, NeverUnderstatesTheTerrainAnywhereInTheDisc) {
  // The mesh's height in a cell never exceeds its highest corner, and the result is the highest
  // corner of every cell the disc touches, so it bounds the height of every point of the disc.
  const TerrainMap map = ridge_map();
  for (double x = -45.0; x <= 45.0; x += 7.5) {
    for (double y = -45.0; y <= 45.0; y += 7.5) {
      const double near = map.max_height_near(x, y, 6.0);
      EXPECT_GE(near, 5.0);
      if (std::hypot(x - 30.0, y) <= 6.0) {
        EXPECT_EQ(near, 80.0);
      }
    }
  }
}

TEST(TerrainMap, OutsideTheGridTheBoundaryIsUsed) {
  const TerrainMap map = ridge_map();
  EXPECT_EQ(map.max_height_near(1000.0, 0.0, 5.0), 5.0);
  EXPECT_TRUE(std::isnan(map.max_height_near(std::numeric_limits<double>::infinity(), 0.0, 1.0)));
  EXPECT_TRUE(std::isnan(map.max_height_near(0.0, 0.0, -1.0)));
}

TEST(TerrainMap, RejectsAnInvalidGrid) {
  EXPECT_THROW(TerrainMap(0, 1, 0.0, 0.0, 1.0, {0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(TerrainMap(1, 1, 0.0, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(TerrainMap(1, 1, 0.0, 0.0, 1.0, {0.0, 0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(
      TerrainMap(1, 1, 0.0, 0.0, 1.0, {0.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN()}),
      std::invalid_argument);
}

TEST(TerrainGuidance, FlatGroundPutsTheFloorAtTheGroundPlusTheClearance) {
  const TerrainMap map = ridge_map();
  const std::vector<PathSample> path = {{-30.0, 0.0, 0.0}, {-30.0, 10.0, 10.0}};
  EXPECT_DOUBLE_EQ(swarm::terrain_floor(map, guidance(), path, 17.0), 55.0);
}

TEST(TerrainGuidance, ARidgeAheadRaisesTheFloorLessTheFartherAwayItIs) {
  const TerrainMap map = ridge_map();
  const TerrainGuidanceParams p = guidance();  // climb budget 1.5 m/s
  const auto floor_with_ridge_at = [&](double distance) {
    return swarm::terrain_floor(map, p, {{30.0, 0.0, distance}}, 15.0);
  };
  // 80 m ridge + 50 m clearance = 130 m, less what a climb at 1.5 m/s gains over distance / 15 s.
  EXPECT_NEAR(floor_with_ridge_at(0.0), 130.0, 1e-12);
  EXPECT_NEAR(floor_with_ridge_at(150.0), 130.0 - 1.5 * 10.0, 1e-12);
  EXPECT_NEAR(floor_with_ridge_at(1500.0), 130.0 - 1.5 * 100.0, 1e-12);
}

TEST(TerrainGuidance, ThePathHelpersCoverTheLookaheadAndALap) {
  const TerrainGuidanceParams p = guidance();
  const std::vector<PathSample> line = swarm::straight_path(Vec3{0.0, 0.0, 50.0}, 1.0, 0.0, p);
  ASSERT_EQ(line.size(), 11U);  // the position and ten samples 10 m apart
  EXPECT_DOUBLE_EQ(line.back().x, 100.0);
  EXPECT_DOUBLE_EQ(line.back().distance, 100.0);
  EXPECT_EQ(swarm::straight_path(Vec3{1.0, 2.0, 3.0}, 0.0, 0.0, p).size(), 1U);

  const std::vector<PathSample> circle =
      swarm::circle_path(Vec3{100.0, 0.0, 50.0}, 0.0, 0.0, 100.0, p);
  // A lap of 628 m at 10 m spacing; every point is on the circle and the first is at the aircraft.
  EXPECT_GE(circle.size(), 63U);
  EXPECT_NEAR(circle.front().x, 100.0, 1e-9);
  for (const PathSample& s : circle) {
    EXPECT_NEAR(std::hypot(s.x, s.y), 100.0, 1e-9);
  }
  // Off the circle, the distance includes the way to it.
  const std::vector<PathSample> joining =
      swarm::circle_path(Vec3{160.0, 0.0, 50.0}, 0.0, 0.0, 100.0, p);
  EXPECT_NEAR(joining.front().distance, 60.0, 1e-9);
}

TEST(TerrainGuidance, ParametersAreChecked) {
  const TerrainMap map = ridge_map();
  TerrainGuidanceParams p = guidance();
  p.climb_fraction = 0.0;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  p = guidance();
  p.clearance = -1.0;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  EXPECT_THROW(swarm::terrain_floor(map, guidance(), {}, 0.0), std::invalid_argument);
  EXPECT_EQ(swarm::terrain_floor(map, guidance(), {}, 10.0),
            -std::numeric_limits<double>::infinity());
}

namespace {

// Four aircraft orbiting over hills that rise to 100 m, with a nominal altitude of 8 m, which is
// below the highest ground. With ground contact on, an aircraft that touches ground
// stops there.
std::string hilly(const std::string& extra) {
  return "version: 3\n"
         "seed: 5\n"
         "duration_s: 90\n"
         "scene: {type: hills, size_m: [2000, 2000], cell_m: 10, amplitude_m: 100,\n"
         "        obstacles: {density_per_m2: 0.0}}\n"
         "swarm: {drones: 4}\n"
         "mission:\n"
         "  waypoints:\n"
         "    - [0, 0, 8]\n"
         "  speed_mps: 17\n"
         "vehicle: {type: fixed_wing}\n"
         "environment: {ground_contact_crashes: true}\n"
         "weather: {visibility_m: 20000}\n"
         "control:\n"
         "  orbit: {radius_m: 150, radius_step_m: 12, altitude_step_m: 10}\n" +
         extra;
}

struct Flown {
  double min_agl = std::numeric_limits<double>::infinity();
  int crashed = 0;
};

Flown fly(const std::string& text) {
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok()) << (r.errors.empty() ? "" : swarm::to_string(r.errors.front()));
  const swarm::ScenarioWorld world = swarm::build_world(r.scenario);
  const std::unique_ptr<swarm::Simulation> sim = world.make_simulation(1);
  Flown out;
  for (std::uint64_t tick = 0; tick < r.scenario.duration_ticks; ++tick) {
    sim->step();
    for (std::size_t i = 0; i < r.scenario.drones; ++i) {
      const Vec3 p = sim->agent(i).position();
      out.min_agl = std::min(out.min_agl, p.z - world.scene->ground_height(p.x, p.y));
    }
  }
  for (std::size_t i = 0; i < r.scenario.drones; ++i) {
    out.crashed += dynamic_cast<const swarm::DroneAgent&>(sim->agent(i)).crashed() ? 1 : 0;
  }
  return out;
}

}  // namespace

TEST(TerrainGuidanceFlight, WithoutGuidanceLowOrbitsOverHillsHitTheGround) {
  const Flown without = fly(hilly(""));
  EXPECT_GT(without.crashed, 0) << "min agl " << without.min_agl;
}

TEST(TerrainGuidanceFlight, WithGuidanceTheAircraftStayWellAboveTheGround) {
  const Flown with = fly(hilly("  terrain: {clearance_m: 50}\n"));
  EXPECT_EQ(with.crashed, 0);
  // The clearance is 50 m over the highest ground near the path; the aircraft follows its
  // altitude command with a lag, so allow a margin.
  EXPECT_GT(with.min_agl, 25.0);
}

TEST(TerrainGuidanceFlight, ARunWithGuidanceIsRepeatable) {
  const std::string text = hilly("  terrain: {clearance_m: 50}\n");
  const swarm::LoadResult r = swarm::load_scenario(text);
  ASSERT_TRUE(r.ok());
  const swarm::ScenarioWorld world = swarm::build_world(r.scenario);
  const auto hash_after = [&](std::size_t threads) {
    const std::unique_ptr<swarm::Simulation> sim = world.make_simulation(threads);
    for (std::uint64_t tick = 0; tick < 1500; ++tick) {
      sim->step();
    }
    return sim->state_hash();
  };
  EXPECT_EQ(hash_after(1), hash_after(3));
}

TEST(TerrainGuidanceFlight, ScenarioKeysNeedTheirVersionAndAnAircraft) {
  EXPECT_FALSE(
      swarm::load_scenario(
          "version: 2\nseed: 1\nduration_s: 5\nswarm: {drones: 2}\nmission:\n  waypoints:\n"
          "    - [0, 0, 60]\n  speed_mps: 17\nvehicle: {type: fixed_wing}\ncontrol:\n"
          "  terrain: {clearance_m: 50}\n")
          .ok());
  EXPECT_FALSE(swarm::load_scenario("version: 3\nseed: 1\nduration_s: 5\nswarm: {drones: 2}\n"
                                    "mission:\n  waypoints:\n    - [0, 0, 20]\n  speed_mps: 3\n"
                                    "control:\n  terrain: {clearance_m: 50}\n")
                   .ok());
  const swarm::Scenario ok = swarm::load_scenario(hilly("  terrain: {clearance_m: 50}\n")).scenario;
  const std::string text = swarm::canonical_text(ok);
  EXPECT_EQ(text.rfind("version: 3\n", 0), 0U);
  const swarm::LoadResult again = swarm::load_scenario(text);
  ASSERT_TRUE(again.ok());
  EXPECT_EQ(swarm::canonical_text(again.scenario), text);
  EXPECT_TRUE(again.scenario.terrain_guidance.enabled);
  EXPECT_TRUE(again.scenario.ground_contact_crashes);
}

TEST(TerrainGuidance, ObstacleTopsAreRaisedIntoTheMap) {
  swarm::SceneData scene;
  scene.terrain.cells_x = 10;
  scene.terrain.cells_y = 10;
  scene.terrain.origin_x = -50.0;
  scene.terrain.origin_y = -50.0;
  scene.terrain.cell_size = 10.0;
  scene.terrain.heights.assign(11 * 11, 5.0);
  swarm::Obstacle tower;
  tower.kind = swarm::ObstacleKind::kCylinder;
  tower.cx = 20.0;
  tower.cy = 0.0;
  tower.base_z = 5.0;
  tower.height = 95.0;
  tower.half_x = 4.0;
  scene.obstacles.push_back(tower);
  const TerrainMap map = swarm::make_terrain_map(scene);
  EXPECT_EQ(map.max_height_near(20.0, 0.0, 1.0), 100.0);
  // Away from the tower the ground is as it was.
  EXPECT_EQ(map.max_height_near(-40.0, 40.0, 1.0), 5.0);
  // A box counts by its bounding rectangle.
  swarm::Obstacle slab;
  slab.kind = swarm::ObstacleKind::kBox;
  slab.cx = -20.0;
  slab.cy = -20.0;
  slab.base_z = 5.0;
  slab.height = 45.0;
  slab.half_x = 12.0;
  slab.half_y = 3.0;
  scene.obstacles.push_back(slab);
  const TerrainMap with_slab = swarm::make_terrain_map(scene);
  EXPECT_EQ(with_slab.max_height_near(-20.0, -20.0, 1.0), 50.0);
  EXPECT_EQ(with_slab.max_height_near(-32.0, -20.0, 1.0), 50.0);
}
