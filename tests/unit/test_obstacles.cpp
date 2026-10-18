#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "swarm/obstacles.hpp"
#include "swarm/terrain.hpp"

using swarm::Obstacle;
using swarm::ObstacleConfig;
using swarm::ObstacleKind;
using swarm::SceneData;
using swarm::TerrainConfig;
using swarm::TerrainType;

namespace {

SceneData terrain(double size, TerrainType type = TerrainType::kField) {
  TerrainConfig c;
  c.size_x = size;
  c.size_y = size;
  c.cell_size = 4.0;
  c.type = type;
  c.amplitude = 25.0;
  c.feature_scale = 80.0;
  return swarm::generate_terrain(c, 5);
}

double bounding_radius(const Obstacle& o) {
  return o.kind == ObstacleKind::kCylinder ? o.half_x
                                           : std::sqrt(o.half_x * o.half_x + o.half_y * o.half_y);
}

SceneData placed_scene(double size, const ObstacleConfig& config, std::uint64_t seed,
                       TerrainType type = TerrainType::kField) {
  SceneData scene = terrain(size, type);
  swarm::place_obstacles(scene, config, seed);
  return scene;
}

ObstacleConfig sparse() {
  ObstacleConfig c;
  c.density_per_m2 = 0.0005;
  return c;
}

}  // namespace

TEST(Obstacles, ZeroDensityPlacesNothing) {
  ObstacleConfig c = sparse();
  c.density_per_m2 = 0.0;
  SceneData scene = terrain(400.0);
  EXPECT_EQ(swarm::place_obstacles(scene, c, 1), 0U);
  EXPECT_TRUE(scene.obstacles.empty());
}

TEST(Obstacles, UnsaturatedSceneReachesTheExactTargetCount) {
  SceneData scene = terrain(800.0);
  const std::size_t placed = swarm::place_obstacles(scene, sparse(), 1);
  EXPECT_EQ(placed, 320U);  // 0.0005 * 800 * 800
  EXPECT_EQ(scene.obstacles.size(), 320U);
  EXPECT_TRUE(swarm::validate(scene));
}

TEST(Obstacles, CrowdedSceneTerminatesWithFewerObstaclesAndKeepsSpacing) {
  ObstacleConfig c = sparse();
  c.density_per_m2 = 0.05;  // far more than fits
  c.min_spacing = 4.0;
  SceneData scene = terrain(120.0);
  const std::size_t placed = swarm::place_obstacles(scene, c, 2);
  const double target = 0.05 * 120.0 * 120.0;
  EXPECT_GT(placed, 0U);
  EXPECT_LT(static_cast<double>(placed), target);
  for (std::size_t i = 0; i < scene.obstacles.size(); ++i) {
    for (std::size_t j = i + 1; j < scene.obstacles.size(); ++j) {
      const auto& a = scene.obstacles[i];
      const auto& b = scene.obstacles[j];
      const double d = std::hypot(a.cx - b.cx, a.cy - b.cy);
      ASSERT_GE(d, bounding_radius(a) + bounding_radius(b) + c.min_spacing - 1e-9);
    }
  }
}

TEST(Obstacles, FootprintsStayInsideTheTerrainWithTheEdgeMargin) {
  ObstacleConfig c = sparse();
  c.edge_margin = 8.0;
  const SceneData scene = placed_scene(400.0, c, 3);
  ASSERT_FALSE(scene.obstacles.empty());
  for (const Obstacle& o : scene.obstacles) {
    const double r = bounding_radius(o);
    ASSERT_GE(o.cx - r, -200.0 + 8.0 - 1e-9);
    ASSERT_LE(o.cx + r, 200.0 - 8.0 + 1e-9);
    ASSERT_GE(o.cy - r, -200.0 + 8.0 - 1e-9);
    ASSERT_LE(o.cy + r, 200.0 - 8.0 + 1e-9);
  }
}

TEST(Obstacles, SpacingIsRespectedBetweenAllPairs) {
  ObstacleConfig c = sparse();
  c.min_spacing = 12.0;
  const SceneData scene = placed_scene(800.0, c, 4);
  for (std::size_t i = 0; i < scene.obstacles.size(); ++i) {
    for (std::size_t j = i + 1; j < scene.obstacles.size(); ++j) {
      const auto& a = scene.obstacles[i];
      const auto& b = scene.obstacles[j];
      ASSERT_GE(std::hypot(a.cx - b.cx, a.cy - b.cy),
                bounding_radius(a) + bounding_radius(b) + 12.0 - 1e-9);
    }
  }
}

TEST(Obstacles, SpawnAreaStaysClear) {
  ObstacleConfig c = sparse();
  c.density_per_m2 = 0.002;
  c.spawn_x = 50.0;
  c.spawn_y = -30.0;
  c.spawn_clear_radius = 60.0;
  const SceneData scene = placed_scene(400.0, c, 5);
  ASSERT_FALSE(scene.obstacles.empty());
  for (const Obstacle& o : scene.obstacles) {
    ASSERT_GE(std::hypot(o.cx - 50.0, o.cy + 30.0), 60.0 + bounding_radius(o) - 1e-9);
  }
}

TEST(Obstacles, SameSeedIsIdenticalDifferentSeedDiffers) {
  const SceneData a = placed_scene(400.0, sparse(), 9, TerrainType::kHills);
  const SceneData b = placed_scene(400.0, sparse(), 9, TerrainType::kHills);
  const SceneData c = placed_scene(400.0, sparse(), 10, TerrainType::kHills);
  EXPECT_EQ(swarm::scene_hash(a), swarm::scene_hash(b));
  EXPECT_NE(swarm::scene_hash(a), swarm::scene_hash(c));
}

TEST(Obstacles, CylinderFractionIsRespected) {
  // Kept sparse on purpose: in a crowded scene large boxes are rejected more often than small
  // cylinders, so the realized cylinder share drifts above the requested fraction.
  ObstacleConfig c = sparse();
  c.density_per_m2 = 0.0005;
  c.cylinder_fraction = 0.0;
  for (const Obstacle& o : placed_scene(1200.0, c, 6).obstacles) {
    ASSERT_EQ(o.kind, ObstacleKind::kBox);
  }
  c.cylinder_fraction = 1.0;
  for (const Obstacle& o : placed_scene(1200.0, c, 6).obstacles) {
    ASSERT_EQ(o.kind, ObstacleKind::kCylinder);
  }
  c.cylinder_fraction = 0.3;
  const SceneData mixed = placed_scene(1200.0, c, 6);
  ASSERT_EQ(mixed.obstacles.size(), 720U);  // target reached, so nothing was biased by rejection
  const auto cylinders = static_cast<double>(
      std::count_if(mixed.obstacles.begin(), mixed.obstacles.end(),
                    [](const Obstacle& o) { return o.kind == ObstacleKind::kCylinder; }));
  EXPECT_NEAR(cylinders / static_cast<double>(mixed.obstacles.size()), 0.3, 0.05);
}

TEST(Obstacles, SizesStayWithinTheConfiguredRanges) {
  ObstacleConfig c = sparse();
  c.density_per_m2 = 0.002;
  const SceneData scene = placed_scene(600.0, c, 7);
  for (const Obstacle& o : scene.obstacles) {
    if (o.kind == ObstacleKind::kBox) {
      ASSERT_GE(o.half_x, c.box_half_min);
      ASSERT_LE(o.half_x, c.box_half_max);
      ASSERT_GE(o.half_y, c.box_half_min);
      ASSERT_LE(o.half_y, c.box_half_max);
      ASSERT_GE(o.height, c.box_height_min);
      ASSERT_LE(o.height, c.box_height_max);
    } else {
      ASSERT_GE(o.half_x, c.cylinder_radius_min);
      ASSERT_LE(o.half_x, c.cylinder_radius_max);
      ASSERT_GE(o.height, c.cylinder_height_min);
      ASSERT_LE(o.height, c.cylinder_height_max);
    }
  }
}

TEST(Obstacles, BaseSitsAtTheEmbedDepthOnFlatGround) {
  ObstacleConfig c = sparse();
  c.embed = 0.75;
  for (const Obstacle& o : placed_scene(400.0, c, 8).obstacles) {
    ASSERT_DOUBLE_EQ(o.base_z, -0.75);
  }
}

TEST(Obstacles, BaseFollowsTheLowestTerrainUnderTheFootprintOnSlopes) {
  ObstacleConfig c = sparse();
  c.density_per_m2 = 0.001;
  c.embed = 0.5;
  const SceneData scene = placed_scene(400.0, c, 12, TerrainType::kHills);
  ASSERT_FALSE(scene.obstacles.empty());
  for (const Obstacle& o : scene.obstacles) {
    // Reference: the lowest terrain height found on a fine grid over the footprint.
    double lowest = std::numeric_limits<double>::max();
    const double ex = o.half_x;
    const double ey = o.kind == ObstacleKind::kCylinder ? o.half_x : o.half_y;
    for (double x = o.cx - ex; x <= o.cx + ex; x += 0.2) {
      for (double y = o.cy - ey; y <= o.cy + ey; y += 0.2) {
        if (o.kind == ObstacleKind::kCylinder &&
            (x - o.cx) * (x - o.cx) + (y - o.cy) * (y - o.cy) > o.half_x * o.half_x) {
          continue;
        }
        lowest = std::min(lowest, swarm::height_at(scene.terrain, x, y));
      }
    }
    ASSERT_NEAR(o.base_z, lowest - 0.5, 0.35);
  }
}

TEST(Obstacles, ExistingObstaclesAreKeptAndRespectedForSpacing) {
  SceneData scene = terrain(300.0);
  Obstacle existing;
  existing.kind = ObstacleKind::kCylinder;
  existing.cx = 100.0;
  existing.cy = 100.0;
  existing.height = 10.0;
  existing.half_x = 1.0;
  scene.obstacles.push_back(existing);

  ObstacleConfig c = sparse();
  c.density_per_m2 = 0.002;
  swarm::place_obstacles(scene, c, 13);
  ASSERT_GT(scene.obstacles.size(), 1U);
  EXPECT_EQ(scene.obstacles[0].cx, 100.0);
  for (std::size_t i = 1; i < scene.obstacles.size(); ++i) {
    const auto& o = scene.obstacles[i];
    ASSERT_GE(std::hypot(o.cx - 100.0, o.cy - 100.0),
              bounding_radius(o) + 1.0 + c.min_spacing - 1e-9);
  }
}

TEST(Obstacles, ResultAlwaysValidates) {
  for (std::uint64_t seed : {1ULL, 2ULL, 3ULL}) {
    EXPECT_TRUE(swarm::validate(placed_scene(500.0, sparse(), seed, TerrainType::kHills)));
  }
}

TEST(Obstacles, GoldenHashPinsPlacementAcrossPlatforms) {
  // Placement uses the PCG stream, sqrt, and basic arithmetic only, so it must match on every
  // compiler and platform. Update the value only for an intended change to the algorithm.
  TerrainConfig t;
  t.size_x = 200.0;
  t.size_y = 200.0;
  t.cell_size = 4.0;
  t.type = TerrainType::kHills;
  t.amplitude = 15.0;
  t.feature_scale = 60.0;
  SceneData scene = swarm::generate_terrain(t, 77);
  ObstacleConfig c;
  c.density_per_m2 = 0.001;
  swarm::place_obstacles(scene, c, 77);
  EXPECT_EQ(scene.obstacles.size(), 40U);
  EXPECT_EQ(swarm::scene_hash(scene), 11716285591898144494ULL);
}

TEST(Obstacles, InvalidArgumentsAreRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto rejects = [](ObstacleConfig c) {
    SceneData scene = terrain(200.0);
    EXPECT_THROW(swarm::place_obstacles(scene, c, 1), std::invalid_argument);
  };
  ObstacleConfig c = sparse();
  c.density_per_m2 = -1.0;
  rejects(c);
  c = sparse();
  c.density_per_m2 = nan;
  rejects(c);
  c = sparse();
  c.min_spacing = -1.0;
  rejects(c);
  c = sparse();
  c.cylinder_fraction = 1.5;
  rejects(c);
  c = sparse();
  c.box_half_min = 7.0;  // above the maximum
  rejects(c);
  c = sparse();
  c.cylinder_radius_min = 0.0;
  rejects(c);
  c = sparse();
  c.box_height_max = nan;
  rejects(c);
  c = sparse();
  c.attempts_per_obstacle = 0;
  rejects(c);
  c = sparse();
  c.spawn_clear_radius = -1.0;
  rejects(c);

  SceneData broken = terrain(200.0);
  broken.terrain.cell_size = 0.0;
  EXPECT_THROW(swarm::place_obstacles(broken, sparse(), 1), std::invalid_argument);
}
