#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "brute_force.hpp"
#include "swarm/rng.hpp"
#include "swarm/scene.hpp"
#include "swarm/terrain.hpp"

using swarm::HitKind;
using swarm::RayHit;
using swarm::Scene;
using swarm::SceneData;
using swarm::Vec3;

namespace {

SceneData flat_scene(double size = 100.0, double cell = 4.0) {
  swarm::TerrainConfig c;
  c.size_x = size;
  c.size_y = size;
  c.cell_size = cell;
  c.type = swarm::TerrainType::kField;
  return swarm::generate_terrain(c, 1);
}

// Heights of the plane z = a x + b y + c, sampled on the scene's grid.
SceneData tilted_scene(double a, double b, double c) {
  SceneData s = flat_scene(160.0, 4.0);
  auto& t = s.terrain;
  for (std::size_t j = 0; j < t.samples_y(); ++j) {
    for (std::size_t i = 0; i < t.samples_x(); ++i) {
      const double x = t.origin_x + static_cast<double>(i) * t.cell_size;
      const double y = t.origin_y + static_cast<double>(j) * t.cell_size;
      t.heights[t.index(i, j)] = a * x + b * y + c;
    }
  }
  return s;
}

SceneData hilly_scene() {
  swarm::TerrainConfig c;
  c.size_x = 400.0;
  c.size_y = 400.0;
  c.cell_size = 4.0;
  c.type = swarm::TerrainType::kHills;
  c.amplitude = 30.0;
  c.feature_scale = 70.0;
  c.octaves = 4;
  return swarm::generate_terrain(c, 17);
}

Vec3 unit(double x, double y, double z) { return swarm::normalized(Vec3{x, y, z}); }

}  // namespace

TEST(RaycastTerrain, VerticalRayHitsFlatGroundAtTheHeight) {
  const Scene scene(flat_scene());
  RayHit hit;
  ASSERT_TRUE(scene.raycast({3.7, -2.1, 10.0}, {0, 0, -1}, 100.0, hit));
  EXPECT_NEAR(hit.distance, 10.0, 1e-12);
  EXPECT_NEAR(hit.point.z, 0.0, 1e-12);
  EXPECT_NEAR(hit.point.x, 3.7, 1e-12);
  EXPECT_EQ(hit.kind, HitKind::kTerrain);
  EXPECT_EQ(hit.obstacle_index, -1);
  EXPECT_NEAR(hit.normal.z, 1.0, 1e-12);
}

TEST(RaycastTerrain, ObliqueRayHitsAtTheAnalyticDistance) {
  const Scene scene(flat_scene());
  RayHit hit;
  ASSERT_TRUE(scene.raycast({0, 0, 10}, unit(1, 0, -1), 100.0, hit));
  EXPECT_NEAR(hit.distance, 10.0 * std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(hit.point.x, 10.0, 1e-9);
  EXPECT_NEAR(hit.point.z, 0.0, 1e-9);
}

TEST(RaycastTerrain, MissesWhenParallelPointingAwayOrOutOfRange) {
  const Scene scene(flat_scene());
  RayHit hit;
  EXPECT_FALSE(scene.raycast({0, 0, 5}, {1, 0, 0}, 100.0, hit));
  EXPECT_FALSE(scene.raycast({0, 0, 5}, {0, 0, 1}, 100.0, hit));
  EXPECT_FALSE(scene.raycast({0, 0, 5}, {0, 0, -1}, 4.9, hit));
}

TEST(RaycastTerrain, HitExactlyAtMaxRangeCounts) {
  const Scene scene(flat_scene());
  RayHit hit;
  EXPECT_TRUE(scene.raycast({0, 0, 5}, {0, 0, -1}, 5.0, hit));
  EXPECT_NEAR(hit.distance, 5.0, 1e-12);
}

TEST(RaycastTerrain, TiltedPlaneMatchesTheAnalyticSolution) {
  const double a = 0.2;
  const double b = -0.1;
  const double c = 3.0;
  const Scene scene(tilted_scene(a, b, c));
  swarm::Rng rng(5, 0);
  int hits = 0;
  for (int n = 0; n < 5000; ++n) {
    const Vec3 origin{rng.uniform(-70.0, 70.0), rng.uniform(-70.0, 70.0), rng.uniform(25.0, 60.0)};
    const Vec3 dir = unit(rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, -0.2));
    const double denom = dir.z - a * dir.x - b * dir.y;
    const double expected = (a * origin.x + b * origin.y + c - origin.z) / denom;
    const Vec3 hit_point = origin + dir * expected;
    const bool inside = std::abs(hit_point.x) <= 80.0 && std::abs(hit_point.y) <= 80.0;
    RayHit hit;
    const bool got = scene.raycast(origin, dir, 1000.0, hit);
    if (inside) {
      ASSERT_TRUE(got);
      ASSERT_NEAR(hit.distance, expected, 1e-9);
      const Vec3 plane_normal = unit(-a, -b, 1.0);
      ASSERT_NEAR(hit.normal.x, plane_normal.x, 1e-9);
      ASSERT_NEAR(hit.normal.y, plane_normal.y, 1e-9);
      ASSERT_NEAR(hit.normal.z, plane_normal.z, 1e-9);
      ++hits;
    }
  }
  EXPECT_GT(hits, 2500);  // enough analytic comparisons; the rest leave the 160 m terrain
}

TEST(RaycastTerrain, ShallowRayCrossesManyCells) {
  const Scene scene(flat_scene(400.0, 4.0));
  RayHit hit;
  const Vec3 dir = unit(std::cos(0.03), std::sin(0.03) * 0.5, -std::sin(0.035));
  ASSERT_TRUE(scene.raycast({-190.0, -90.0, 5.0}, dir, 1000.0, hit));
  EXPECT_NEAR(hit.point.z, 0.0, 1e-9);
  EXPECT_NEAR(hit.distance, 5.0 / -dir.z, 1e-6);
  EXPECT_GT(hit.distance, 100.0);  // well over twenty cells
}

TEST(RaycastTerrain, RaysThroughVerticesAndAlongCellBordersStillHit) {
  const SceneData data = flat_scene(100.0, 4.0);
  const Scene scene(data);
  const double vx = data.terrain.origin_x + 8 * 4.0;
  const double vy = data.terrain.origin_y + 6 * 4.0;
  RayHit hit;

  ASSERT_TRUE(scene.raycast({vx, vy, 10.0}, {0, 0, -1}, 50.0, hit));  // straight down a vertex
  EXPECT_NEAR(hit.distance, 10.0, 1e-9);

  // A diagonal ray whose ground track passes exactly through grid vertices.
  ASSERT_TRUE(scene.raycast({vx - 12.0, vy - 12.0, 12.0}, unit(1, 1, -1), 100.0, hit));
  EXPECT_NEAR(hit.point.x, vx, 1e-9);
  EXPECT_NEAR(hit.point.y, vy, 1e-9);

  // A ray running along a grid line.
  ASSERT_TRUE(scene.raycast({vx, data.terrain.origin_y + 2.0, 1.0}, unit(0, 1, -0.05), 100.0, hit));
  EXPECT_NEAR(hit.point.z, 0.0, 1e-9);
  EXPECT_NEAR(hit.point.x, vx, 1e-9);
}

TEST(RaycastTerrain, RayStartingOutsideTheTerrainCanEnterIt) {
  const Scene scene(flat_scene(100.0, 4.0));
  RayHit hit;
  ASSERT_TRUE(scene.raycast({-150.0, 0.0, 10.0}, unit(120.0, 0.0, -10.0), 500.0, hit));
  EXPECT_NEAR(hit.point.x, -30.0, 1e-9);
  EXPECT_NEAR(hit.point.z, 0.0, 1e-9);
  EXPECT_NEAR(hit.distance, std::sqrt(120.0 * 120.0 + 10.0 * 10.0), 1e-9);
}

TEST(RaycastTerrain, RaysThatNeverEnterOrAlreadyLeftMiss) {
  const Scene scene(flat_scene(100.0, 4.0));
  RayHit hit;
  EXPECT_FALSE(scene.raycast({-150.0, 0.0, 10.0}, unit(-1, 0, -0.1), 500.0, hit));   // away
  EXPECT_FALSE(scene.raycast({-150.0, 80.0, 10.0}, unit(1, 0, -0.01), 500.0, hit));  // beside
  EXPECT_FALSE(scene.raycast({0.0, 0.0, 10.0}, unit(1, 0, 0), 500.0, hit));          // level, exits
  EXPECT_FALSE(scene.raycast({60.0, 0.0, 10.0}, unit(0, 0, -1), 500.0, hit));  // outside, down
}

TEST(RaycastTerrain, RayFromBelowHitsTheSurfaceAndTheNormalFacesTheOrigin) {
  const Scene scene(flat_scene());
  RayHit hit;
  ASSERT_TRUE(scene.raycast({0, 0, -5}, {0, 0, 1}, 50.0, hit));
  EXPECT_NEAR(hit.distance, 5.0, 1e-12);
  EXPECT_NEAR(hit.normal.z, -1.0, 1e-12);
}

TEST(RaycastTerrain, InvalidRaysDoNotHit) {
  const Scene scene(flat_scene());
  RayHit hit;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(scene.raycast({0, 0, 5}, {0, 0, 0}, 50.0, hit));   // zero direction
  EXPECT_FALSE(scene.raycast({0, 0, 5}, {0, 0, -2}, 50.0, hit));  // not unit length
  EXPECT_FALSE(scene.raycast({0, 0, 5}, {0, 0, -1}, 0.0, hit));   // no range
  EXPECT_FALSE(scene.raycast({0, 0, 5}, {0, 0, -1}, -1.0, hit));
  EXPECT_FALSE(scene.raycast({nan, 0, 5}, {0, 0, -1}, 50.0, hit));
  EXPECT_FALSE(scene.raycast({0, 0, 5}, {nan, 0, -1}, 50.0, hit));
  EXPECT_FALSE(scene.raycast({0, 0, 5}, {0, 0, -1}, nan, hit));
}

TEST(RaycastTerrain, AgreesWithBruteForceOnRandomRaysOverHills) {
  const SceneData data = hilly_scene();
  const Scene scene(data);
  swarm::Rng rng(31, 0);
  int hits = 0;
  int misses = 0;
  for (int n = 0; n < 20000; ++n) {
    const Vec3 origin{rng.uniform(-220.0, 220.0), rng.uniform(-220.0, 220.0),
                      rng.uniform(-35.0, 70.0)};
    const Vec3 dir = swarm::normalized(Vec3{rng.normal(), rng.normal(), rng.normal()});
    const double range = rng.uniform(20.0, 400.0);

    double expected = 0.0;
    const bool want =
        swarm::testing::brute_force_terrain(data.terrain, origin, dir, range, expected);
    RayHit hit;
    const bool got = scene.raycast(origin, dir, range, hit);
    ASSERT_EQ(got, want) << "ray " << n;
    if (want) {
      ASSERT_NEAR(hit.distance, expected, 1e-7) << "ray " << n;
      ASSERT_LT(swarm::dot(hit.normal, dir), 0.0);
      ASSERT_NEAR(swarm::norm(hit.normal), 1.0, 1e-12);
      ++hits;
    } else {
      ++misses;
    }
  }
  // The sample must exercise both outcomes, or the comparison proves little.
  EXPECT_GT(hits, 3000);
  EXPECT_GT(misses, 3000);
}

TEST(RaycastTerrain, GroundHeightMatchesTheTerrainQuery) {
  const SceneData data = hilly_scene();
  const Scene scene(data);
  EXPECT_DOUBLE_EQ(scene.ground_height(12.5, -33.1), swarm::height_at(data.terrain, 12.5, -33.1));
}

TEST(RaycastTerrain, ConstructionRejectsInvalidData) {
  SceneData bad = flat_scene();
  bad.terrain.heights.pop_back();
  EXPECT_THROW(Scene{bad}, std::invalid_argument);
}
