#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "brute_force.hpp"
#include "swarm/geometry.hpp"
#include "swarm/obstacles.hpp"
#include "swarm/rng.hpp"
#include "swarm/scene.hpp"
#include "swarm/terrain.hpp"

using swarm::HitKind;
using swarm::Obstacle;
using swarm::ObstacleKind;
using swarm::PrimitiveHit;
using swarm::RayHit;
using swarm::Scene;
using swarm::SceneData;
using swarm::Vec3;

namespace {

Obstacle box(double cx, double cy, double base, double hx, double hy, double height) {
  Obstacle o;
  o.kind = ObstacleKind::kBox;
  o.cx = cx;
  o.cy = cy;
  o.base_z = base;
  o.half_x = hx;
  o.half_y = hy;
  o.height = height;
  return o;
}

Obstacle cylinder(double cx, double cy, double base, double radius, double height) {
  Obstacle o;
  o.kind = ObstacleKind::kCylinder;
  o.cx = cx;
  o.cy = cy;
  o.base_z = base;
  o.half_x = radius;
  o.height = height;
  return o;
}

SceneData flat_scene(double size = 200.0, double cell = 4.0) {
  swarm::TerrainConfig c;
  c.size_x = size;
  c.size_y = size;
  c.cell_size = cell;
  c.type = swarm::TerrainType::kField;
  return swarm::generate_terrain(c, 1);
}

SceneData with(SceneData scene, std::vector<Obstacle> obstacles) {
  scene.obstacles = std::move(obstacles);
  return scene;
}

void expect_hit(const Scene& scene, const Vec3& origin, const Vec3& dir, double distance,
                const Vec3& normal, HitKind kind, int index) {
  RayHit hit;
  ASSERT_TRUE(scene.raycast(origin, dir, 500.0, hit));
  EXPECT_NEAR(hit.distance, distance, 1e-9);
  EXPECT_NEAR(hit.normal.x, normal.x, 1e-9);
  EXPECT_NEAR(hit.normal.y, normal.y, 1e-9);
  EXPECT_NEAR(hit.normal.z, normal.z, 1e-9);
  EXPECT_EQ(hit.kind, kind);
  EXPECT_EQ(hit.obstacle_index, index);
  const Vec3 expected_point = origin + dir * distance;
  EXPECT_NEAR(hit.point.x, expected_point.x, 1e-9);
  EXPECT_NEAR(hit.point.y, expected_point.y, 1e-9);
  EXPECT_NEAR(hit.point.z, expected_point.z, 1e-9);
}

}  // namespace

TEST(RaycastObstacles, BoxFacesAreHitAtTheAnalyticDistanceWithTheRightNormal) {
  // Box centered (10, 5), half extents 2 x 3, z from 1 to 5. Floating so the bottom face is
  // reachable.
  const Scene scene(with(flat_scene(), {box(10, 5, 1, 2, 3, 4)}));
  const HitKind k = HitKind::kBox;
  expect_hit(scene, {30, 5, 3}, {-1, 0, 0}, 18.0, {1, 0, 0}, k, 0);    // +x face at x = 12
  expect_hit(scene, {-10, 5, 3}, {1, 0, 0}, 18.0, {-1, 0, 0}, k, 0);   // -x face at x = 8
  expect_hit(scene, {10, 20, 3}, {0, -1, 0}, 12.0, {0, 1, 0}, k, 0);   // +y face at y = 8
  expect_hit(scene, {10, -10, 3}, {0, 1, 0}, 12.0, {0, -1, 0}, k, 0);  // -y face at y = 2
  expect_hit(scene, {10, 5, 20}, {0, 0, -1}, 15.0, {0, 0, 1}, k, 0);   // top at z = 5
  expect_hit(scene, {10, 5, 0.5}, {0, 0, 1}, 0.5, {0, 0, -1}, k, 0);   // bottom at z = 1
}

TEST(RaycastObstacles, CylinderSideAndCapsAreHitAtTheAnalyticDistance) {
  // Cylinder at (-20, 10), radius 3, z from 2 to 8.
  const Scene scene(with(flat_scene(), {cylinder(-20, 10, 2, 3, 6)}));
  const HitKind k = HitKind::kCylinder;
  expect_hit(scene, {0, 10, 4}, {-1, 0, 0}, 17.0, {1, 0, 0}, k, 0);
  expect_hit(scene, {-20, 10, 20}, {0, 0, -1}, 12.0, {0, 0, 1}, k, 0);  // top at z = 8
  expect_hit(scene, {-20, 10, 0.5}, {0, 0, 1}, 1.5, {0, 0, -1}, k, 0);  // bottom at z = 2
  // Off-center chord: the wall is hit where x = -20 + sqrt(9 - 1).
  const double x_hit = -20.0 + std::sqrt(8.0);
  expect_hit(scene, {-10, 11, 4}, {-1, 0, 0}, -10.0 - x_hit, {std::sqrt(8.0) / 3.0, 1.0 / 3.0, 0},
             k, 0);
}

TEST(RaycastObstacles, TangentRaysCountAsHitsAndNearMissesDoNot) {
  const Scene scene(with(flat_scene(), {cylinder(-20, 10, 0, 3, 6)}));
  RayHit hit;
  EXPECT_TRUE(scene.raycast({0, 13.0, 3}, {-1, 0, 0}, 100.0, hit));     // tangent
  EXPECT_TRUE(scene.raycast({0, 12.999, 3}, {-1, 0, 0}, 100.0, hit));   // just inside
  EXPECT_FALSE(scene.raycast({0, 13.001, 3}, {-1, 0, 0}, 100.0, hit));  // just outside
}

TEST(RaycastObstacles, NearestOfTerrainAndObstacleWins) {
  const Scene scene(with(flat_scene(), {box(0, 0, 0, 5, 5, 6)}));
  RayHit hit;
  // From above the box: the roof (z = 6) is hit before the ground.
  ASSERT_TRUE(scene.raycast({0, 0, 20}, {0, 0, -1}, 100.0, hit));
  EXPECT_EQ(hit.kind, HitKind::kBox);
  EXPECT_NEAR(hit.distance, 14.0, 1e-9);
  // From above ground beside it: the ground.
  ASSERT_TRUE(scene.raycast({30, 0, 20}, {0, 0, -1}, 100.0, hit));
  EXPECT_EQ(hit.kind, HitKind::kTerrain);
  EXPECT_EQ(hit.obstacle_index, -1);
}

TEST(RaycastObstacles, AnObstacleHiddenBehindARidgeIsNotHit) {
  SceneData data = flat_scene();
  auto& t = data.terrain;
  for (std::size_t j = 0; j < t.samples_y(); ++j) {
    for (std::size_t i = 0; i < t.samples_x(); ++i) {
      const double x = t.origin_x + static_cast<double>(i) * t.cell_size;
      if (std::abs(x) <= 4.0) {
        t.heights[t.index(i, j)] = 20.0;  // a 20 m ridge across the middle
      }
    }
  }
  data.obstacles = {box(30, 0, 0, 3, 3, 10), box(-30, 0, 0, 3, 3, 10)};
  const Scene scene(data);
  RayHit hit;

  // Toward the box behind the ridge: the ridge is hit first.
  ASSERT_TRUE(scene.raycast({-15, 0, 3}, {1, 0, 0}, 200.0, hit));
  EXPECT_EQ(hit.kind, HitKind::kTerrain);
  EXPECT_LT(hit.distance, 20.0);
  // Away from the ridge toward the box in front of it: the box.
  ASSERT_TRUE(scene.raycast({-15, 0, 3}, {-1, 0, 0}, 200.0, hit));
  EXPECT_EQ(hit.kind, HitKind::kBox);
  EXPECT_EQ(hit.obstacle_index, 1);
  EXPECT_NEAR(hit.distance, 12.0, 1e-9);
}

TEST(RaycastObstacles, ALargeObstacleSpanningManyCellsIsFoundFromEveryApproach) {
  const Scene scene(with(flat_scene(200.0, 4.0), {box(0, 0, 0, 20, 20, 10)}));
  RayHit hit;
  const Vec3 targets[] = {{0, 0, 5}, {15, -12, 5}, {-19, 19, 9}, {19.5, 0, 0.5}};
  const Vec3 origins[] = {{80, 0, 5}, {-80, 10, 6}, {5, 90, 8}, {0, -90, 4}, {60, 60, 40}};
  for (const Vec3& target : targets) {
    for (const Vec3& origin : origins) {
      const Vec3 dir = swarm::normalized(target - origin);
      ASSERT_TRUE(scene.raycast(origin, dir, 500.0, hit));
      ASSERT_EQ(hit.kind, HitKind::kBox);
      ASSERT_EQ(hit.obstacle_index, 0);
    }
  }
}

TEST(RaycastObstacles, AnObstacleIgnoresRaysThatStartInsideIt) {
  const Scene scene(with(flat_scene(), {box(10, 5, 0, 2, 3, 4), cylinder(-20, 10, 0, 3, 6)}));
  RayHit hit;
  EXPECT_FALSE(scene.raycast({10, 5, 2}, {1, 0, 0}, 100.0, hit));    // inside the box, level
  EXPECT_FALSE(scene.raycast({-20, 10, 3}, {0, 1, 0}, 100.0, hit));  // inside the cylinder, level
  ASSERT_TRUE(scene.raycast({10, 5, 2}, {0, 0, -1}, 100.0, hit));    // inside the box, down
  EXPECT_EQ(hit.kind, HitKind::kTerrain);
  EXPECT_NEAR(hit.distance, 2.0, 1e-9);
}

TEST(RaycastObstacles, ObstaclesBehindTheRayAreNotHit) {
  const Scene scene(with(flat_scene(), {box(30, 0, 0, 2, 2, 6)}));
  RayHit hit;
  EXPECT_FALSE(scene.raycast({0, 0, 3}, {-1, 0, 0}, 100.0, hit));
  EXPECT_FALSE(scene.raycast({0, 0, 3}, {1, 0, 0}, 20.0, hit));  // beyond max range
  EXPECT_TRUE(scene.raycast({0, 0, 3}, {1, 0, 0}, 30.0, hit));
}

TEST(RaycastObstacles, PrimitiveFunctionsHonorTheirContractDirectly) {
  PrimitiveHit hit;
  const Obstacle b = box(0, 0, 0, 1, 1, 2);
  EXPECT_TRUE(swarm::intersect_box(b, {5, 0, 1}, {-1, 0, 0}, 10.0, hit));
  EXPECT_NEAR(hit.t, 4.0, 1e-12);
  EXPECT_FALSE(swarm::intersect_box(b, {5, 3, 1}, {-1, 0, 0}, 10.0, hit));  // passes beside
  EXPECT_FALSE(swarm::intersect_box(b, {5, 0, 5}, {-1, 0, 0}, 10.0, hit));  // passes above
  EXPECT_TRUE(swarm::intersect_box(b, {5, 1, 1}, {-1, 0, 0}, 10.0, hit));   // grazes a side face
  const Obstacle c = cylinder(0, 0, 0, 1, 2);
  EXPECT_TRUE(swarm::intersect_cylinder(c, {5, 0, 1}, {-1, 0, 0}, 10.0, hit));
  EXPECT_NEAR(hit.t, 4.0, 1e-12);
  EXPECT_FALSE(swarm::intersect_cylinder(c, {5, 0, 3}, {-1, 0, 0}, 10.0, hit));  // above the top
}

TEST(RaycastObstacles, AgreesWithBruteForceOnRandomRaysOverHillsWithManyObstacles) {
  swarm::TerrainConfig tc;
  tc.size_x = 400.0;
  tc.size_y = 400.0;
  tc.cell_size = 4.0;
  tc.type = swarm::TerrainType::kHills;
  tc.amplitude = 25.0;
  tc.feature_scale = 90.0;
  SceneData data = swarm::generate_terrain(tc, 8);
  swarm::ObstacleConfig oc;
  oc.density_per_m2 = 0.0019;
  oc.min_spacing = 2.0;
  oc.spawn_clear_radius = 0.0;
  swarm::place_obstacles(data, oc, 8);
  ASSERT_GT(data.obstacles.size(), 250U);
  const Scene scene(data);

  swarm::Rng rng(77, 0);
  int obstacle_hits = 0;
  int terrain_hits = 0;
  int misses = 0;
  const int rays = 50000;
  for (int n = 0; n < rays; ++n) {
    Vec3 origin{rng.uniform(-200.0, 200.0), rng.uniform(-200.0, 200.0), rng.uniform(-5.0, 50.0)};
    Vec3 dir;
    if (n % 2 == 0) {
      dir = swarm::normalized(Vec3{rng.normal(), rng.normal(), rng.normal()});
    } else {
      // Aim at a random obstacle so a good share of the rays reach one.
      const Obstacle& target =
          data.obstacles[rng.below(static_cast<std::uint32_t>(data.obstacles.size()))];
      const Vec3 aim{target.cx + rng.uniform(-target.half_x, target.half_x),
                     target.cy + rng.uniform(-target.half_x, target.half_x),
                     target.base_z + rng.uniform(0.0, target.height)};
      origin = aim + swarm::normalized(Vec3{rng.normal(), rng.normal(), rng.uniform(-0.3, 1.0)}) *
                         rng.uniform(15.0, 80.0);
      dir = swarm::normalized(aim - origin);
    }
    const double range = rng.uniform(20.0, 300.0);

    const auto want = swarm::testing::brute_force_scene(data, origin, dir, range);
    RayHit hit;
    const bool got = scene.raycast(origin, dir, range, hit);
    ASSERT_EQ(got, want.hit) << "ray " << n;
    if (!want.hit) {
      ++misses;
      continue;
    }
    ASSERT_NEAR(hit.distance, want.distance, 1e-7) << "ray " << n;
    ASSERT_EQ(hit.kind != HitKind::kTerrain, want.is_obstacle) << "ray " << n;
    if (want.is_obstacle) {
      ASSERT_EQ(hit.obstacle_index, want.obstacle_index) << "ray " << n;
      ++obstacle_hits;
    } else {
      ++terrain_hits;
    }
    ASSERT_LE(swarm::dot(hit.normal, dir), 1e-12);
  }
  EXPECT_GT(obstacle_hits, 5000);
  EXPECT_GT(terrain_hits, 3000);
  EXPECT_GT(misses, 2000);
}

TEST(RaycastObstacles, ResultsDoNotDependOnTheInternalCellSize) {
  // The same geometry on terrains with different cell sizes must give identical answers.
  SceneData reference = flat_scene(200.0, 4.0);
  swarm::ObstacleConfig oc;
  oc.density_per_m2 = 0.002;
  oc.spawn_clear_radius = 0.0;
  swarm::place_obstacles(reference, oc, 5);
  ASSERT_GT(reference.obstacles.size(), 50U);

  std::vector<Scene> scenes;
  for (double cell : {2.0, 4.0, 8.0, 20.0}) {
    SceneData s = flat_scene(200.0, cell);
    s.obstacles = reference.obstacles;
    scenes.emplace_back(s);
  }

  swarm::Rng rng(6, 0);
  for (int n = 0; n < 20000; ++n) {
    const Vec3 origin{rng.uniform(-95.0, 95.0), rng.uniform(-95.0, 95.0), rng.uniform(0.5, 30.0)};
    const Vec3 dir = swarm::normalized(Vec3{rng.normal(), rng.normal(), rng.normal() - 0.3});
    RayHit first;
    const bool first_hit = scenes[0].raycast(origin, dir, 150.0, first);
    for (std::size_t s = 1; s < scenes.size(); ++s) {
      RayHit other;
      const bool other_hit = scenes[s].raycast(origin, dir, 150.0, other);
      ASSERT_EQ(first_hit, other_hit) << "ray " << n << " scene " << s;
      if (first_hit) {
        ASSERT_NEAR(first.distance, other.distance, 1e-9);
        ASSERT_EQ(first.kind, other.kind);
        ASSERT_EQ(first.obstacle_index, other.obstacle_index);
      }
    }
  }
}
