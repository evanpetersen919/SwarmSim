#pragma once

// Reference implementations for checking the accelerated ray casting. They walk every triangle
// (and every obstacle) instead of using the grid, and use an independent triangle test.

#include <cmath>
#include <cstddef>

#include "swarm/format/scene.hpp"
#include "swarm/geometry.hpp"
#include "swarm/vec3.hpp"

namespace swarm::testing {

// Nearest hit on the terrain by testing every triangle with a plane intersection followed by a
// barycentric point-in-triangle test. Returns true and sets `best_distance` on a hit.
inline bool brute_force_terrain(const Heightfield& t, const Vec3& o, const Vec3& d,
                                double max_range, double& best_distance) {
  bool found = false;
  best_distance = max_range;
  const double cs = t.cell_size;
  const auto test = [&](const Vec3& p0, const Vec3& p1, const Vec3& p2) {
    const Vec3 n = cross(p1 - p0, p2 - p0);
    const double denom = dot(n, d);
    if (std::abs(denom) < 1e-14) {
      return;
    }
    const double dist = dot(n, p0 - o) / denom;
    if (dist < 0.0 || dist > best_distance) {
      return;
    }
    const Vec3 p = o + d * dist;
    const Vec3 v0 = p1 - p0;
    const Vec3 v1 = p2 - p0;
    const Vec3 v2 = p - p0;
    const double d00 = dot(v0, v0);
    const double d01 = dot(v0, v1);
    const double d11 = dot(v1, v1);
    const double d20 = dot(v2, v0);
    const double d21 = dot(v2, v1);
    const double inv = 1.0 / (d00 * d11 - d01 * d01);
    const double bv = (d11 * d20 - d01 * d21) * inv;
    const double bw = (d00 * d21 - d01 * d20) * inv;
    if (bv >= -1e-9 && bw >= -1e-9 && bv + bw <= 1.0 + 1e-9) {
      best_distance = dist;
      found = true;
    }
  };
  for (std::size_t j = 0; j < t.cells_y; ++j) {
    for (std::size_t i = 0; i < t.cells_x; ++i) {
      const double x0 = t.origin_x + static_cast<double>(i) * cs;
      const double y0 = t.origin_y + static_cast<double>(j) * cs;
      const Vec3 a{x0, y0, t.heights[t.index(i, j)]};
      const Vec3 b{x0 + cs, y0, t.heights[t.index(i + 1, j)]};
      const Vec3 c{x0 + cs, y0 + cs, t.heights[t.index(i + 1, j + 1)]};
      const Vec3 dd{x0, y0 + cs, t.heights[t.index(i, j + 1)]};
      test(a, b, c);
      test(a, c, dd);
    }
  }
  return found;
}

struct BruteHit {
  bool hit = false;
  double distance = 0.0;
  bool is_obstacle = false;
  int obstacle_index = -1;
};

// Nearest of the terrain and every obstacle, tried exhaustively.
inline BruteHit brute_force_scene(const SceneData& scene, const Vec3& o, const Vec3& d,
                                  double max_range) {
  BruteHit best;
  double terrain_distance = 0.0;
  if (brute_force_terrain(scene.terrain, o, d, max_range, terrain_distance)) {
    best.hit = true;
    best.distance = terrain_distance;
  }
  for (std::size_t n = 0; n < scene.obstacles.size(); ++n) {
    const Obstacle& obstacle = scene.obstacles[n];
    PrimitiveHit ph;
    const bool hit = obstacle.kind == ObstacleKind::kCylinder
                         ? intersect_cylinder(obstacle, o, d, max_range, ph)
                         : intersect_box(obstacle, o, d, max_range, ph);
    if (hit && (!best.hit || ph.t < best.distance)) {
      best.hit = true;
      best.distance = ph.t;
      best.is_obstacle = true;
      best.obstacle_index = static_cast<int>(n);
    }
  }
  return best;
}

}  // namespace swarm::testing
