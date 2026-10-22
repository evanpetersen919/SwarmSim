#include "swarm/scene.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "swarm/geometry.hpp"
#include "swarm/terrain.hpp"

namespace swarm {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kBarycentricSlack = 1e-9;

// Moller-Trumbore, two-sided, with a small slack on the barycentric bounds so rays through shared
// edges and vertices cannot slip between triangles. Returns the ray parameter or -1 for no hit.
double intersect_triangle(const Vec3& origin, const Vec3& dir, const Vec3& v0, const Vec3& v1,
                          const Vec3& v2, double parallel_epsilon) {
  const Vec3 e1 = v1 - v0;
  const Vec3 e2 = v2 - v0;
  const Vec3 p = cross(dir, e2);
  const double det = dot(e1, p);
  if (std::abs(det) < parallel_epsilon) {
    return -1.0;
  }
  const double inv = 1.0 / det;
  const Vec3 s = origin - v0;
  const double u = dot(s, p) * inv;
  if (u < -kBarycentricSlack || u > 1.0 + kBarycentricSlack) {
    return -1.0;
  }
  const Vec3 q = cross(s, e1);
  const double v = dot(dir, q) * inv;
  if (v < -kBarycentricSlack || u + v > 1.0 + kBarycentricSlack) {
    return -1.0;
  }
  const double t = dot(e2, q) * inv;
  return t >= 0.0 ? t : -1.0;
}

}  // namespace

Scene::Scene(SceneData data) : data_(std::move(data)) {
  std::string reason;
  if (!validate(data_, &reason)) {
    throw std::invalid_argument("Scene: invalid scene data: " + reason);
  }

  // Register every obstacle in each terrain cell its footprint overlaps.
  const Heightfield& t = data_.terrain;
  const auto last_i = static_cast<long long>(t.cells_x) - 1;
  const auto last_j = static_cast<long long>(t.cells_y) - 1;
  const auto cell_of = [&](double v, double origin, long long last) {
    return std::clamp(static_cast<long long>(std::floor((v - origin) / t.cell_size)), 0LL, last);
  };
  cell_obstacles_.resize(static_cast<std::size_t>(t.cells_x) * t.cells_y);
  for (std::size_t n = 0; n < data_.obstacles.size(); ++n) {
    const Obstacle& o = data_.obstacles[n];
    const double ex = o.half_x;
    const double ey = o.kind == ObstacleKind::kCylinder ? o.half_x : o.half_y;
    const long long i0 = cell_of(o.cx - ex, t.origin_x, last_i);
    const long long i1 = cell_of(o.cx + ex, t.origin_x, last_i);
    const long long j0 = cell_of(o.cy - ey, t.origin_y, last_j);
    const long long j1 = cell_of(o.cy + ey, t.origin_y, last_j);
    for (long long j = j0; j <= j1; ++j) {
      for (long long i = i0; i <= i1; ++i) {
        cell_obstacles_[static_cast<std::size_t>(j) * t.cells_x + static_cast<std::size_t>(i)]
            .push_back(static_cast<std::uint32_t>(n));
      }
    }
  }
}

double Scene::ground_height(double x, double y) const { return height_at(data_.terrain, x, y); }

double Scene::surface_height(double x, double y) const {
  double h = ground_height(x, y);
  const Heightfield& t = data_.terrain;
  const double fi = std::floor((x - t.origin_x) / t.cell_size);
  const double fj = std::floor((y - t.origin_y) / t.cell_size);
  if (!(fi >= 0.0 && fj >= 0.0 && fi < static_cast<double>(t.cells_x) &&
        fj < static_cast<double>(t.cells_y))) {
    return h;  // outside the terrain's extent no obstacle exists
  }
  const auto cell = static_cast<std::size_t>(fj) * t.cells_x + static_cast<std::size_t>(fi);
  for (const std::uint32_t n : cell_obstacles_[cell]) {
    const Obstacle& o = data_.obstacles[n];
    const double dx = x - o.cx;
    const double dy = y - o.cy;
    const bool inside = o.kind == ObstacleKind::kCylinder
                            ? dx * dx + dy * dy <= o.half_x * o.half_x
                            : std::abs(dx) <= o.half_x && std::abs(dy) <= o.half_y;
    if (inside) {
      h = std::max(h, o.base_z + o.height);
    }
  }
  return h;
}

bool Scene::raycast(const Vec3& origin, const Vec3& direction, double max_range,
                    RayHit& hit) const {
  if (!(max_range > 0.0) || !std::isfinite(max_range) || !std::isfinite(origin.x) ||
      !std::isfinite(origin.y) || !std::isfinite(origin.z) || !std::isfinite(direction.x) ||
      !std::isfinite(direction.y) || !std::isfinite(direction.z)) {
    return false;
  }
  if (std::abs(norm_squared(direction) - 1.0) > 1e-9) {
    return false;
  }

  const Heightfield& terrain = data_.terrain;
  const double cs = terrain.cell_size;
  const double x_lo = terrain.origin_x;
  const double x_hi = terrain.origin_x + terrain.extent_x();
  const double y_lo = terrain.origin_y;
  const double y_hi = terrain.origin_y + terrain.extent_y();

  // Clip the ray to the terrain's xy extent.
  double t_enter = 0.0;
  double t_leave = max_range;
  const auto clip = [&](double o, double d, double lo, double hi) {
    if (d == 0.0) {
      return o >= lo && o <= hi;
    }
    double t1 = (lo - o) / d;
    double t2 = (hi - o) / d;
    if (t1 > t2) {
      std::swap(t1, t2);
    }
    t_enter = std::max(t_enter, t1);
    t_leave = std::min(t_leave, t2);
    return true;
  };
  if (!clip(origin.x, direction.x, x_lo, x_hi) || !clip(origin.y, direction.y, y_lo, y_hi) ||
      t_enter > t_leave) {
    return false;
  }

  // Walk the grid cells the clipped ray passes through, in order.
  const Vec3 start = origin + direction * t_enter;
  const auto last_i = static_cast<long long>(terrain.cells_x) - 1;
  const auto last_j = static_cast<long long>(terrain.cells_y) - 1;
  long long i = std::clamp(static_cast<long long>(std::floor((start.x - x_lo) / cs)), 0LL, last_i);
  long long j = std::clamp(static_cast<long long>(std::floor((start.y - y_lo) / cs)), 0LL, last_j);

  const int step_i = direction.x > 0.0 ? 1 : (direction.x < 0.0 ? -1 : 0);
  const int step_j = direction.y > 0.0 ? 1 : (direction.y < 0.0 ? -1 : 0);
  double t_max_x = kInf;
  double t_max_y = kInf;
  double t_delta_x = kInf;
  double t_delta_y = kInf;
  if (step_i != 0) {
    const double border = x_lo + static_cast<double>(i + (step_i > 0 ? 1 : 0)) * cs;
    t_max_x = (border - origin.x) / direction.x;
    t_delta_x = cs / std::abs(direction.x);
  }
  if (step_j != 0) {
    const double border = y_lo + static_cast<double>(j + (step_j > 0 ? 1 : 0)) * cs;
    t_max_y = (border - origin.y) / direction.y;
    t_delta_y = cs / std::abs(direction.y);
  }

  const double parallel_epsilon = 1e-12 * cs * cs;
  double best_t = kInf;
  Vec3 best_normal;
  HitKind best_kind = HitKind::kNone;
  int best_obstacle = -1;

  for (;;) {
    // Terrain triangles of cell (i, j): (a, b, c) and (a, c, d).
    const auto ci = static_cast<std::size_t>(i);
    const auto cj = static_cast<std::size_t>(j);
    const double x0 = x_lo + static_cast<double>(i) * cs;
    const double y0 = y_lo + static_cast<double>(j) * cs;
    const Vec3 a{x0, y0, terrain.heights[terrain.index(ci, cj)]};
    const Vec3 b{x0 + cs, y0, terrain.heights[terrain.index(ci + 1, cj)]};
    const Vec3 c{x0 + cs, y0 + cs, terrain.heights[terrain.index(ci + 1, cj + 1)]};
    const Vec3 d{x0, y0 + cs, terrain.heights[terrain.index(ci, cj + 1)]};

    const double t1 = intersect_triangle(origin, direction, a, b, c, parallel_epsilon);
    if (t1 >= 0.0 && t1 <= max_range && t1 < best_t) {
      best_t = t1;
      best_normal = normalized(cross(b - a, c - a));
      best_kind = HitKind::kTerrain;
      best_obstacle = -1;
    }
    const double t2 = intersect_triangle(origin, direction, a, c, d, parallel_epsilon);
    if (t2 >= 0.0 && t2 <= max_range && t2 < best_t) {
      best_t = t2;
      best_normal = normalized(cross(c - a, d - a));
      best_kind = HitKind::kTerrain;
      best_obstacle = -1;
    }

    // Obstacles overlapping this cell. A hit may lie beyond the cell, which is fine: the stopping
    // rule below only accepts a hit that is no farther than the end of the cells already walked.
    for (std::uint32_t index : cell_obstacles_[cj * terrain.cells_x + ci]) {
      const Obstacle& obstacle = data_.obstacles[index];
      PrimitiveHit primitive;
      const bool hit_obstacle =
          obstacle.kind == ObstacleKind::kCylinder
              ? intersect_cylinder(obstacle, origin, direction, max_range, primitive)
              : intersect_box(obstacle, origin, direction, max_range, primitive);
      if (hit_obstacle && primitive.t < best_t) {
        best_t = primitive.t;
        best_normal = primitive.normal;
        best_kind = obstacle.kind == ObstacleKind::kCylinder ? HitKind::kCylinder : HitKind::kBox;
        best_obstacle = static_cast<int>(index);
      }
    }

    const double t_exit = std::min(t_max_x, t_max_y);
    // Every hit in a later cell is at or beyond this cell's exit, so a hit inside this cell's
    // span is the nearest one.
    if (best_t <= t_exit || t_exit >= t_leave) {
      break;
    }
    if (t_max_x < t_max_y) {
      i += step_i;
      t_max_x += t_delta_x;
    } else {
      j += step_j;
      t_max_y += t_delta_y;
    }
    if (i < 0 || i > last_i || j < 0 || j > last_j) {
      break;
    }
  }

  if (!(best_t <= max_range)) {
    return false;
  }
  hit.distance = best_t;
  hit.point = origin + direction * best_t;
  hit.normal = dot(best_normal, direction) > 0.0 ? -best_normal : best_normal;
  hit.kind = best_kind;
  hit.obstacle_index = best_obstacle;
  return true;
}

}  // namespace swarm
