#include "swarm/scene.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

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
}

double Scene::ground_height(double x, double y) const { return height_at(data_.terrain, x, y); }

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
    }
    const double t2 = intersect_triangle(origin, direction, a, c, d, parallel_epsilon);
    if (t2 >= 0.0 && t2 <= max_range && t2 < best_t) {
      best_t = t2;
      best_normal = normalized(cross(c - a, d - a));
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
  hit.kind = HitKind::kTerrain;
  hit.obstacle_index = -1;
  return true;
}

}  // namespace swarm
