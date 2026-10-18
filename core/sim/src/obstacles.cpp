#include "swarm/obstacles.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "swarm/rng.hpp"
#include "swarm/streams.hpp"
#include "swarm/terrain.hpp"

namespace swarm {

namespace {

// Unit circle sampled every 22.5 degrees, as literals so no libm call is involved.
constexpr double kCircle[16][2] = {
    {1.0, 0.0},
    {0.9238795325112867, 0.3826834323650898},
    {0.7071067811865476, 0.7071067811865476},
    {0.3826834323650898, 0.9238795325112867},
    {0.0, 1.0},
    {-0.3826834323650898, 0.9238795325112867},
    {-0.7071067811865476, 0.7071067811865476},
    {-0.9238795325112867, 0.3826834323650898},
    {-1.0, 0.0},
    {-0.9238795325112867, -0.3826834323650898},
    {-0.7071067811865476, -0.7071067811865476},
    {-0.3826834323650898, -0.9238795325112867},
    {0.0, -1.0},
    {0.3826834323650898, -0.9238795325112867},
    {0.7071067811865476, -0.7071067811865476},
    {0.9238795325112867, -0.3826834323650898},
};

bool finite_positive(double v) { return std::isfinite(v) && v > 0.0; }
bool finite_non_negative(double v) { return std::isfinite(v) && v >= 0.0; }

bool valid_range(double lo, double hi) {
  return finite_positive(lo) && std::isfinite(hi) && lo <= hi;
}

void validate_config(const ObstacleConfig& c) {
  if (!finite_non_negative(c.density_per_m2) || !finite_non_negative(c.min_spacing) ||
      !finite_non_negative(c.spawn_clear_radius) || !finite_non_negative(c.edge_margin) ||
      !finite_non_negative(c.embed)) {
    throw std::invalid_argument(
        "ObstacleConfig: densities, spacings, and margins must be finite "
        "and non-negative");
  }
  if (!(c.cylinder_fraction >= 0.0 && c.cylinder_fraction <= 1.0)) {
    throw std::invalid_argument("ObstacleConfig: cylinder_fraction must be in [0, 1]");
  }
  if (!valid_range(c.box_half_min, c.box_half_max) ||
      !valid_range(c.box_height_min, c.box_height_max) ||
      !valid_range(c.cylinder_radius_min, c.cylinder_radius_max) ||
      !valid_range(c.cylinder_height_min, c.cylinder_height_max)) {
    throw std::invalid_argument("ObstacleConfig: size ranges must be positive with min <= max");
  }
  if (!std::isfinite(c.spawn_x) || !std::isfinite(c.spawn_y)) {
    throw std::invalid_argument("ObstacleConfig: spawn point must be finite");
  }
  if (c.attempts_per_obstacle < 1) {
    throw std::invalid_argument("ObstacleConfig: attempts_per_obstacle must be at least 1");
  }
}

double bounding_radius(const Obstacle& o) {
  if (o.kind == ObstacleKind::kCylinder) {
    return o.half_x;
  }
  return std::sqrt(o.half_x * o.half_x + o.half_y * o.half_y);
}

// Lowest terrain height under the footprint: the center, points around the perimeter spaced at
// most a quarter cell apart, and every terrain vertex inside the footprint.
double lowest_footprint_height(const Heightfield& t, const Obstacle& o) {
  double lowest = height_at(t, o.cx, o.cy);
  const auto consider = [&](double x, double y) { lowest = std::min(lowest, height_at(t, x, y)); };

  if (o.kind == ObstacleKind::kCylinder) {
    for (const auto& p : kCircle) {
      consider(o.cx + p[0] * o.half_x, o.cy + p[1] * o.half_x);
    }
  } else {
    const double step = 0.25 * t.cell_size;
    const auto along = [&](double half) {
      const int n = std::max(1, static_cast<int>(std::ceil(2.0 * half / step)));
      return n;
    };
    const int nx = along(o.half_x);
    const int ny = along(o.half_y);
    for (int k = 0; k <= nx; ++k) {
      const double x = o.cx - o.half_x + 2.0 * o.half_x * static_cast<double>(k) / nx;
      consider(x, o.cy - o.half_y);
      consider(x, o.cy + o.half_y);
    }
    for (int k = 0; k <= ny; ++k) {
      const double y = o.cy - o.half_y + 2.0 * o.half_y * static_cast<double>(k) / ny;
      consider(o.cx - o.half_x, y);
      consider(o.cx + o.half_x, y);
    }
  }

  // Terrain vertices inside the footprint's bounding box and the footprint itself.
  const double ex = o.half_x;
  const double ey = o.kind == ObstacleKind::kCylinder ? o.half_x : o.half_y;
  const auto lo_i = static_cast<long long>(std::floor((o.cx - ex - t.origin_x) / t.cell_size));
  const auto hi_i = static_cast<long long>(std::ceil((o.cx + ex - t.origin_x) / t.cell_size));
  const auto lo_j = static_cast<long long>(std::floor((o.cy - ey - t.origin_y) / t.cell_size));
  const auto hi_j = static_cast<long long>(std::ceil((o.cy + ey - t.origin_y) / t.cell_size));
  for (long long j = std::max(0LL, lo_j); j <= std::min<long long>(hi_j, t.cells_y); ++j) {
    for (long long i = std::max(0LL, lo_i); i <= std::min<long long>(hi_i, t.cells_x); ++i) {
      const double x = t.origin_x + static_cast<double>(i) * t.cell_size;
      const double y = t.origin_y + static_cast<double>(j) * t.cell_size;
      const double dx = x - o.cx;
      const double dy = y - o.cy;
      const bool inside = o.kind == ObstacleKind::kCylinder
                              ? dx * dx + dy * dy <= o.half_x * o.half_x
                              : std::abs(dx) <= o.half_x && std::abs(dy) <= o.half_y;
      if (inside) {
        lowest = std::min(
            lowest, t.heights[t.index(static_cast<std::size_t>(i), static_cast<std::size_t>(j))]);
      }
    }
  }
  return lowest;
}

// Uniform grid of obstacle indices, so a candidate is only compared with nearby obstacles.
class SpatialGrid {
 public:
  SpatialGrid(double min_x, double min_y, double extent_x, double extent_y, double cell)
      : min_x_(min_x), min_y_(min_y), cell_(cell) {
    nx_ = static_cast<long long>(std::ceil(extent_x / cell)) + 1;
    ny_ = static_cast<long long>(std::ceil(extent_y / cell)) + 1;
    cells_.resize(static_cast<std::size_t>(nx_ * ny_));
  }

  void add(double x, double y, std::size_t index) {
    cells_[slot(clamp_x(x), clamp_y(y))].push_back(index);
  }

  // Calls fn(index) for every obstacle in the 3x3 block of cells around (x, y).
  template <typename Fn>
  void for_each_near(double x, double y, Fn&& fn) const {
    const long long cx = clamp_x(x);
    const long long cy = clamp_y(y);
    for (long long j = std::max(0LL, cy - 1); j <= std::min(cy + 1, ny_ - 1); ++j) {
      for (long long i = std::max(0LL, cx - 1); i <= std::min(cx + 1, nx_ - 1); ++i) {
        for (std::size_t index : cells_[slot(i, j)]) {
          fn(index);
        }
      }
    }
  }

 private:
  long long clamp_x(double x) const {
    return std::clamp(static_cast<long long>(std::floor((x - min_x_) / cell_)), 0LL, nx_ - 1);
  }
  long long clamp_y(double y) const {
    return std::clamp(static_cast<long long>(std::floor((y - min_y_) / cell_)), 0LL, ny_ - 1);
  }
  std::size_t slot(long long i, long long j) const { return static_cast<std::size_t>(j * nx_ + i); }

  double min_x_;
  double min_y_;
  double cell_;
  long long nx_ = 0;
  long long ny_ = 0;
  std::vector<std::vector<std::size_t>> cells_;
};

}  // namespace

void validate(const ObstacleConfig& config) { validate_config(config); }

std::size_t place_obstacles(SceneData& scene, const ObstacleConfig& config, std::uint64_t seed) {
  validate_config(config);
  std::string reason;
  if (!validate(scene, &reason)) {
    throw std::invalid_argument("place_obstacles: invalid scene: " + reason);
  }

  const Heightfield& t = scene.terrain;
  const double area = t.extent_x() * t.extent_y();
  const double target_real = std::floor(config.density_per_m2 * area + 0.5);
  if (!(target_real <= static_cast<double>(kMaxSceneObstacles))) {
    throw std::invalid_argument("place_obstacles: target count exceeds the obstacle limit");
  }
  const auto target = static_cast<std::size_t>(target_real);
  if (scene.obstacles.size() + target > kMaxSceneObstacles) {
    throw std::invalid_argument("place_obstacles: obstacle limit would be exceeded");
  }

  // Largest bounding radius that any new obstacle can have, for sizing the spatial grid. Existing
  // obstacles are compared by their own radius, so the 3x3 search must cover the largest of both.
  double max_radius = std::max(config.cylinder_radius_max, std::sqrt(2.0) * config.box_half_max);
  for (const Obstacle& o : scene.obstacles) {
    max_radius = std::max(max_radius, bounding_radius(o));
  }
  const double cell = 2.0 * max_radius + config.min_spacing;
  SpatialGrid grid(t.origin_x, t.origin_y, t.extent_x(), t.extent_y(), cell);
  for (std::size_t i = 0; i < scene.obstacles.size(); ++i) {
    grid.add(scene.obstacles[i].cx, scene.obstacles[i].cy, i);
  }

  Rng rng(seed, kStreamScene);
  const auto lerp_range = [](double lo, double hi, double u) { return lo + (hi - lo) * u; };

  std::size_t placed = 0;
  const std::uint64_t budget = static_cast<std::uint64_t>(target) * config.attempts_per_obstacle;
  for (std::uint64_t attempt = 0; placed < target && attempt < budget; ++attempt) {
    // Always six draws, accepted or not.
    const double ux = rng.uniform();
    const double uy = rng.uniform();
    const double uk = rng.uniform();
    const double ua = rng.uniform();
    const double ub = rng.uniform();
    const double uh = rng.uniform();

    Obstacle o;
    if (uk < config.cylinder_fraction) {
      o.kind = ObstacleKind::kCylinder;
      o.half_x = lerp_range(config.cylinder_radius_min, config.cylinder_radius_max, ua);
      o.half_y = 0.0;
      o.height = lerp_range(config.cylinder_height_min, config.cylinder_height_max, uh);
    } else {
      o.kind = ObstacleKind::kBox;
      o.half_x = lerp_range(config.box_half_min, config.box_half_max, ua);
      o.half_y = lerp_range(config.box_half_min, config.box_half_max, ub);
      o.height = lerp_range(config.box_height_min, config.box_height_max, uh);
    }
    const double radius = bounding_radius(o);

    const double min_x = t.origin_x + config.edge_margin + radius;
    const double max_x = t.origin_x + t.extent_x() - config.edge_margin - radius;
    const double min_y = t.origin_y + config.edge_margin + radius;
    const double max_y = t.origin_y + t.extent_y() - config.edge_margin - radius;
    if (!(min_x <= max_x) || !(min_y <= max_y)) {
      continue;  // this obstacle cannot fit inside the allowed area
    }
    o.cx = lerp_range(min_x, max_x, ux);
    o.cy = lerp_range(min_y, max_y, uy);

    const double sx = o.cx - config.spawn_x;
    const double sy = o.cy - config.spawn_y;
    const double clear = config.spawn_clear_radius + radius;
    if (sx * sx + sy * sy < clear * clear) {
      continue;
    }

    bool crowded = false;
    grid.for_each_near(o.cx, o.cy, [&](std::size_t index) {
      const Obstacle& other = scene.obstacles[index];
      const double dx = o.cx - other.cx;
      const double dy = o.cy - other.cy;
      const double gap = radius + bounding_radius(other) + config.min_spacing;
      if (dx * dx + dy * dy < gap * gap) {
        crowded = true;
      }
    });
    if (crowded) {
      continue;
    }

    o.base_z = lowest_footprint_height(t, o) - config.embed;
    grid.add(o.cx, o.cy, scene.obstacles.size());
    scene.obstacles.push_back(o);
    ++placed;
  }
  return placed;
}

}  // namespace swarm
