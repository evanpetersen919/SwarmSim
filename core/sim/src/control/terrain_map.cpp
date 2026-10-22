#include "swarm/control/terrain_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace swarm {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite(double v) { return std::isfinite(v); }

}  // namespace

TerrainMap::TerrainMap(std::uint32_t cells_x, std::uint32_t cells_y, double origin_x,
                       double origin_y, double cell_size, std::vector<double> heights)
    : cells_x_(cells_x),
      cells_y_(cells_y),
      origin_x_(origin_x),
      origin_y_(origin_y),
      cell_size_(cell_size),
      heights_(std::move(heights)) {
  if (cells_x_ == 0 || cells_y_ == 0) {
    throw std::invalid_argument("TerrainMap: the grid needs at least one cell on each axis");
  }
  if (!finite(origin_x_) || !finite(origin_y_) || !finite(cell_size_) || !(cell_size_ > 0.0)) {
    throw std::invalid_argument("TerrainMap: origin must be finite and cell_size positive");
  }
  const std::size_t expected =
      (static_cast<std::size_t>(cells_x_) + 1U) * (static_cast<std::size_t>(cells_y_) + 1U);
  if (heights_.size() != expected) {
    throw std::invalid_argument("TerrainMap: heights must hold (cells_x + 1) * (cells_y + 1)");
  }
  for (double h : heights_) {
    if (!finite(h)) {
      throw std::invalid_argument("TerrainMap: heights must be finite");
    }
  }
}

double TerrainMap::max_height_near(double x, double y, double radius) const {
  if (!finite(x) || !finite(y) || !finite(radius) || radius < 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // The cells that the disc touches, as sample index ranges: the samples at the corners of those
  // cells. Clamped to the grid, which is the nearest-boundary rule for a disc outside it.
  const auto lo = [&](double v, double origin, std::uint32_t cells) {
    const double i = std::floor((v - radius - origin) / cell_size_);
    return static_cast<std::int64_t>(std::clamp(i, 0.0, static_cast<double>(cells)));
  };
  const auto hi = [&](double v, double origin, std::uint32_t cells) {
    const double i = std::ceil((v + radius - origin) / cell_size_);
    return static_cast<std::int64_t>(std::clamp(i, 0.0, static_cast<double>(cells)));
  };
  const std::int64_t i0 = lo(x, origin_x_, cells_x_);
  const std::int64_t i1 = hi(x, origin_x_, cells_x_);
  const std::int64_t j0 = lo(y, origin_y_, cells_y_);
  const std::int64_t j1 = hi(y, origin_y_, cells_y_);
  const std::size_t stride = static_cast<std::size_t>(cells_x_) + 1U;
  double best = -std::numeric_limits<double>::infinity();
  for (std::int64_t j = j0; j <= j1; ++j) {
    for (std::int64_t i = i0; i <= i1; ++i) {
      best = std::max(best,
                      heights_[static_cast<std::size_t>(j) * stride + static_cast<std::size_t>(i)]);
    }
  }
  return best;
}

void validate(const TerrainGuidanceParams& p) {
  if (!finite(p.clearance) || !finite(p.footprint) || !finite(p.climb_rate) ||
      !finite(p.climb_fraction) || !finite(p.sample_spacing) || !finite(p.lookahead)) {
    throw std::invalid_argument("TerrainGuidanceParams: values must be finite");
  }
  if (p.clearance < 0.0 || p.footprint < 0.0) {
    throw std::invalid_argument(
        "TerrainGuidanceParams: clearance and footprint must not be negative");
  }
  if (!(p.climb_rate > 0.0) || !(p.sample_spacing > 0.0) || !(p.lookahead > 0.0)) {
    throw std::invalid_argument(
        "TerrainGuidanceParams: climb_rate, sample_spacing, and lookahead must be positive");
  }
  if (!(p.climb_fraction > 0.0) || p.climb_fraction > 1.0) {
    throw std::invalid_argument("TerrainGuidanceParams: climb_fraction must be in (0, 1]");
  }
}

double terrain_floor(const TerrainMap& map, const TerrainGuidanceParams& params,
                     const std::vector<PathSample>& path, double speed) {
  validate(params);
  if (!finite(speed) || !(speed > 0.0)) {
    throw std::invalid_argument("terrain_floor: speed must be positive and finite");
  }
  const double budget = params.climb_fraction * params.climb_rate;
  double floor = -std::numeric_limits<double>::infinity();
  for (const PathSample& s : path) {
    const double ground = map.max_height_near(s.x, s.y, params.footprint);
    floor = std::max(floor, ground + params.clearance - budget * s.distance / speed);
  }
  return floor;
}

std::vector<PathSample> straight_path(const Vec3& position, double dir_x, double dir_y,
                                      const TerrainGuidanceParams& params) {
  validate(params);
  std::vector<PathSample> path;
  path.push_back({position.x, position.y, 0.0});
  const double norm = std::hypot(dir_x, dir_y);
  if (!(norm > 0.0) || !finite(norm)) {
    return path;
  }
  const double ux = dir_x / norm;
  const double uy = dir_y / norm;
  for (double d = params.sample_spacing; d <= params.lookahead; d += params.sample_spacing) {
    path.push_back({position.x + ux * d, position.y + uy * d, d});
  }
  return path;
}

std::vector<PathSample> circle_path(const Vec3& position, double cx, double cy, double radius,
                                    const TerrainGuidanceParams& params) {
  validate(params);
  if (!finite(radius) || !(radius > 0.0)) {
    throw std::invalid_argument("circle_path: radius must be positive and finite");
  }
  std::vector<PathSample> path;
  const double angle0 = std::atan2(position.y - cy, position.x - cx);
  const double to_circle = std::abs(std::hypot(position.x - cx, position.y - cy) - radius);
  const double lap = 2.0 * kPi * radius;
  for (double d = 0.0; d <= lap; d += params.sample_spacing) {
    const double a = angle0 + d / radius;
    path.push_back({cx + radius * std::cos(a), cy + radius * std::sin(a), to_circle + d});
  }
  return path;
}

}  // namespace swarm
