#include "swarm/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "swarm/streams.hpp"

namespace swarm {

namespace {

// splitmix64 finalizer: a well-mixed 64-bit hash of a 64-bit value.
std::uint64_t mix(std::uint64_t z) {
  z += 0x9E3779B97F4A7C15ULL;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

// Gradient at a lattice point: one of eight fixed directions chosen by hash.
void gradient_at(std::uint64_t seed, std::int64_t ix, std::int64_t iy, double& gx, double& gy) {
  static constexpr double kGradients[8][2] = {{1, 1}, {-1, 1}, {1, -1}, {-1, -1},
                                              {1, 0}, {-1, 0}, {0, 1},  {0, -1}};
  const std::uint64_t h =
      mix(mix(mix(seed) + static_cast<std::uint64_t>(ix)) + static_cast<std::uint64_t>(iy));
  const auto& g = kGradients[h & 7U];
  gx = g[0];
  gy = g[1];
}

double fade(double t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }

double lerp(double a, double b, double t) { return a + t * (b - a); }

bool positive_finite(double v) { return std::isfinite(v) && v > 0.0; }

void validate_config(const TerrainConfig& c) {
  if (!positive_finite(c.size_x) || !positive_finite(c.size_y) || !positive_finite(c.cell_size)) {
    throw std::invalid_argument("TerrainConfig: sizes and cell size must be finite and positive");
  }
  if (c.cell_size > c.size_x || c.cell_size > c.size_y) {
    throw std::invalid_argument("TerrainConfig: cell size is larger than the terrain");
  }
  if (std::ceil(c.size_x / c.cell_size) > kMaxSceneCellsPerAxis ||
      std::ceil(c.size_y / c.cell_size) > kMaxSceneCellsPerAxis) {
    throw std::invalid_argument("TerrainConfig: too many cells; increase cell_size");
  }
  if (c.type == TerrainType::kHills) {
    if (!std::isfinite(c.amplitude) || c.amplitude < 0.0) {
      throw std::invalid_argument("TerrainConfig: amplitude must be finite and non-negative");
    }
    if (!positive_finite(c.feature_scale)) {
      throw std::invalid_argument("TerrainConfig: feature_scale must be finite and positive");
    }
    if (c.octaves < 1 || c.octaves > 16) {
      throw std::invalid_argument("TerrainConfig: octaves must be between 1 and 16");
    }
    if (!(c.persistence > 0.0 && c.persistence <= 1.0)) {
      throw std::invalid_argument("TerrainConfig: persistence must be in (0, 1]");
    }
  }
}

}  // namespace

double gradient_noise(std::uint64_t seed, double x, double y) {
  const double fx = std::floor(x);
  const double fy = std::floor(y);
  const auto ix = static_cast<std::int64_t>(fx);
  const auto iy = static_cast<std::int64_t>(fy);
  const double dx = x - fx;
  const double dy = y - fy;

  const auto corner = [&](std::int64_t cx, std::int64_t cy, double ox, double oy) {
    double gx = 0.0;
    double gy = 0.0;
    gradient_at(seed, cx, cy, gx, gy);
    return gx * ox + gy * oy;
  };

  const double n00 = corner(ix, iy, dx, dy);
  const double n10 = corner(ix + 1, iy, dx - 1.0, dy);
  const double n01 = corner(ix, iy + 1, dx, dy - 1.0);
  const double n11 = corner(ix + 1, iy + 1, dx - 1.0, dy - 1.0);

  const double u = fade(dx);
  const double v = fade(dy);
  return lerp(lerp(n00, n10, u), lerp(n01, n11, u), v);
}

double height_at(const Heightfield& t, double x, double y) {
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double max_u = static_cast<double>(t.cells_x);
  const double max_v = static_cast<double>(t.cells_y);
  const double gu = std::clamp((x - t.origin_x) / t.cell_size, 0.0, max_u);
  const double gv = std::clamp((y - t.origin_y) / t.cell_size, 0.0, max_v);

  const double cell_i = std::min(std::floor(gu), max_u - 1.0);
  const double cell_j = std::min(std::floor(gv), max_v - 1.0);
  const double u = gu - cell_i;
  const double v = gv - cell_j;
  const auto i = static_cast<std::size_t>(cell_i);
  const auto j = static_cast<std::size_t>(cell_j);

  const double h00 = t.heights[t.index(i, j)];
  const double h10 = t.heights[t.index(i + 1, j)];
  const double h01 = t.heights[t.index(i, j + 1)];
  const double h11 = t.heights[t.index(i + 1, j + 1)];

  if (u >= v) {  // triangle (a, b, c)
    return h00 + u * (h10 - h00) + v * (h11 - h10);
  }
  return h00 + u * (h11 - h01) + v * (h01 - h00);  // triangle (a, c, d)
}

SceneData generate_terrain(const TerrainConfig& config, std::uint64_t seed) {
  validate_config(config);

  SceneData scene;
  Heightfield& t = scene.terrain;
  t.cells_x = static_cast<std::uint32_t>(std::ceil(config.size_x / config.cell_size));
  t.cells_y = static_cast<std::uint32_t>(std::ceil(config.size_y / config.cell_size));
  t.cell_size = config.cell_size;
  t.origin_x = -0.5 * t.extent_x();
  t.origin_y = -0.5 * t.extent_y();
  t.heights.assign(t.samples_x() * t.samples_y(), 0.0);

  if (config.type == TerrainType::kField) {
    return scene;
  }

  const std::uint64_t base_seed = mix(seed ^ kStreamScene);
  // Sum of octave amplitudes, accumulated by repeated multiplication (no libm pow), so the
  // normalization keeps every height within +-amplitude.
  double amplitude_sum = 0.0;
  double octave_amplitude = 1.0;
  for (std::uint32_t o = 0; o < config.octaves; ++o) {
    amplitude_sum += octave_amplitude;
    octave_amplitude *= config.persistence;
  }

  for (std::size_t j = 0; j < t.samples_y(); ++j) {
    for (std::size_t i = 0; i < t.samples_x(); ++i) {
      const double x = t.origin_x + static_cast<double>(i) * t.cell_size;
      const double y = t.origin_y + static_cast<double>(j) * t.cell_size;
      double sum = 0.0;
      double amp = 1.0;
      double frequency = 1.0 / config.feature_scale;
      for (std::uint32_t o = 0; o < config.octaves; ++o) {
        sum += amp * gradient_noise(mix(base_seed + o), x * frequency, y * frequency);
        amp *= config.persistence;
        frequency *= 2.0;
      }
      const double normalized = std::clamp(sum / amplitude_sum, -1.0, 1.0);
      t.heights[t.index(i, j)] = config.amplitude * normalized + 0.0;  // + 0.0 maps -0.0 to +0.0
    }
  }
  return scene;
}

}  // namespace swarm
