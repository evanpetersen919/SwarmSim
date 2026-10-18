#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "swarm/rng.hpp"
#include "swarm/terrain.hpp"

using swarm::SceneData;
using swarm::TerrainConfig;
using swarm::TerrainType;

namespace {

TerrainConfig hills(double amplitude = 30.0, double scale = 100.0, std::uint32_t octaves = 4) {
  TerrainConfig c;
  c.size_x = 400.0;
  c.size_y = 400.0;
  c.cell_size = 4.0;
  c.type = TerrainType::kHills;
  c.amplitude = amplitude;
  c.feature_scale = scale;
  c.octaves = octaves;
  return c;
}

// Sum of absolute differences between neighboring samples: a roughness measure.
double roughness(const swarm::Heightfield& t) {
  double sum = 0.0;
  for (std::size_t j = 0; j < t.samples_y(); ++j) {
    for (std::size_t i = 0; i + 1 < t.samples_x(); ++i) {
      sum += std::abs(t.heights[t.index(i + 1, j)] - t.heights[t.index(i, j)]);
    }
  }
  return sum;
}

// Sum of absolute second differences along x: a measure of high-frequency detail.
double curvature(const swarm::Heightfield& t) {
  double sum = 0.0;
  for (std::size_t j = 0; j < t.samples_y(); ++j) {
    for (std::size_t i = 1; i + 1 < t.samples_x(); ++i) {
      sum += std::abs(t.heights[t.index(i + 1, j)] - 2.0 * t.heights[t.index(i, j)] +
                      t.heights[t.index(i - 1, j)]);
    }
  }
  return sum;
}

// A heightfield sampling the plane z = 2 + 0.3 x - 0.1 y.
swarm::Heightfield planar_field() {
  swarm::Heightfield t;
  t.cells_x = 6;
  t.cells_y = 4;
  t.origin_x = -3.0;
  t.origin_y = 1.0;
  t.cell_size = 2.0;
  t.heights.resize(t.samples_x() * t.samples_y());
  for (std::size_t j = 0; j < t.samples_y(); ++j) {
    for (std::size_t i = 0; i < t.samples_x(); ++i) {
      const double x = t.origin_x + static_cast<double>(i) * t.cell_size;
      const double y = t.origin_y + static_cast<double>(j) * t.cell_size;
      t.heights[t.index(i, j)] = 2.0 + 0.3 * x - 0.1 * y;
    }
  }
  return t;
}

}  // namespace

TEST(Terrain, FieldIsExactlyFlat) {
  TerrainConfig c;
  c.type = TerrainType::kField;
  const SceneData s = swarm::generate_terrain(c, 99);
  for (double h : s.terrain.heights) {
    ASSERT_EQ(h, 0.0);
  }
  EXPECT_TRUE(s.obstacles.empty());
  EXPECT_TRUE(swarm::validate(s));
}

TEST(Terrain, GridDimensionsFollowTheExtentRoundingRule) {
  TerrainConfig c;
  c.size_x = 100.0;
  c.size_y = 60.0;
  c.cell_size = 10.0;
  SceneData s = swarm::generate_terrain(c, 1);
  EXPECT_EQ(s.terrain.cells_x, 10U);
  EXPECT_EQ(s.terrain.cells_y, 6U);
  EXPECT_EQ(s.terrain.heights.size(), 11U * 7U);
  EXPECT_DOUBLE_EQ(s.terrain.origin_x, -50.0);
  EXPECT_DOUBLE_EQ(s.terrain.origin_y, -30.0);

  c.size_x = 105.0;  // rounds up to 11 cells
  s = swarm::generate_terrain(c, 1);
  EXPECT_EQ(s.terrain.cells_x, 11U);
  EXPECT_DOUBLE_EQ(s.terrain.extent_x(), 110.0);
  EXPECT_DOUBLE_EQ(s.terrain.origin_x, -55.0);
}

TEST(GradientNoise, IsZeroAtEveryLatticePoint) {
  for (std::uint64_t seed : {0ULL, 1ULL, 12345ULL, 0xFFFFFFFFFFFFFFFFULL}) {
    for (int ix = -6; ix <= 6; ++ix) {
      for (int iy = -6; iy <= 6; ++iy) {
        ASSERT_EQ(swarm::gradient_noise(seed, ix, iy), 0.0);
      }
    }
  }
}

TEST(GradientNoise, StaysWithinUnitRangeAndIsNotConstant) {
  swarm::Rng rng(3, 0);
  double sum = 0.0;
  double sum_sq = 0.0;
  const int n = 200000;
  for (int i = 0; i < n; ++i) {
    const double x = rng.uniform(-50.0, 50.0);
    const double y = rng.uniform(-50.0, 50.0);
    const double v = swarm::gradient_noise(7, x, y);
    ASSERT_LE(std::abs(v), 1.0);
    sum += v;
    sum_sq += v * v;
  }
  const double mean = sum / n;
  const double sd = std::sqrt(sum_sq / n - mean * mean);
  EXPECT_NEAR(mean, 0.0, 0.01);
  EXPECT_GT(sd, 0.2);
}

TEST(GradientNoise, IsContinuous) {
  swarm::Rng rng(4, 0);
  for (int i = 0; i < 20000; ++i) {
    const double x = rng.uniform(-20.0, 20.0);
    const double y = rng.uniform(-20.0, 20.0);
    const double step = 1e-3;
    const double d =
        std::abs(swarm::gradient_noise(5, x + step, y) - swarm::gradient_noise(5, x, y));
    ASSERT_LE(d, 6.0 * step);
  }
}

TEST(GradientNoise, SeedChangesTheField) {
  int differing = 0;
  for (int i = 0; i < 100; ++i) {
    const double x = 0.37 + i * 0.91;
    const double y = 0.11 + i * 0.53;
    if (swarm::gradient_noise(1, x, y) != swarm::gradient_noise(2, x, y)) {
      ++differing;
    }
  }
  EXPECT_GT(differing, 90);
}

TEST(Terrain, HillsAreDeterministicAndSeedSensitive) {
  const SceneData a = swarm::generate_terrain(hills(), 42);
  const SceneData b = swarm::generate_terrain(hills(), 42);
  const SceneData c = swarm::generate_terrain(hills(), 43);
  EXPECT_EQ(swarm::scene_hash(a), swarm::scene_hash(b));
  EXPECT_NE(swarm::scene_hash(a), swarm::scene_hash(c));
  EXPECT_TRUE(swarm::validate(a));
}

TEST(Terrain, HillsStayWithinAmplitudeAndHaveSpread) {
  const SceneData s = swarm::generate_terrain(hills(25.0, 120.0, 5), 7);
  double sum = 0.0;
  double sum_sq = 0.0;
  for (double h : s.terrain.heights) {
    ASSERT_LE(std::abs(h), 25.0);
    sum += h;
    sum_sq += h * h;
  }
  const double n = static_cast<double>(s.terrain.heights.size());
  const double mean = sum / n;
  const double sd = std::sqrt(sum_sq / n - mean * mean);
  EXPECT_LT(std::abs(mean), 0.25 * 25.0);
  EXPECT_GT(sd, 0.05 * 25.0);
}

TEST(Terrain, ZeroAmplitudeHillsAreFlat) {
  const SceneData s = swarm::generate_terrain(hills(0.0), 5);
  for (double h : s.terrain.heights) {
    ASSERT_EQ(h, 0.0);
  }
}

TEST(Terrain, SmallerFeatureScaleIsRougher) {
  const double smooth = roughness(swarm::generate_terrain(hills(30.0, 400.0, 1), 11).terrain);
  const double rough = roughness(swarm::generate_terrain(hills(30.0, 40.0, 1), 11).terrain);
  EXPECT_GT(rough, 2.0 * smooth);
}

TEST(Terrain, MoreOctavesAddHighFrequencyDetail) {
  // Second differences respond to high-frequency detail. First differences do not, because the
  // octave amplitudes are normalized, so extra octaves also shrink the dominant low frequency.
  const double one = curvature(swarm::generate_terrain(hills(30.0, 200.0, 1), 11).terrain);
  const double six = curvature(swarm::generate_terrain(hills(30.0, 200.0, 6), 11).terrain);
  EXPECT_GT(six, 1.5 * one);
}

TEST(Terrain, GoldenHashPinsTheGeneratorAcrossPlatforms) {
  // Uses only integer hashing and basic IEEE arithmetic, so it must match on every compiler and
  // platform. If this fails after an intended change to the generator, update the value.
  TerrainConfig c;
  c.size_x = 64.0;
  c.size_y = 48.0;
  c.cell_size = 4.0;
  c.type = TerrainType::kHills;
  c.amplitude = 20.0;
  c.feature_scale = 30.0;
  c.octaves = 3;
  c.persistence = 0.5;
  EXPECT_EQ(swarm::scene_hash(swarm::generate_terrain(c, 2024)), 1908170542786555081ULL);
}

TEST(Terrain, InvalidConfigurationsAreRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto rejects = [](TerrainConfig c) {
    EXPECT_THROW(swarm::generate_terrain(c, 1), std::invalid_argument);
  };
  TerrainConfig c = hills();
  c.size_x = 0.0;
  rejects(c);
  c = hills();
  c.size_y = -5.0;
  rejects(c);
  c = hills();
  c.cell_size = 0.0;
  rejects(c);
  c = hills();
  c.cell_size = nan;
  rejects(c);
  c = hills();
  c.cell_size = 500.0;  // larger than the 400 m terrain
  rejects(c);
  c = hills();
  c.size_x = 10000.0;
  c.cell_size = 1.0;  // 10,000 cells exceeds the limit
  rejects(c);
  c = hills();
  c.octaves = 0;
  rejects(c);
  c = hills();
  c.octaves = 17;
  rejects(c);
  c = hills();
  c.persistence = 0.0;
  rejects(c);
  c = hills();
  c.persistence = 1.5;
  rejects(c);
  c = hills();
  c.amplitude = -1.0;
  rejects(c);
  c = hills();
  c.feature_scale = 0.0;
  rejects(c);
}

TEST(HeightAt, MatchesGridHeightsAtSamples) {
  const SceneData s = swarm::generate_terrain(hills(), 21);
  const auto& t = s.terrain;
  for (std::size_t j = 0; j < t.samples_y(); j += 7) {
    for (std::size_t i = 0; i < t.samples_x(); i += 7) {
      const double x = t.origin_x + static_cast<double>(i) * t.cell_size;
      const double y = t.origin_y + static_cast<double>(j) * t.cell_size;
      ASSERT_NEAR(swarm::height_at(t, x, y), t.heights[t.index(i, j)], 1e-12);
    }
  }
}

TEST(HeightAt, ReproducesAPlaneExactly) {
  const swarm::Heightfield t = planar_field();
  swarm::Rng rng(8, 0);
  for (int i = 0; i < 20000; ++i) {
    const double x = rng.uniform(t.origin_x, t.origin_x + t.extent_x());
    const double y = rng.uniform(t.origin_y, t.origin_y + t.extent_y());
    ASSERT_NEAR(swarm::height_at(t, x, y), 2.0 + 0.3 * x - 0.1 * y, 1e-12);
  }
}

TEST(HeightAt, IsContinuousAcrossCellBordersAndDiagonals) {
  const SceneData s = swarm::generate_terrain(hills(), 33);
  const auto& t = s.terrain;
  const double eps = 1e-9;
  swarm::Rng rng(2, 0);
  for (int n = 0; n < 5000; ++n) {
    const auto i = static_cast<double>(rng.below(t.cells_x - 1) + 1U);
    const auto j = static_cast<double>(rng.below(t.cells_y - 1) + 1U);
    const double x = t.origin_x + i * t.cell_size;
    const double y = t.origin_y + j * t.cell_size;
    // Across a vertical border, a horizontal border, and along the diagonal of a cell.
    ASSERT_NEAR(swarm::height_at(t, x - eps, y + 0.3), swarm::height_at(t, x + eps, y + 0.3), 1e-6);
    ASSERT_NEAR(swarm::height_at(t, x + 0.3, y - eps), swarm::height_at(t, x + 0.3, y + eps), 1e-6);
    const double d = rng.uniform(0.1, t.cell_size - 0.1);
    ASSERT_NEAR(swarm::height_at(t, x + d - eps, y + d + eps),
                swarm::height_at(t, x + d + eps, y + d - eps), 1e-6);
  }
}

TEST(HeightAt, ClampsToTheBoundaryOutsideTheGrid) {
  const swarm::Heightfield t = planar_field();
  const double x_max = t.origin_x + t.extent_x();
  const double y_max = t.origin_y + t.extent_y();
  EXPECT_NEAR(swarm::height_at(t, x_max + 50.0, 2.0), swarm::height_at(t, x_max, 2.0), 1e-12);
  EXPECT_NEAR(swarm::height_at(t, -100.0, y_max + 9.0), swarm::height_at(t, t.origin_x, y_max),
              1e-12);
  EXPECT_TRUE(std::isnan(swarm::height_at(t, std::nan(""), 0.0)));
  EXPECT_TRUE(std::isnan(swarm::height_at(t, 0.0, std::numeric_limits<double>::infinity())));
}
