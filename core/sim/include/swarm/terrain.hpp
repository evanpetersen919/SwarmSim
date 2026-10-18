#pragma once

#include <cstdint>

#include "swarm/format/scene.hpp"

namespace swarm {

enum class TerrainType {
  kField,  // exactly flat at height 0
  kHills,  // fractal gradient noise
};

struct TerrainConfig {
  double size_x = 800.0;  // meters; rounded up to a whole number of cells
  double size_y = 800.0;
  double cell_size = 4.0;  // meters
  TerrainType type = TerrainType::kField;
  double amplitude = 30.0;       // meters; hills stay within [-amplitude, +amplitude]
  double feature_scale = 200.0;  // meters; wavelength of the first octave
  std::uint32_t octaves = 4;     // 1 to 16
  double persistence = 0.5;      // amplitude ratio between successive octaves, in (0, 1]
};

// Throws std::invalid_argument if the configuration is invalid; generate_terrain checks the same.
void validate(const TerrainConfig& config);

// Builds a terrain-only scene centered on the origin. Throws std::invalid_argument for an invalid
// configuration. Deterministic from (config, seed), and independent of the platform: it uses
// integer hashing and basic arithmetic only, never libm.
SceneData generate_terrain(const TerrainConfig& config, std::uint64_t seed);

// 2D gradient noise, zero at integer lattice points and within [-1, 1]. Exposed for tests.
double gradient_noise(std::uint64_t seed, double x, double y);

// Height of the triangulated terrain surface at (x, y), using exactly the triangle layout
// documented in swarm/format/scene.hpp. Outside the grid, the nearest point on the grid boundary
// is used. Returns NaN if x or y is not finite. The heightfield must be valid.
double height_at(const Heightfield& terrain, double x, double y);

}  // namespace swarm
