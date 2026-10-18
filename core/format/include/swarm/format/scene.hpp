#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// Scene geometry as plain data: a terrain heightfield plus box and cylinder obstacles. The
// simulation generates it, the LiDAR casts rays against it, and the Unreal plugin rebuilds the
// same scene from it. This header is compiled into the plugin, which builds without C++
// exceptions, so errors are status codes and the API works on in-memory buffers.
//
// Coordinates follow the conventions in docs/MODELS.md: meters, +z up.
//
// Terrain triangulation (every consumer must use exactly this): cell (i, j) has corners
//   a = (i, j), b = (i+1, j), c = (i+1, j+1), d = (i, j+1)
// and is split into the triangles (a, b, c) and (a, c, d). Heights are never interpolated
// bilinearly, because a bilinear surface and this mesh disagree inside a cell.

namespace swarm {

inline constexpr std::uint32_t kSceneVersion = 1;
inline constexpr std::uint32_t kMaxSceneCellsPerAxis = 4096;
inline constexpr std::uint32_t kMaxSceneObstacles = 1000000;

enum class ObstacleKind : std::uint8_t {
  kBox = 0,       // axis-aligned in x and y; half extents half_x, half_y
  kCylinder = 1,  // vertical axis; radius stored in half_x, half_y unused
};

// Solid from base_z up to base_z + height.
struct Obstacle {
  ObstacleKind kind = ObstacleKind::kBox;
  double cx = 0.0;
  double cy = 0.0;
  double base_z = 0.0;
  double height = 0.0;
  double half_x = 0.0;
  double half_y = 0.0;
};

// Regular grid of height samples. Sample (i, j) sits at
// (origin_x + i * cell_size, origin_y + j * cell_size).
struct Heightfield {
  std::uint32_t cells_x = 0;
  std::uint32_t cells_y = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double cell_size = 1.0;
  std::vector<double> heights;  // (cells_x + 1) * (cells_y + 1), x varying fastest

  std::size_t samples_x() const { return static_cast<std::size_t>(cells_x) + 1U; }
  std::size_t samples_y() const { return static_cast<std::size_t>(cells_y) + 1U; }
  std::size_t index(std::size_t i, std::size_t j) const { return j * samples_x() + i; }
  double extent_x() const { return static_cast<double>(cells_x) * cell_size; }
  double extent_y() const { return static_cast<double>(cells_y) * cell_size; }
};

struct SceneData {
  Heightfield terrain;
  std::vector<Obstacle> obstacles;
};

enum class SceneStatus {
  kOk,
  kBadMagic,
  kUnsupportedVersion,
  kTruncated,
  kCorrupt,        // well-formed container, invalid contents
  kLimitExceeded,  // a count is larger than the format allows
  kInvalid,        // serialize() was given an invalid scene
};

const char* to_string(SceneStatus status);

// Structural and numeric checks: sizes agree, everything is finite, cell size and obstacle
// dimensions are positive, counts are within limits. On failure, `reason` (if given) says why.
bool validate(const SceneData& scene, std::string* reason = nullptr);

// FNV-1a over every field. Equal scenes hash equal; any change to a height, obstacle, or grid
// parameter changes the hash.
std::uint64_t scene_hash(const SceneData& scene);

// Layout, little-endian:
//   "SWSC" | u32 version | u32 cells_x | u32 cells_y | f64 origin_x | f64 origin_y | f64 cell_size
//   | u32 obstacle_count | heights (f64 each) | obstacles
// Obstacle: u8 kind | f64 cx | f64 cy | f64 base_z | f64 height | f64 half_x | f64 half_y
SceneStatus serialize_scene(const SceneData& scene, std::vector<std::uint8_t>& out);
SceneStatus parse_scene(const std::uint8_t* data, std::size_t size, SceneData& out);

}  // namespace swarm
