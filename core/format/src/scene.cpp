#include "swarm/format/scene.hpp"

#include <cmath>
#include <cstring>
#include <utility>

#include "byte_io.hpp"
#include "swarm/format/hash.hpp"

namespace swarm {

namespace {

constexpr std::uint8_t kMagic[4] = {'S', 'W', 'S', 'C'};
constexpr std::size_t kFixedHeaderBytes = 4 + 4 + 8 + 8 + 8 + 4;  // after magic and version
constexpr std::size_t kObstacleBytes = 1 + 6 * 8;

using detail::get_f64;
using detail::get_u32;
using detail::put_f64;
using detail::put_u32;
using detail::put_u8;

bool fail(std::string* reason, const char* message) {
  if (reason != nullptr) {
    *reason = message;
  }
  return false;
}

std::uint64_t sample_count(const Heightfield& t) {
  return static_cast<std::uint64_t>(t.cells_x + 1ULL) * (t.cells_y + 1ULL);
}

}  // namespace

const char* to_string(SceneStatus status) {
  switch (status) {
    case SceneStatus::kOk:
      return "ok";
    case SceneStatus::kBadMagic:
      return "bad magic";
    case SceneStatus::kUnsupportedVersion:
      return "unsupported version";
    case SceneStatus::kTruncated:
      return "truncated";
    case SceneStatus::kCorrupt:
      return "corrupt";
    case SceneStatus::kLimitExceeded:
      return "limit exceeded";
    case SceneStatus::kInvalid:
      return "invalid scene";
  }
  return "unknown";
}

bool validate(const SceneData& scene, std::string* reason) {
  const Heightfield& t = scene.terrain;
  if (t.cells_x < 1 || t.cells_y < 1) {
    return fail(reason, "terrain needs at least one cell in each direction");
  }
  if (t.cells_x > kMaxSceneCellsPerAxis || t.cells_y > kMaxSceneCellsPerAxis) {
    return fail(reason, "terrain has too many cells");
  }
  if (!std::isfinite(t.origin_x) || !std::isfinite(t.origin_y)) {
    return fail(reason, "terrain origin is not finite");
  }
  if (!std::isfinite(t.cell_size) || !(t.cell_size > 0.0)) {
    return fail(reason, "cell size must be finite and positive");
  }
  if (t.heights.size() != sample_count(t)) {
    return fail(reason, "heights array does not match the grid size");
  }
  for (double h : t.heights) {
    if (!std::isfinite(h)) {
      return fail(reason, "a terrain height is not finite");
    }
  }
  if (scene.obstacles.size() > kMaxSceneObstacles) {
    return fail(reason, "too many obstacles");
  }
  for (const Obstacle& o : scene.obstacles) {
    if (o.kind != ObstacleKind::kBox && o.kind != ObstacleKind::kCylinder) {
      return fail(reason, "unknown obstacle kind");
    }
    if (!std::isfinite(o.cx) || !std::isfinite(o.cy) || !std::isfinite(o.base_z) ||
        !std::isfinite(o.height) || !std::isfinite(o.half_x) || !std::isfinite(o.half_y)) {
      return fail(reason, "an obstacle field is not finite");
    }
    if (!(o.height > 0.0) || !(o.half_x > 0.0)) {
      return fail(reason, "obstacle height and size must be positive");
    }
    if (o.kind == ObstacleKind::kBox && !(o.half_y > 0.0)) {
      return fail(reason, "box half extents must be positive");
    }
  }
  return true;
}

std::uint64_t scene_hash(const SceneData& scene) {
  const Heightfield& t = scene.terrain;
  Fnv1a64 hash;
  hash.update_u64(kSceneVersion);
  hash.update_u64(t.cells_x);
  hash.update_u64(t.cells_y);
  hash.update_f64(t.origin_x);
  hash.update_f64(t.origin_y);
  hash.update_f64(t.cell_size);
  hash.update_u64(t.heights.size());
  for (double h : t.heights) {
    hash.update_f64(h);
  }
  hash.update_u64(scene.obstacles.size());
  for (const Obstacle& o : scene.obstacles) {
    hash.update_u64(static_cast<std::uint64_t>(o.kind));
    hash.update_f64(o.cx);
    hash.update_f64(o.cy);
    hash.update_f64(o.base_z);
    hash.update_f64(o.height);
    hash.update_f64(o.half_x);
    hash.update_f64(o.half_y);
  }
  return hash.digest();
}

SceneStatus serialize_scene(const SceneData& scene, std::vector<std::uint8_t>& out) {
  if (!validate(scene)) {
    return SceneStatus::kInvalid;
  }
  const Heightfield& t = scene.terrain;
  std::vector<std::uint8_t> bytes;
  bytes.reserve(4 + 4 + kFixedHeaderBytes + t.heights.size() * 8 +
                scene.obstacles.size() * kObstacleBytes);
  bytes.insert(bytes.end(), kMagic, kMagic + sizeof(kMagic));
  put_u32(bytes, kSceneVersion);
  put_u32(bytes, t.cells_x);
  put_u32(bytes, t.cells_y);
  put_f64(bytes, t.origin_x);
  put_f64(bytes, t.origin_y);
  put_f64(bytes, t.cell_size);
  put_u32(bytes, static_cast<std::uint32_t>(scene.obstacles.size()));
  for (double h : t.heights) {
    put_f64(bytes, h);
  }
  for (const Obstacle& o : scene.obstacles) {
    put_u8(bytes, static_cast<std::uint8_t>(o.kind));
    put_f64(bytes, o.cx);
    put_f64(bytes, o.cy);
    put_f64(bytes, o.base_z);
    put_f64(bytes, o.height);
    put_f64(bytes, o.half_x);
    put_f64(bytes, o.half_y);
  }
  out = std::move(bytes);
  return SceneStatus::kOk;
}

SceneStatus parse_scene(const std::uint8_t* data, std::size_t size, SceneData& out) {
  std::size_t pos = 0;
  const auto has = [&](std::size_t count) { return size - pos >= count; };

  if (!has(4)) {
    return SceneStatus::kTruncated;
  }
  if (std::memcmp(data, kMagic, sizeof(kMagic)) != 0) {
    return SceneStatus::kBadMagic;
  }
  pos += 4;

  if (!has(4)) {
    return SceneStatus::kTruncated;
  }
  if (get_u32(data + pos) != kSceneVersion) {
    return SceneStatus::kUnsupportedVersion;
  }
  pos += 4;

  if (!has(kFixedHeaderBytes)) {
    return SceneStatus::kTruncated;
  }
  SceneData scene;
  Heightfield& t = scene.terrain;
  t.cells_x = get_u32(data + pos);
  t.cells_y = get_u32(data + pos + 4);
  t.origin_x = get_f64(data + pos + 8);
  t.origin_y = get_f64(data + pos + 16);
  t.cell_size = get_f64(data + pos + 24);
  const std::uint32_t obstacle_count = get_u32(data + pos + 32);
  pos += kFixedHeaderBytes;

  if (t.cells_x > kMaxSceneCellsPerAxis || t.cells_y > kMaxSceneCellsPerAxis ||
      obstacle_count > kMaxSceneObstacles) {
    return SceneStatus::kLimitExceeded;
  }

  const std::uint64_t samples = sample_count(t);
  const std::uint64_t body = samples * 8ULL + static_cast<std::uint64_t>(obstacle_count) *
                                                  static_cast<std::uint64_t>(kObstacleBytes);
  const std::uint64_t remaining = size - pos;
  if (remaining < body) {
    return SceneStatus::kTruncated;
  }
  if (remaining > body) {
    return SceneStatus::kCorrupt;
  }

  t.heights.resize(static_cast<std::size_t>(samples));
  for (double& h : t.heights) {
    h = get_f64(data + pos);
    pos += 8;
  }
  scene.obstacles.resize(obstacle_count);
  for (Obstacle& o : scene.obstacles) {
    const std::uint8_t kind = data[pos];
    if (kind > static_cast<std::uint8_t>(ObstacleKind::kCylinder)) {
      return SceneStatus::kCorrupt;
    }
    o.kind = static_cast<ObstacleKind>(kind);
    o.cx = get_f64(data + pos + 1);
    o.cy = get_f64(data + pos + 9);
    o.base_z = get_f64(data + pos + 17);
    o.height = get_f64(data + pos + 25);
    o.half_x = get_f64(data + pos + 33);
    o.half_y = get_f64(data + pos + 41);
    pos += kObstacleBytes;
  }

  if (!validate(scene)) {
    return SceneStatus::kCorrupt;
  }
  out = std::move(scene);
  return SceneStatus::kOk;
}

}  // namespace swarm
