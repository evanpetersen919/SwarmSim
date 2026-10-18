#pragma once

#include <cstdint>
#include <vector>

#include "swarm/format/scene.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

enum class HitKind : std::uint8_t { kNone, kTerrain, kBox, kCylinder };

struct RayHit {
  double distance = 0.0;
  Vec3 point;
  Vec3 normal;  // unit length, faces the ray origin
  HitKind kind = HitKind::kNone;
  int obstacle_index = -1;  // index into SceneData::obstacles, or -1 for terrain
};

// What sensors see of the world. Implementations must be safe to query from many threads at once.
class IScene {
 public:
  virtual ~IScene() = default;

  // Finds the nearest intersection along the ray within [0, max_range]. `direction` must be a unit
  // vector (within 1e-9 in squared length); otherwise, and for non-finite input or a non-positive
  // range, there is no hit. Returns true and fills `hit` on a hit.
  virtual bool raycast(const Vec3& origin, const Vec3& direction, double max_range,
                       RayHit& hit) const = 0;

  // Terrain height at (x, y), clamped to the terrain boundary outside it.
  virtual double ground_height(double x, double y) const = 0;
};

// Terrain and obstacles from a SceneData, with a spatial index for ray casting.
//
// Obstacles are solid: a ray that starts inside one ignores it. Geometry outside the terrain's
// xy extent does not exist, so obstacles should lie within it.
//
// Terrain is the triangulated surface described in swarm/format/scene.hpp and is two-sided: a ray
// that starts below the surface hits it from below. Rays are clipped to the terrain's xy extent;
// nothing exists outside it. Queries are const and use no caches, so they are thread-safe.
class Scene : public IScene {
 public:
  // Throws std::invalid_argument if `data` fails validate().
  explicit Scene(SceneData data);

  const SceneData& data() const { return data_; }

  bool raycast(const Vec3& origin, const Vec3& direction, double max_range,
               RayHit& hit) const override;
  double ground_height(double x, double y) const override;

 private:
  SceneData data_;
  // For each terrain cell (row-major), the indices of obstacles whose footprint overlaps it.
  std::vector<std::vector<std::uint32_t>> cell_obstacles_;
};

}  // namespace swarm
