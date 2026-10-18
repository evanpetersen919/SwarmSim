#pragma once

#include "swarm/format/scene.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

struct PrimitiveHit {
  double t = 0.0;  // distance along the ray
  Vec3 normal;     // unit length, faces the ray origin
};

// Ray against a solid axis-aligned box obstacle (kind kBox). `direction` must be a unit vector.
//
// Reports the point where the ray enters the solid, if that is within [0, max_range]. A ray that
// starts inside the solid does not hit it. A ray that grazes an edge or corner exactly counts as
// a hit.
bool intersect_box(const Obstacle& box, const Vec3& origin, const Vec3& direction, double max_range,
                   PrimitiveHit& hit);

// Same contract for a vertical cylinder (kind kCylinder, radius in half_x). A tangent ray to the
// side counts as a hit.
bool intersect_cylinder(const Obstacle& cylinder, const Vec3& origin, const Vec3& direction,
                        double max_range, PrimitiveHit& hit);

}  // namespace swarm
