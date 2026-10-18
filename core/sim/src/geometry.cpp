#include "swarm/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace swarm {

bool intersect_box(const Obstacle& box, const Vec3& origin, const Vec3& direction, double max_range,
                   PrimitiveHit& hit) {
  const double lo[3] = {box.cx - box.half_x, box.cy - box.half_y, box.base_z};
  const double hi[3] = {box.cx + box.half_x, box.cy + box.half_y, box.base_z + box.height};
  const double o[3] = {origin.x, origin.y, origin.z};
  const double d[3] = {direction.x, direction.y, direction.z};

  double t_near = -std::numeric_limits<double>::infinity();
  double t_far = std::numeric_limits<double>::infinity();
  int entry_axis = 0;
  for (int axis = 0; axis < 3; ++axis) {
    if (d[axis] == 0.0) {
      if (o[axis] < lo[axis] || o[axis] > hi[axis]) {
        return false;  // parallel to this slab and outside it
      }
      continue;
    }
    double t1 = (lo[axis] - o[axis]) / d[axis];
    double t2 = (hi[axis] - o[axis]) / d[axis];
    if (t1 > t2) {
      std::swap(t1, t2);
    }
    if (t1 > t_near) {
      t_near = t1;
      entry_axis = axis;
    }
    t_far = std::min(t_far, t2);
  }

  // t_near < 0 means the origin is inside the box or the box is behind the ray.
  if (t_near < 0.0 || t_near > t_far || t_near > max_range) {
    return false;
  }

  Vec3 normal;
  const double sign = d[entry_axis] > 0.0 ? -1.0 : 1.0;
  if (entry_axis == 0) {
    normal = {sign, 0.0, 0.0};
  } else if (entry_axis == 1) {
    normal = {0.0, sign, 0.0};
  } else {
    normal = {0.0, 0.0, sign};
  }
  hit.t = t_near;
  hit.normal = normal;
  return true;
}

bool intersect_cylinder(const Obstacle& cylinder, const Vec3& origin, const Vec3& direction,
                        double max_range, PrimitiveHit& hit) {
  const double r = cylinder.half_x;
  const double z0 = cylinder.base_z;
  const double z1 = cylinder.base_z + cylinder.height;
  const double ox = origin.x - cylinder.cx;
  const double oy = origin.y - cylinder.cy;

  // A ray that starts inside the solid does not hit it.
  if (ox * ox + oy * oy < r * r && origin.z > z0 && origin.z < z1) {
    return false;
  }

  bool found = false;
  double best_t = max_range;
  Vec3 best_normal;
  const auto consider = [&](double t, const Vec3& normal) {
    if (t >= 0.0 && t <= best_t) {
      best_t = t;
      best_normal = normal;
      found = true;
    }
  };

  // Side wall: the nearest root of |xy|^2 = r^2, accepted if it lies between the caps.
  const double a = direction.x * direction.x + direction.y * direction.y;
  if (a > 0.0) {
    const double b = 2.0 * (ox * direction.x + oy * direction.y);
    const double c = ox * ox + oy * oy - r * r;
    const double disc = b * b - 4.0 * a * c;
    if (disc >= 0.0) {
      const double root = std::sqrt(disc);
      const double t = (-b - root) / (2.0 * a);
      const double z = origin.z + t * direction.z;
      if (t >= 0.0 && z >= z0 && z <= z1) {
        const double px = ox + t * direction.x;
        const double py = oy + t * direction.y;
        consider(t, {px / r, py / r, 0.0});
      }
    }
  }

  // Caps: entered from above through the top disc or from below through the bottom disc.
  if (direction.z != 0.0) {
    const auto cap = [&](double z, double normal_z) {
      const double t = (z - origin.z) / direction.z;
      const double px = ox + t * direction.x;
      const double py = oy + t * direction.y;
      if (px * px + py * py <= r * r) {
        consider(t, {0.0, 0.0, normal_z});
      }
    };
    if (direction.z < 0.0 && origin.z >= z1) {
      cap(z1, 1.0);
    } else if (direction.z > 0.0 && origin.z <= z0) {
      cap(z0, -1.0);
    }
  }

  if (!found) {
    return false;
  }
  hit.t = best_t;
  hit.normal = best_normal;
  return true;
}

}  // namespace swarm
