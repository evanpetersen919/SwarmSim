#pragma once

#include <cmath>

namespace swarm {

// Plain 3-vector of doubles. Deliberately minimal and dependency-free; matrix work arrives with
// the estimators in a later phase.
struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

constexpr Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
constexpr Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
constexpr Vec3 operator-(Vec3 a) { return {-a.x, -a.y, -a.z}; }
constexpr Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
constexpr Vec3 operator*(double s, Vec3 a) { return a * s; }
constexpr Vec3 operator/(Vec3 a, double s) { return {a.x / s, a.y / s, a.z / s}; }

inline Vec3& operator+=(Vec3& a, Vec3 b) { return a = a + b; }
inline Vec3& operator-=(Vec3& a, Vec3 b) { return a = a - b; }
inline Vec3& operator*=(Vec3& a, double s) { return a = a * s; }

// Exact comparison, intended for tests and hashing, not for tolerance checks.
constexpr bool operator==(Vec3 a, Vec3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
constexpr bool operator!=(Vec3 a, Vec3 b) { return !(a == b); }

constexpr double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
constexpr double norm_squared(Vec3 a) { return dot(a, a); }
inline double norm(Vec3 a) { return std::sqrt(norm_squared(a)); }
inline double distance(Vec3 a, Vec3 b) { return norm(a - b); }

// Unit vector in the direction of `a`, or the zero vector if `a` is zero.
inline Vec3 normalized(Vec3 a) {
  const double n = norm(a);
  return n > 0.0 ? a / n : Vec3{};
}

// Scales `a` down to length `max_norm` if it is longer, keeping its direction. A non-positive
// `max_norm` yields the zero vector.
inline Vec3 clamp_norm(Vec3 a, double max_norm) {
  if (!(max_norm > 0.0)) {
    return Vec3{};
  }
  const double n2 = norm_squared(a);
  if (n2 <= max_norm * max_norm) {
    return a;
  }
  return a * (max_norm / std::sqrt(n2));
}

}  // namespace swarm
