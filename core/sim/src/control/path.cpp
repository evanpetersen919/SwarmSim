#include "swarm/control/path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swarm {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// s taken into [0, length).
double wrap_length(double s, double length) {
  double w = std::fmod(s, length);
  if (w < 0.0) {
    w += length;
  }
  return w;
}

}  // namespace

void validate(const PatternParams& p) {
  const auto finite_value = [](double v) { return std::isfinite(v); };
  if (!finite_value(p.heading) || !finite_value(p.leg_length) || !finite_value(p.turn_radius) ||
      !finite_value(p.lane_spacing) || !finite_value(p.stagger_s) ||
      !finite_value(p.join_fraction)) {
    throw std::invalid_argument("PatternParams: values must be finite");
  }
  if (p.leg_length < 0.0 || !(p.turn_radius > 0.0) || !(p.lane_spacing > 0.0) ||
      p.stagger_s < 0.0 || !(p.join_fraction > 0.0)) {
    throw std::invalid_argument(
        "PatternParams: leg_length and stagger_s must not be negative; turn_radius, "
        "lane_spacing, and join_fraction must be positive");
  }
}

StadiumPath::StadiumPath(double cx, double cy, double heading, double leg_length, double radius,
                         int direction)
    : cx_(cx),
      cy_(cy),
      ux_(std::cos(heading)),
      uy_(std::sin(heading)),
      px_(-std::sin(heading) * direction),
      py_(std::cos(heading) * direction),
      leg_(leg_length),
      radius_(radius),
      length_(2.0 * leg_length + 2.0 * kPi * radius),
      direction_(direction) {
  if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(heading) ||
      !std::isfinite(leg_length) || !std::isfinite(radius)) {
    throw std::invalid_argument("StadiumPath: values must be finite");
  }
  if (leg_length < 0.0 || !(radius > 0.0)) {
    throw std::invalid_argument("StadiumPath: leg_length must not be negative, radius positive");
  }
  if (direction != 1 && direction != -1) {
    throw std::invalid_argument("StadiumPath: direction must be +1 or -1");
  }
}

StadiumPath::Point StadiumPath::at(double s_in) const {
  const double s = wrap_length(s_in, length_);
  const double half = 0.5 * leg_;
  const double arc = kPi * radius_;
  // The point in the frame (u, p): `a` along the straights, `b` across.
  double a = 0.0;
  double b = 0.0;
  double ta = 1.0;  // tangent in the same frame
  double tb = 0.0;
  double curvature = 0.0;
  if (s < leg_) {
    a = -half + s;
    b = -radius_;
  } else if (s < leg_ + arc) {
    const double phi = -0.5 * kPi + (s - leg_) / radius_;
    a = half + radius_ * std::cos(phi);
    b = radius_ * std::sin(phi);
    ta = -std::sin(phi);
    tb = std::cos(phi);
    curvature = static_cast<double>(direction_) / radius_;
  } else if (s < 2.0 * leg_ + arc) {
    a = half - (s - leg_ - arc);
    b = radius_;
    ta = -1.0;
    tb = 0.0;
  } else {
    const double phi = 0.5 * kPi + (s - 2.0 * leg_ - arc) / radius_;
    a = -half + radius_ * std::cos(phi);
    b = radius_ * std::sin(phi);
    ta = -std::sin(phi);
    tb = std::cos(phi);
    curvature = static_cast<double>(direction_) / radius_;
  }
  Point p;
  p.x = cx_ + a * ux_ + b * px_;
  p.y = cy_ + a * uy_ + b * py_;
  p.tangent_x = ta * ux_ + tb * px_;
  p.tangent_y = ta * uy_ + tb * py_;
  p.curvature = curvature;
  p.s = s;
  return p;
}

StadiumPath::Nearest StadiumPath::nearest(double x, double y) const {
  const double half = 0.5 * leg_;
  const double arc = kPi * radius_;
  const double a = (x - cx_) * ux_ + (y - cy_) * uy_;
  const double b = (x - cx_) * px_ + (y - cy_) * py_;

  double best_s = 0.0;
  double best_d = std::numeric_limits<double>::infinity();
  const auto consider = [&](double s, double d) {
    if (d < best_d) {
      best_d = d;
      best_s = s;
    }
  };

  // The straights.
  const double along = std::clamp(a, -half, half);
  consider(along + half, std::hypot(a - along, b + radius_));
  consider(leg_ + arc + (half - along), std::hypot(a - along, b - radius_));

  // A half circle around (center_a, 0) whose angle runs from phi0 to phi0 + pi.
  const auto arc_nearest = [&](double center_a, double phi0, double s0) {
    const double va = a - center_a;
    const double vb = b;
    const double r = std::hypot(va, vb);
    // The angle of the query from the arc's center, taken into [phi0, phi0 + 2 pi).
    double psi = r > 1e-12 ? std::atan2(vb, va) : phi0 + 0.5 * kPi;
    psi = phi0 + wrap_length(psi - phi0, 2.0 * kPi);
    double phi = psi;
    if (psi > phi0 + kPi) {
      // Beyond the end of the arc: the nearer end is the nearer endpoint.
      const double to_end = psi - (phi0 + kPi);
      const double to_start = 2.0 * kPi - (psi - phi0);
      phi = to_end < to_start ? phi0 + kPi : phi0;
    }
    const double pa = center_a + radius_ * std::cos(phi);
    const double pb = radius_ * std::sin(phi);
    consider(s0 + radius_ * (phi - phi0), std::hypot(a - pa, b - pb));
  };
  arc_nearest(half, -0.5 * kPi, leg_);
  arc_nearest(-half, 0.5 * kPi, 2.0 * leg_ + arc);

  Nearest out;
  out.point = at(best_s);
  out.distance = best_d;
  // Positive when the query is to the left of the path, in the world frame.
  const double nx = -out.point.tangent_y;
  const double ny = out.point.tangent_x;
  out.cross_track = (x - out.point.x) * nx + (y - out.point.y) * ny;
  return out;
}

StadiumPath pattern_path(std::uint32_t slot, std::uint32_t n, const PatternParams& params,
                         const OrbitParams& orbit) {
  if (n == 0 || slot >= n) {
    throw std::invalid_argument("pattern_path: slot must be below the number of drones");
  }
  validate(params);
  validate(orbit);
  switch (params.kind) {
    case PatternKind::kRacetrack: {
      const double radius = params.turn_radius + static_cast<double>(slot % 2U) * orbit.radius_step;
      return StadiumPath(orbit.center.x, orbit.center.y, params.heading, params.leg_length, radius,
                         +1);
    }
    case PatternKind::kLanes: {
      const double offset =
          (static_cast<double>(slot) - 0.5 * static_cast<double>(n - 1)) * params.lane_spacing;
      const double cx = orbit.center.x - std::sin(params.heading) * offset;
      const double cy = orbit.center.y + std::cos(params.heading) * offset;
      return StadiumPath(cx, cy, params.heading, params.leg_length, params.turn_radius,
                         slot % 2U == 0U ? +1 : -1);
    }
    case PatternKind::kOrbit:
      break;
  }
  throw std::invalid_argument("pattern_path: the orbit pattern has no stadium path");
}

PatternReference pattern_reference(std::uint32_t slot, std::uint32_t n, double time_s,
                                   const PatternParams& params, const OrbitParams& orbit) {
  if (!std::isfinite(time_s)) {
    throw std::invalid_argument("pattern_reference: time must be finite");
  }
  const StadiumPath own = pattern_path(slot, n, params, orbit);
  const StadiumPath first = pattern_path(0, n, params, orbit);
  // The same loop time for every slot, so the spacing along the loops does not drift.
  const double period = first.length() / orbit.speed;
  PatternReference ref;
  ref.speed = own.length() / period;
  const double start = params.stagger_s > 0.0
                           ? -static_cast<double>(slot) * params.stagger_s * ref.speed
                           : static_cast<double>(slot) * own.length() / static_cast<double>(n);
  const StadiumPath::Point p = own.at(start + ref.speed * time_s);
  ref.s = p.s;
  ref.position = {p.x, p.y, orbit.center.z + static_cast<double>(slot) * orbit.altitude_step};
  ref.velocity = {ref.speed * p.tangent_x, ref.speed * p.tangent_y, 0.0};
  return ref;
}

Vec3 path_guidance(const Vec3& position, const Vec3& velocity, const StadiumPath& path,
                   double speed, double z_ref, double phase_error, const PatternParams& params,
                   const OrbitParams& orbit, double gain_scale) {
  if (!finite(position) || !finite(velocity) || !std::isfinite(z_ref) ||
      !std::isfinite(phase_error) || !std::isfinite(gain_scale) || gain_scale < 0.0) {
    throw std::invalid_argument(
        "path_guidance: inputs must be finite and the gain scale not negative");
  }
  if (!(std::isfinite(speed) && speed > 0.0)) {
    throw std::invalid_argument("path_guidance: speed must be positive");
  }
  validate(params);
  validate(orbit);

  const StadiumPath::Nearest near = path.nearest(position.x, position.y);
  const double nx = -near.point.tangent_y;
  const double ny = near.point.tangent_x;
  const bool joining = near.distance > params.join_fraction * path.radius();
  const double phase = joining ? 0.0 : phase_error;

  const double cut = std::clamp(-orbit.radial_gain * near.cross_track / path.radius(), -1.0, 1.0);
  double dx = near.point.tangent_x + cut * nx;
  double dy = near.point.tangent_y + cut * ny;
  const double dn = std::hypot(dx, dy);
  dx /= dn;
  dy /= dn;

  const double desired =
      std::clamp(speed + orbit.phase_gain * phase, orbit.min_speed + orbit.speed_margin,
                 orbit.max_speed - orbit.speed_margin);
  const double centripetal = desired * desired * near.point.curvature;
  const double kv = orbit.velocity_gain * gain_scale;

  Vec3 a;
  a.x = centripetal * nx + kv * (desired * dx - velocity.x);
  a.y = centripetal * ny + kv * (desired * dy - velocity.y);
  a.z = gain_scale *
        (orbit.altitude_gain * (z_ref - position.z) - orbit.altitude_damping * velocity.z);
  return a;
}

double path_phase_error(const StadiumPath& path, const Vec3& position, double s_ref) {
  if (!finite(position) || !std::isfinite(s_ref)) {
    throw std::invalid_argument("path_phase_error: inputs must be finite");
  }
  const StadiumPath::Nearest near = path.nearest(position.x, position.y);
  double d = std::fmod(s_ref - near.point.s, path.length());
  if (d > 0.5 * path.length()) {
    d -= path.length();
  } else if (d <= -0.5 * path.length()) {
    d += path.length();
  }
  return d;
}

std::vector<PathSample> path_samples(const StadiumPath& path, const Vec3& position,
                                     const TerrainGuidanceParams& params) {
  validate(params);
  const StadiumPath::Nearest near = path.nearest(position.x, position.y);
  std::vector<PathSample> out;
  for (double d = 0.0; d <= path.length(); d += params.sample_spacing) {
    const StadiumPath::Point p = path.at(near.point.s + d);
    out.push_back({p.x, p.y, near.distance + d});
  }
  return out;
}

}  // namespace swarm
