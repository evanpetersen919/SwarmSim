#include "swarm/control/orbit.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swarm {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

double wrap(double angle) {
  double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
  if (wrapped <= 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

}  // namespace

void validate(const OrbitParams& p) {
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  const auto non_negative = [](double v) { return std::isfinite(v) && v >= 0.0; };
  if (!finite(p.center)) {
    throw std::invalid_argument("OrbitParams: center must be finite");
  }
  if (!positive(p.radius) || !positive(p.speed) || !positive(p.min_speed) ||
      !positive(p.max_speed) || !positive(p.max_accel) || !positive(p.lookahead_s)) {
    throw std::invalid_argument(
        "OrbitParams: radius, speeds, max_accel, and lookahead must be "
        "finite and positive");
  }
  if (!non_negative(p.radius_step) || !non_negative(p.altitude_step) ||
      !non_negative(p.speed_margin) || !non_negative(p.radial_gain) ||
      !non_negative(p.phase_gain) || !non_negative(p.velocity_gain) ||
      !non_negative(p.altitude_gain) || !non_negative(p.altitude_damping)) {
    throw std::invalid_argument(
        "OrbitParams: steps, margin, and gains must be finite and "
        "not negative");
  }
  if (!(p.min_speed + p.speed_margin < p.speed && p.speed < p.max_speed - p.speed_margin)) {
    throw std::invalid_argument(
        "OrbitParams: speed must be inside the speed limits by more than speed_margin");
  }
  if (!(std::isfinite(p.consensus_weight) && p.consensus_weight >= 0.0 &&
        p.consensus_weight <= 1.0) ||
      !positive(p.consensus_sigma_floor)) {
    throw std::invalid_argument(
        "OrbitParams: consensus_weight must be in [0, 1] and consensus_sigma_floor positive");
  }
  const SeparationParams& s = p.separation;
  if (!positive(s.d_safe) || !non_negative(s.kappa) || !positive(s.max_margin) ||
      !positive(s.gain) || !positive(s.max_accel) || s.max_margin < s.d_safe) {
    throw std::invalid_argument("OrbitParams: separation parameters are invalid");
  }
}

OrbitReference orbit_reference(std::uint32_t slot, std::uint32_t n, double time_s,
                               const OrbitParams& params) {
  if (n == 0 || slot >= n) {
    throw std::invalid_argument("orbit_reference: slot must be below the number of drones");
  }
  if (!std::isfinite(time_s)) {
    throw std::invalid_argument("orbit_reference: time must be finite");
  }
  validate(params);

  const double omega = params.speed / params.radius;
  OrbitReference ref;
  ref.radius = params.radius + static_cast<double>(slot % 2U) * params.radius_step;
  ref.speed = omega * ref.radius;
  ref.angle = wrap(2.0 * kPi * static_cast<double>(slot) / static_cast<double>(n) + omega * time_s);
  const double c = std::cos(ref.angle);
  const double s = std::sin(ref.angle);
  ref.position = {params.center.x + ref.radius * c, params.center.y + ref.radius * s,
                  params.center.z + static_cast<double>(slot) * params.altitude_step};
  ref.velocity = {-ref.speed * s, ref.speed * c, 0.0};
  return ref;
}

double orbit_phase_error(const Vec3& position, const Vec3& center, double reference_angle) {
  if (!finite(position) || !finite(center) || !std::isfinite(reference_angle)) {
    throw std::invalid_argument("orbit_phase_error: inputs must be finite");
  }
  const double own = std::atan2(position.y - center.y, position.x - center.x);
  return wrap(reference_angle - own);
}

Vec3 orbit_guidance(const Vec3& position, const Vec3& velocity, const Vec3& center, double radius,
                    double speed, double z_ref, double phase_error, const OrbitParams& params,
                    double gain_scale) {
  if (!finite(position) || !finite(velocity) || !finite(center) || !std::isfinite(z_ref) ||
      !std::isfinite(phase_error) || !std::isfinite(gain_scale) || gain_scale < 0.0) {
    throw std::invalid_argument(
        "orbit_guidance: inputs must be finite and the gain scale not "
        "negative");
  }
  if (!(std::isfinite(radius) && radius > 0.0) || !(std::isfinite(speed) && speed > 0.0)) {
    throw std::invalid_argument("orbit_guidance: radius and speed must be positive");
  }
  validate(params);

  double px = position.x - center.x;
  double py = position.y - center.y;
  double r = std::hypot(px, py);
  if (r < 1e-9) {
    px = 1.0;  // at the center the outward direction is undefined; choose +x
    py = 0.0;
    r = 1.0;
  }
  const double erx = px / r;
  const double ery = py / r;
  const double etx = -ery;
  const double ety = erx;

  const double cut = std::clamp(params.radial_gain * (r - radius) / radius, -1.0, 1.0);
  double dx = etx - cut * erx;
  double dy = ety - cut * ery;
  const double dn = std::hypot(dx, dy);
  dx /= dn;
  dy /= dn;

  const double desired =
      std::clamp(speed + params.phase_gain * radius * phase_error,
                 params.min_speed + params.speed_margin, params.max_speed - params.speed_margin);
  const double centripetal = desired * desired / radius;
  const double kv = params.velocity_gain * gain_scale;

  Vec3 a;
  a.x = -centripetal * erx + kv * (desired * dx - velocity.x);
  a.y = -centripetal * ery + kv * (desired * dy - velocity.y);
  a.z = gain_scale *
        (params.altitude_gain * (z_ref - position.z) - params.altitude_damping * velocity.z);
  return a;
}

bool orbit_consensus_phase_error(std::uint32_t self, std::uint32_t n, double time_s,
                                 const Vec3& position, const std::vector<OrbitNeighbor>& neighbors,
                                 const OrbitParams& params, double& error) {
  if (!finite(position)) {
    throw std::invalid_argument("orbit_consensus_phase_error: position must be finite");
  }
  const OrbitReference own = orbit_reference(self, n, time_s, params);  // validates the rest
  const double own_angle = std::atan2(position.y - params.center.y, position.x - params.center.x);
  double sum_sin = 0.0;
  double sum_cos = 0.0;
  double total = 0.0;
  for (const OrbitNeighbor& nb : neighbors) {
    if (nb.slot == self || nb.slot >= n || !finite(nb.position) || !(nb.sigma >= 0.0) ||
        !std::isfinite(nb.sigma)) {
      continue;
    }
    const OrbitReference ref = orbit_reference(nb.slot, n, time_s, params);
    const double neighbor_angle =
        std::atan2(nb.position.y - params.center.y, nb.position.x - params.center.x);
    const double e = wrap(neighbor_angle - wrap(ref.angle - own.angle) - own_angle);
    const double floor = params.consensus_sigma_floor;
    const double w = 1.0 / (nb.sigma * nb.sigma + floor * floor);
    sum_sin += w * std::sin(e);
    sum_cos += w * std::cos(e);
    total += w;
  }
  if (!(total > 0.0)) {
    return false;
  }
  error = std::atan2(sum_sin, sum_cos);
  return true;
}

Vec3 orbit_separation_accel(std::uint32_t self, const Vec3& position, const Vec3& velocity,
                            const std::vector<OrbitNeighbor>& neighbors, const OrbitParams& params,
                            double margin_scale) {
  if (!finite(position) || !finite(velocity) ||
      !(std::isfinite(margin_scale) && margin_scale > 0.0)) {
    throw std::invalid_argument(
        "orbit_separation_accel: own state must be finite and the margin "
        "scale positive");
  }
  validate(params);
  const SeparationParams& sep = params.separation;

  Vec3 total;
  for (const OrbitNeighbor& n : neighbors) {
    if (n.slot == self || !finite(n.position) || !finite(n.velocity) || !(n.sigma >= 0.0) ||
        !std::isfinite(n.sigma)) {
      continue;
    }
    const double margin = std::min(sep.d_safe * margin_scale + sep.kappa * n.sigma, sep.max_margin);
    const Vec3 r = n.position - position;
    const Vec3 v = n.velocity - velocity;
    const double vv = norm_squared(v);
    double t_star = 0.0;
    if (vv > 0.0) {
      t_star = std::clamp(-dot(r, v) / vv, 0.0, params.lookahead_s);
    }
    const Vec3 miss = r + v * t_star;
    const double miss_distance = norm(miss);
    if (!(miss_distance < margin)) {
      continue;
    }
    Vec3 away;
    if (miss_distance > 1e-9) {
      away = miss / -miss_distance;
    } else {
      // Exactly on a collision course there is no side to dodge to: turn right of the own heading,
      // as in aviation, so two aircraft head-on pass each other on their right.
      const double speed_h = std::hypot(velocity.x, velocity.y);
      away = speed_h > 1e-9 ? Vec3{velocity.y / speed_h, -velocity.x / speed_h, 0.0}
                            : (self > n.slot ? Vec3{1.0, 0.0, 0.0} : Vec3{-1.0, 0.0, 0.0});
    }
    const double weight = self > n.slot ? 1.0 : 0.5;
    const double magnitude = std::min(sep.gain * (margin - miss_distance), sep.max_accel);
    total += away * (weight * magnitude);
  }
  return clamp_norm(total, sep.max_accel);
}

}  // namespace swarm
