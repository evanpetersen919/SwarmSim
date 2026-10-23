#include "swarm/control/avoid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace swarm {

namespace {
bool finite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
}  // namespace

void validate(const AvoidParams& p) {
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!positive(p.horizon_s) || !positive(p.hold_s) || !positive(p.turn_time_s) ||
      !positive(p.max_lateral_accel) || !positive(p.max_vertical_accel)) {
    throw std::invalid_argument(
        "AvoidParams: horizon, hold, turn time, and the accelerations must be positive");
  }
  if (!(std::isfinite(p.bank_fraction) && p.bank_fraction > 0.0 && p.bank_fraction <= 1.0)) {
    throw std::invalid_argument("AvoidParams: bank_fraction must be in (0, 1]");
  }
  if (p.clearance_m && !(std::isfinite(*p.clearance_m) && *p.clearance_m >= 0.0)) {
    throw std::invalid_argument("AvoidParams: clearance_m must be finite and not negative");
  }
}

double reachable_climb(double v0, double climb_limit, double accel, double t) {
  if (!std::isfinite(v0) || !std::isfinite(climb_limit) || !std::isfinite(accel) ||
      !std::isfinite(t) || !(climb_limit > 0.0) || !(accel > 0.0) || t < 0.0) {
    throw std::invalid_argument("reachable_climb: invalid input");
  }
  const double v = std::min(v0, climb_limit);
  const double ramp = (climb_limit - v) / accel;
  if (t <= ramp) {
    return v * t + 0.5 * accel * t * t;
  }
  return climb_limit * t - (climb_limit - v) * (climb_limit - v) / (2.0 * accel);
}

AvoidResult assess_avoidance(const AvoidParams& params, const AvoidInput& in) {
  validate(params);
  if (!finite(in.position) || !finite(in.velocity) || !finite(in.sample_position) ||
      !finite(in.guidance_accel) || !std::isfinite(in.clearance) ||
      !std::isfinite(in.climb_budget) || !std::isfinite(in.climb_limit) ||
      !std::isfinite(in.margin_sigmas) || !(std::isfinite(in.min_speed) && in.min_speed > 0.0)) {
    throw std::invalid_argument("assess_avoidance: the input must be finite");
  }
  AvoidResult result;
  const double horizontal = std::hypot(in.velocity.x, in.velocity.y);
  if (in.sample == nullptr || !in.sample->available || in.sample->rays.empty() ||
      in.sample->columns == 0 || !(horizontal > 1e-9)) {
    return result;
  }
  const std::vector<ForwardRay>& rays = in.sample->rays;
  const std::size_t columns = in.sample->columns;
  const double hx = in.velocity.x / horizontal;
  const double hy = in.velocity.y / horizontal;
  const double speed = std::max(horizontal, in.min_speed);
  const double clearance = params.clearance_m ? *params.clearance_m : in.clearance;
  const bool kinematic = params.feasibility == AvoidFeasibility::kKinematic && in.climb_limit > 0.0;

  // Per column: the signed angle from the heading to the column, positive to the left, whether it
  // is blocked, and the time to its nearest unclearable return.
  std::vector<double> azimuth(columns, 0.0);
  std::vector<bool> seen(columns, false);
  std::vector<bool> blocked(columns, false);
  std::vector<double> time_to(columns, std::numeric_limits<double>::infinity());
  for (std::size_t i = 0; i < rays.size(); ++i) {
    const ForwardRay& ray = rays[i];
    const std::size_t c = i % columns;
    if (!seen[c]) {
      seen[c] = true;
      azimuth[c] = std::atan2(hx * ray.direction.y - hy * ray.direction.x,
                              hx * ray.direction.x + hy * ray.direction.y);
    }
    if (!ray.hit) {
      continue;
    }
    const double margin = in.margin_sigmas * ray.sigma;
    const double range = std::max(0.0, ray.range + (ray.direction.z > 0.0 ? margin : -margin));
    const Vec3 point = in.sample_position + ray.direction * range;
    const double distance = std::hypot(point.x - in.position.x, point.y - in.position.y);
    const double t = distance / speed;
    if (t >= params.horizon_s) {
      continue;
    }
    const double needed = point.z + clearance - in.position.z;
    const double reachable =
        kinematic ? reachable_climb(in.velocity.z, in.climb_limit, params.max_vertical_accel, t)
                  : in.climb_budget * t;
    if (needed > reachable) {
      blocked[c] = true;
      time_to[c] = std::min(time_to[c], t);
    }
  }

  // The column nearest the heading decides whether there is a threat.
  std::size_t center = columns;
  for (std::size_t c = 0; c < columns; ++c) {
    if (seen[c] && (center == columns || std::abs(azimuth[c]) < std::abs(azimuth[center]))) {
      center = c;
    }
  }
  if (center == columns || !blocked[center]) {
    return result;
  }
  result.threatened = true;
  result.time_to_threat = time_to[center];

  // The heading the free column is chosen nearest to: the aircraft's own (angle 0), or with rejoin
  // the way the guidance is already turning it, with the side a tie goes to.
  double preferred = 0.0;
  double tie_side = -1.0;  // the right
  if (params.rejoin) {
    const double gx = in.guidance_accel.x;
    const double gy = in.guidance_accel.y;
    if (std::hypot(gx, gy) > 1e-9) {
      const double px = in.velocity.x + gx * params.turn_time_s;
      const double py = in.velocity.y + gy * params.turn_time_s;
      if (std::hypot(px, py) > 1e-9) {
        preferred = std::atan2(hx * py - hy * px, hx * px + hy * py);
        const double sideways = hx * gy - hy * gx;  // positive to the left
        if (std::abs(sideways) > 1e-12) {
          tie_side = sideways > 0.0 ? 1.0 : -1.0;
        }
      }
    }
  }

  // The free column nearest that heading; on a tie, the one on the preferred side.
  std::size_t target = columns;
  for (std::size_t c = 0; c < columns; ++c) {
    if (!seen[c] || blocked[c]) {
      continue;
    }
    if (target == columns) {
      target = c;
      continue;
    }
    const double dc = std::abs(azimuth[c] - preferred);
    const double dt = std::abs(azimuth[target] - preferred);
    if (dc < dt - 1e-12 ||
        (std::abs(dc - dt) <= 1e-12 && azimuth[c] * tie_side > azimuth[target] * tie_side)) {
      target = c;
    }
  }
  if (target == columns) {
    // Every column is blocked: toward the one that gives the most time.
    result.all_blocked = true;
    for (std::size_t c = 0; c < columns; ++c) {
      if (seen[c] && (target == columns || time_to[c] > time_to[target])) {
        target = c;
      }
    }
  }
  result.turn_angle = azimuth[target];
  const double limit = params.bank_fraction * params.max_lateral_accel;
  result.turn_accel = std::clamp(speed * result.turn_angle / params.turn_time_s, -limit, limit);
  if (std::abs(result.turn_accel) > 1e-9) {
    const double turn_rate = std::abs(result.turn_accel) / speed;
    result.slow = result.time_to_threat < std::abs(result.turn_angle) / turn_rate;
  }
  if (result.all_blocked) {
    result.turn_accel = result.turn_angle > 0.0 ? limit : -limit;
    result.slow = true;
  }
  return result;
}

}  // namespace swarm
