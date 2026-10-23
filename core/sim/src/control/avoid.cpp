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
}

AvoidResult assess_avoidance(const AvoidParams& params, const AvoidInput& in) {
  validate(params);
  if (!finite(in.position) || !finite(in.velocity) || !finite(in.sample_position) ||
      !std::isfinite(in.clearance) || !std::isfinite(in.climb_budget) ||
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
    const double needed = point.z + in.clearance - in.position.z;
    if (needed > in.climb_budget * t) {
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

  // The free column nearest the heading, the right one on a tie.
  std::size_t target = columns;
  for (std::size_t c = 0; c < columns; ++c) {
    if (!seen[c] || blocked[c]) {
      continue;
    }
    if (target == columns || std::abs(azimuth[c]) < std::abs(azimuth[target]) - 1e-12 ||
        (std::abs(std::abs(azimuth[c]) - std::abs(azimuth[target])) <= 1e-12 &&
         azimuth[c] < azimuth[target])) {
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
