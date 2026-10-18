#include "swarm/control/formation_control.hpp"

#include <cmath>
#include <stdexcept>

namespace swarm {

namespace {

bool finite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

void validate(const FormationParams& p) {
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!(std::isfinite(p.alpha) && p.alpha >= 0.0 && p.alpha <= 1.0)) {
    throw std::invalid_argument("FormationParams: alpha must be in [0, 1]");
  }
  if (!(std::isfinite(p.kp) && p.kp >= 0.0) || !(std::isfinite(p.kd) && p.kd >= 0.0)) {
    throw std::invalid_argument("FormationParams: gains must be finite and non-negative");
  }
  if (!positive(p.sigma_floor) || !positive(p.max_accel)) {
    throw std::invalid_argument("FormationParams: sigma_floor and max_accel must be positive");
  }
}

bool usable(const FormationNeighbor& n, std::uint32_t self, std::size_t slots) {
  const NeighborEstimate& e = n.estimate;
  return e.valid && n.slot != self && n.slot < slots && finite(e.position) && finite(e.velocity) &&
         std::isfinite(e.position_sigma) && e.position_sigma >= 0.0;
}

}  // namespace

FormationTarget formation_target(std::uint32_t self, const Vec3& own_position,
                                 const Vec3& own_velocity, double own_sigma,
                                 const std::vector<FormationNeighbor>& neighbors,
                                 const std::vector<Vec3>& offsets,
                                 const MissionReference& reference, const FormationParams& params) {
  validate(params);
  if (self >= offsets.size()) {
    throw std::invalid_argument("formation_target: slot is outside the formation");
  }
  if (!finite(own_position) || !finite(own_velocity) ||
      !(std::isfinite(own_sigma) && own_sigma >= 0.0)) {
    throw std::invalid_argument("formation_target: the own estimate must be finite");
  }

  const double floor_sq = params.sigma_floor * params.sigma_floor;
  const auto weight = [&](double sigma) { return 1.0 / (sigma * sigma + floor_sq); };

  // Weighted sums over this drone and every usable neighbor.
  double w_own = weight(own_sigma);
  double w_sum = w_own;
  Vec3 center_sum = (own_position - offsets[self]) * w_own;
  Vec3 velocity_sum = own_velocity * w_own;
  std::size_t used = 0;
  for (const FormationNeighbor& n : neighbors) {
    if (!usable(n, self, offsets.size())) {
      continue;
    }
    const double w = weight(n.estimate.position_sigma);
    w_sum += w;
    center_sum += (n.estimate.position - offsets[n.slot]) * w;
    velocity_sum += n.estimate.velocity * w;
    ++used;
  }

  const double alpha = used == 0 ? 1.0 : params.alpha;
  const Vec3 center_consensus = center_sum / w_sum;
  const Vec3 velocity_consensus = velocity_sum / w_sum;

  FormationTarget target;
  target.position = (reference.position * alpha + center_consensus * (1.0 - alpha)) + offsets[self];
  target.velocity = reference.velocity * alpha + velocity_consensus * (1.0 - alpha);
  target.neighbors_used = used;
  return target;
}

Vec3 formation_accel(const FormationTarget& target, const Vec3& own_position,
                     const Vec3& own_velocity, const FormationParams& params) {
  validate(params);
  const Vec3 accel =
      (target.position - own_position) * params.kp + (target.velocity - own_velocity) * params.kd;
  return clamp_norm(accel, params.max_accel);
}

}  // namespace swarm
