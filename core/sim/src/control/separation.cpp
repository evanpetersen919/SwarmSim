#include "swarm/control/separation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swarm {

namespace {

bool finite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

}  // namespace

Vec3 separation_accel(std::uint32_t self, const Vec3& own_position,
                      const std::vector<SeparationNeighbor>& neighbors,
                      const SeparationParams& params) {
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!positive(params.d_safe) || !positive(params.gain) || !positive(params.max_accel) ||
      !(std::isfinite(params.max_margin) && params.max_margin >= params.d_safe) ||
      !(std::isfinite(params.kappa) && params.kappa >= 0.0)) {
    throw std::invalid_argument("SeparationParams: invalid parameter");
  }
  if (!finite(own_position)) {
    throw std::invalid_argument("separation_accel: own position must be finite");
  }

  Vec3 total;
  for (const SeparationNeighbor& n : neighbors) {
    if (n.slot == self || !finite(n.position) || !(std::isfinite(n.sigma) && n.sigma >= 0.0)) {
      continue;
    }
    const double margin = std::min(params.d_safe + params.kappa * n.sigma, params.max_margin);
    const Vec3 away = own_position - n.position;
    const double d = norm(away);
    if (!(d < margin)) {
      continue;
    }
    Vec3 direction;
    if (d > 0.0) {
      direction = away / d;
    } else {
      direction = Vec3{self > n.slot ? 1.0 : -1.0, 0.0, 0.0};
    }
    double magnitude = params.gain * (margin - d);
    if (magnitude > params.max_accel) {
      magnitude = params.max_accel;
    }
    total += direction * magnitude;
  }
  return clamp_norm(total, params.max_accel);
}

Vec3 combine_accel(const Vec3& formation, const Vec3& separation, double max_accel) {
  if (!(std::isfinite(max_accel) && max_accel > 0.0)) {
    throw std::invalid_argument("combine_accel: max_accel must be finite and positive");
  }
  const Vec3 kept = clamp_norm(separation, max_accel);
  const double headroom = max_accel - norm(kept);
  return kept + clamp_norm(formation, headroom);
}

}  // namespace swarm
