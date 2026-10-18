#include "swarm/position_fix.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "swarm/streams.hpp"

namespace swarm {

namespace {

void validate_config(const PositionFixConfig& c) {
  const auto fraction = [](double f) { return f > 0.0 && f <= 1.0; };
  if (!(std::isfinite(c.base_sigma) && c.base_sigma > 0.0)) {
    throw std::invalid_argument("PositionFixConfig: base_sigma must be finite and positive");
  }
  if (!fraction(c.reference_fraction) || !fraction(c.min_fraction)) {
    throw std::invalid_argument("PositionFixConfig: fractions must be in (0, 1]");
  }
  if (!(std::isfinite(c.max_sigma) && c.max_sigma >= c.base_sigma)) {
    throw std::invalid_argument("PositionFixConfig: max_sigma must be finite and >= base_sigma");
  }
}

}  // namespace

PositionFixSensor::PositionFixSensor(const PositionFixConfig& config, std::uint64_t seed,
                                     std::uint32_t drone_id)
    : config_(config), rng_(seed, kStreamFixBase + drone_id) {
  validate_config(config_);
}

PositionFix PositionFixSensor::measure(const Vec3& true_position, const Scan& scan) {
  if (!std::isfinite(true_position.x) || !std::isfinite(true_position.y) ||
      !std::isfinite(true_position.z)) {
    throw std::invalid_argument("PositionFixSensor: true position must be finite");
  }

  // The draws come first and are unconditional.
  const double nx = rng_.normal();
  const double ny = rng_.normal();
  const double nz = rng_.normal();

  std::size_t valid = 0;
  for (const LidarReturn& r : scan.returns) {
    valid += r.valid ? 1U : 0U;
  }
  const double fraction =
      scan.returns.empty() ? 0.0
                           : static_cast<double>(valid) / static_cast<double>(scan.returns.size());

  PositionFix fix;
  if (!(fraction > 0.0) || fraction < config_.min_fraction) {
    fix.sigma = config_.max_sigma;
    return fix;
  }

  const double sigma = std::min(
      config_.max_sigma, config_.base_sigma * std::sqrt(config_.reference_fraction / fraction));
  fix.available = true;
  fix.sigma = sigma;
  fix.position = {true_position.x + sigma * nx, true_position.y + sigma * ny,
                  true_position.z + sigma * nz};
  return fix;
}

}  // namespace swarm
