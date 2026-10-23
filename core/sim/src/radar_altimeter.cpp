#include "swarm/radar_altimeter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "swarm/streams.hpp"

namespace swarm {

void validate(const RadarAltimeterConfig& c) {
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!positive(c.sigma_floor_m) || !positive(c.sigma_per_m) || !positive(c.max_range_m)) {
    throw std::invalid_argument(
        "RadarAltimeterConfig: sigmas and range must be finite and positive");
  }
  if (!(c.dropout_probability >= 0.0 && c.dropout_probability < 1.0)) {
    throw std::invalid_argument("RadarAltimeterConfig: dropout_probability must be in [0, 1)");
  }
  if (c.period_ticks < 1) {
    throw std::invalid_argument("RadarAltimeterConfig: period_ticks must be at least 1");
  }
}

RadarAltimeter::RadarAltimeter(const RadarAltimeterConfig& config, std::uint64_t seed,
                               std::uint32_t drone_id)
    : config_(config), rng_(seed, kStreamAltimeterBase + drone_id) {
  validate(config_);
}

AltimeterSample RadarAltimeter::measure(const Vec3& true_position, double surface_z) {
  if (!std::isfinite(true_position.x) || !std::isfinite(true_position.y) ||
      !std::isfinite(true_position.z) || !std::isfinite(surface_z)) {
    throw std::invalid_argument("RadarAltimeter: position and surface height must be finite");
  }
  // The draws come first and are unconditional.
  const double noise = rng_.normal();
  const double dropout = rng_.uniform();

  AltimeterSample sample;
  const double height = true_position.z - surface_z;
  if (!(height > 0.0) || height > config_.max_range_m || dropout < config_.dropout_probability) {
    return sample;
  }
  const double sigma = std::max(config_.sigma_floor_m, config_.sigma_per_m * height);
  sample.available = true;
  sample.sigma = sigma;
  sample.height = std::max(0.0, height + sigma * noise);
  return sample;
}

}  // namespace swarm
