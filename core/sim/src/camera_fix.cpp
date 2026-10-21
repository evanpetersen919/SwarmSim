#include "swarm/camera_fix.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "swarm/streams.hpp"

namespace swarm {

namespace {

void validate_config(const CameraFixConfig& c) {
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!positive(c.sigma_floor_m) || !positive(c.sigma_per_m_agl) || !positive(c.min_agl_m) ||
      !positive(c.max_agl_m) || !positive(c.max_sigma_m)) {
    throw std::invalid_argument("CameraFixConfig: sigmas and heights must be finite and positive");
  }
  if (!(c.min_agl_m < c.max_agl_m)) {
    throw std::invalid_argument("CameraFixConfig: min_agl_m must be below max_agl_m");
  }
  if (!(c.max_sigma_m >= c.sigma_floor_m)) {
    throw std::invalid_argument("CameraFixConfig: max_sigma_m must be at least sigma_floor_m");
  }
  if (!(c.min_transmission > 0.0 && c.min_transmission < 1.0)) {
    throw std::invalid_argument("CameraFixConfig: min_transmission must be in (0, 1)");
  }
  if (!(c.dropout_probability >= 0.0 && c.dropout_probability < 1.0)) {
    throw std::invalid_argument("CameraFixConfig: dropout_probability must be in [0, 1)");
  }
}

}  // namespace

CameraFixSensor::CameraFixSensor(const CameraFixConfig& config, std::uint64_t seed,
                                 std::uint32_t drone_id)
    : config_(config), rng_(seed, kStreamFixBase + drone_id) {
  validate_config(config_);
}

PositionFix CameraFixSensor::measure(const Vec3& true_position, double ground_z,
                                     const Weather& weather) {
  if (!std::isfinite(true_position.x) || !std::isfinite(true_position.y) ||
      !std::isfinite(true_position.z) || !std::isfinite(ground_z)) {
    throw std::invalid_argument("CameraFixSensor: position and ground height must be finite");
  }
  // The draws come first and are unconditional.
  const double nx = rng_.normal();
  const double ny = rng_.normal();
  const double nz = rng_.normal();
  const double dropout = rng_.uniform();

  PositionFix fix;
  fix.sigma = config_.max_sigma_m;
  const double agl = true_position.z - ground_z;
  if (!(agl >= config_.min_agl_m) || agl > config_.max_agl_m) {
    return fix;
  }
  const double transmission = std::exp(-weather.extinction_per_m() * agl);
  if (!(transmission >= config_.min_transmission) || dropout < config_.dropout_probability) {
    return fix;
  }
  const double sigma =
      std::min(config_.max_sigma_m,
               std::max(config_.sigma_floor_m, config_.sigma_per_m_agl * agl) / transmission);
  fix.available = true;
  fix.sigma = sigma;
  fix.position = {true_position.x + sigma * nx, true_position.y + sigma * ny,
                  true_position.z + sigma * nz};
  return fix;
}

}  // namespace swarm
