#include "swarm/lidar.hpp"

#include <cmath>
#include <stdexcept>

#include "swarm/dynamics.hpp"
#include "swarm/streams.hpp"

namespace swarm {

namespace {

constexpr double kDegToRad = kPi / 180.0;

bool finite_non_negative(double v) { return std::isfinite(v) && v >= 0.0; }

void validate_config(const LidarConfig& c) {
  if (c.beams < 1 || c.azimuth_samples < 1) {
    throw std::invalid_argument("LidarConfig: beams and azimuth_samples must be at least 1");
  }
  if (!std::isfinite(c.vertical_fov_min_deg) || !std::isfinite(c.vertical_fov_max_deg) ||
      c.vertical_fov_min_deg > c.vertical_fov_max_deg || c.vertical_fov_min_deg < -90.0 ||
      c.vertical_fov_max_deg > 90.0) {
    throw std::invalid_argument(
        "LidarConfig: vertical field of view must satisfy -90 <= min <= max <= 90 degrees");
  }
  if (!(c.horizontal_fov_deg > 0.0 && c.horizontal_fov_deg <= 360.0)) {
    throw std::invalid_argument("LidarConfig: horizontal_fov_deg must be in (0, 360]");
  }
  if (!finite_non_negative(c.min_range) || !std::isfinite(c.max_range) ||
      !(c.max_range > c.min_range)) {
    throw std::invalid_argument("LidarConfig: need 0 <= min_range < max_range");
  }
  if (!finite_non_negative(c.range_noise_sigma) ||
      !finite_non_negative(c.range_noise_sigma_per_m)) {
    throw std::invalid_argument("LidarConfig: noise sigmas must be finite and non-negative");
  }
  if (!(c.dropout_probability >= 0.0 && c.dropout_probability <= 1.0)) {
    throw std::invalid_argument("LidarConfig: dropout_probability must be in [0, 1]");
  }
}

}  // namespace

Lidar::Lidar(const LidarConfig& config, std::uint64_t seed, std::uint32_t drone_id)
    : config_(config), rng_(seed, kStreamLidarBase + drone_id) {
  validate_config(config_);

  pattern_.reserve(ray_count());
  const double fov = config_.horizontal_fov_deg * kDegToRad;
  for (std::uint32_t b = 0; b < config_.beams; ++b) {
    const double elevation_deg =
        config_.beams == 1
            ? 0.5 * (config_.vertical_fov_min_deg + config_.vertical_fov_max_deg)
            : config_.vertical_fov_min_deg +
                  (config_.vertical_fov_max_deg - config_.vertical_fov_min_deg) *
                      static_cast<double>(b) / static_cast<double>(config_.beams - 1);
    const double elevation = elevation_deg * kDegToRad;
    const double horizontal = std::cos(elevation);
    const double vertical = std::sin(elevation);
    for (std::uint32_t k = 0; k < config_.azimuth_samples; ++k) {
      const double azimuth = -0.5 * fov + (static_cast<double>(k) + 0.5) * fov /
                                              static_cast<double>(config_.azimuth_samples);
      pattern_.push_back(
          {horizontal * std::cos(azimuth), horizontal * std::sin(azimuth), vertical});
    }
  }
}

Vec3 Lidar::beam_direction(std::uint32_t beam, std::uint32_t azimuth, double yaw) const {
  const Vec3 rel = pattern_[static_cast<std::size_t>(beam) * config_.azimuth_samples + azimuth];
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {rel.x * c - rel.y * s, rel.x * s + rel.y * c, rel.z};
}

Scan Lidar::scan(const Vec3& origin, double yaw, const IScene& scene) {
  Scan out;
  out.beams = config_.beams;
  out.azimuth_samples = config_.azimuth_samples;
  out.returns.assign(pattern_.size(), LidarReturn{});

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (std::size_t i = 0; i < pattern_.size(); ++i) {
    // The random draws come first and are unconditional.
    const double noise = rng_.normal();
    const double dropout = rng_.uniform();

    const Vec3& rel = pattern_[i];
    const Vec3 direction{rel.x * c - rel.y * s, rel.x * s + rel.y * c, rel.z};
    RayHit hit;
    if (!scene.raycast(origin, direction, config_.max_range, hit)) {
      continue;
    }
    if (hit.distance < config_.min_range) {
      continue;
    }
    if (dropout < config_.dropout_probability) {
      continue;
    }
    const double sigma = config_.range_noise_sigma + config_.range_noise_sigma_per_m * hit.distance;
    out.returns[i].valid = true;
    out.returns[i].range = hit.distance + sigma * noise;
  }
  return out;
}

}  // namespace swarm
