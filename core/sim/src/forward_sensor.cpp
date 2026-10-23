#include "swarm/forward_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "swarm/streams.hpp"

namespace swarm {

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}  // namespace

void validate(const ForwardSensorConfig& c) {
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!(std::isfinite(c.horizontal_fov_deg) && c.horizontal_fov_deg > 0.0 &&
        c.horizontal_fov_deg <= 180.0)) {
    throw std::invalid_argument("ForwardSensorConfig: horizontal_fov_deg must be in (0, 180]");
  }
  if (!(std::isfinite(c.vertical_fov_deg) && c.vertical_fov_deg >= 0.0 &&
        c.vertical_fov_deg < 90.0)) {
    throw std::invalid_argument("ForwardSensorConfig: vertical_fov_deg must be in [0, 90)");
  }
  if (!(std::isfinite(c.pitch_deg) && c.pitch_deg > -90.0 && c.pitch_deg < 90.0)) {
    throw std::invalid_argument("ForwardSensorConfig: pitch_deg must be in (-90, 90)");
  }
  if (c.azimuth_rays < 1 || c.elevation_rays < 1) {
    throw std::invalid_argument("ForwardSensorConfig: at least one ray on each axis is required");
  }
  if (!positive(c.max_range_m) || !positive(c.sigma_floor_m) || !positive(c.sigma_per_m)) {
    throw std::invalid_argument(
        "ForwardSensorConfig: range and sigmas must be finite and positive");
  }
  if (!(c.dropout_probability >= 0.0 && c.dropout_probability < 1.0)) {
    throw std::invalid_argument("ForwardSensorConfig: dropout_probability must be in [0, 1)");
  }
  if (!(c.min_transmission > 0.0 && c.min_transmission <= 1.0)) {
    throw std::invalid_argument("ForwardSensorConfig: min_transmission must be in (0, 1]");
  }
  if (c.period_ticks < 1) {
    throw std::invalid_argument("ForwardSensorConfig: period_ticks must be at least 1");
  }
}

ForwardSensor::ForwardSensor(const ForwardSensorConfig& config, std::uint64_t seed,
                             std::uint32_t drone_id)
    : config_(config), rng_(seed, kStreamForwardBase + drone_id) {
  validate(config_);
}

ForwardSample ForwardSensor::measure(const Vec3& position, double heading_x, double heading_y,
                                     const IScene& scene, const Weather& weather) {
  if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      !std::isfinite(heading_x) || !std::isfinite(heading_y)) {
    throw std::invalid_argument("ForwardSensor: position and heading must be finite");
  }
  const double length = std::hypot(heading_x, heading_y);
  if (!(length > 0.0)) {
    throw std::invalid_argument("ForwardSensor: the heading must not be zero");
  }
  weather.validate();
  const double hx = heading_x / length;
  const double hy = heading_y / length;
  const double alpha = weather.extinction_per_m();

  const std::uint32_t rows = config_.elevation_rays;
  const std::uint32_t columns = config_.azimuth_rays;
  ForwardSample sample;
  sample.available = true;
  sample.rays.reserve(static_cast<std::size_t>(rows) * columns);
  for (std::uint32_t row = 0; row < rows; ++row) {
    const double elevation_deg =
        config_.pitch_deg +
        (rows == 1 ? 0.0
                   : config_.vertical_fov_deg *
                         (static_cast<double>(row) / static_cast<double>(rows - 1) - 0.5));
    const double elevation = elevation_deg * kDegToRad;
    for (std::uint32_t column = 0; column < columns; ++column) {
      // A positive azimuth is to the left of the heading; the first column is the leftmost.
      const double azimuth_deg =
          columns == 1 ? 0.0
                       : config_.horizontal_fov_deg *
                             (0.5 - static_cast<double>(column) / static_cast<double>(columns - 1));
      const double azimuth = azimuth_deg * kDegToRad;
      const double cos_e = std::cos(elevation);
      const double dx = hx * std::cos(azimuth) - hy * std::sin(azimuth);
      const double dy = hx * std::sin(azimuth) + hy * std::cos(azimuth);

      // The draws come first and are unconditional.
      const double noise = rng_.normal();
      const double dropout = rng_.uniform();

      ForwardRay ray;
      ray.direction = Vec3{cos_e * dx, cos_e * dy, std::sin(elevation)};
      RayHit hit;
      if (scene.raycast(position, ray.direction, config_.max_range_m, hit)) {
        const double transmission = std::exp(-alpha * hit.distance);
        if (transmission >= config_.min_transmission && !(dropout < config_.dropout_probability)) {
          ray.hit = true;
          ray.sigma =
              std::max(config_.sigma_floor_m, config_.sigma_per_m * hit.distance) / transmission;
          ray.range = std::max(0.0, hit.distance + ray.sigma * noise);
        }
      }
      sample.rays.push_back(ray);
    }
  }
  return sample;
}

}  // namespace swarm
