#include "swarm/wind.hpp"

#include <cmath>
#include <stdexcept>

#include "swarm/dynamics.hpp"
#include "swarm/rng.hpp"
#include "swarm/streams.hpp"

namespace swarm {

namespace {

bool finite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// The wavelengths run from this fraction of the largest scale up to the scale itself.
constexpr double kSmallestWavelengthFraction = 0.2;
// A wave also changes on its own at this fraction of (wavenumber * rms) radians per second.
constexpr double kEddyRate = 0.5;

}  // namespace

void validate(const WindParams& params) {
  if (!finite(params.mean) || !std::isfinite(params.gust_sigma) ||
      !std::isfinite(params.gust_scale) || !std::isfinite(params.vertical_ratio)) {
    throw std::invalid_argument("WindParams: values must be finite");
  }
  if (params.gust_sigma < 0.0) {
    throw std::invalid_argument("WindParams: gust_sigma must not be negative");
  }
  if (!(params.gust_scale > 0.0)) {
    throw std::invalid_argument("WindParams: gust_scale must be positive");
  }
  if (params.vertical_ratio < 0.0 || params.vertical_ratio > 1.0) {
    throw std::invalid_argument("WindParams: vertical_ratio must be in [0, 1]");
  }
}

WindField::WindField(const WindParams& params, std::uint64_t seed) : params_(params) {
  validate(params_);
  Rng rng(seed, kStreamWind);
  for (Wave& w : waves_) {
    // Draw everything for a wave whether or not the gusts are on, so the field a seed gives does
    // not depend on the other settings.
    const double direction = rng.uniform(0.0, 2.0 * kPi);
    const double fraction = rng.uniform();  // wavelength is log-uniform in its range
    const double wavelength =
        params_.gust_scale * std::exp(std::log(kSmallestWavelengthFraction) * (1.0 - fraction));
    const double k = 2.0 * kPi / wavelength;
    w.kx = k * std::cos(direction);
    w.ky = k * std::sin(direction);
    w.phase = rng.uniform(0.0, 2.0 * kPi);
    w.phase_z = rng.uniform(0.0, 2.0 * kPi);
    const double gust_direction = rng.uniform(0.0, 2.0 * kPi);
    w.dir_x = std::cos(gust_direction);
    w.dir_y = std::sin(gust_direction);
    w.omega = w.kx * params_.mean.x + w.ky * params_.mean.y + kEddyRate * k * params_.gust_sigma;
  }
  // Each wave has a mean square of a^2 / 2, so kWaves waves give the horizontal variance
  // 2 sigma^2 (both components) when a = 2 sigma / sqrt(kWaves).
  amplitude_ = 2.0 * params_.gust_sigma / std::sqrt(static_cast<double>(kWaves));
  amplitude_z_ = params_.vertical_ratio * params_.gust_sigma * std::sqrt(2.0 / kWaves);
}

Vec3 WindField::at(const Vec3& position, double time_s) const {
  if (!finite(position) || !std::isfinite(time_s)) {
    throw std::invalid_argument("WindField::at: position and time must be finite");
  }
  Vec3 wind = params_.mean;
  if (params_.gust_sigma == 0.0) {
    return wind;
  }
  for (const Wave& w : waves_) {
    const double arg = w.kx * position.x + w.ky * position.y - w.omega * time_s;
    const double horizontal = amplitude_ * std::sin(arg + w.phase);
    wind.x += horizontal * w.dir_x;
    wind.y += horizontal * w.dir_y;
    wind.z += amplitude_z_ * std::sin(arg + w.phase_z);
  }
  return wind;
}

}  // namespace swarm
