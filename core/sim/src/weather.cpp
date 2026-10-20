#include "swarm/weather.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace swarm {

namespace {
constexpr double kMaxFogConstant = 20.0;
}

Weather Weather::fog(double visibility_m) {
  Weather w;
  w.visibility_m = visibility_m;
  return w;
}

Weather Weather::rain(double rate_mm_h, double visibility_m) {
  Weather w;
  w.rain_rate_mm_h = rate_mm_h;
  w.visibility_m = visibility_m;
  return w;
}

void Weather::validate() const {
  const auto finite = [](double v) { return std::isfinite(v); };
  if (!finite(visibility_m) || !(visibility_m > 0.0)) {
    throw std::invalid_argument("Weather: visibility must be finite and positive");
  }
  if (!finite(fog_constant) || !(fog_constant > 0.0) || fog_constant > kMaxFogConstant) {
    throw std::invalid_argument("Weather: fog constant must be positive and at most 20");
  }
  if (!finite(rain_rate_mm_h) || rain_rate_mm_h < 0.0) {
    throw std::invalid_argument("Weather: rain rate must be finite and non-negative");
  }
  if (!finite(rain_coefficient) || rain_coefficient < 0.0) {
    throw std::invalid_argument("Weather: rain coefficient must be finite and non-negative");
  }
  if (!finite(rain_exponent) || !(rain_exponent > 0.0)) {
    throw std::invalid_argument("Weather: rain exponent must be finite and positive");
  }
}

double Weather::extinction_per_m() const {
  const double fog = fog_constant / visibility_m;
  const double rain =
      rain_rate_mm_h > 0.0 ? rain_coefficient * std::pow(rain_rate_mm_h, rain_exponent) : 0.0;
  return fog + rain;
}

double Weather::intensity(double reflectivity, double range) const {
  return reflectivity * std::exp(-2.0 * extinction_per_m() * range);
}

double Weather::max_detection_range(double reflectivity, double threshold) const {
  const double alpha = extinction_per_m();
  if (!(alpha > 0.0)) {
    return std::numeric_limits<double>::infinity();
  }
  if (!(threshold > 0.0)) {
    return std::numeric_limits<double>::infinity();
  }
  if (reflectivity <= threshold) {
    return 0.0;  // even at zero range the return is too weak
  }
  return std::log(reflectivity / threshold) / (2.0 * alpha);
}

}  // namespace swarm
