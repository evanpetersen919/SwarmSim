#pragma once

#include <cstdint>

#include "swarm/lidar.hpp"
#include "swarm/rng.hpp"
#include "swarm/sensor_data.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

struct PositionFixConfig {
  double base_sigma = 0.5;  // meters per axis when the valid-return fraction equals the reference
  double reference_fraction = 0.5;  // in (0, 1]
  double min_fraction = 0.05;  // in (0, 1]; below this fraction of valid returns there is no fix
  double max_sigma = 20.0;     // meters; the reported sigma never exceeds this
};

// A simulated position fix whose availability and accuracy depend on what the LiDAR saw.
//
// This is a sensor model, not localization. It is allowed to read the true position, as the IMU
// model reads the true motion, and reports `true + noise`. What it takes from the scan is how
// informative the scan was: with f the fraction of rays that returned,
//   no fix if f < min_fraction,
//   sigma = min(max_sigma, base_sigma * sqrt(reference_fraction / f)) otherwise,
// so variance is inversely proportional to the number of valid returns. Fog, rain, and open sky
// therefore degrade or remove the fix. Estimators receive only the PositionFix.
//
// Every call draws three normals whether or not a fix results, on stream kStreamFixBase + drone id,
// so the noise of one call does not depend on earlier calls' outcomes.
class PositionFixSensor {
 public:
  // Throws std::invalid_argument for an invalid configuration.
  PositionFixSensor(const PositionFixConfig& config, std::uint64_t seed, std::uint32_t drone_id);

  // Throws std::invalid_argument if `true_position` is not finite.
  PositionFix measure(const Vec3& true_position, const Scan& scan);

  const PositionFixConfig& config() const { return config_; }

 private:
  PositionFixConfig config_;
  Rng rng_;
};

}  // namespace swarm
