#pragma once

#include <cstdint>

#include "swarm/rng.hpp"
#include "swarm/sensor_data.hpp"
#include "swarm/vec3.hpp"
#include "swarm/weather.hpp"

namespace swarm {

// Parameters of the camera position fix. Every value is an assumption: no source gives how well
// an aircraft's camera fixes its position as a function of visibility, and the navigation of the
// aircraft this models is not public (ADR 0010). docs/MODELS.md lists the values and their basis.
struct CameraFixConfig {
  double sigma_floor_m = 3.0;         // best accuracy per axis, in clear air at low altitude
  double sigma_per_m_agl = 0.02;      // accuracy lost per meter above ground (ground sample size)
  double min_agl_m = 10.0;            // below this height above ground there is no fix
  double max_agl_m = 1500.0;          // above this there is no fix
  double min_transmission = 0.2;      // below this one-way transmission to the ground, no fix
  double max_sigma_m = 100.0;         // the reported sigma never exceeds this
  double dropout_probability = 0.05;  // chance that a fix that could be made is not
};

// A simulated position fix from a downward camera. A sensor model, not localization: it may read
// the true position and reports `true + noise`, or no fix.
//
// With h the height above ground and alpha the weather's extinction (fog and rain), the one-way
// transmission over the slant range to the ground, straight down, is
//   T = exp(-alpha * h)
// and the result is
//   no fix if h is outside [min_agl_m, max_agl_m], or T < min_transmission, or the dropout draw
//   hits;
//   otherwise sigma = min(max_sigma_m, max(sigma_floor_m, sigma_per_m_agl * h) / T).
// So fog and rain degrade the fix before they remove it, and shorten the height it works from.
//
// Every call draws three normals and one uniform whether or not a fix results, on stream
// kStreamFixBase + drone id, so the noise of one call does not depend on earlier outcomes. A drone
// has either this sensor or the LiDAR fix, never both, so they share the stream.
class CameraFixSensor {
 public:
  // Throws std::invalid_argument for an invalid configuration.
  CameraFixSensor(const CameraFixConfig& config, std::uint64_t seed, std::uint32_t drone_id);

  // `ground_z` is the terrain height under the aircraft. Throws std::invalid_argument for a
  // non-finite position or ground height.
  PositionFix measure(const Vec3& true_position, double ground_z, const Weather& weather);

  const CameraFixConfig& config() const { return config_; }

 private:
  CameraFixConfig config_;
  Rng rng_;
};

}  // namespace swarm
