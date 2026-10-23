#pragma once

#include <cstdint>

#include "swarm/rng.hpp"
#include "swarm/sensor_data.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

// Parameters of the radar altimeter (ADR 0012). Every value is an assumption: public sources
// describe the payload of the aircraft this models only as an EO/IR camera. docs/MODELS.md lists
// the values and their basis.
struct RadarAltimeterConfig {
  double sigma_floor_m = 1.0;         // best accuracy, close to the surface
  double sigma_per_m = 0.01;          // accuracy lost per meter of height
  double max_range_m = 1500.0;        // above this height there is no reading
  double dropout_probability = 0.02;  // chance that a reading that could be made is not
  std::uint32_t period_ticks = 10;    // a reading is attempted on ticks divisible by this
};

// Throws std::invalid_argument unless the sigmas and range are finite and positive, the dropout
// probability is in [0, 1), and the period is at least 1.
void validate(const RadarAltimeterConfig& config);

// A simulated radar altimeter. A sensor model: it may read the truth. It measures straight down
// from the aircraft to the highest solid surface below it, the terrain or the top of an obstacle,
// and reports
//   no reading if the height is not above zero or is above max_range_m, or the dropout draw hits;
//   otherwise height + sigma * N(0, 1), with sigma = max(sigma_floor_m, sigma_per_m * height).
// It is a radar, so weather does not affect it, unlike the camera fix.
//
// Every call draws one normal and one uniform whether or not a reading results, on stream
// kStreamAltimeterBase + drone id, so the noise of one call does not depend on earlier outcomes.
class RadarAltimeter {
 public:
  // Throws std::invalid_argument for an invalid configuration.
  RadarAltimeter(const RadarAltimeterConfig& config, std::uint64_t seed, std::uint32_t drone_id);

  // `surface_z` is the height of the highest solid surface under the aircraft. Throws
  // std::invalid_argument for a non-finite position or surface height.
  AltimeterSample measure(const Vec3& true_position, double surface_z);

  const RadarAltimeterConfig& config() const { return config_; }

 private:
  RadarAltimeterConfig config_;
  Rng rng_;
};

}  // namespace swarm
