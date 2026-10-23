#pragma once

#include <cstdint>

#include "swarm/rng.hpp"
#include "swarm/scene.hpp"
#include "swarm/sensor_data.hpp"
#include "swarm/vec3.hpp"
#include "swarm/weather.hpp"

namespace swarm {

// Parameters of the forward obstacle sensor (ADR 0013). Every value is an assumption: public
// sources describe the payload of the aircraft this models only as an EO/IR camera, and say
// nothing about obstacle detection, a field of view, or a range. docs/MODELS.md lists the values
// and their basis.
struct ForwardSensorConfig {
  double horizontal_fov_deg =
      40.0;                        // the rays are spread evenly over this, centered on the heading
  double vertical_fov_deg = 20.0;  // and over this, centered on the pitch offset
  double pitch_deg = -5.0;         // elevation of the center of the fan; negative looks down
  std::uint32_t azimuth_rays = 7;
  std::uint32_t elevation_rays = 3;
  double max_range_m = 800.0;         // nothing farther than this is detected
  double sigma_floor_m = 2.0;         // best range accuracy, in clear air at short range
  double sigma_per_m = 0.01;          // accuracy lost per meter of range
  double dropout_probability = 0.05;  // chance that a return that could be detected is not
  double min_transmission = 0.2;      // below this one-way transmission to the surface, no return
  std::uint32_t period_ticks = 10;    // a reading is attempted on ticks divisible by this
};

// Throws std::invalid_argument unless the field of view angles are in range (the horizontal in
// (0, 180], the vertical in [0, 90)), the pitch is in (-90, 90), there is at least one ray on each
// axis, the range and sigmas are finite and positive, the dropout probability is in [0, 1), the
// minimum transmission is in (0, 1], and the period is at least 1.
void validate(const ForwardSensorConfig& config);

// A simulated forward obstacle sensor: what a forward EO/IR camera with an obstacle detector would
// report, as a fan of range rays and not as an image. A sensor model: it may read the truth.
//
// The fan points along the horizontal heading (given as a vector, not necessarily unit). Ray
// (row, column) has elevation pitch + vertical_fov * (row / (rows - 1) - 1/2) and an azimuth of
// horizontal_fov * (1/2 - column / (columns - 1)) to the left of the heading; one row or one
// column sits at the center. Rays are in row-major order from the lowest row and the leftmost
// column.
//
// Each ray is cast against the scene up to max_range_m. With alpha the weather's extinction (fog
// and rain) and d the distance to the first surface, the one-way transmission is T = exp(-alpha *
// d), and the ray reports
//   no return if nothing is hit, or T < min_transmission, or the dropout draw hits;
//   otherwise range = d + sigma * N(0, 1), floored at 0, with
//   sigma = max(sigma_floor_m, sigma_per_m * d) / T.
// So fog and rain shorten the distance at which a surface is seen, and blur it before it is lost.
//
// Every ray draws one normal and one uniform whether or not it hits anything, on stream
// kStreamForwardBase + drone id, so the noise on a ray does not depend on the scene or on the
// outcome of other rays. Ray directions use sin and cos, so readings are reproducible on the same
// binary and platform but not guaranteed bit-identical across platforms.
class ForwardSensor {
 public:
  // Throws std::invalid_argument for an invalid configuration.
  ForwardSensor(const ForwardSensorConfig& config, std::uint64_t seed, std::uint32_t drone_id);

  // Throws std::invalid_argument for a non-finite position or heading, a zero heading, or invalid
  // weather.
  ForwardSample measure(const Vec3& position, double heading_x, double heading_y,
                        const IScene& scene, const Weather& weather);

  const ForwardSensorConfig& config() const { return config_; }

 private:
  ForwardSensorConfig config_;
  Rng rng_;
};

}  // namespace swarm
