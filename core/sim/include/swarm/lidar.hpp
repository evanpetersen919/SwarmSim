#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "swarm/rng.hpp"
#include "swarm/scene.hpp"
#include "swarm/vec3.hpp"
#include "swarm/weather.hpp"

namespace swarm {

struct LidarConfig {
  std::uint32_t beams = 16;              // vertical channels
  std::uint32_t azimuth_samples = 90;    // rays per beam per scan
  double vertical_fov_min_deg = -30.0;   // elevation of the lowest beam
  double vertical_fov_max_deg = 10.0;    // elevation of the highest beam
  double horizontal_fov_deg = 360.0;     // centered on the sensor's forward direction
  double min_range = 0.5;                // meters; closer hits are not reported
  double max_range = 60.0;               // meters; farther hits are not reported
  double range_noise_sigma = 0.02;       // meters, constant term
  double range_noise_sigma_per_m = 0.0;  // meters of sigma per meter of range
  double dropout_probability = 0.0;      // chance that a valid return is lost, in [0, 1]
  double terrain_reflectivity = 0.3;     // relative reflectivity of the ground, in (0, 1]
  double obstacle_reflectivity = 0.6;    // relative reflectivity of obstacles, in (0, 1]
  // A return weaker than this relative intensity is not detected, in [0, 1). With clear air the
  // intensity is the reflectivity, so a threshold below it never hides a return by itself.
  double intensity_threshold = 0.05;
};

struct LidarReturn {
  bool valid = false;
  double range = 0.0;      // measured, with noise; 0 when not valid
  double intensity = 0.0;  // relative intensity after attenuation; 0 when not valid
};

// One revolution. Returns are indexed beam * azimuth_samples + azimuth.
struct Scan {
  std::uint32_t beams = 0;
  std::uint32_t azimuth_samples = 0;
  std::vector<LidarReturn> returns;

  const LidarReturn& at(std::uint32_t beam, std::uint32_t azimuth) const {
    return returns[static_cast<std::size_t>(beam) * azimuth_samples + azimuth];
  }
};

// Spinning LiDAR mounted level on the drone at its position, yawing with it.
//
// Beam `b` has elevation spread evenly over the vertical field of view (a single beam sits at the
// middle of it). Sample `k` has azimuth `-fov/2 + (k + 0.5) * fov / N` relative to the drone's
// heading. Motion during the revolution is ignored: the whole scan is taken at one pose.
//
// Weather attenuates each return (see swarm/weather.hpp); a return whose relative intensity falls
// below `intensity_threshold` is not detected. Intensity uses the true range, not the noisy one.
//
// Every scan draws the same random values per ray, whether or not the ray hits anything: one
// normal for range noise, then one uniform for dropout. Changing the scene therefore never shifts
// the noise on unrelated rays. Uses stream kStreamLidarBase + drone id.
//
// Beam directions use sin and cos, so scans are reproducible on the same binary and platform but
// not guaranteed bit-identical across platforms.
class Lidar {
 public:
  // Throws std::invalid_argument for an invalid configuration.
  Lidar(const LidarConfig& config, std::uint64_t seed, std::uint32_t drone_id);

  const LidarConfig& config() const { return config_; }
  std::size_t ray_count() const {
    return static_cast<std::size_t>(config_.beams) * config_.azimuth_samples;
  }

  // Unit direction of one ray for a sensor with the given yaw (radians).
  Vec3 beam_direction(std::uint32_t beam, std::uint32_t azimuth, double yaw) const;

  // Takes one scan in clear air.
  Scan scan(const Vec3& origin, double yaw, const IScene& scene);
  // Takes one scan through the given weather. Throws std::invalid_argument for invalid weather.
  Scan scan(const Vec3& origin, double yaw, const IScene& scene, const Weather& weather);

 private:
  LidarConfig config_;
  Rng rng_;
  // Unit directions relative to the heading, one per ray, in scan order.
  std::vector<Vec3> pattern_;
};

}  // namespace swarm
