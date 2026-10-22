#pragma once

#include <array>
#include <cstdint>

#include "swarm/vec3.hpp"

namespace swarm {

// Wind as a steady part and a gust field (ADR 0011).
//
// The gust field is a sum of traveling waves with seeded phases, directions, and wavelengths, so
// every aircraft sees the same air: the field is smooth in space and time, and a function of
// position, time, and seed alone, which keeps a run independent of thread scheduling. The waves
// are carried along by the steady wind (frozen turbulence), with a slow change of their own.
//
// The gust is not a model of any one spectrum. It has the requested rms and a largest scale, and
// nothing more; docs/MODELS.md says so.
struct WindParams {
  Vec3 mean;                    // m/s, the direction the air moves toward; z is usually 0
  double gust_sigma = 0.0;      // m/s, rms of each horizontal gust component; 0 means no gusts
  double gust_scale = 100.0;    // m, wavelength of the largest eddies
  double vertical_ratio = 0.5;  // vertical rms as a fraction of the horizontal
};

// Throws std::invalid_argument unless every value is finite, the sigma is not negative, the scale
// is positive, and the vertical ratio is in [0, 1].
void validate(const WindParams& params);

class WindField {
 public:
  // The number of waves summed.
  static constexpr int kWaves = 8;

  // Throws std::invalid_argument for invalid parameters.
  WindField(const WindParams& params, std::uint64_t seed);

  // The air velocity at `position` and `time_s`: the steady wind plus the gust. Throws
  // std::invalid_argument for a non-finite position or time.
  Vec3 at(const Vec3& position, double time_s) const;

  const WindParams& params() const { return params_; }

 private:
  struct Wave {
    double kx = 0.0;
    double ky = 0.0;
    double omega = 0.0;
    double phase = 0.0;
    double phase_z = 0.0;
    double dir_x = 0.0;  // unit direction of the horizontal gust of this wave
    double dir_y = 0.0;
  };
  WindParams params_;
  std::array<Wave, kWaves> waves_;
  double amplitude_ = 0.0;
  double amplitude_z_ = 0.0;
};

}  // namespace swarm
