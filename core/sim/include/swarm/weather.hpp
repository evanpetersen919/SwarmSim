#pragma once

namespace swarm {

// Atmospheric conditions that attenuate LiDAR returns.
//
// Fog and haze set the extinction coefficient from the visibility,
//   alpha_fog = fog_constant / visibility
// where the default 3.912 is the Koschmieder constant ln(1/0.02), the 2% contrast threshold. The
// constant depends on how visibility is defined: a 5% threshold, ln(1/0.05) = 2.996, gives
// about 3, which is the value Rasshofer et al. (2011) used for a 905 nm automotive laser radar.
// See docs/MODELS.md. Rain adds a power law in the rain rate
//   alpha_rain = rain_coefficient * rate^rain_exponent
// and the two add. A return from a surface of reflectivity rho at range R has relative intensity
//   rho * exp(-2 * alpha * R)
// (the beam travels out and back).
//
// The rain constants are placeholders chosen to give an extinction of a few per kilometer in
// moderate rain. They are not taken from a source and have not been validated; see
// docs/MODELS.md. The fog relation is the standard one and is the part that gets validated.
// Default of Weather::fog_constant.
inline constexpr double kKoschmiederConstant = 3.912;

struct Weather {
  double visibility_m = 20000.0;               // meteorological visibility, meters
  double fog_constant = kKoschmiederConstant;  // alpha_fog * visibility, dimensionless
  double rain_rate_mm_h = 0.0;                 // rainfall rate, millimeters per hour
  double rain_coefficient = 4.0e-4;            // 1/m per (mm/h)^rain_exponent. Placeholder.
  double rain_exponent = 0.6;                  // Placeholder.

  static Weather clear() { return Weather{}; }
  static Weather fog(double visibility_m);
  static Weather rain(double rate_mm_h, double visibility_m = 20000.0);

  // Throws std::invalid_argument for non-finite values, non-positive visibility, fog constant or
  // exponent, a fog constant above 20, or negative rain rate or coefficient.
  void validate() const;

  // Extinction coefficient in 1/m.
  double extinction_per_m() const;

  // Relative intensity of a return from a surface with the given reflectivity at `range` meters.
  double intensity(double reflectivity, double range) const;

  // Largest range at which a surface of the given reflectivity still reaches `threshold`
  // intensity. Infinite when the extinction is zero.
  double max_detection_range(double reflectivity, double threshold) const;
};

}  // namespace swarm
