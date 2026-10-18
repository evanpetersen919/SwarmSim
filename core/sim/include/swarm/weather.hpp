#pragma once

namespace swarm {

// Atmospheric conditions that attenuate LiDAR returns.
//
// Fog and haze set the extinction coefficient through the Koschmieder relation
//   alpha_fog = 3.912 / visibility
// (3.912 = ln(1/0.02), the 2% contrast threshold). Rain adds a power law in the rain rate
//   alpha_rain = rain_coefficient * rate^rain_exponent
// and the two add. A return from a surface of reflectivity rho at range R has relative intensity
//   rho * exp(-2 * alpha * R)
// (the beam travels out and back).
//
// The rain constants are placeholders chosen to give an extinction of a few per kilometer in
// moderate rain. They are not taken from a source and have not been validated; see
// docs/MODELS.md. The fog relation is the standard one and is the part that gets validated.
struct Weather {
  double visibility_m = 20000.0;     // meteorological visibility, meters
  double rain_rate_mm_h = 0.0;       // rainfall rate, millimeters per hour
  double rain_coefficient = 4.0e-4;  // 1/m per (mm/h)^rain_exponent. Placeholder.
  double rain_exponent = 0.6;        // Placeholder.

  static Weather clear() { return Weather{}; }
  static Weather fog(double visibility_m);
  static Weather rain(double rate_mm_h, double visibility_m = 20000.0);

  // Throws std::invalid_argument for non-finite values, non-positive visibility or exponent, or
  // negative rain rate or coefficient.
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
