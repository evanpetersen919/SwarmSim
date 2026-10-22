#pragma once
// Controller headers may include only this directory, swarm/estimation/, sensor_data.hpp,
// vec3.hpp, drone_command.hpp, message.hpp, and swarm/format/{byte_io,hash}.hpp. They must
// never include vehicle truth.
// Enforced by scripts/check_core_invariants.sh.

#include <cstdint>
#include <vector>

#include "swarm/control/orbit.hpp"
#include "swarm/control/terrain_map.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

// Search and loiter patterns for a vehicle that cannot stop (ADR 0011). They are standard
// patterns, and assumptions: public sources do not describe what the real aircraft fly.
//
//   kOrbit      the circle of orbit.hpp, one per slot around a shared center
//   kRacetrack  every aircraft flies the same stadium (two straights and two semicircles) around
//               the center, on concentric stadiums and spread along the loop
//   kLanes      each aircraft flies its own stadium, side by side across the area, with adjacent
//               lanes flown in opposite directions
enum class PatternKind : std::uint8_t {
  kOrbit = 0,
  kRacetrack = 1,
  kLanes = 2,
};

struct PatternParams {
  PatternKind kind = PatternKind::kOrbit;
  double heading = 0.0;         // rad, direction of the straights, counterclockwise from +x
  double leg_length = 600.0;    // m, length of each straight
  double turn_radius = 100.0;   // m, radius of the semicircles of the base stadium
  double lane_spacing = 300.0;  // m between the centers of adjacent lanes (kLanes)
  // Seconds between aircraft along the loop: aircraft i is i * stagger_s of flying behind aircraft
  // 0. 0 spreads the aircraft evenly along the loop. The press reports launches 10 to 15 s apart.
  double stagger_s = 0.0;
  // An aircraft farther than this fraction of the turn radius from its path is joining it: it
  // flies at the nominal speed toward the path and does not try to hold its phase.
  double join_fraction = 0.5;
};

// Throws std::invalid_argument unless every value is finite, the leg length is not negative, the
// radii and spacing are positive, the stagger is not negative, and the join fraction is positive.
void validate(const PatternParams& params);

// A closed stadium path: straights of length `leg_length` along +u at the lateral offset -R and
// along -u at +R, joined by semicircles of radius R at the ends, flown counterclockwise
// (`direction` +1) or clockwise (-1). With a zero leg length it is a circle. Distance along the
// path is `s` in [0, length), starting at the beginning of the first straight.
class StadiumPath {
 public:
  // Throws std::invalid_argument for a non-finite or non-positive radius, a negative or
  // non-finite leg length, a non-finite center, or a direction other than +1 or -1.
  StadiumPath(double cx, double cy, double heading, double leg_length, double radius,
              int direction);

  double length() const { return length_; }
  double radius() const { return radius_; }

  struct Point {
    double x = 0.0;
    double y = 0.0;
    double tangent_x = 1.0;  // unit tangent in the direction of flight
    double tangent_y = 0.0;
    double curvature = 0.0;  // 1 / m, positive when the path turns left
    double s = 0.0;
  };

  // The point at distance `s` along the path; `s` is taken modulo the length.
  Point at(double s) const;

  struct Nearest {
    Point point;
    double cross_track = 0.0;  // m, positive when the query is to the left of the path
    double distance = 0.0;     // m, to the nearest point
  };

  // The point of the path nearest to (x, y).
  Nearest nearest(double x, double y) const;

 private:
  double cx_, cy_;
  double ux_, uy_;  // unit direction of the straights
  double px_, py_;  // unit left normal in the direction of flight's frame, flipped for clockwise
  double leg_;
  double radius_;
  double length_;
  int direction_;
};

// The path of slot `slot` of `n` under `params`, the orbit parameters supplying the center, the
// radius step, and the altitude step. For kRacetrack, odd slots fly the stadium widened by
// radius_step, as odd slots of an orbit fly a larger circle. For kLanes the lanes are centered on
// the center across the heading, `lane_spacing` apart, with direction alternating by slot.
// Throws std::invalid_argument for kOrbit, a slot not below n, or invalid parameters.
StadiumPath pattern_path(std::uint32_t slot, std::uint32_t n, const PatternParams& params,
                         const OrbitParams& orbit);

// Where slot `slot` is supposed to be at `time_s`, and how fast. Every slot completes a loop in
// the same time, `length of slot 0's loop / orbit.speed`, so the spacing along the loops does not
// drift; a longer loop is flown faster. The altitude is the center altitude plus slot *
// altitude_step.
struct PatternReference {
  Vec3 position;
  Vec3 velocity;
  double s = 0.0;      // distance along the slot's own path
  double speed = 0.0;  // m/s along the path
};
PatternReference pattern_reference(std::uint32_t slot, std::uint32_t n, double time_s,
                                   const PatternParams& params, const OrbitParams& orbit);

// Acceleration that takes an aircraft onto the path and along it. With the nearest point p*, the
// path's unit tangent t and left normal n there, and e the cross-track error (positive to the
// left):
//   direction     = normalize(t + clamp(-radial_gain * e / radius, -1, 1) * n)
//   desired speed = clamp(speed + phase_gain * phase_error, min + margin, max - margin)
//   horizontal a  = desired speed^2 * curvature * n + velocity_gain * gain_scale *
//                   (desired speed * direction - v)
//   vertical a    = gain_scale * (altitude_gain * (z_ref - z) - altitude_damping * v_z)
// `phase_error` is in meters along the path, positive when the aircraft is behind its reference.
// While the aircraft is joining (more than join_fraction * radius from the path) the phase error
// is taken as zero. Throws std::invalid_argument for non-finite input, a non-positive speed or a
// negative gain scale.
Vec3 path_guidance(const Vec3& position, const Vec3& velocity, const StadiumPath& path,
                   double speed, double z_ref, double phase_error, const PatternParams& params,
                   const OrbitParams& orbit, double gain_scale);

// The signed distance in meters along the path from the aircraft's nearest point to the reference
// distance `s_ref`, wrapped into (-length / 2, length / 2]; positive when the reference is ahead.
double path_phase_error(const StadiumPath& path, const Vec3& position, double s_ref);

// The samples of the path ahead of the aircraft, from its nearest point once around, for terrain
// guidance. The distance counts from the aircraft to that point and then along the path.
std::vector<PathSample> path_samples(const StadiumPath& path, const Vec3& position,
                                     const TerrainGuidanceParams& params);

}  // namespace swarm
