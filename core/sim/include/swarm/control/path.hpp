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
//   kFormation  slot 0 flies a lead track (straights and turns of a stadium path) and the others
//               hold scheduled offsets from it: in trail, in echelon right or left, or in a vee.
//               This follows the one sequence that is documented for the ALTIUS-600: launches
//               10 to 15 s apart, then an echelon, a left echelon, and a vee (Inside Unmanned
//               Systems, "ALEs: Multiplatform Force Multipliers").
enum class PatternKind : std::uint8_t {
  kOrbit = 0,
  kRacetrack = 1,
  kLanes = 2,
  kFormation = 3,
};

// The shape of a formation: where slot i holds relative to the lead, in meters back along the
// lead's heading and to its left.
//   kTrail          i * (stagger_s * speed) back, no offset to the side
//   kEchelonRight   i * along back, i * across to the right
//   kEchelonLeft    i * along back, i * across to the left
//   kVee            k * along back and k * across to the side, with k = ceil(i / 2), odd slots on
//                   the left and even slots on the right
enum class FormationStyle : std::uint8_t {
  kTrail = 0,
  kEchelonRight = 1,
  kEchelonLeft = 2,
  kVee = 3,
};

// From `at_s` seconds on, the formation takes `shape`, moving into it over `transition_s`.
struct FormationPhase {
  double at_s = 0.0;
  FormationStyle shape = FormationStyle::kTrail;
};

struct FormationPlan {
  std::vector<FormationPhase> phases;  // sorted by at_s; empty means trail throughout
  double along = 50.0;                 // m back per rank
  double across = 30.0;                // m to the side per rank
  // Time to move from one shape to the next. A fixed-wing aircraft closes or opens a gap at a few
  // meters per second at most (the speed margin of its limits), so changing sides takes tens of
  // seconds and closing a trail of several launch intervals takes over a minute.
  double transition_s = 25.0;
  double position_gain = 0.15;  // 1/s^2: acceleration per meter of position error
  double velocity_gain = 0.8;   // 1/s: acceleration per m/s of velocity error
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
  FormationPlan formation;  // used by kFormation
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
  Vec3 acceleration;   // of the reference; zero except for a formation
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

// The lead track of a formation: the stadium path of `params`, translated so that the lead is at
// the center of `orbit` at time 0, flying along `params.heading`. The lead flies it at
// `orbit.speed`, so it is at distance `orbit.speed * t` along it at time t.
StadiumPath formation_lead(const PatternParams& params, const OrbitParams& orbit);

// Acceleration that holds a formation slot: the reference's acceleration plus gains on its
// position and velocity errors, the horizontal part limited to `orbit.max_accel`, and the
// vertical part the orbit's altitude law toward `z_ref`.
Vec3 formation_guidance(const Vec3& position, const Vec3& velocity, const PatternReference& ref,
                        double z_ref, const PatternParams& params, const OrbitParams& orbit,
                        double gain_scale);

// The samples of the track ahead of slot `slot` at time `time_s`, shifted by the slot's offset,
// for terrain guidance: `params.lookahead` meters of the lead track from where the lead is, at
// `params.sample_spacing`.
std::vector<PathSample> formation_samples(std::uint32_t slot, double time_s,
                                          const PatternParams& params, const OrbitParams& orbit,
                                          const TerrainGuidanceParams& terrain);

// The signed distance in meters along the path from the aircraft's nearest point to the reference
// distance `s_ref`, wrapped into (-length / 2, length / 2]; positive when the reference is ahead.
double path_phase_error(const StadiumPath& path, const Vec3& position, double s_ref);

// The samples of the path ahead of the aircraft, from its nearest point once around, for terrain
// guidance. The distance counts from the aircraft to that point and then along the path.
std::vector<PathSample> path_samples(const StadiumPath& path, const Vec3& position,
                                     const TerrainGuidanceParams& params);

}  // namespace swarm
