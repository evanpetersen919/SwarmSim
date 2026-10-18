#pragma once

// Controller headers may include only this directory, swarm/estimation/, sensor_data.hpp,
// vec3.hpp, drone_command.hpp, and swarm/format/byte_io.hpp. They must never include vehicle truth.
// Enforced by scripts/check_core_invariants.sh.

#include <vector>

#include "swarm/vec3.hpp"

namespace swarm {

struct Mission {
  std::vector<Vec3> waypoints;  // at least one; consecutive repeats are allowed
  double speed = 5.0;           // cruise speed along the path, m/s, finite and positive
  bool loop = false;            // after the last waypoint, return to the first and repeat
};

// Where the shared mission reference is at some time.
struct MissionReference {
  Vec3 position;
  Vec3 velocity;
  // True once a non-looping mission has reached its last waypoint, where it stays. A looping
  // mission is never finished.
  bool finished = false;
};

// A polyline through the waypoints, traversed at constant speed starting at time 0. It is a pure
// function of time, so every drone that has the same mission computes the same reference without
// exchanging any message.
//
// Position is continuous everywhere; velocity is the direction of the current segment times the
// speed, so it changes at corners. A looping mission includes the closing segment from the last
// waypoint back to the first. Zero-length segments take no time. A mission whose total length is
// zero holds at its first waypoint with zero velocity.
class MissionPath {
 public:
  // Throws std::invalid_argument for no waypoints, a non-finite waypoint, or a speed that is not
  // finite and positive.
  explicit MissionPath(const Mission& mission);

  // Times before 0 are treated as 0. Throws std::invalid_argument if `time_s` is not a number.
  MissionReference at(double time_s) const;

  // Total length of one pass of the path in meters, including the closing segment when looping.
  double length() const { return length_; }
  // Time to travel one pass, in seconds.
  double duration() const { return length_ / mission_.speed; }

 private:
  Mission mission_;
  // Segment i runs from point(i) to point(i + 1 mod n) and starts at cumulative_[i] meters. Only
  // segments of positive length are kept.
  struct Segment {
    Vec3 from;
    Vec3 direction;  // unit length
    double start;    // meters along the path
    double length;   // meters
  };
  std::vector<Segment> segments_;
  double length_ = 0.0;
};

}  // namespace swarm
