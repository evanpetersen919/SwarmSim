#include "swarm/control/mission.hpp"

#include <cmath>
#include <stdexcept>

namespace swarm {

namespace {

bool finite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

}  // namespace

MissionPath::MissionPath(const Mission& mission) : mission_(mission) {
  if (mission.waypoints.empty()) {
    throw std::invalid_argument("Mission: at least one waypoint is required");
  }
  if (!(std::isfinite(mission.speed) && mission.speed > 0.0)) {
    throw std::invalid_argument("Mission: speed must be finite and positive");
  }
  for (const Vec3& w : mission.waypoints) {
    if (!finite(w)) {
      throw std::invalid_argument("Mission: waypoints must be finite");
    }
  }

  const std::size_t n = mission.waypoints.size();
  const std::size_t count = mission.loop ? n : n - 1;  // a loop adds the closing segment
  double travelled = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const Vec3& from = mission.waypoints[i];
    const Vec3& to = mission.waypoints[(i + 1) % n];
    const double len = distance(from, to);
    if (!(len > 0.0)) {
      continue;  // a repeated waypoint takes no time
    }
    segments_.push_back({from, (to - from) / len, travelled, len});
    travelled += len;
  }
  length_ = travelled;
}

MissionReference MissionPath::at(double time_s) const {
  if (std::isnan(time_s)) {
    throw std::invalid_argument("MissionPath::at: time must be a number");
  }
  const double t = time_s > 0.0 ? time_s : 0.0;

  MissionReference ref;
  if (segments_.empty()) {
    // A single point, or every waypoint identical: hold there.
    ref.position = mission_.waypoints.front();
    ref.finished = !mission_.loop;
    return ref;
  }

  double s = mission_.speed * t;
  if (mission_.loop) {
    s = std::fmod(s, length_);
  } else if (s >= length_) {
    ref.position = segments_.back().from + segments_.back().direction * segments_.back().length;
    ref.finished = true;
    return ref;
  }

  // The last segment that starts at or before s. Segments are few, so a linear scan is fine.
  std::size_t index = 0;
  while (index + 1 < segments_.size() && segments_[index + 1].start <= s) {
    ++index;
  }
  const Segment& seg = segments_[index];
  ref.position = seg.from + seg.direction * (s - seg.start);
  ref.velocity = seg.direction * mission_.speed;
  return ref;
}

}  // namespace swarm
