#pragma once

// Helpers for metric tests that build logs by hand: a scenario with a fixed reference, snapshots
// with chosen positions, and the log that holds them. No simulation is involved.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "swarm/control/formation.hpp"
#include "swarm/format/log.hpp"
#include "swarm/format/snapshot.hpp"
#include "swarm/scenario/metrics.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"

namespace swarm::testing {

using Bytes = std::vector<std::uint8_t>;

inline constexpr double kSpacing = 20.0;
inline const Vec3 kCenter{0.0, 0.0, 20.0};

// A scenario whose mission is a single waypoint, so the reference never moves, with dt of one
// second so that tick k describes time k + 1.
inline Scenario make_scenario(std::uint32_t drones, std::uint64_t ticks, double settle_s = 0.0) {
  Scenario s;
  s.name = "hand_built";
  s.dt = 1.0;
  s.duration_ticks = ticks;
  s.drones = drones;
  s.formation_shape = swarm::FormationShape::kLine;
  s.formation_spacing_m = kSpacing;
  s.mission.waypoints = {kCenter};
  s.metrics.settle_s = settle_s;
  s.metrics.collision_radius_m = 1.0;
  return s;
}

inline std::vector<Vec3> slots(const Scenario& s) {
  return swarm::make_formation(s.formation_shape, s.drones, s.formation_spacing_m);
}

inline AgentSnapshot at(Vec3 p, bool alive = true) {
  AgentSnapshot a;
  a.alive = alive;
  a.position = {p.x, p.y, p.z};
  return a;
}

// A snapshot with every drone on its slot, displaced by `shift` (a vector per drone).
inline Snapshot on_slots(const Scenario& s, const std::vector<Vec3>& shift) {
  Snapshot snap;
  const std::vector<Vec3> offsets = slots(s);
  for (std::uint32_t i = 0; i < s.drones; ++i) {
    snap.agents.push_back(at(kCenter + offsets[i] + shift[i]));
  }
  return snap;
}

inline Snapshot on_slots(const Scenario& s) {
  return on_slots(s, std::vector<Vec3>(s.drones, Vec3{}));
}

inline Bytes make_log(const Scenario& s, const std::vector<Snapshot>& snapshots) {
  swarm::LogHeader header;
  header.seed = 1;
  header.dt = s.dt;
  header.agent_count = s.drones;
  header.config = std::string(swarm::kScenarioLogTag) + swarm::canonical_text(s);
  swarm::LogWriter writer;
  EXPECT_EQ(writer.begin(header), swarm::LogStatus::kOk);
  for (std::size_t k = 0; k < snapshots.size(); ++k) {
    swarm::LogRecord r;
    r.tick = k;
    r.hash = k + 1;
    EXPECT_EQ(swarm::encode_snapshot(snapshots[k], r.blob), swarm::SnapshotStatus::kOk);
    EXPECT_EQ(writer.append(r), swarm::LogStatus::kOk);
  }
  EXPECT_EQ(writer.finish(), swarm::LogStatus::kOk);
  return writer.bytes();
}

inline swarm::MetricsOutcome analyze(const Bytes& bytes) {
  return swarm::analyze_log(bytes.data(), bytes.size());
}

}  // namespace swarm::testing
