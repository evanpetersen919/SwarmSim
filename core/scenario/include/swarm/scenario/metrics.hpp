#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace swarm {

enum class MetricsStatus {
  kOk,
  kLogError,        // the log could not be read; `message` says why
  kNotScenarioLog,  // the log is not a scenario log (for example a demo log)
  kBadScenario,     // the scenario in the log header does not load
  kBadSnapshot,     // a record's blob does not decode, or disagrees with the header
  kNoRecords,       // the log has no records
};

const char* to_string(MetricsStatus status);

// Everything computed from a log. A value that cannot be computed from the log (no drone alive, no
// record at or after the settle time) is empty, never zero.
struct MetricsResult {
  std::uint64_t ticks = 0;  // records in the log
  std::uint32_t drones = 0;
  double dt = 0.0;

  // Formation error: the root mean square, over alive drones and over the records considered, of
  // the distance from a drone's true position to its slot on the mission reference. The state
  // after the step of tick k is at time (k + 1) * dt, and the reference is evaluated there.
  // `formation_rmse` covers every record with drone state; `formation_rmse_settled` only those at
  // or after `metrics.settle_s`. Failed drones are excluded.
  std::optional<double> formation_rmse;
  std::optional<double> formation_rmse_settled;
  std::uint32_t alive_at_end = 0;  // alive drones in the last record with drone state

  // Smallest true distance between any two drones, alive or failed, over every record with drone
  // state.
  std::optional<double> min_separation;

  // Collision episodes: a pair starts one when its distance falls below
  // `metrics.collision_radius_m` (a distance equal to the radius is not inside) and ends it when
  // the distance is back at or above the radius. A pair already inside in the first record starts
  // an episode there. The count is over all pairs.
  std::uint64_t collision_episodes = 0;
};

struct MetricsOutcome {
  MetricsStatus status = MetricsStatus::kOk;
  std::string message;  // empty when ok
  MetricsResult result;
  bool ok() const { return status == MetricsStatus::kOk; }
};

// Computes the metrics from a scenario log. Reads the log bytes and nothing else: the scenario it
// needs (mission, formation, metric settings) is in the header. Records without drone state (the
// ticks a snapshot stride skips) are skipped for the state metrics.
MetricsOutcome analyze_log(const std::uint8_t* data, std::size_t size);

}  // namespace swarm
