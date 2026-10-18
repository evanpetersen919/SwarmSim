#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

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

enum class DisturbanceKind {
  kJamEnd,  // the end of a finite jammer window
  kFault,   // a drone failing
};

// How the formation recovered from one disturbance. With E(k) the formation RMSE over alive drones
// at tick k, and t_d the disturbance tick, recovery is the first tick t_r >= t_d from which E stays
// at or below `metrics.recovery_threshold_m` for `metrics.recovery_hold_s`. Then `seconds` is
// (t_r - t_d) * dt and `censored` is false. If E never does that before the log ends, `censored` is
// true and `seconds` is the time from t_d to the end of the log: a lower bound on the recovery
// time, not a recovery time. Records without drone state are skipped.
struct Recovery {
  DisturbanceKind kind = DisturbanceKind::kJamEnd;
  std::uint64_t tick = 0;  // t_d
  double seconds = 0.0;
  bool censored = false;
};

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

  // Messages, from the events in the records (every tick has them, whatever the snapshot stride).
  // A scheduled message is delivered at its delivery tick. One whose delivery tick is after the
  // last record is still in flight when the log ends and is counted as such, and is left out of
  // the latency and of the delivery ratio.
  std::uint64_t messages_sent = 0;       // every event
  std::uint64_t messages_delivered = 0;  // scheduled, with a delivery tick within the log
  std::uint64_t messages_in_flight = 0;
  std::uint64_t dropped_loss = 0;
  std::uint64_t dropped_rate = 0;
  std::uint64_t dropped_jam = 0;
  // delivered / (delivered + all drops). Empty when no message has been decided.
  std::optional<double> delivery_ratio;
  // Percentiles of (delivery tick - send tick) * dt over delivered messages, by the nearest-rank
  // rule: the value at index ceil(p * n) - 1 of the sorted list. Empty when none was delivered.
  std::optional<double> latency_p50_s;
  std::optional<double> latency_p95_s;

  // Recovery after each disturbance seen in the log, in tick order (a jam end before a fault at
  // the same tick). Empty when the scenario has none that the log covers.
  std::vector<Recovery> recoveries;
  // The longest `seconds` among `recoveries`, censored ones included; empty if there are none.
  std::optional<double> recovery_longest;
  bool any_recovery_censored = false;
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
