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

// An aircraft is on station when it is within this distance of its orbit reference point.
inline constexpr double kStationRadiusM = 30.0;

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
  std::uint64_t ticks = 0;    // records in the log
  std::string scenario_name;  // from the scenario in the log header
  std::uint64_t seed = 0;
  std::uint32_t drones = 0;
  double dt = 0.0;

  // Formation error: the root mean square, over alive drones and over the records considered, of
  // the distance from a drone's true position to its slot on the mission reference. The state
  // after the step of tick k is at time (k + 1) * dt, and the reference is evaluated there.
  // For a fixed-wing scenario the reference of a drone is its point on the orbit at that time
  // (orbit.hpp), and the recovery metrics below use the same error.
  // `formation_rmse` covers every record with drone state; `formation_rmse_settled` only those at
  // or after `metrics.settle_s`. Failed drones are excluded.
  std::optional<double> formation_rmse;
  std::optional<double> formation_rmse_settled;
  std::uint32_t alive_at_end = 0;  // alive drones in the last record with drone state

  // Orbit metrics, for a fixed-wing scenario only; empty for a quad. Over every alive drone and
  // every record with drone state, with R_i, z_i, and theta_i the radius, altitude, and angle of
  // the drone's orbit reference at the time of the state:
  //   orbit_radial_rmse    RMS of (horizontal distance from the center - R_i), meters
  //   orbit_altitude_rmse  RMS of (altitude - z_i), meters
  //   orbit_phase_rmse     RMS of R_i * wrap(theta_i - own angle), the arc length of the phase
  //                        error in meters; positive when the drone is behind its reference
  //   time_on_station      the fraction of samples within kStationRadiusM of the reference
  std::optional<double> orbit_radial_rmse;
  std::optional<double> orbit_altitude_rmse;
  std::optional<double> orbit_phase_rmse;
  std::optional<double> time_on_station;
  // With a pattern other than the orbit (ADR 0011) the reference of a drone is its point on its
  // stadium path, `orbit_radial_rmse` is the RMS cross-track error, and `orbit_phase_rmse` the RMS
  // distance along the path to its reference point. The altitude reference is the nominal layer,
  // so with terrain guidance `orbit_altitude_rmse` includes the climb over high ground.
  // Height above ground, for a fixed-wing scenario whose scene can be built (a scene file that
  // is missing leaves them empty). `min_agl` is the lowest height of any aircraft in any record
  // above the highest surface under it, meters: the terrain, or the top of an obstacle whose
  // footprint contains the point. `ground_contacts` the number of aircraft that were at or below
  // that surface in some record, which is the number that crashed when the scenario sets
  // `ground_contact_crashes`.
  std::optional<double> min_agl;
  // The mean of the same height over every aircraft and every record, meters: what staying clear
  // of the ground costs in height (ADR 0013).
  std::optional<double> mean_agl;
  std::optional<std::uint32_t> ground_contacts;

  // The share of the alive drones, counted over every record with drone state, that were in each
  // fallback state: degraded communication, degraded navigation, and safe hold. The rest of the
  // share is the nominal state. Empty when no drone was alive in any record. A run in which no
  // drone left the nominal state has 0 in all three, and a run with the fallback switched off
  // still reports the state its machine reached.
  std::optional<double> fallback_comms;
  std::optional<double> fallback_navigation;
  std::optional<double> fallback_safe_hold;

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
