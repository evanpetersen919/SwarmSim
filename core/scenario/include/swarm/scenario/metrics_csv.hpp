#pragma once

#include <string>

#include "swarm/scenario/metrics.hpp"

namespace swarm {

// CSV for metrics: one header row and one row per run, fixed columns, written by these two
// functions only, so files from different runs can be concatenated as they are.
//
// Columns, in order:
//   scenario, seed, drones, dt_s, ticks, duration_s,
//   formation_rmse_m, formation_rmse_settled_m, alive_at_end,
//   min_separation_m, collision_episodes,
//   recovery_count, recovery_longest_s, recovery_censored,
//   messages_sent, messages_delivered, messages_in_flight,
//   dropped_loss, dropped_rate, dropped_jam,
//   delivery_ratio, latency_p50_s, latency_p95_s,
//   orbit_radial_rmse_m, orbit_altitude_rmse_m, orbit_phase_rmse_m, time_on_station, min_agl_m,
//   ground_contacts
// The last four are empty for a quad scenario (see MetricsResult).
//
// Rules: lines end with a line feed. Whole numbers are written in decimal. Other numbers are
// written with "%.9g", which does not depend on the locale because the project never sets one. A
// value that does not exist is an empty cell, never zero. `recovery_censored` is 1 if any
// recovery result was censored and 0 if none was, and empty when the run had no disturbance. A
// scenario name containing a comma, a quote, or a line break is quoted, with quotes doubled.
std::string metrics_csv_header();

// The row for one run, ending with a line feed.
std::string metrics_csv_row(const MetricsResult& result);

}  // namespace swarm
