#include "swarm/scenario/metrics_csv.hpp"

#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>

namespace swarm {

namespace {

std::string number(double value) {
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.9g", value);
  return buf;
}

std::string optional_number(const std::optional<double>& value) {
  return value ? number(*value) : std::string();
}

std::string whole(std::uint64_t value) { return std::to_string(value); }

std::string field(const std::string& text) {
  if (text.find_first_of(",\"\r\n") == std::string::npos) {
    return text;
  }
  std::string out = "\"";
  for (char c : text) {
    if (c == '"') {
      out += '"';
    }
    out += c;
  }
  return out + "\"";
}

}  // namespace

std::string metrics_csv_header() {
  return "scenario,seed,drones,dt_s,ticks,duration_s,"
         "formation_rmse_m,formation_rmse_settled_m,alive_at_end,"
         "min_separation_m,collision_episodes,"
         "recovery_count,recovery_longest_s,recovery_censored,"
         "messages_sent,messages_delivered,messages_in_flight,"
         "dropped_loss,dropped_rate,dropped_jam,"
         "delivery_ratio,latency_p50_s,latency_p95_s,"
         "orbit_radial_rmse_m,orbit_altitude_rmse_m,orbit_phase_rmse_m,time_on_station,"
         "min_agl_m,ground_contacts\n";
}

std::string metrics_csv_row(const MetricsResult& r) {
  std::string row;
  // Count the cells, not look at the text: an empty first cell still needs its separator after it.
  std::size_t cells = 0;
  const auto push = [&](const std::string& cell) {
    if (cells++ > 0) {
      row += ',';
    }
    row += cell;
  };

  push(field(r.scenario_name));
  push(whole(r.seed));
  push(whole(r.drones));
  push(number(r.dt));
  push(whole(r.ticks));
  push(number(static_cast<double>(r.ticks) * r.dt));
  push(optional_number(r.formation_rmse));
  push(optional_number(r.formation_rmse_settled));
  push(whole(r.alive_at_end));
  push(optional_number(r.min_separation));
  push(whole(r.collision_episodes));
  push(whole(r.recoveries.size()));
  push(optional_number(r.recovery_longest));
  push(r.recoveries.empty() ? std::string() : std::string(r.any_recovery_censored ? "1" : "0"));
  push(whole(r.messages_sent));
  push(whole(r.messages_delivered));
  push(whole(r.messages_in_flight));
  push(whole(r.dropped_loss));
  push(whole(r.dropped_rate));
  push(whole(r.dropped_jam));
  push(optional_number(r.delivery_ratio));
  push(optional_number(r.latency_p50_s));
  push(optional_number(r.latency_p95_s));
  push(optional_number(r.orbit_radial_rmse));
  push(optional_number(r.orbit_altitude_rmse));
  push(optional_number(r.orbit_phase_rmse));
  push(optional_number(r.time_on_station));
  push(optional_number(r.min_agl));
  push(r.ground_contacts ? whole(*r.ground_contacts) : std::string());
  return row + "\n";
}

}  // namespace swarm
