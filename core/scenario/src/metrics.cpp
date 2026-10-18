#include "swarm/scenario/metrics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "swarm/control/formation.hpp"
#include "swarm/control/mission.hpp"
#include "swarm/format/log.hpp"
#include "swarm/format/snapshot.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

const char* to_string(MetricsStatus status) {
  switch (status) {
    case MetricsStatus::kOk:
      return "ok";
    case MetricsStatus::kLogError:
      return "log error";
    case MetricsStatus::kNotScenarioLog:
      return "not a scenario log";
    case MetricsStatus::kBadScenario:
      return "bad scenario";
    case MetricsStatus::kBadSnapshot:
      return "bad snapshot";
    case MetricsStatus::kNoRecords:
      return "no records";
  }
  return "unknown";
}

namespace {

MetricsOutcome failure(MetricsStatus status, std::string message) {
  MetricsOutcome outcome;
  outcome.status = status;
  outcome.message = std::move(message);
  return outcome;
}

Vec3 to_vec(const std::array<double, 3>& a) { return Vec3{a[0], a[1], a[2]}; }

}  // namespace

MetricsOutcome analyze_log(const std::uint8_t* data, std::size_t size) {
  LogReader reader(data, size);
  LogStatus status = reader.read_header();
  if (status != LogStatus::kOk) {
    return failure(MetricsStatus::kLogError, to_string(status));
  }
  const LogHeader& header = reader.header();
  if (!is_scenario_log(header)) {
    return failure(MetricsStatus::kNotScenarioLog, "the log does not hold a scenario");
  }
  const std::string tag = kScenarioLogTag;
  const LoadResult loaded = load_scenario(header.config.substr(tag.size()), "scenario");
  if (!loaded.ok()) {
    return failure(MetricsStatus::kBadScenario, to_string(loaded.errors.front()));
  }
  const Scenario& scenario = loaded.scenario;
  if (header.agent_count != scenario.drones || header.dt != scenario.dt) {
    return failure(MetricsStatus::kBadScenario, "the log header disagrees with its scenario");
  }

  const std::uint32_t n = scenario.drones;
  const MissionPath path(scenario.mission);
  const std::vector<Vec3> offsets =
      make_formation(scenario.formation_shape, n, scenario.formation_spacing_m);
  const double radius = scenario.metrics.collision_radius_m;

  MetricsOutcome outcome;
  MetricsResult& out = outcome.result;
  out.drones = n;
  out.dt = scenario.dt;

  double sum_all = 0.0;
  std::uint64_t count_all = 0;
  double sum_settled = 0.0;
  std::uint64_t count_settled = 0;
  std::vector<std::pair<std::uint64_t, double>> trace;  // (tick, formation RMSE), where defined
  std::vector<char> inside(static_cast<std::size_t>(n) * (n > 0 ? n - 1 : 0) / 2, 0);

  LogRecord record;
  Snapshot snapshot;
  for (;;) {
    status = reader.next(record);
    if (status == LogStatus::kEndOfLog) {
      break;
    }
    if (status != LogStatus::kOk) {
      return failure(MetricsStatus::kLogError, to_string(status));
    }
    ++out.ticks;

    const SnapshotStatus decoded =
        decode_snapshot(record.blob.data(), record.blob.size(), snapshot);
    if (decoded != SnapshotStatus::kOk) {
      return failure(
          MetricsStatus::kBadSnapshot,
          std::string("tick ") + std::to_string(record.tick) + ": " + to_string(decoded));
    }
    if (snapshot.agents.empty()) {
      continue;  // a tick the snapshot stride skipped
    }
    if (snapshot.agents.size() != n) {
      return failure(MetricsStatus::kBadSnapshot, "tick " + std::to_string(record.tick) +
                                                      ": the snapshot has " +
                                                      std::to_string(snapshot.agents.size()) +
                                                      " drones, the scenario " + std::to_string(n));
    }

    // Formation error, at the time the state describes.
    const double time = static_cast<double>(record.tick + 1) * scenario.dt;
    const Vec3 center = path.at(time).position;
    const bool settled = time >= scenario.metrics.settle_s;
    std::uint32_t alive = 0;
    for (std::uint32_t i = 0; i < n; ++i) {
      const AgentSnapshot& a = snapshot.agents[i];
      if (!a.alive) {
        continue;
      }
      ++alive;
      const double e2 = norm_squared(to_vec(a.position) - (center + offsets[i]));
      sum_all += e2;
      ++count_all;
      if (settled) {
        sum_settled += e2;
        ++count_settled;
      }
    }
    out.alive_at_end = alive;
    if (alive > 0) {
      double sum = 0.0;
      for (std::uint32_t i = 0; i < n; ++i) {
        if (snapshot.agents[i].alive) {
          sum += norm_squared(to_vec(snapshot.agents[i].position) - (center + offsets[i]));
        }
      }
      trace.emplace_back(record.tick, std::sqrt(sum / static_cast<double>(alive)));
    }

    // Separation and collisions, over every pair, alive or not.
    std::size_t pair = 0;
    for (std::uint32_t i = 0; i < n; ++i) {
      for (std::uint32_t j = i + 1; j < n; ++j, ++pair) {
        const double d =
            distance(to_vec(snapshot.agents[i].position), to_vec(snapshot.agents[j].position));
        if (!out.min_separation || d < *out.min_separation) {
          out.min_separation = d;
        }
        const bool now_inside = d < radius;
        if (now_inside && inside[pair] == 0) {
          ++out.collision_episodes;
        }
        inside[pair] = now_inside ? 1 : 0;
      }
    }
  }

  if (out.ticks == 0) {
    return failure(MetricsStatus::kNoRecords, "the log has no records");
  }
  if (count_all > 0) {
    out.formation_rmse = std::sqrt(sum_all / static_cast<double>(count_all));
  }
  if (count_settled > 0) {
    out.formation_rmse_settled = std::sqrt(sum_settled / static_cast<double>(count_settled));
  }

  // Recovery after each disturbance the log covers.
  std::vector<Recovery> disturbances;
  for (const Jammer& j : scenario.channel.jammers) {
    if (j.end_tick != std::numeric_limits<std::uint64_t>::max()) {
      disturbances.push_back({DisturbanceKind::kJamEnd, j.end_tick, 0.0, false});
    }
  }
  for (const Fault& f : scenario.faults) {
    disturbances.push_back({DisturbanceKind::kFault, f.tick, 0.0, false});
  }
  std::stable_sort(disturbances.begin(), disturbances.end(),
                   [](const Recovery& a, const Recovery& b) {
                     return a.tick != b.tick ? a.tick < b.tick : a.kind < b.kind;
                   });

  const double threshold = scenario.metrics.recovery_threshold_m;
  const std::uint64_t hold_ticks = static_cast<std::uint64_t>(
      std::max(1.0, std::ceil(scenario.metrics.recovery_hold_s / scenario.dt - 1e-9)));
  for (Recovery r : disturbances) {
    if (r.tick >= out.ticks) {
      continue;  // after the end of the log: not observed
    }
    bool recovered = false;
    bool in_run = false;
    std::uint64_t run_start = 0;
    for (const auto& point : trace) {
      if (point.first < r.tick) {
        continue;
      }
      if (point.second <= threshold) {
        if (!in_run) {
          in_run = true;
          run_start = point.first;
        }
        if (point.first - run_start + 1 >= hold_ticks) {
          recovered = true;
          break;
        }
      } else {
        in_run = false;
      }
    }
    if (recovered) {
      r.seconds = static_cast<double>(run_start - r.tick) * scenario.dt;
    } else {
      r.censored = true;
      r.seconds = static_cast<double>(out.ticks - r.tick) * scenario.dt;
    }
    out.recoveries.push_back(r);
  }
  for (const Recovery& r : out.recoveries) {
    if (!out.recovery_longest || r.seconds > *out.recovery_longest) {
      out.recovery_longest = r.seconds;
    }
    out.any_recovery_censored = out.any_recovery_censored || r.censored;
  }
  return outcome;
}

}  // namespace swarm
