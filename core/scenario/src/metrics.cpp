#include "swarm/scenario/metrics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "swarm/control/formation.hpp"
#include "swarm/control/mission.hpp"
#include "swarm/control/orbit.hpp"
#include "swarm/control/path.hpp"
#include "swarm/format/log.hpp"
#include "swarm/format/snapshot.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"
#include "swarm/scene.hpp"
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
  const bool fixed_wing = scenario.vehicle.kind == VehicleKind::kFixedWing;
  const OrbitParams orbit = fixed_wing ? orbit_params(scenario) : OrbitParams{};

  // The ground under the aircraft, when the scene can be built.
  std::unique_ptr<Scene> scene;
  if (fixed_wing) {
    SceneData ground;
    std::string scene_error;
    if (build_scene_data(scenario, ground, scene_error)) {
      scene = std::make_unique<Scene>(std::move(ground));
    }
  }
  std::vector<char> touched(n, 0);
  double lowest_agl = std::numeric_limits<double>::infinity();
  const bool pattern = fixed_wing && scenario.pattern.kind != PatternKind::kOrbit;
  // With terrain guidance the altitude is raised over high ground, so the distance to the nominal
  // reference would count the climb as being off station; then only the horizontal distance counts.
  const bool horizontal_station = scenario.terrain_guidance.enabled;
  const auto station_error2 = [&](double e2_full, const Vec3& p, const Vec3& target) {
    if (!horizontal_station) {
      return e2_full;
    }
    const double dx = p.x - target.x;
    const double dy = p.y - target.y;
    return dx * dx + dy * dy;
  };

  MetricsOutcome outcome;
  MetricsResult& out = outcome.result;
  out.scenario_name = scenario.name;
  out.seed = scenario.seed;
  out.drones = n;
  out.dt = scenario.dt;

  double sum_all = 0.0;
  std::uint64_t count_all = 0;
  double sum_settled = 0.0;
  std::uint64_t count_settled = 0;
  double sum_radial = 0.0;
  double sum_altitude = 0.0;
  double sum_phase = 0.0;
  std::uint64_t count_on_station = 0;
  std::uint64_t count_orbit = 0;
  std::vector<std::uint64_t> latencies_ticks;  // delivery tick - send tick, per delivered message
  std::vector<std::uint64_t> delivery_ticks_pending;    // scheduled messages, checked at the end
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
    for (const SnapshotEvent& e : snapshot.events) {
      ++out.messages_sent;
      switch (e.kind) {
        case SnapshotEventKind::kScheduled:
          if (e.deliver_tick < e.send_tick) {
            return failure(MetricsStatus::kBadSnapshot,
                           "tick " + std::to_string(record.tick) +
                               ": a message is delivered before it is sent");
          }
          latencies_ticks.push_back(e.deliver_tick - e.send_tick);
          delivery_ticks_pending.push_back(e.deliver_tick);
          break;
        case SnapshotEventKind::kDroppedLoss:
          ++out.dropped_loss;
          break;
        case SnapshotEventKind::kDroppedRate:
          ++out.dropped_rate;
          break;
        case SnapshotEventKind::kDroppedJam:
          ++out.dropped_jam;
          break;
      }
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
    // Where each drone should be: its slot on the mission reference, or its point on the orbit.
    std::vector<OrbitReference> orbit_refs;
    std::vector<PatternReference> pattern_refs;
    std::vector<Vec3> targets(n);
    for (std::uint32_t i = 0; i < n; ++i) {
      if (pattern) {
        pattern_refs.push_back(pattern_reference(i, n, time, scenario.pattern, orbit));
        targets[i] = pattern_refs.back().position;
      } else if (fixed_wing) {
        orbit_refs.push_back(orbit_reference(i, n, time, orbit));
        targets[i] = orbit_refs.back().position;
      } else {
        targets[i] = center + offsets[i];
      }
    }
    std::uint32_t alive = 0;
    for (std::uint32_t i = 0; i < n; ++i) {
      const AgentSnapshot& a = snapshot.agents[i];
      if (scene) {
        const double agl = a.position[2] - scene->surface_height(a.position[0], a.position[1]);
        lowest_agl = std::min(lowest_agl, agl);
        if (agl <= 0.0) {
          touched[i] = 1;
        }
      }
      if (!a.alive) {
        continue;
      }
      ++alive;
      const double e2 = norm_squared(to_vec(a.position) - targets[i]);
      if (pattern) {
        const PatternReference& ref = pattern_refs[i];
        const Vec3 p = to_vec(a.position);
        double cross = 0.0;
        double phase = 0.0;
        if (scenario.pattern.kind == PatternKind::kFormation) {
          // Against the aircraft's own place in the formation: the error to its left (across) and
          // behind (along) the direction the reference is flying.
          const double tx = ref.velocity.x / ref.speed;
          const double ty = ref.velocity.y / ref.speed;
          const double ex = p.x - ref.position.x;
          const double ey = p.y - ref.position.y;
          cross = ex * -ty + ey * tx;
          phase = -(ex * tx + ey * ty);
        } else {
          const StadiumPath own = pattern_path(i, n, scenario.pattern, orbit);
          cross = own.nearest(p.x, p.y).cross_track;
          phase = path_phase_error(own, p, ref.s);
        }
        const double altitude = p.z - ref.position.z;
        sum_radial += cross * cross;
        sum_altitude += altitude * altitude;
        sum_phase += phase * phase;
        count_on_station +=
            station_error2(e2, p, targets[i]) <= kStationRadiusM * kStationRadiusM ? 1U : 0U;
        ++count_orbit;
      } else if (fixed_wing) {
        const OrbitReference& ref = orbit_refs[i];
        const Vec3 p = to_vec(a.position);
        const double dx = p.x - orbit.center.x;
        const double dy = p.y - orbit.center.y;
        const double radial = std::hypot(dx, dy) - ref.radius;
        const double altitude = p.z - ref.position.z;
        const double phase = ref.radius * wrap_angle(ref.angle - std::atan2(dy, dx));
        sum_radial += radial * radial;
        sum_altitude += altitude * altitude;
        sum_phase += phase * phase;
        count_on_station +=
            station_error2(e2, p, targets[i]) <= kStationRadiusM * kStationRadiusM ? 1U : 0U;
        ++count_orbit;
      }
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
          sum += norm_squared(to_vec(snapshot.agents[i].position) - targets[i]);
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
  if (count_orbit > 0) {
    const double samples = static_cast<double>(count_orbit);
    out.orbit_radial_rmse = std::sqrt(sum_radial / samples);
    out.orbit_altitude_rmse = std::sqrt(sum_altitude / samples);
    out.orbit_phase_rmse = std::sqrt(sum_phase / samples);
    out.time_on_station = static_cast<double>(count_on_station) / samples;
  }
  if (scene) {
    out.min_agl = lowest_agl;
    out.ground_contacts = static_cast<std::uint32_t>(std::count(touched.begin(), touched.end(), 1));
  }

  // Messages. The last tick in the log is out.ticks - 1; anything scheduled after it is in flight.
  std::vector<std::uint64_t> delivered_latency;
  for (std::size_t k = 0; k < latencies_ticks.size(); ++k) {
    if (delivery_ticks_pending[k] < out.ticks) {
      delivered_latency.push_back(latencies_ticks[k]);
    } else {
      ++out.messages_in_flight;
    }
  }
  out.messages_delivered = delivered_latency.size();
  const std::uint64_t decided =
      out.messages_delivered + out.dropped_loss + out.dropped_rate + out.dropped_jam;
  if (decided > 0) {
    out.delivery_ratio = static_cast<double>(out.messages_delivered) / static_cast<double>(decided);
  }
  if (!delivered_latency.empty()) {
    std::sort(delivered_latency.begin(), delivered_latency.end());
    const std::uint64_t count = delivered_latency.size();
    // Nearest rank with integer arithmetic: ceil(0.50 * n) and ceil(0.95 * n).
    const std::uint64_t rank50 = (50 * count + 99) / 100;
    const std::uint64_t rank95 = (95 * count + 99) / 100;
    out.latency_p50_s = static_cast<double>(delivered_latency[rank50 - 1]) * scenario.dt;
    out.latency_p95_s = static_cast<double>(delivered_latency[rank95 - 1]) * scenario.dt;
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
