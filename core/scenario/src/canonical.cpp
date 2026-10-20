#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>

#include "swarm/scenario/scenario.hpp"

namespace swarm {

namespace {

// Seventeen significant digits round-trip every double. `inf` is written for infinity, which only
// a jammer radius may be.
std::string num(double v) {
  if (std::isinf(v)) {
    return "inf";
  }
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

std::string uint(std::uint64_t v) { return std::to_string(v); }

std::string quoted(const std::string& text) {
  std::string out = "\"";
  for (char c : text) {
    const unsigned char u = static_cast<unsigned char>(c);
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (u < 0x20 || u == 0x7f) {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "\\x%02x", static_cast<unsigned>(u));
      out += buf;
    } else {
      out += c;
    }
  }
  return out + "\"";
}

const char* shape_name(FormationShape s) {
  switch (s) {
    case FormationShape::kLine:
      return "line";
    case FormationShape::kGrid:
      return "grid";
    case FormationShape::kWedge:
      break;
  }
  return "wedge";
}

const char* predictor_name(PredictorChoice p) {
  switch (p) {
    case PredictorChoice::kDeadReckoning:
      return "dead_reckoning";
    case PredictorChoice::kHoldLast:
      return "hold_last";
    case PredictorChoice::kOracle:
      return "oracle";
    case PredictorChoice::kKalman:
      break;
  }
  return "kalman";
}

const char* loss_name(LossModel m) {
  switch (m) {
    case LossModel::kIndependent:
      return "independent";
    case LossModel::kBurst:
      return "burst";
    case LossModel::kNone:
      break;
  }
  return "none";
}

}  // namespace

std::string canonical_text(const Scenario& s) {
  const auto seconds = [&](std::uint64_t ticks) { return num(static_cast<double>(ticks) * s.dt); };
  std::string o;
  const auto line = [&](const std::string& text) { o += text + "\n"; };

  line("version: " + uint(kScenarioVersion));
  line("name: " + quoted(s.name));
  line("seed: " + uint(s.seed));
  line("dt: " + num(s.dt));
  line("duration_s: " + seconds(s.duration_ticks));

  line("scene:");
  line(std::string("  type: ") + (s.terrain.type == TerrainType::kHills ? "hills" : "field"));
  line("  size_m: [" + num(s.terrain.size_x) + ", " + num(s.terrain.size_y) + "]");
  line("  cell_m: " + num(s.terrain.cell_size));
  line("  amplitude_m: " + num(s.terrain.amplitude));
  line("  obstacles:");
  line("    density_per_m2: " + num(s.obstacles.density_per_m2));
  line("    spawn_clear_radius_m: " + num(s.obstacles.spawn_clear_radius));

  line("swarm:");
  line("  drones: " + uint(s.drones));
  line("  formation:");
  line(std::string("    shape: ") + shape_name(s.formation_shape));
  line("    spacing_m: " + num(s.formation_spacing_m));
  line("  start:");
  line("    columns: " + uint(s.start.columns));
  line("    spacing_m: " + num(s.start.spacing_m));
  line("    jitter_m: " + num(s.start.jitter_m));
  line("    altitude_m: " + num(s.start.altitude_m));
  line("    altitude_jitter_m: " + num(s.start.altitude_jitter_m));

  line("mission:");
  line("  waypoints:");
  for (const Vec3& w : s.mission.waypoints) {
    line("    - [" + num(w.x) + ", " + num(w.y) + ", " + num(w.z) + "]");
  }
  line("  speed_mps: " + num(s.mission.speed));
  line(std::string("  loop: ") + (s.mission.loop ? "true" : "false"));

  line("weather:");
  line("  visibility_m: " + num(s.weather.visibility_m));
  line("  rain_mm_h: " + num(s.weather.rain_rate_mm_h));

  line("sensors:");
  line("  lidar:");
  line("    beams: " + uint(s.lidar_beams));
  line("    azimuth_samples: " + uint(s.lidar_azimuth_samples));
  line("  fix_period_ticks: " + uint(s.fix_period_ticks));

  line("comms:");
  line("  latency_ticks: " + uint(s.channel.latency_ticks));
  line("  jitter_ticks: " + uint(s.channel.jitter_ticks));
  line(std::string("  loss_model: ") + loss_name(s.channel.loss_model));
  line("  loss: " + num(s.channel.loss));
  line("  mean_burst: " + num(s.channel.mean_burst));
  line("  rate_limit_per_tick: " + uint(s.channel.max_msgs_per_sender_per_tick));
  if (s.channel.jammers.empty()) {
    line("  jammers: []");
  } else {
    line("  jammers:");
    for (const Jammer& j : s.channel.jammers) {
      line("    - center: [" + num(j.center.x) + ", " + num(j.center.y) + ", " + num(j.center.z) +
           "]");
      line("      radius_m: " + num(j.radius));
      line(std::string("      mode: ") + (j.mode == JamMode::kPulsed ? "pulsed" : "constant"));
      line("      from_s: " + seconds(j.start_tick));
      if (j.end_tick != std::numeric_limits<std::uint64_t>::max()) {
        line("      to_s: " + seconds(j.end_tick));
      }
      if (j.mode == JamMode::kPulsed) {
        line("      period_ticks: " + uint(j.pulse_period_ticks));
        line("      duty: " + num(j.pulse_duty));
        line("      offset_ticks: " + uint(j.pulse_offset_ticks));
      }
      line("      loss: " + num(j.loss_inside));
      line(std::string("      affects: ") +
           (j.affects == JamAffects::kEither ? "either" : "receiver"));
    }
  }

  line("control:");
  line(std::string("  predictor: ") + predictor_name(s.predictor));
  if (!s.fallback_enabled) {
    line("  fallback: false");
  }
  if (!s.separation_enabled) {
    line("  separation: false");
  }
  line("  report_period_ticks: " + uint(s.report_period_ticks));
  line("  staleness_ticks: " + uint(s.staleness_ticks));

  if (s.faults.empty()) {
    line("faults: []");
  } else {
    line("faults:");
    for (const Fault& f : s.faults) {
      line("  - drone: " + uint(f.drone));
      line("    at_s: " + seconds(f.tick));
    }
  }

  if (s.commands.empty()) {
    line("commands: []");
  } else {
    line("commands:");
    for (const ScenarioCommand& c : s.commands) {
      line("  - at_s: " + seconds(c.tick));
      line(std::string("    kind: ") + (c.kind == CommandKind::kHold ? "hold" : "resume"));
    }
  }

  line("metrics:");
  line("  settle_s: " + num(s.metrics.settle_s));
  line("  collision_radius_m: " + num(s.metrics.collision_radius_m));
  line("  recovery_threshold_m: " + num(s.metrics.recovery_threshold_m));
  line("  recovery_hold_s: " + num(s.metrics.recovery_hold_s));

  line("log:");
  line("  snapshot_every_ticks: " + uint(s.snapshot_every_ticks));
  return o;
}

}  // namespace swarm
