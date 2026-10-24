#include <algorithm>
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

// Sixteen lowercase hexadecimal digits, as `swarm_sim` prints hashes.
std::string hash_text(std::uint64_t v) {
  char buf[20];
  std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(v));
  return buf;
}

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
    case PredictorChoice::kTurning:
      return "turning";
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

bool wind_used(const Scenario& s) {
  return s.wind.mean.x != 0.0 || s.wind.mean.y != 0.0 || s.wind.gust_sigma != 0.0;
}

namespace {
bool environment_used(const Scenario& s) { return wind_used(s) || s.ground_contact_crashes; }
}  // namespace

std::uint64_t scenario_version(const Scenario& s) {
  if (s.vehicle.dynamics == FixedWingDynamics::kEnergy || environment_used(s) ||
      s.terrain_guidance.enabled || s.pattern.kind != PatternKind::kOrbit || s.altimeter_used ||
      s.forward_used || s.avoid.enabled) {
    return kScenarioFlightVersion;
  }
  return s.vehicle.kind == VehicleKind::kQuad ? 1 : kScenarioVehicleVersion;
}

std::string canonical_text(const Scenario& s) {
  const auto seconds = [&](std::uint64_t ticks) { return num(static_cast<double>(ticks) * s.dt); };
  std::string o;
  const auto line = [&](const std::string& text) { o += text + "\n"; };

  // A scenario is written as the lowest version that can express it, so a scenario that uses no
  // newer feature has the text, and the log header, it always had.
  line("version: " + uint(scenario_version(s)));
  line("name: " + quoted(s.name));
  line("seed: " + uint(s.seed));
  line("dt: " + num(s.dt));
  line("duration_s: " + seconds(s.duration_ticks));

  line("scene:");
  if (s.scene_file.used) {
    line("  source: file");
    line("  path: " + quoted(s.scene_file.path));
    if (s.scene_file.hash) {
      line("  hash: " + hash_text(*s.scene_file.hash));
    }
  } else {
    line(std::string("  type: ") + (s.terrain.type == TerrainType::kHills ? "hills" : "field"));
    line("  size_m: [" + num(s.terrain.size_x) + ", " + num(s.terrain.size_y) + "]");
    line("  cell_m: " + num(s.terrain.cell_size));
    line("  amplitude_m: " + num(s.terrain.amplitude));
    line("  obstacles:");
    line("    density_per_m2: " + num(s.obstacles.density_per_m2));
    line("    spawn_clear_radius_m: " + num(s.obstacles.spawn_clear_radius));
  }

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

  if (s.vehicle.kind == VehicleKind::kFixedWing) {
    const FixedWingParams& p = s.vehicle.fixed_wing;
    line("vehicle:");
    line("  type: fixed_wing");
    if (s.vehicle.dynamics == FixedWingDynamics::kSimple) {
      line("  min_speed_mps: " + num(p.min_speed));
    }
    line("  max_speed_mps: " + num(p.max_speed));
    line("  max_accel_mps2: " + num(p.max_accel));
    line("  max_decel_mps2: " + num(p.max_decel));
    line("  max_bank_deg: " + num(p.max_bank * 180.0 / kPi));
    line("  max_climb_rate_mps: " + num(p.max_climb_rate));
    line("  max_descent_rate_mps: " + num(p.max_descent_rate));
    line("  max_vertical_accel_mps2: " + num(p.max_vertical_accel));
    if (s.vehicle.dynamics == FixedWingDynamics::kEnergy) {
      const EnergyParams& e = s.vehicle.energy;
      line("  dynamics: energy");
      line("  energy:");
      line("    mass_kg: " + num(e.mass));
      line("    wing_area_m2: " + num(e.wing_area));
      line("    wingspan_m: " + num(e.wingspan));
      line("    cd0: " + num(e.cd0));
      line("    oswald: " + num(e.oswald));
      line("    cl_max: " + num(e.cl_max));
      line("    air_density_kg_m3: " + num(e.air_density));
      line("    max_thrust_n: " + num(e.max_thrust));
      line("    max_power_w: " + num(e.max_power));
      line("    stall_margin: " + num(e.stall_margin));
      line("    max_roll_rate_rad_s: " + num(e.max_roll_rate));
      line("    roll_tau_s: " + num(e.roll_tau));
      line("    pitch_tau_s: " + num(e.pitch_tau));
      line("    max_path_rate_rad_s: " + num(e.max_path_rate));
      line("    max_path_angle_rad: " + num(e.max_path_angle));
    }
  }
  if (environment_used(s)) {
    line("environment:");
    if (wind_used(s)) {
      line("  wind:");
      line("    mean_mps: [" + num(s.wind.mean.x) + ", " + num(s.wind.mean.y) + "]");
      line("    gust_mps: " + num(s.wind.gust_sigma));
      line("    gust_scale_m: " + num(s.wind.gust_scale));
      line("    vertical_ratio: " + num(s.wind.vertical_ratio));
    }
    if (s.ground_contact_crashes) {
      line("  ground_contact_crashes: true");
    }
  }
  line("weather:");
  line("  visibility_m: " + num(s.weather.visibility_m));
  line("  rain_mm_h: " + num(s.weather.rain_rate_mm_h));
  if (s.weather.fog_constant != kKoschmiederConstant) {
    line("  fog_constant: " + num(s.weather.fog_constant));
  }

  line("sensors:");
  if (s.vehicle.kind == VehicleKind::kFixedWing) {
    const CameraFixConfig& f = s.camera;
    line("  camera:");
    line("    sigma_floor_m: " + num(f.sigma_floor_m));
    line("    sigma_per_m_agl: " + num(f.sigma_per_m_agl));
    line("    min_agl_m: " + num(f.min_agl_m));
    line("    max_agl_m: " + num(f.max_agl_m));
    line("    min_transmission: " + num(f.min_transmission));
    line("    max_sigma_m: " + num(f.max_sigma_m));
    line("    dropout_probability: " + num(f.dropout_probability));
    if (s.altimeter_used) {
      const RadarAltimeterConfig& a = s.altimeter;
      line("  radar_altimeter:");
      line("    sigma_floor_m: " + num(a.sigma_floor_m));
      line("    sigma_per_m: " + num(a.sigma_per_m));
      line("    max_range_m: " + num(a.max_range_m));
      line("    dropout_probability: " + num(a.dropout_probability));
      line("    period_ticks: " + uint(a.period_ticks));
    }
    if (s.forward_used) {
      const ForwardSensorConfig& fw = s.forward;
      line("  forward:");
      line("    horizontal_fov_deg: " + num(fw.horizontal_fov_deg));
      line("    vertical_fov_deg: " + num(fw.vertical_fov_deg));
      line("    pitch_deg: " + num(fw.pitch_deg));
      line("    azimuth_rays: " + uint(fw.azimuth_rays));
      line("    elevation_rays: " + uint(fw.elevation_rays));
      line("    max_range_m: " + num(fw.max_range_m));
      line("    sigma_floor_m: " + num(fw.sigma_floor_m));
      line("    sigma_per_m: " + num(fw.sigma_per_m));
      line("    dropout_probability: " + num(fw.dropout_probability));
      line("    min_transmission: " + num(fw.min_transmission));
      line("    period_ticks: " + uint(fw.period_ticks));
    }
  } else {
    line("  lidar:");
    line("    beams: " + uint(s.lidar_beams));
    line("    azimuth_samples: " + uint(s.lidar_azimuth_samples));
  }
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
  if (s.vehicle.kind == VehicleKind::kFixedWing) {
    const OrbitParams& orbit = s.orbit;
    line("  orbit:");
    line("    radius_m: " + num(orbit.radius));
    line("    radius_step_m: " + num(orbit.radius_step));
    line("    altitude_step_m: " + num(orbit.altitude_step));
    line("    phase_gain: " + num(orbit.phase_gain));
    line("    radial_gain: " + num(orbit.radial_gain));
    line("    lookahead_s: " + num(orbit.lookahead_s));
    line("    consensus_weight: " + num(orbit.consensus_weight));
    line("    safe_distance_m: " + num(orbit.separation.d_safe));
    if (s.pattern.kind != PatternKind::kOrbit) {
      const PatternParams& q = s.pattern;
      line("  pattern:");
      line(std::string("    type: ") + (q.kind == PatternKind::kRacetrack ? "racetrack"
                                        : q.kind == PatternKind::kLanes   ? "lanes"
                                                                          : "formation"));
      line("    heading_deg: " + num(q.heading * 180.0 / kPi));
      line("    leg_length_m: " + num(q.leg_length));
      line("    turn_radius_m: " + num(q.turn_radius));
      line("    lane_spacing_m: " + num(q.lane_spacing));
      line("    stagger_s: " + num(q.stagger_s));
      line("    join_fraction: " + num(q.join_fraction));
      if (q.kind == PatternKind::kFormation) {
        const FormationPlan& f = q.formation;
        line("    formation:");
        line("      along_m: " + num(f.along));
        line("      across_m: " + num(f.across));
        line("      transition_s: " + num(f.transition_s));
        line("      position_gain: " + num(f.position_gain));
        line("      velocity_gain: " + num(f.velocity_gain));
        // Keys added after the formation pattern are written only when they are used, so the text
        // of earlier scenarios does not change.
        if (f.altitude_stagger != 0.0) {
          line("      altitude_stagger_m: " + num(f.altitude_stagger));
        }
        if (std::any_of(f.phases.begin(), f.phases.end(),
                        [](const FormationPhase& p) { return p.shape == FormationStyle::kRake; })) {
          line("      rake:");
          line("        agl_m: " + num(f.rake_agl));
          line("        hfov_deg: " + num(f.rake_hfov_deg));
          line("        overlap: " + num(f.rake_overlap));
        }
        if (!f.phases.empty()) {
          line("      phases:");
          for (const FormationPhase& phase : f.phases) {
            line("        - at_s: " + num(phase.at_s));
            line(std::string("          shape: ") +
                 (phase.shape == FormationStyle::kTrail          ? "trail"
                  : phase.shape == FormationStyle::kEchelonRight ? "echelon_right"
                  : phase.shape == FormationStyle::kEchelonLeft  ? "echelon_left"
                  : phase.shape == FormationStyle::kRake         ? "rake"
                                                                 : "vee"));
          }
        }
      }
    }
    if (s.terrain_guidance.enabled) {
      const TerrainGuidanceParams& t = s.terrain_guidance;
      line("  terrain:");
      line("    clearance_m: " + num(t.clearance));
      line("    footprint_m: " + num(t.footprint));
      line("    climb_fraction: " + num(t.climb_fraction));
      line("    sample_spacing_m: " + num(t.sample_spacing));
      line("    lookahead_m: " + num(t.lookahead));
      const TerrainMapFaults& faults = s.map_faults;
      if (faults.offset_m != 0.0) {
        line("    offset_m: " + num(faults.offset_m));
      }
      if (faults.error_sigma_m > 0.0) {
        line("    error_sigma_m: " + num(faults.error_sigma_m));
        line("    error_correlation_m: " + num(faults.error_correlation_m));
      }
      if (!faults.omit.empty()) {
        line("    omit:");
        for (const OmitRect& r : faults.omit) {
          line("      - x_m: [" + num(r.x_min) + ", " + num(r.x_max) + "]");
          line("        y_m: [" + num(r.y_min) + ", " + num(r.y_max) + "]");
        }
      }
    }
    if (s.avoid.enabled) {
      const AvoidParams& av = s.avoid;
      line("  avoid:");
      line("    horizon_s: " + num(av.horizon_s));
      line("    hold_s: " + num(av.hold_s));
      line("    bank_fraction: " + num(av.bank_fraction));
      if (av.rejoin) {
        line("    rejoin: true");
      }
      if (av.feasibility == AvoidFeasibility::kKinematic) {
        line("    feasibility: kinematic");
      }
      if (av.clearance_m) {
        line("    clearance_m: " + num(*av.clearance_m));
      }
      if (av.urgent_s > 0.0) {
        line("    urgent_s: " + num(av.urgent_s));
      }
      if (av.turn_first) {
        line("    turn_first: true");
      }
      if (av.catchup_cap_mps) {
        line("    catchup_cap_mps: " + num(*av.catchup_cap_mps));
      }
    }
    if (s.share.mode != ShareMode::kOff) {
      const ShareParams& sh = s.share;
      line("  share:");
      line(std::string("    mode: ") + (sh.mode == ShareMode::kMemory ? "memory" : "share"));
      line("    ttl_s: " + num(sh.ttl_s));
      line("    points_per_message: " + uint(sh.points_per_message));
      if (sh.period_ticks > 0) {
        line("    period_ticks: " + uint(sh.period_ticks));
      }
      line("    repeat_s: " + num(sh.repeat_s));
      line("    min_sources: " + uint(sh.min_sources));
    }
  }
  line("  report_period_ticks: " + uint(s.report_period_ticks));
  line("  staleness_ticks: " + uint(s.staleness_ticks));
  if (s.drop_stale_after_ticks > 0) {
    line("  drop_stale_after_ticks: " + uint(s.drop_stale_after_ticks));
  }

  if (s.faults.empty()) {
    line("faults: []");
  } else {
    line("faults:");
    for (const Fault& f : s.faults) {
      line("  - drone: " + uint(f.drone));
      line("    at_s: " + seconds(f.tick));
      if (f.kind == FaultKind::kFalseHazards) {
        line("    kind: false_hazards");
      }
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
