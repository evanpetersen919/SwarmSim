#include "swarm/scenario/world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>

#include "swarm/rng.hpp"
#include "swarm/scenario/scene_file.hpp"
#include "swarm/streams.hpp"

namespace swarm {

std::vector<Vec3> start_positions(const Scenario& s) {
  const StartLayout& layout = s.start;
  const std::uint32_t columns = layout.columns;
  const std::uint32_t rows = (s.drones + columns - 1) / columns;
  const Vec3 center = s.mission.waypoints.front();

  Rng rng(s.seed, kStreamStart);
  std::vector<Vec3> starts;
  starts.reserve(s.drones);
  for (std::uint32_t i = 0; i < s.drones; ++i) {
    const double col = static_cast<double>(i % columns) - 0.5 * static_cast<double>(columns - 1);
    const double row = static_cast<double>(i / columns) - 0.5 * static_cast<double>(rows - 1);
    // Three draws per drone, always, so the jitter amounts never shift another drone's draws.
    const double jx = rng.uniform(-1.0, 1.0);
    const double jy = rng.uniform(-1.0, 1.0);
    const double jz = rng.uniform(-1.0, 1.0);
    if (s.vehicle.kind == VehicleKind::kFixedWing) {
      // An aircraft starts on station: on its slot of the orbit, with the jitter added. Launching
      // and joining the orbit are not part of what is studied, and a swarm launched in a grid at
      // one altitude has to cross itself to spread onto its circles. The grid keys do not apply.
      const Vec3 slot =
          s.pattern.kind == PatternKind::kOrbit
              ? orbit_reference(i, s.drones, 0.0, orbit_params(s)).position
              : pattern_reference(i, s.drones, 0.0, s.pattern, orbit_params(s)).position;
      starts.push_back({slot.x + jx * layout.jitter_m, slot.y + jy * layout.jitter_m,
                        slot.z + jz * layout.altitude_jitter_m});
      continue;
    }
    starts.push_back({center.x + col * layout.spacing_m + jx * layout.jitter_m,
                      center.y + row * layout.spacing_m + jy * layout.jitter_m,
                      layout.altitude_m + jz * layout.altitude_jitter_m});
  }
  return starts;
}

namespace {

std::string hash_text(std::uint64_t v) {
  char buf[20];
  std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(v));
  return buf;
}

}  // namespace

OrbitParams orbit_params(const Scenario& s) {
  OrbitParams p = s.orbit;
  if (!s.mission.waypoints.empty()) {
    p.center = s.mission.waypoints.front();
  }
  p.speed = s.mission.speed;
  p.min_speed = s.vehicle.fixed_wing.min_speed;
  p.max_speed = s.vehicle.fixed_wing.max_speed;
  return p;
}

bool build_scene_data(const Scenario& s, SceneData& data, std::string& error) {
  if (s.scene_file.used) {
    if (!read_scene_file(s.scene_file.path, data, error)) {
      return false;
    }
    const std::uint64_t hash = scene_hash(data);
    if (s.scene_file.hash && *s.scene_file.hash != hash) {
      error = "the scene file " + s.scene_file.path + " has hash " + hash_text(hash) +
              " but the scenario expects " + hash_text(*s.scene_file.hash);
      return false;
    }
    return true;
  }
  data = generate_terrain(s.terrain, s.seed);
  ObstacleConfig obstacles = s.obstacles;
  obstacles.spawn_x = s.mission.waypoints.front().x;  // keep the start area clear
  obstacles.spawn_y = s.mission.waypoints.front().y;
  place_obstacles(data, obstacles, s.seed);
  return true;
}

TerrainMap make_terrain_map(const SceneData& scene) {
  const Heightfield& field = scene.terrain;
  std::vector<double> heights = field.heights;
  const std::size_t stride = field.samples_x();
  for (const Obstacle& o : scene.obstacles) {
    const double top = o.base_z + o.height;
    const double hx = o.half_x;
    const double hy = o.kind == ObstacleKind::kCylinder ? o.half_x : o.half_y;
    // The samples whose cells the footprint's bounding box touches.
    const auto first = [&](double v, double origin) {
      return std::max(0.0, std::floor((v - origin) / field.cell_size));
    };
    const auto last = [&](double v, double origin, std::size_t samples) {
      return std::min(static_cast<double>(samples - 1), std::ceil((v - origin) / field.cell_size));
    };
    const double i0 = first(o.cx - hx, field.origin_x);
    const double i1 = last(o.cx + hx, field.origin_x, field.samples_x());
    const double j0 = first(o.cy - hy, field.origin_y);
    const double j1 = last(o.cy + hy, field.origin_y, field.samples_y());
    for (double j = j0; j <= j1; j += 1.0) {
      for (double i = i0; i <= i1; i += 1.0) {
        double& h = heights[static_cast<std::size_t>(j) * stride + static_cast<std::size_t>(i)];
        h = std::max(h, top);
      }
    }
  }
  return TerrainMap(field.cells_x, field.cells_y, field.origin_x, field.origin_y, field.cell_size,
                    std::move(heights));
}

ScenarioWorld build_world(const Scenario& s) {
  ScenarioWorld world;
  world.scenario = s;

  SceneData data;
  std::string scene_error;
  if (!build_scene_data(s, data, scene_error)) {
    throw std::invalid_argument("build_world: " + scene_error);
  }
  if (s.scene_file.used) {
    // The world's scenario records the file's hash, so a log's header names the exact scene.
    world.scenario.scene_file.hash = scene_hash(data);
  }
  world.scene = std::make_unique<Scene>(std::move(data));
  std::shared_ptr<const TerrainMap> terrain_map;
  if (s.terrain_guidance.enabled) {
    terrain_map = std::make_shared<const TerrainMap>(make_terrain_map(world.scene->data()));
  }

  world.sim.dt = s.dt;
  world.sim.seed = s.seed;
  world.sim.channel = s.channel;

  world.offsets = make_formation(s.formation_shape, s.drones, s.formation_spacing_m);
  const std::vector<Vec3> starts = start_positions(s);

  for (std::uint32_t i = 0; i < s.drones; ++i) {
    DroneAgentConfig c;
    c.brain.id = i;
    c.brain.num_drones = s.drones;
    c.brain.dt = s.dt;
    c.brain.offsets = world.offsets;
    c.brain.mission = s.mission;
    c.brain.pose.dt = s.dt;
    c.brain.predictor_config.dt = s.dt;
    switch (s.predictor) {
      case PredictorChoice::kDeadReckoning:
        c.brain.predictor = PredictorKind::kDeadReckoning;
        break;
      case PredictorChoice::kTurning:
        c.brain.predictor = PredictorKind::kTurning;
        break;
      case PredictorChoice::kHoldLast:
        c.brain.predictor = PredictorKind::kHoldLast;
        break;
      case PredictorChoice::kKalman:
      case PredictorChoice::kOracle:
        c.brain.predictor = PredictorKind::kKalman;
        break;
    }
    c.brain.fallback_enabled = s.fallback_enabled;
    c.brain.separation_enabled = s.separation_enabled;
    c.brain.report_period_ticks = s.report_period_ticks;
    c.brain.staleness_ticks = s.staleness_ticks;
    c.start = starts[i];
    c.vehicle.max_accel = c.brain.separation.max_accel;
    c.vehicle_kind = s.vehicle.kind;
    if (s.vehicle.kind == VehicleKind::kFixedWing) {
      c.brain.guidance = Guidance::kOrbit;
      c.brain.orbit = orbit_params(s);
      c.brain.pattern = s.pattern;
    }
    c.camera = s.camera;
    c.fixed_wing = s.vehicle.fixed_wing;
    c.crash_on_ground = s.ground_contact_crashes;
    if (s.terrain_guidance.enabled && s.vehicle.kind == VehicleKind::kFixedWing) {
      c.brain.terrain = terrain_map;
      c.brain.terrain_guidance = s.terrain_guidance;
      c.brain.terrain_guidance.climb_rate = s.vehicle.fixed_wing.max_climb_rate;
    }
    c.dynamics = s.vehicle.dynamics;
    c.energy = s.vehicle.energy;
    c.wind = s.wind;
    if (s.vehicle.kind == VehicleKind::kFixedWing) {
      // An aircraft starts flying along its orbit, at the speed of its slot. At rest it would be at
      // minimum speed along its heading after one step, and the pose estimator would start with the
      // wrong velocity.
      c.start_velocity =
          s.pattern.kind == PatternKind::kOrbit
              ? orbit_reference(i, s.drones, 0.0, c.brain.orbit).velocity
              : pattern_reference(i, s.drones, 0.0, s.pattern, c.brain.orbit).velocity;
      if (s.terrain_guidance.enabled) {
        // Over high ground an aircraft starts at the altitude that clears it, with its layer.
        const double ground = world.scene->ground_height(starts[i].x, starts[i].y);
        const double slot_offset = static_cast<double>(i) * c.brain.orbit.altitude_step;
        c.start.z = std::max(c.start.z, ground + s.terrain_guidance.clearance + slot_offset);
      }
    }
    c.lidar.beams = s.lidar_beams;
    c.lidar.azimuth_samples = s.lidar_azimuth_samples;
    c.weather = s.weather;
    c.seed = s.seed;
    c.fix_period_ticks = s.fix_period_ticks;
    c.oracle = s.predictor == PredictorChoice::kOracle;
    for (const Fault& fault : s.faults) {
      if (fault.drone == i) {
        c.fail_tick = fault.tick;
      }
    }
    world.agents.push_back(std::move(c));
  }

  if (!s.commands.empty()) {
    GroundStationConfig station;
    station.drones = s.drones;
    for (const ScenarioCommand& c : s.commands) {
      station.schedule.push_back({c.tick, c.kind});
    }
    world.station = std::move(station);
  }

  // The brain rejects slots that are not farther apart than the separation margin. Say so here,
  // in terms of the scenario.
  if (s.drones > 1) {
    const double margin = world.agents.front().brain.separation.max_margin;
    for (std::size_t a = 0; a < world.offsets.size(); ++a) {
      for (std::size_t b = a + 1; b < world.offsets.size(); ++b) {
        if (!(distance(world.offsets[a], world.offsets[b]) > margin)) {
          throw std::invalid_argument(
              "build_world: swarm.formation.spacing_m must exceed the separation margin (" +
              std::to_string(margin) + " m)");
        }
      }
    }
  }
  return world;
}

std::unique_ptr<Simulation> ScenarioWorld::make_simulation(
    std::size_t threads, const ChannelFactory& channel_factory) const {
  std::vector<std::unique_ptr<IAgent>> built;
  built.reserve(agents.size());
  for (const DroneAgentConfig& config : agents) {
    built.push_back(std::make_unique<DroneAgent>(config, *scene));
  }
  if (station) {
    built.push_back(std::make_unique<GroundStationAgent>(*station));
  }
  if (!channel_factory) {
    return std::make_unique<Simulation>(sim, std::move(built), threads);
  }
  ChannelConfig channel = sim.channel;
  channel.seed = sim.seed;  // as the simulation seeds its own channel
  std::unique_ptr<IChannel> made =
      channel_factory(static_cast<std::uint32_t>(built.size()), channel);
  if (made == nullptr) {
    throw std::invalid_argument("make_simulation: the channel factory returned no channel");
  }
  return std::make_unique<Simulation>(sim, std::move(built), threads, std::move(made));
}

}  // namespace swarm
