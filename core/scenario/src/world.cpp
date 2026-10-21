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
      const Vec3 slot = orbit_reference(i, s.drones, 0.0, orbit_params(s)).position;
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

ScenarioWorld build_world(const Scenario& s) {
  ScenarioWorld world;
  world.scenario = s;

  SceneData data;
  if (s.scene_file.used) {
    std::string error;
    if (!read_scene_file(s.scene_file.path, data, error)) {
      throw std::invalid_argument("build_world: " + error);
    }
    // The world's scenario records the file's hash, so a log's header names the exact scene.
    const std::uint64_t hash = scene_hash(data);
    if (s.scene_file.hash && *s.scene_file.hash != hash) {
      throw std::invalid_argument("build_world: the scene file " + s.scene_file.path +
                                  " has hash " + hash_text(hash) + " but the scenario expects " +
                                  hash_text(*s.scene_file.hash));
    }
    world.scenario.scene_file.hash = hash;
  } else {
    data = generate_terrain(s.terrain, s.seed);
    ObstacleConfig obstacles = s.obstacles;
    obstacles.spawn_x = s.mission.waypoints.front().x;  // keep the start area clear
    obstacles.spawn_y = s.mission.waypoints.front().y;
    place_obstacles(data, obstacles, s.seed);
  }
  world.scene = std::make_unique<Scene>(std::move(data));

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
    }
    c.camera = s.camera;
    c.fixed_wing = s.vehicle.fixed_wing;
    if (s.vehicle.kind == VehicleKind::kFixedWing) {
      // An aircraft starts flying along its orbit, at the speed of its slot. At rest it would be at
      // minimum speed along its heading after one step, and the pose estimator would start with the
      // wrong velocity.
      c.start_velocity = orbit_reference(i, s.drones, 0.0, c.brain.orbit).velocity;
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
