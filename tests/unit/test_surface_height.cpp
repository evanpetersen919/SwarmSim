#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "swarm/drone_agent.hpp"
#include "swarm/format/scene.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"
#include "swarm/scene.hpp"
#include "swarm/simulation.hpp"
#include "swarm/terrain.hpp"

using swarm::Obstacle;
using swarm::ObstacleKind;
using swarm::Scene;
using swarm::SceneData;

namespace {

SceneData flat_scene() {
  swarm::TerrainConfig c;
  c.size_x = 2000.0;
  c.size_y = 2000.0;
  c.cell_size = 10.0;
  c.type = swarm::TerrainType::kField;
  c.amplitude = 0.0;
  return swarm::generate_terrain(c, 1);
}

Obstacle make_obstacle(ObstacleKind kind, double cx, double cy, double half_x, double half_y,
                       double base_z, double height) {
  Obstacle o;
  o.kind = kind;
  o.cx = cx;
  o.cy = cy;
  o.half_x = half_x;
  o.half_y = half_y;
  o.base_z = base_z;
  o.height = height;
  return o;
}

}  // namespace

TEST(SurfaceHeight, WithoutObstaclesItIsTheGround) {
  const Scene scene(flat_scene());
  EXPECT_EQ(scene.surface_height(30.0, -40.0), scene.ground_height(30.0, -40.0));
}

TEST(SurfaceHeight, ABoxAndACylinderRaiseTheSurfaceOnlyInsideTheirFootprints) {
  SceneData data = flat_scene();
  data.obstacles.push_back(make_obstacle(ObstacleKind::kBox, 100.0, 0.0, 20.0, 10.0, 0.0, 80.0));
  data.obstacles.push_back(
      make_obstacle(ObstacleKind::kCylinder, -100.0, 0.0, 15.0, 0.0, 0.0, 60.0));
  const Scene scene(std::move(data));
  const double ground = scene.ground_height(0.0, 0.0);

  EXPECT_DOUBLE_EQ(scene.surface_height(100.0, 0.0), 80.0 + 0.0);
  EXPECT_DOUBLE_EQ(scene.surface_height(119.0, 9.0), 80.0);
  EXPECT_EQ(scene.surface_height(121.0, 0.0), scene.ground_height(121.0, 0.0));
  EXPECT_EQ(scene.surface_height(100.0, 11.0), scene.ground_height(100.0, 11.0));

  EXPECT_DOUBLE_EQ(scene.surface_height(-100.0, 0.0), 60.0);
  EXPECT_DOUBLE_EQ(scene.surface_height(-100.0, 14.0), 60.0);
  // The corner of the cylinder's bounding box is outside the circle.
  EXPECT_EQ(scene.surface_height(-86.0, 14.0), scene.ground_height(-86.0, 14.0));
  EXPECT_DOUBLE_EQ(scene.surface_height(0.0, 0.0), ground);
}

TEST(SurfaceHeight, OutsideTheTerrainExtentItIsTheClampedGround) {
  SceneData data = flat_scene();
  data.obstacles.push_back(make_obstacle(ObstacleKind::kBox, 0.0, 0.0, 20.0, 20.0, 0.0, 80.0));
  const Scene scene(std::move(data));
  EXPECT_EQ(scene.surface_height(1.0e6, 0.0), scene.ground_height(1.0e6, 0.0));
}

namespace {

std::string wall_scene_path(bool with_wall) {
  SceneData data = flat_scene();
  if (with_wall) {
    // A wall across the orbit's start, far taller than the aircraft's altitude.
    data.obstacles.push_back(
        make_obstacle(ObstacleKind::kBox, 150.0, 0.0, 40.0, 400.0, 0.0, 300.0));
  }
  std::vector<std::uint8_t> bytes;
  EXPECT_EQ(swarm::serialize_scene(data, bytes), swarm::SceneStatus::kOk);
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      (with_wall ? "swarmsim_test_surface_wall.swscene" : "swarmsim_test_surface_open.swscene");
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out.write(reinterpret_cast<const char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
  return path.generic_string();
}

struct Flown {
  int crashed = 0;
  double min_agl = 1.0e9;
};

Flown fly(bool with_wall) {
  const std::string text =
      "version: 3\n"
      "seed: 5\n"
      "duration_s: 40\n"
      "scene: {source: file, path: \"" +
      wall_scene_path(with_wall) +
      "\"}\n"
      "swarm: {drones: 2}\n"
      "mission:\n"
      "  waypoints:\n"
      "    - [0, 0, 30]\n"
      "  speed_mps: 17\n"
      "vehicle: {type: fixed_wing}\n"
      "environment: {ground_contact_crashes: true}\n"
      "weather: {visibility_m: 20000}\n"
      "control:\n"
      "  orbit: {radius_m: 150, radius_step_m: 12, altitude_step_m: 10}\n";
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok()) << (r.errors.empty() ? "" : swarm::to_string(r.errors.front()));
  const swarm::ScenarioWorld world = swarm::build_world(r.scenario);
  const std::unique_ptr<swarm::Simulation> sim = world.make_simulation(1);
  Flown out;
  for (std::uint64_t tick = 0; tick < r.scenario.duration_ticks; ++tick) {
    sim->step();
    for (std::size_t i = 0; i < r.scenario.drones; ++i) {
      const swarm::Vec3 p = sim->agent(i).position();
      out.min_agl = std::min(out.min_agl, p.z - world.scene->surface_height(p.x, p.y));
    }
  }
  for (std::size_t i = 0; i < r.scenario.drones; ++i) {
    out.crashed += dynamic_cast<const swarm::DroneAgent&>(sim->agent(i)).crashed() ? 1 : 0;
  }
  return out;
}

}  // namespace

TEST(SurfaceHeightFlight, AnAircraftFlyingIntoATallObstacleCrashesAndOpenGroundDoesNot) {
  const Flown open = fly(false);
  EXPECT_EQ(open.crashed, 0);
  EXPECT_GT(open.min_agl, 0.0);

  const Flown wall = fly(true);
  EXPECT_GT(wall.crashed, 0);
}
