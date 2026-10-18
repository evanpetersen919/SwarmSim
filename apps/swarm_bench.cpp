// Measures ray casting and LiDAR scan throughput on a generated scene. Not a test: timings depend
// on the machine, so run it in a Release build and report the hardware, compiler, and commit with
// any numbers taken from it.

#include <chrono>
#include <cstdint>
#include <cstdio>

#include "swarm/lidar.hpp"
#include "swarm/obstacles.hpp"
#include "swarm/rng.hpp"
#include "swarm/scene.hpp"
#include "swarm/terrain.hpp"

namespace {

using Clock = std::chrono::steady_clock;

double seconds_since(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

}  // namespace

int main() {
  swarm::TerrainConfig terrain;
  terrain.size_x = 800.0;
  terrain.size_y = 800.0;
  terrain.cell_size = 4.0;
  terrain.type = swarm::TerrainType::kHills;
  terrain.amplitude = 30.0;
  terrain.feature_scale = 200.0;
  swarm::SceneData data = swarm::generate_terrain(terrain, 1);
  swarm::ObstacleConfig obstacles;
  obstacles.density_per_m2 = 0.002;
  const std::size_t placed = swarm::place_obstacles(data, obstacles, 1);
  const swarm::Scene scene(data);

  std::printf("scene: %.0f x %.0f m, %zu terrain cells, %zu obstacles\n", terrain.size_x,
              terrain.size_y, static_cast<std::size_t>(data.terrain.cells_x) * data.terrain.cells_y,
              placed);

  // Raw ray casting: random rays from random points above the terrain.
  {
    swarm::Rng rng(7, 0);
    const int rays = 2000000;
    int hits = 0;
    const auto start = Clock::now();
    for (int i = 0; i < rays; ++i) {
      const swarm::Vec3 origin{rng.uniform(-390.0, 390.0), rng.uniform(-390.0, 390.0),
                               rng.uniform(5.0, 60.0)};
      const swarm::Vec3 dir =
          swarm::normalized(swarm::Vec3{rng.normal(), rng.normal(), rng.normal() - 0.5});
      swarm::RayHit hit;
      hits += scene.raycast(origin, dir, 60.0, hit) ? 1 : 0;
    }
    const double elapsed = seconds_since(start);
    std::printf("raycast: %d rays in %.3f s = %.2f M rays/s (%d hits)\n", rays, elapsed,
                rays / elapsed / 1e6, hits);
  }

  // LiDAR scans with the default 16 x 90 configuration.
  {
    swarm::Lidar lidar(swarm::LidarConfig{}, 7, 0);
    const int scans = 2000;
    std::size_t valid = 0;
    swarm::Rng rng(9, 0);
    const auto start = Clock::now();
    for (int i = 0; i < scans; ++i) {
      const swarm::Vec3 origin{rng.uniform(-300.0, 300.0), rng.uniform(-300.0, 300.0), 25.0};
      const swarm::Scan scan = lidar.scan(origin, rng.uniform(-3.0, 3.0), scene);
      for (const auto& r : scan.returns) {
        valid += r.valid ? 1U : 0U;
      }
    }
    const double elapsed = seconds_since(start);
    std::printf(
        "lidar %ux%u: %d scans in %.3f s = %.0f scans/s, %.2f M rays/s (%zu valid returns)\n",
        lidar.config().beams, lidar.config().azimuth_samples, scans, elapsed, scans / elapsed,
        static_cast<double>(scans) * static_cast<double>(lidar.ray_count()) / elapsed / 1e6, valid);
  }
  return 0;
}
