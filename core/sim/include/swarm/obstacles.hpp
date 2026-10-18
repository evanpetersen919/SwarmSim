#pragma once

#include <cstddef>
#include <cstdint>

#include "swarm/format/scene.hpp"

namespace swarm {

struct ObstacleConfig {
  double density_per_m2 = 0.0005;  // obstacles per square meter of terrain
  double min_spacing = 5.0;        // meters between the bounding circles of two obstacles
  double cylinder_fraction = 0.5;  // share of obstacles that are cylinders, in [0, 1]

  double box_half_min = 2.0;  // half extent of a box in x and in y, each drawn from this range
  double box_half_max = 6.0;
  double box_height_min = 4.0;
  double box_height_max = 20.0;
  double cylinder_radius_min = 0.5;
  double cylinder_radius_max = 1.5;
  double cylinder_height_min = 4.0;
  double cylinder_height_max = 15.0;

  double spawn_x = 0.0;  // no obstacle comes closer than spawn_clear_radius to this point
  double spawn_y = 0.0;
  double spawn_clear_radius = 30.0;

  double edge_margin = 5.0;  // gap kept between an obstacle's bounding circle and the terrain edge
  double embed = 0.5;  // meters an obstacle's base is buried below the lowest footprint height
  std::uint32_t attempts_per_obstacle = 30;
};

// Appends obstacles to `scene` by seeded rejection sampling and returns how many were placed.
//
// The target count is round(density * terrain area). Each attempt draws the same six random
// values whether or not it is accepted, then is rejected if its bounding circle leaves the
// allowed area, comes too close to the spawn point, or comes within `min_spacing` of the bounding
// circle of any obstacle already in the scene. The attempt budget is `attempts_per_obstacle` times
// the target, so a crowded scene ends with fewer obstacles than the target instead of looping.
//
// Spacing and clearance use bounding circles (a box's is the circle through its corners), which
// is conservative for boxes. The base of each obstacle is set to the lowest sampled terrain height
// under its footprint minus `embed`, so obstacles do not float on slopes. The sampling is dense
// but not exhaustive, which is what `embed` absorbs.
//
// Throws std::invalid_argument for an invalid configuration, an invalid scene, or a target that
// would exceed the format's obstacle limit. Uses random stream kStreamScene.
std::size_t place_obstacles(SceneData& scene, const ObstacleConfig& config, std::uint64_t seed);

}  // namespace swarm
